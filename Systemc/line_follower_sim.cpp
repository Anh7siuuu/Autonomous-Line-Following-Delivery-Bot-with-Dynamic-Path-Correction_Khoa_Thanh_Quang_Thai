#include <systemc.h>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <cmath>
#include <thread>
#include <mutex>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <cstring>

#define SC_MS sc_time(10, SC_MS)

// ====================== Cảm biến hồng ngoại (IR Sensor) =========================
SC_MODULE(IRSensor) {
    sc_in<bool> ir_ol_in, ir_il_in, ir_ir_in, ir_or_in;
    SC_CTOR(IRSensor) { SC_THREAD(monitor); }
    void monitor() { while (true) { wait(SC_MS); } }
};

// ====================== Driver & Motor =====================
SC_MODULE(L298N) {
    sc_in<double> pwm_in; sc_out<double> speed_out;
    SC_CTOR(L298N) { SC_METHOD(update); sensitive << pwm_in; }
    void update() { speed_out.write(pwm_in.read()); }
};

SC_MODULE(DCMotor) {
    sc_in<double> speed_in; sc_out<double> actual_out;
    SC_CTOR(DCMotor) { SC_METHOD(run); sensitive << speed_in; }
    void run() { actual_out.write(speed_in.read()); }
};

// ====================== Môi trường Line =================
SC_MODULE(LineEnvironment) {
    sc_in<double> motor_left_speed, motor_right_speed;
    sc_out<bool> ir_ol_out, ir_il_out, ir_ir_out, ir_or_out;

    double pos = 0.0;
    const double drift_rate = 0.0015;
    const double SENSOR_POS[4] = {-0.3, -0.1, 0.1, 0.3};
    const double LINE_WIDTH_HALF = 0.06;

    SC_CTOR(LineEnvironment) {
        SC_THREAD(simulate);
        sensitive << motor_left_speed << motor_right_speed;
    }

    void simulate() {
        while (true) {
            double vL = motor_left_speed.read();
            double vR = motor_right_speed.read();
            pos += ((vL - vR) / 190.0) * 0.02 + drift_rate;
            pos = std::min(2.0, std::max(-2.0, pos));

            double rel_line = 0.0 - pos;
            ir_ol_out.write(std::abs(SENSOR_POS[0] - rel_line) < LINE_WIDTH_HALF);
            ir_il_out.write(std::abs(SENSOR_POS[1] - rel_line) < LINE_WIDTH_HALF);
            ir_ir_out.write(std::abs(SENSOR_POS[2] - rel_line) < LINE_WIDTH_HALF);
            ir_or_out.write(std::abs(SENSOR_POS[3] - rel_line) < LINE_WIDTH_HALF);
            wait(SC_MS);
        }
    }
};

// ====================== Controller (PID + Kalman + WiFi) ======================
SC_MODULE(Controller) {
    sc_in<bool> ir_ol_in, ir_il_in, ir_ir_in, ir_or_in;
    sc_in<double> actual_l, actual_r;
    sc_out<double> pwm_l, pwm_r;

    // Params
    double Kp = 35.0, Ki = 0.05, Kd = 20.0;
    double Q = 0.01, R = 0.1, Kt = 0.01; // Kalman Params
    std::mutex mtx;

    // Kalman State
    double x_est = 0.0, P = 1.0;
    
    // PID State
    double integral = 0.0, last_err = 0.0, base_speed = 90.0;

    int server_fd; bool running = true; std::thread net_thread;

    SC_CTOR(Controller) {
        SC_THREAD(control_loop);
        net_thread = std::thread(&Controller::wifi_listener, this);
    }
    ~Controller() { running = false; shutdown(server_fd, SHUT_RDWR); close(server_fd); if(net_thread.joinable()) net_thread.join(); }

    void wifi_listener() {
        struct sockaddr_in addr; int addrlen = sizeof(addr);
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        int opt = 1; setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
        addr.sin_family = AF_INET; addr.sin_addr.s_addr = INADDR_ANY; addr.sin_port = htons(8888);
        bind(server_fd, (struct sockaddr *)&addr, sizeof(addr));
        listen(server_fd, 3);

        char buf[1024];
        while (running) {
            int sock = accept(server_fd, (struct sockaddr *)&addr, (socklen_t*)&addrlen);
            if (sock < 0) continue;
            int len = read(sock, buf, 1024);
            if (len > 0) {
                buf[len] = '\0';
                double nkp, nki, nkd, nq, nr, nkt;
                if (sscanf(buf, "%lf %lf %lf %lf %lf %lf", &nkp, &nki, &nkd, &nq, &nr, &nkt) == 6) {
                    std::lock_guard<std::mutex> lock(mtx);
                    Kp=nkp; Ki=nki; Kd=nkd; Q=nq; R=nr; Kt=nkt;
                }
            }
            close(sock);
        }
    }

    double kalman_filter(double measurement) {
        // Predict
        P = P + Q;
        // Update
        double K = P / (P + R);
        x_est = x_est + K * (measurement - x_est);
        P = (1 - K) * P;
        return x_est;
    }

    void control_loop() {
        std::cout << "\n[WIFI] Tuning: echo \"Kp Ki Kd Q R Kt\" | nc localhost 8888" << std::endl;
        std::cout << std::setw(115) << std::setfill('-') << "" << std::setfill(' ') << std::endl;
        std::cout << "| Time | RawErr | FilErr | PIDOut | SpdL | SpdR |  Kp  |  Ki  |  Kd  |  Q   |  R   |  Kt  |" << std::endl;
        std::cout << std::setw(115) << std::setfill('-') << "" << std::setfill(' ') << std::endl;

        while (true) {
            double raw_err = 0; int active = 0;
            if(ir_ol_in.read()){raw_err-=3; active++;} if(ir_il_in.read()){raw_err-=1; active++;}
            if(ir_ir_in.read()){raw_err+=1; active++;} if(ir_or_in.read()){raw_err+=3; active++;}
            if(active > 0) raw_err /= active; else raw_err = last_err;

            double filtered_err = kalman_filter(raw_err);

            std::lock_guard<std::mutex> lock(mtx);
            double p_term = Kp * filtered_err;
            integral = std::min(100.0, std::max(-100.0, integral + filtered_err * 0.01));
            double derivative = (filtered_err - last_err) / 0.01;
            double turn = p_term + Ki * integral + Kd * derivative;

            double L = std::min(190.0, std::max(0.0, base_speed + turn));
            double R_spd = std::min(190.0, std::max(0.0, base_speed - turn));

            pwm_l.write(L); pwm_r.write(R_spd);
            last_err = filtered_err;

            static int c = 0;
            if (c++ % 10 == 0) {
                std::cout << "| " << std::setw(4) << sc_time_stamp().to_seconds() << " | " 
                          << std::setw(6) << raw_err << " | " << std::setw(6) << filtered_err << " | "
                          << std::setw(6) << turn << " | " << std::setw(4) << (int)L << " | " << std::setw(4) << (int)R_spd << " | "
                          << std::fixed << std::setprecision(2) << Kp << " | " << Ki << " | " << Kd << " | " << Q << " | " << R << " | " << Kt << " |" << "\r" << std::flush;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            wait(SC_MS);
        }
    }
};

int sc_main(int argc, char* argv[]) {
    sc_signal<bool> s1, s2, s3, s4;
    sc_signal<double> pL, pR, aL, aR, mL, mR;

    LineEnvironment env("Env");
    env.motor_left_speed(aL); env.motor_right_speed(aR);
    env.ir_ol_out(s1); env.ir_il_out(s2); env.ir_ir_out(s3); env.ir_or_out(s4);

    L298N dL("DL"); dL.pwm_in(pL); dL.speed_out(mL);
    L298N dR("DR"); dR.pwm_in(pR); dR.speed_out(mR);

    DCMotor mL_m("ML"); mL_m.speed_in(mL); mL_m.actual_out(aL);
    DCMotor mR_m("MR"); mR_m.speed_in(mR); mR_m.actual_out(aR);

    Controller ctrl("Ctrl");
    ctrl.ir_ol_in(s1); ctrl.ir_il_in(s2); ctrl.ir_ir_in(s3); ctrl.ir_or_in(s4);
    ctrl.actual_l(aL); ctrl.actual_r(aR); ctrl.pwm_l(pL); ctrl.pwm_r(pR);

    sc_start(300, SC_SEC);
    return 0;
}