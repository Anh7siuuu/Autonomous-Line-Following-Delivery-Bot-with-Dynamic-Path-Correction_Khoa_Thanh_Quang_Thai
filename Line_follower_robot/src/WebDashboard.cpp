#include "WebDashboard.h"

const char WebDashboard::index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><title>ESP32 Dashboard</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
  body{font-family:'Courier New', monospace;background:#111;color:#0f0;padding:5px;text-align:center;font-size:14px;margin:0}
  .box{border:1px solid #0f0;padding:10px;margin:5px auto;max-width:450px;border-radius:5px;background:#1a1a1a}
  
  /* Cảm biến */
  .sensor{display:inline-block;width:30px;height:30px;border:2px solid #555;border-radius:50%;margin:2px;background:#000;transition:0.1s}
  .active{background:#0f0;box-shadow:0 0 10px #0f0;border-color:#0f0}
  
  /* Input & Button */
  .input-group {display: flex; justify-content: space-between; align-items: center; margin-bottom: 5px;}
  label {font-weight:bold;width:40px;text-align:left}
  input[type=number]{width:60%;background:#000;color:#0f0;border:1px solid #0f0;padding:5px;text-align:center;border-radius:4px}
  button{width:100%;padding:10px;margin-top:5px;font-weight:bold;border-radius:4px;cursor:pointer;border:none;color:#fff}
  .btn-green{background:#060;border:1px solid #0f0}.btn-red{background:#800;border:1px solid #f00}.btn-blue{background:#048;border:1px solid #08f}
  
  /* Thông số Realtime */
  .info-row {display:flex; justify-content:space-around; margin-top:10px; border-top:1px dashed #333; padding-top:5px; color:#ccc}
  .val-box {text-align:center} .val-num {color:#fff; font-weight:bold; font-size:16px}
  
  /* Canvas Đồ thị */
  canvas {background: #000; width: 100%; height: 150px; border: 1px solid #333; display: block; margin-top: 5px;}
</style></head><body>

<div class="box">
  <div id="s0" class="sensor"></div><div id="s1" class="sensor"></div>
  <div id="s2" class="sensor"></div><div id="s3" class="sensor"></div>
  
  <canvas id="chartEl" width="400" height="150"></canvas>
  
  <div class="info-row">
    <div class="val-box">ERR<br><span id="err" class="val-num" style="color:yellow">0.0</span></div>
    <div class="val-box">PWM L<br><span id="pl" class="val-num" style="color:cyan">0</span></div>
    <div class="val-box">PWM R<br><span id="pr" class="val-num" style="color:cyan">0</span></div>
    <div class="val-box">Loop<br><span id="lt" class="val-num">0</span> us</div>
  </div>
</div>

<div class="box" style="display:flex; gap:10px;">
  <button class="btn-green" onclick="cmd('START')">RUN</button>
  <button class="btn-red" onclick="cmd('STOP')">STOP</button>
</div>

<div class="box">
  <div class="input-group"><label>Kp</label><input type="number" step="0.1" id="kp"></div>
  <div class="input-group"><label>Kd</label><input type="number" step="0.1" id="kd"></div>
  <div class="input-group"><label>Spd</label><input type="number" step="1" id="bs"></div>
  <hr style="border-color:#333">
  <div class="input-group"><label>Q</label><input type="number" step="0.001" id="q"></div>
  <div class="input-group"><label>R</label><input type="number" step="0.001" id="r"></div>
  <div class="input-group"><label>Kt</label><input type="number" step="0.01" id="kt"></div>
  <button class="btn-blue" onclick="tx()">UPDATE</button>
</div>

<script>
var ws;
var isConnected = false;

// --- PHẦN VẼ ĐỒ THỊ (KHÔNG CẦN THƯ VIỆN) ---
var ctx = document.getElementById('chartEl').getContext('2d');
var width = 400, height = 150;
var dataErr = new Array(100).fill(0); // Mảng chứa 100 điểm dữ liệu Error
var dataPid = new Array(100).fill(0); // Mảng chứa 100 điểm dữ liệu PID

function drawGraph() {
  ctx.clearRect(0, 0, width, height);
  
  // Vẽ trục giữa
  ctx.strokeStyle = '#333'; ctx.beginPath(); ctx.moveTo(0, height/2); ctx.lineTo(width, height/2); ctx.stroke();
  
  // Vẽ đường Error (Màu Đỏ)
  ctx.strokeStyle = '#f00'; ctx.lineWidth = 2; ctx.beginPath();
  for(let i=0; i<dataErr.length; i++) {
    // Map Error: Giả sử max error là +-4 -> Map vào chiều cao canvas
    let y = (height/2) - (dataErr[i] * 15); 
    if(i==0) ctx.moveTo(i*4, y); else ctx.lineTo(i*4, y);
  }
  ctx.stroke();

  // Vẽ đường PID Output (Màu Xanh lá)
  ctx.strokeStyle = '#0f0'; ctx.lineWidth = 1; ctx.beginPath();
  for(let i=0; i<dataPid.length; i++) {
    // Map PID: Giả sử PID max +-50 -> Map vào chiều cao
    let y = (height/2) - (dataPid[i] * 1); 
    if(i==0) ctx.moveTo(i*4, y); else ctx.lineTo(i*4, y);
  }
  ctx.stroke();
}

// --- QUẢN LÝ KẾT NỐI WEBSOCKET (CHỐNG ĐƠ) ---
function initWS() {
  ws = new WebSocket(`ws://${location.hostname}/ws`);
  
  ws.onopen = () => { 
    console.log("Connected"); 
    isConnected = true; 
    document.body.style.opacity = "1"; // Sáng lên khi kết nối
  };
  
  ws.onclose = () => { 
    console.log("Disconnected. Retrying..."); 
    isConnected = false; 
    document.body.style.opacity = "0.5"; // Mờ đi khi mất kết nối
    setTimeout(initWS, 2000); // Tự kết nối lại sau 2 giây
  };

  ws.onmessage = (e) => {
    var d = JSON.parse(e.data);
    
    // Cập nhật Cảm biến
    for(let i=0;i<4;i++) document.getElementById('s'+i).classList.toggle('active',d.s[i]);
    
    // Cập nhật Số liệu
    document.getElementById('err').innerText = d.e.toFixed(2);
    document.getElementById('lt').innerText = d.t;
    document.getElementById('pl').innerText = d.pl; // PWM Left
    document.getElementById('pr').innerText = d.pr; // PWM Right
    
    // Cập nhật mảng dữ liệu cho đồ thị (Cuộn dữ liệu)
    dataErr.push(d.e); dataErr.shift(); // Thêm mới vào cuối, xóa đầu
    dataPid.push(d.p); dataPid.shift(); 
    requestAnimationFrame(drawGraph); // Vẽ lại

    // Sync Config (Chỉ cập nhật khi server gửi gói tin chứa config)
    if(d.kp !== undefined){
       setVal('kp', d.kp); setVal('kd', d.kd); setVal('bs', d.bs);
       setVal('q', d.q);   setVal('r', d.r);   setVal('kt', d.kt);
    }
  };
}

// Helper set value input
function setVal(id, val) { if(document.activeElement.id !== id) document.getElementById(id).value = val; }

// Helper gửi lệnh
function tx(){
  var msg = {
    t: 'cfg',
    kp: parseFloat(document.getElementById('kp').value),
    kd: parseFloat(document.getElementById('kd').value),
    bs: parseInt(document.getElementById('bs').value),
    q:  parseFloat(document.getElementById('q').value),
    r:  parseFloat(document.getElementById('r').value),
    kt: parseFloat(document.getElementById('kt').value)
  };
  ws.send(JSON.stringify(msg));
}
function cmd(c){ws.send(JSON.stringify({t:'cmd',v:c}));}

// Khởi chạy
window.onload = initWS;
</script></body></html>
)rawliteral";

WebDashboard::WebDashboard(const char* ssid, const char* pass) 
    : server(80), ws("/ws"), ssid(ssid), pass(pass) {}

void WebDashboard::begin() {
    // Cấu hình Access Point
    WiFi.disconnect();
    WiFi.mode(WIFI_AP);
    IPAddress local_IP(192, 168, 4, 1);
    IPAddress gateway(192, 168, 4, 1);
    IPAddress subnet(255, 255, 255, 0);
    WiFi.softAPConfig(local_IP, gateway, subnet);
    WiFi.softAP(ssid, pass);
    
    Serial.println("AP Started. IP: 192.168.4.1");

    ws.onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
        this->onEvent(server, client, type, arg, data, len);
    });
    server.addHandler(&ws);
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send_P(200, "text/html", index_html);
    });
    server.begin();
}

void WebDashboard::notifyClients() {
    StaticJsonDocument<512> doc;
    
    // 1. Data Telemetry (Gửi liên tục)
    JsonArray s = doc.createNestedArray("s");
    s.add(shared.sensorRaw[0]); s.add(shared.sensorRaw[1]);
    s.add(shared.sensorRaw[2]); s.add(shared.sensorRaw[3]);
    
    doc["e"]  = shared.currentError;
    doc["p"]  = shared.pidOutput;    // PID Output (Để vẽ đồ thị)
    doc["t"]  = shared.loopTime;
    doc["pl"] = shared.pwmLeft;      // [MỚI] Tốc độ bánh trái
    doc["pr"] = shared.pwmRight;     // [MỚI] Tốc độ bánh phải
    
    // // 2. Data Config (Gửi kèm luôn cho chắc, nhưng JS sẽ tự lọc)
    // doc["kp"] = shared.Kp; doc["kd"] = shared.Kd; doc["bs"] = shared.baseSpeed;
    // doc["q"]  = shared.Q;  doc["r"]  = shared.R;  doc["kt"] = shared.Kt;

    char buf[512];
    serializeJson(doc, buf);
    ws.textAll(buf);
}

void WebDashboard::cleanup() { ws.cleanupClients(); }

void WebDashboard::onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
    if (type == WS_EVT_DATA) {
        AwsFrameInfo *info = (AwsFrameInfo*)arg;
        if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
            StaticJsonDocument<512> doc;
            DeserializationError err = deserializeJson(doc, data);
            if(err) return;

            const char* t = doc["t"];
            if (strcmp(t, "cfg") == 0) {
                if(doc.containsKey("kp")) shared.Kp = doc["kp"];
                if(doc.containsKey("kd")) shared.Kd = doc["kd"];
                if(doc.containsKey("bs")) shared.baseSpeed = doc["bs"];
                if(doc.containsKey("q"))  shared.Q  = doc["q"];
                if(doc.containsKey("r"))  shared.R  = doc["r"];
                if(doc.containsKey("kt")) shared.Kt = doc["kt"];
            } else if (strcmp(t, "cmd") == 0) {
                const char* v = doc["v"];
                if (strcmp(v, "START") == 0) shared.isRunning = true;
                if (strcmp(v, "STOP") == 0) shared.isRunning = false;
            }
        }
    }
}