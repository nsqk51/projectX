#include <MPU6050_tockn.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>

MPU6050 mpu6050(Wire);
WebServer server(80);

// ========== НАСТРОЙКИ Wi-Fi ==========
const char* ssid = "ESP32_Balance_Bot";
const char* password = "12345678";

// --- ПИНЫ ---
const int PWMA = 13, AIN2 = 12, AIN1 = 14, STBY = 27, BIN1 = 26, BIN2 = 25, PWMB = 33;

// --- PWM ---
const int freq = 20000;
const int resolution = 8;

// --- PID (Базовые настройки) ---
float Kp = 25.0; 
float Ki = 0.4;  
float Kd = 1.2;  

// --- УГОЛ ---
float targetAngle = 0.1; 
float moveOffset = 0; 

// --- ОГРАНИЧЕНИЯ ---
int maxSpeed = 250; 

// --- ПЕРЕМЕННЫЕ ---
float angleFiltered = 0;
float error = 0;
float integral = 0;
float gyroRate = 0;
float output = 0;
float lastOutput = 0;
bool isBalancing = false;

// Высокоточный таймер на микросекундах
unsigned long lastTimeMicros = 0;
const unsigned long dt_us = 10000; 

// ========== HTML СТРАНИЦА ==========
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="ru">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no">
    <title>Балансирующая система</title>
    <style>
        * { box-sizing: border-box; user-select: none; }
        body {
            font-family: 'Segoe UI', Arial, sans-serif;
            background: linear-gradient(135deg, #1a1a2e 0%, #16213e 100%);
            color: #eee;
            margin: 0;
            padding: 15px;
            min-height: 100vh;
        }
        .container { max-width: 550px; margin: 0 auto; }
        h1 { font-size: 22px; text-align: center; margin: 0 0 5px 0; color: #0f0; letter-spacing: 1px; }
        .subtitle { text-align: center; font-size: 11px; color: #ff0; margin-bottom: 25px; letter-spacing: 2px; }
        .card {
            background: rgba(0,0,0,0.6);
            backdrop-filter: blur(10px);
            border-radius: 12px;
            padding: 20px;
            margin-bottom: 15px;
            border: 1px solid #333;
        }
        .card-title {
            font-size: 14px;
            font-weight: bold;
            margin-bottom: 15px;
            padding-bottom: 8px;
            border-bottom: 2px solid #0f0;
            letter-spacing: 1px;
        }
        .param-desc {
            font-size: 11px;
            color: #888;
            margin-top: 5px;
            padding-left: 5px;
            border-left: 2px solid #0f0;
        }
        .sensor-item {
            background: rgba(0,0,0,0.5);
            border-radius: 8px;
            padding: 12px;
            text-align: center;
            margin-bottom: 10px;
        }
        .sensor-label { font-size: 11px; color: #aaa; margin-bottom: 5px; letter-spacing: 1px; }
        .sensor-value { font-size: 28px; font-weight: bold; font-family: monospace; }
        .slider-group { margin-bottom: 20px; }
        .slider-label {
            display: flex;
            justify-content: space-between;
            margin-bottom: 8px;
            font-size: 13px;
            font-weight: bold;
        }
        .slider-label span:last-child { color: #0f0; font-family: monospace; }
        input[type="range"] {
            width: 100%;
            height: 6px;
            -webkit-appearance: none;
            background: #333;
            border-radius: 3px;
        }
        input[type="range"]::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 20px;
            height: 20px;
            border-radius: 50%;
            background: #0f0;
            cursor: pointer;
        }
        .button-group {
            display: flex;
            flex-wrap: wrap;
            gap: 10px;
            margin-top: 10px;
        }
        button {
            background: #2a2a4a;
            border: 1px solid #0f0;
            color: #0f0;
            padding: 12px 20px;
            border-radius: 6px;
            font-size: 13px;
            cursor: pointer;
            font-weight: bold;
            flex: 1;
            letter-spacing: 1px;
        }
        button:active { transform: scale(0.98); background: #0f0; color: #000; }
        .btn-danger { border-color: #f00; color: #f00; }
        .btn-warning { border-color: #ff0; color: #ff0; }
        .status-led {
            display: inline-block;
            width: 10px;
            height: 10px;
            border-radius: 50%;
            margin-right: 8px;
        }
        .led-green { background: #0f0; box-shadow: 0 0 5px #0f0; animation: pulse 1s infinite; }
        .led-red { background: #f00; }
        @keyframes pulse { 0%, 100% { opacity: 1; } 50% { opacity: 0.5; } }
        hr { border-color: #333; margin: 15px 0; }
    </style>
</head>
<body>
<div class="container">
    <h1>БАЛАНСИРУЮЩАЯ СИСТЕМА</h1>
    <div class="subtitle">МОНИТОРИНГ И НАСТРОЙКА ПАРАМЕТРОВ ПИД</div>
    
    <div class="card">
        <div class="card-title">
            <div id="statusLed" class="status-led led-red"></div>
            <span id="statusText">ОСТАНОВЛЕН</span>
        </div>
        <div class="sensor-item">
            <div class="sensor-label">ТЕКУЩИЙ УГОЛ ОТКЛОНЕНИЯ</div>
            <div class="sensor-value"><span id="angle">0.0</span>°</div>
        </div>
        <div class="sensor-item">
            <div class="sensor-label">ВЫХОДНОЙ СИГНАЛ УПРАВЛЕНИЯ</div>
            <div class="sensor-value"><span id="output">0</span> PWM</div>
        </div>
    </div>
    
    <div class="card">
        <div class="card-title">КОЭФФИЦИЕНТЫ ПИД-РЕГУЛЯТОРА</div>
        
        <div class="slider-group">
            <div class="slider-label"><span>Пропорциональный (Kp) = <span id="kpVal">25.0</span></span><span>СИЛА ВОЗВРАТА</span></div>
            <input type="range" id="kpSlider" min="5" max="60" step="1" value="25" oninput="updateKp(this.value)">
            <div class="param-desc">Определяет силу противодействия наклону. При избытке возникает вибрация.</div>
        </div>
        
        <div class="slider-group">
            <div class="slider-label"><span>Интегральный (Ki) = <span id="kiVal">0.40</span></span><span>УСТРАНЕНИЕ СТАТИЧЕСКОЙ ОШИБКИ</span></div>
            <input type="range" id="kiSlider" min="0" max="2" step="0.05" value="0.4" oninput="updateKi(this.value)">
            <div class="param-desc">Компенсирует постоянный наклон и уход с точки. Избыток вызывает раскачку.</div>
        </div>
        
        <div class="slider-group">
            <div class="slider-label"><span>Дифференциальный (Kd) = <span id="kdVal">1.2</span></span><span>ДЕМПФИРОВАНИЕ</span></div>
            <input type="range" id="kdSlider" min="0" max="5" step="0.1" value="1.2" oninput="updateKd(this.value)">
            <div class="param-desc">Гасит колебания. Избыточное значение приводит к резким рывкам двигателей.</div>
        </div>
        
        <hr>
        
        <div class="slider-group">
            <div class="slider-label"><span>Смещение нуля (targetAngle) = <span id="targetVal">0.1</span>°</span><span>НУЛЕВАЯ ТОЧКА</span></div>
            <input type="range" id="targetSlider" min="-5.0" max="5.0" step="0.1" value="0.1" oninput="updateTarget(this.value)">
            <div class="param-desc">Компенсация физического дисбаланса конструкции (смещение центра масс).</div>
        </div>
        
        <div class="slider-group">
            <div class="slider-label"><span>Лимит ШИМ (MaxSpeed) = <span id="maxSpeedVal">250</span></span><span>ОГРАНИЧЕНИЕ МОЩНОСТИ</span></div>
            <input type="range" id="maxSpeedSlider" min="150" max="255" step="5" value="250" oninput="updateMaxSpeed(this.value)">
            <div class="param-desc">Аппаратное ограничение максимального заполнения ШИМ-сигнала.</div>
        </div>
    </div>
    
    <div class="card">
        <div class="card-title">РЕЖИМЫ РАБОТЫ</div>
        <div class="button-group">
            <button onclick="sendCmd('1')">ЗАПУСК</button>
            <button onclick="sendCmd('0')" class="btn-danger">ОСТАНОВ</button>
            <button onclick="sendCmd('X')" class="btn-warning">КАЛИБРОВКА ДАТЧИКА</button>
        </div>
    </div>
</div>

<script>
    function updateKp(val) {
        document.getElementById('kpVal').innerHTML = parseFloat(val).toFixed(1);
        fetch('/setpid?kp=' + val + '&ki=' + document.getElementById('kiSlider').value + '&kd=' + document.getElementById('kdSlider').value);
    }
    function updateKi(val) {
        document.getElementById('kiVal').innerHTML = parseFloat(val).toFixed(2);
        fetch('/setpid?kp=' + document.getElementById('kpSlider').value + '&ki=' + val + '&kd=' + document.getElementById('kdSlider').value);
    }
    function updateKd(val) {
        document.getElementById('kdVal').innerHTML = parseFloat(val).toFixed(1);
        fetch('/setpid?kp=' + document.getElementById('kpSlider').value + '&ki=' + document.getElementById('kiSlider').value + '&kd=' + val);
    }
    function updateTarget(val) {
        document.getElementById('targetVal').innerHTML = parseFloat(val).toFixed(1);
        fetch('/settarget?val=' + val);
    }
    function updateMaxSpeed(val) {
        document.getElementById('maxSpeedVal').innerHTML = val;
        fetch('/setmaxspeed?val=' + val);
    }
    function sendCmd(cmd) {
        fetch('/cmd?val=' + cmd);
        if(cmd === '1') {
            document.getElementById('statusText').innerHTML = 'БАЛАНСИРОВКА';
            document.getElementById('statusLed').className = 'status-led led-green';
        }
        if(cmd === '0') {
            document.getElementById('statusText').innerHTML = 'ОСТАНОВЛЕН';
            document.getElementById('statusLed').className = 'status-led led-red';
        }
    }
    function updateData() {
        fetch('/data')
            .then(response => response.json())
            .then(data => {
                document.getElementById('angle').innerHTML = data.angle.toFixed(1);
                document.getElementById('output').innerHTML = Math.abs(data.output);
                if(data.balancing) {
                    document.getElementById('statusText').innerHTML = 'БАЛАНСИРОВКА';
                    document.getElementById('statusLed').className = 'status-led led-green';
                } else {
                    document.getElementById('statusText').innerHTML = 'ОСТАНОВЛЕН';
                    document.getElementById('statusLed').className = 'status-led led-red';
                }
            });
    }
    setInterval(updateData, 100);
</script>
</body>
</html>
)rawliteral";

// ========== ОБРАБОТЧИКИ WEB-ЗАПРОСОВ ==========
void handleRoot() { server.send(200, "text/html", index_html); }

void handleCommand() {
  if (server.hasArg("val")) {
    String cmd = server.arg("val");
    char c = cmd.charAt(0);
    
    if (c == 'X' || c == 'x') calibrateMPU();
    else if (c == '1') { 
      isBalancing = true; 
      Serial.println("РОБОТ НАЧАЛ РАБОТУ (WEB)");
    }
    else if (c == '0') { 
      isBalancing = false; 
      drive(0);
      Serial.println("STOP (WEB)"); 
    }
    else if (c == 'F' || c == 'f') { moveOffset = -1.0; }
    else if (c == 'B' || c == 'b') { moveOffset = 1.0; }
    else if (c == 'S' || c == 's') { moveOffset = 0; }
    server.send(200, "text/plain", "OK");
  }
}

void handleSetPID() {
  if (server.hasArg("kp")) { Kp = server.arg("kp").toFloat(); }
  if (server.hasArg("ki")) { Ki = server.arg("ki").toFloat(); }
  if (server.hasArg("kd")) { Kd = server.arg("kd").toFloat(); }
  Serial.print("PID: Kp="); Serial.print(Kp);
  Serial.print(" Ki="); Serial.print(Ki);
  Serial.print(" Kd="); Serial.println(Kd);
  server.send(200, "text/plain", "OK");
}

void handleSetTarget() {
  if (server.hasArg("val")) { 
    targetAngle = server.arg("val").toFloat();
    Serial.print("Целевой угол: ");
    Serial.println(targetAngle);
  }
  server.send(200, "text/plain", "OK");
}

void handleSetMaxSpeed() {
  if (server.hasArg("val")) { 
    maxSpeed = server.arg("val").toInt();
    Serial.print("MaxSpeed: ");
    Serial.println(maxSpeed);
  }
  server.send(200, "text/plain", "OK");
}

void handleData() {
  String json = "{\"angle\":" + String(angleFiltered) + 
                ",\"output\":" + String(abs(output)) +
                ",\"balancing\":" + String(isBalancing ? "true" : "false") + "}";
  server.send(200, "application/json", json);
}

// ========== SETUP ==========
void setup() {
  Serial.begin(115200); 
  Wire.begin(21, 22);

  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);

  ledcAttach(PWMA, freq, resolution);
  ledcAttach(PWMB, freq, resolution);

  mpu6050.begin();
  
  WiFi.softAP(ssid, password);
  server.on("/", handleRoot);
  server.on("/cmd", handleCommand);
  server.on("/setpid", handleSetPID);
  server.on("/settarget", handleSetTarget);
  server.on("/setmaxspeed", handleSetMaxSpeed);
  server.on("/data", handleData);
  server.begin();

  Serial.println("ГОТОВ. Команды принимаются через веб-интерфейс.");
  Serial.print("Wi-Fi AP: "); Serial.println(ssid);
  Serial.print("IP: "); Serial.println(WiFi.softAPIP());
}

// ========== УПРАВЛЕНИЕ МОТОРАМИ ==========
void drive(int speed) {
  if (!isBalancing) {
    ledcWrite(PWMA, 0);
    ledcWrite(PWMB, 0);
    return;
  }

  speed = constrain(speed, -255, 255);

  if (speed > 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
  }

  int pwm = abs(speed);
  
  if (pwm > 0) {
    pwm = map(pwm, 0, 255, 40, 255); 
  }

  ledcWrite(PWMA, constrain(pwm, 0, 255));
  ledcWrite(PWMB, constrain(pwm, 0, 255));
}

// ========== КАЛИБРОВКА ==========
void calibrateMPU() {
  Serial.println("КАЛИБРОВКА... НЕ ДВИГАЙ РОБОТА");
  delay(2000);
  mpu6050.calcGyroOffsets(true);
  angleFiltered = 0;
  integral = 0;
  lastOutput = 0;
  Serial.println("КАЛИБРОВКА ЗАВЕРШЕНА");
}

// ========== LOOP ==========
void loop() {
  server.handleClient(); 

  unsigned long now = micros();
  if (now - lastTimeMicros >= dt_us) {
    float dt = (now - lastTimeMicros) / 1000000.0;
    lastTimeMicros = now;

    mpu6050.update();
    
    float angle = mpu6050.getAngleX(); 
    float rawGyro = mpu6050.getGyroX(); 

    angleFiltered = angle; 
    gyroRate = rawGyro; 

    if (!isBalancing) {
      return;
    }

    if (abs(angleFiltered) > 45) {
      isBalancing = false;
      drive(0);
      Serial.println("STATUS: РОБОТ УПАЛ");
      return;
    }

    // ---------------- PID ----------------
    error = angleFiltered - targetAngle; 

    if (abs(error) < 5) integral += error * dt;
    else integral = 0;
    integral = constrain(integral, -100, 100);

    float p_term = Kp * error;
    float i_term = Ki * integral;
    float d_term = Kd * gyroRate; 

    output = p_term + i_term + d_term;

    if (abs(error) > 8) {
      output += 35 * (error > 0 ? 1 : -1);
    }

    output = constrain(output, -250, 250);
    
    float maxChange = 120; 
    if (output > lastOutput + maxChange) output = lastOutput + maxChange;
    if (output < lastOutput - maxChange) output = lastOutput - maxChange;
    lastOutput = output;

    // --- ОТЛАДОЧНЫЙ ЛОГ ---
    Serial.print("BAL_DATA:");
    Serial.print("dt:"); Serial.print(dt, 4);
    Serial.print(",Ang:"); Serial.print(angleFiltered, 2);
    Serial.print(",Err:"); Serial.print(error, 2);
    Serial.print(",P:"); Serial.print(p_term, 1);
    Serial.print(",I:"); Serial.print(i_term, 1);
    Serial.print(",D:"); Serial.print(d_term, 1);
    Serial.print(",Out:"); Serial.println(output, 0);

    drive((int)output);
  }
}