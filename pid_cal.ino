/*
 * pid_cal.ino
 * 
 * Real-time wireless PID tuning and telemetry server for ESP32 UAV.
 * 
 * This firmware hosts a mobile-friendly web interface to remotely 
 * adjust Cascaded PID gains (Angle and Rate loops) in real-time 
 * during tethered flight testing. It is primarily used for:
 * - Iteratively tuning flight dynamics and stability limits without recompiling.
 * - Streaming live MPU6050 telemetry (Pitch, Roll) via asynchronous JSON fetching.
 * - Managing remote arming, IMU calibration, and emergency failsafe disarming.
 */

#include <WiFi.h>
#include <WebServer.h>
#include <ESP32Servo.h>
#include <Wire.h>

const char* ssid     = "ABCD";   // Change to your network
const char* password = "12345678"; // Change password

const int MPU_ADDR = 0x68;
Servo mot1, mot2, mot3, mot4;

const int p12 = 12; // Front Right
const int p27 = 27; // Front Left  
const int p13 = 13; // Rear Right  
const int p14 = 14; // Rear Left   

enum State { DISARMED, CALIBRATING, ARMED };
volatile State sysState = DISARMED;

volatile int target_pwm = 1100;       
volatile int max_allowed_pwm = 1400;  
volatile unsigned long lastContactMs = 0;
uint32_t LoopTimer;

int calib_warmup_count = 0;
int calib_sample_count = 0;
float acc_pitch_sum = 0, acc_roll_sum = 0;
long gx_sum = 0, gy_sum = 0;

volatile float pitch = 0.0f, roll = 0.0f;
volatile float gyro_rate_pitch = 0.0f, gyro_rate_roll = 0.0f;

float pitch_acc_offset = 0.0f, roll_acc_offset = 0.0f;
float gyro_x_offset = 0.0f, gyro_y_offset = 0.0f;
bool cf_init = false;

#define DT 0.004f            
#define GYRO_SENS 65.5f      
#define ALPHA_ARMED 0.991f   
#define ALPHA_DISARMED 0.970f

volatile float PAngleRoll = 2.5, IAngleRoll = 0.2, DAngleRoll = 0.007;
volatile float PAnglePitch = 2.5, IAnglePitch = 0.2, DAnglePitch = 0.007;

volatile float PRateRoll = 0.7, IRateRoll = 1.5, DRateRoll = 0.0088;
volatile float PRatePitch = 0.7, IRatePitch = 1.5, DRatePitch = 0.0088;

float prev_err_angle_p = 0, prev_iterm_angle_p = 0;
float prev_err_angle_r = 0, prev_iterm_angle_r = 0;
float prev_err_rate_p = 0,  prev_iterm_rate_p = 0;
float prev_err_rate_r = 0,  prev_iterm_rate_r = 0;

WebServer server(80);

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Flight Controller Tuning</title>
<style>
  body { font-family: Arial, sans-serif; background: #121212; color: #fff; text-align: center; padding: 10px; }
  .box { background: #1e1e1e; padding: 15px; border-radius: 8px; margin-bottom: 15px; }
  .grid { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 10px; text-align: center; }
  input[type=number] { width: 80%; padding: 8px; font-size: 16px; text-align: center; font-weight: bold; border-radius: 5px; border: none; }
  input[type=text] { width: 100px; padding: 8px; font-size: 16px; text-align: center; font-weight: bold; border-radius: 5px; border: none; }
  .btn { padding: 15px; font-size: 18px; font-weight: bold; border: none; border-radius: 5px; cursor: pointer; width: 100%; margin-top: 10px; }
  .btn-arm { background: #4CAF50; color: white; }
  .btn-disarm { background: #f44336; color: white; font-size: 22px; padding: 20px;}
  .btn-cal { background: #2196F3; color: white; }
  .btn-upd { background: #ff9800; color: white; }
  .power-grid { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
  h3 { color: #00e5ff; margin-bottom: 5px; }
  #status { font-size: 20px; font-weight: bold; color: #ffeb3b; }
</style>
</head><body>
  <h2>Cascaded PID Controller</h2>
  <div class="box"><p>STATUS: <span id="status">DISARMED</span></p></div>
  
  <div class="box">
    <h3>Telemetry</h3>
    Pitch: <input type="text" id="p_val" readonly> &deg; | Roll: <input type="text" id="r_val" readonly> &deg;
  </div>

  <div class="box">
    <h3>Motor Power Limits</h3>
    <div class="power-grid">
      <div><b>Base Throttle</b><br><input type="number" id="pwm_in" value="1100" min="1000" max="2000"></div>
      <div><b style="color:#f44336;">Absolute Max</b><br><input type="number" id="pwm_max" value="1400" min="1000" max="2000"></div>
    </div>
  </div>

  <div class="box">
    <h3>Angle Loop (Outer)</h3>
    <div class="grid">
      <div><b>Pitch P</b><br><input type="number" id="pap" step="0.1" value="2.0"></div>
      <div><b>Pitch I</b><br><input type="number" id="iap" step="0.1" value="0.5"></div>
      <div><b>Pitch D</b><br><input type="number" id="dap" step="0.001" value="0.007"></div>
      <div><b>Roll P</b><br><input type="number" id="par" step="0.1" value="2.0"></div>
      <div><b>Roll I</b><br><input type="number" id="iar" step="0.1" value="0.5"></div>
      <div><b>Roll D</b><br><input type="number" id="dar" step="0.001" value="0.007"></div>
    </div>
  </div>

  <div class="box">
    <h3>Rate Loop (Inner)</h3>
    <div class="grid">
      <div><b>Pitch P</b><br><input type="number" id="prp" step="0.01" value="0.625"></div>
      <div><b>Pitch I</b><br><input type="number" id="irp" step="0.1" value="2.1"></div>
      <div><b>Pitch D</b><br><input type="number" id="drp" step="0.001" value="0.0088"></div>
      <div><b>Roll P</b><br><input type="number" id="prr" step="0.01" value="0.625"></div>
      <div><b>Roll I</b><br><input type="number" id="irr" step="0.1" value="2.1"></div>
      <div><b>Roll D</b><br><input type="number" id="drr" step="0.001" value="0.0088"></div>
    </div>
  </div>

  <button class="btn btn-upd" onclick="updatePID()">UPDATE GAINS & PWM</button>
  <button class="btn btn-arm" onclick="sendCmd('arm')">ARM MOTORS</button>
  <button class="btn btn-cal" onclick="sendCmd('cal')">CALIBRATE IMU</button>
  <button class="btn btn-disarm" onclick="sendCmd('disarm')">EMERGENCY STOP</button>

<script>
  function sendCmd(action) { fetch("/" + action).catch(e => console.log(e)); }
  function updatePID() {
    let url = "/update?pwm=" + document.getElementById('pwm_in').value +
              "&pwm_max=" + document.getElementById('pwm_max').value +
              "&pap=" + document.getElementById('pap').value + "&iap=" + document.getElementById('iap').value + "&dap=" + document.getElementById('dap').value +
              "&par=" + document.getElementById('par').value + "&iar=" + document.getElementById('iar').value + "&dar=" + document.getElementById('dar').value +
              "&prp=" + document.getElementById('prp').value + "&irp=" + document.getElementById('irp').value + "&drp=" + document.getElementById('drp').value +
              "&prr=" + document.getElementById('prr').value + "&irr=" + document.getElementById('irr').value + "&drr=" + document.getElementById('drr').value;
    fetch(url).then(r => alert("Updated!")).catch(e => alert("Error"));
  }
  
  // RECURSIVE FETCH: Prevents the browser from DDoSing the ESP32
  function fetchTelemetry() {
    fetch('/data')
      .then(r => r.json())
      .then(d => {
        document.getElementById('p_val').value = d.p.toFixed(2);
        document.getElementById('r_val').value = d.r.toFixed(2);
        let stat = document.getElementById('status');
        if(d.s == 0) { stat.innerText = "DISARMED"; stat.style.color = "#ffeb3b"; }
        if(d.s == 1) { stat.innerText = "CALIBRATING"; stat.style.color = "#2196F3"; }
        if(d.s == 2) { stat.innerText = "ARMED - MOTORS LIVE"; stat.style.color = "#4CAF50"; }
        
        // Schedule next update ONLY after this one completes
        setTimeout(fetchTelemetry, 250);
      })
      .catch(e => {
        console.log("Telemetry error, retrying...", e);
        // If WiFi lags, wait a full second before trying again so we don't crash the ESP
        setTimeout(fetchTelemetry, 1000); 
      });
  }
  
  // Start the telemetry loop
  fetchTelemetry();
</script>
</body></html>
)rawliteral";

void webTask(void *pvParameters) {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.print("\nWiFi OK. Open IP in Browser: "); Serial.println(WiFi.localIP());
  lastContactMs = millis();

  server.on("/", [](){ 
    lastContactMs = millis(); 
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", index_html); 
  });
  
  server.on("/data", [](){
    lastContactMs = millis();

    char json[100];
    snprintf(json, sizeof(json), "{\"p\":%.2f,\"r\":%.2f,\"s\":%d}", 
            (float)(pitch - pitch_acc_offset), (float)(roll - roll_acc_offset), sysState);
    
    server.sendHeader("Connection", "close"); 
    server.send(200, "application/json", json);
  });

  server.on("/arm", [](){ 
    lastContactMs = millis(); 
    if(sysState != CALIBRATING) sysState = ARMED; 
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", "OK"); 
  });

  server.on("/disarm", [](){ 
    lastContactMs = millis(); 
    sysState = DISARMED; 
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", "OK"); 
  });

  server.on("/cal", [](){ 
    lastContactMs = millis(); 
    if(sysState != ARMED) { cf_init = false; sysState = CALIBRATING; } 
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", "OK"); 
  });

  server.on("/update", [](){
    lastContactMs = millis();
    if(server.hasArg("pwm")) target_pwm = server.arg("pwm").toInt();
    if(server.hasArg("pwm_max")) max_allowed_pwm = constrain(server.arg("pwm_max").toInt(), 1000, 2000); 
    
    if (target_pwm > max_allowed_pwm) target_pwm = max_allowed_pwm; 

    if(server.hasArg("pap")) PAnglePitch = server.arg("pap").toFloat();
    if(server.hasArg("iap")) IAnglePitch = server.arg("iap").toFloat();
    if(server.hasArg("dap")) DAnglePitch = server.arg("dap").toFloat();
    if(server.hasArg("par")) PAngleRoll = server.arg("par").toFloat();
    if(server.hasArg("iar")) IAngleRoll = server.arg("iar").toFloat();
    if(server.hasArg("dar")) DAngleRoll = server.arg("dar").toFloat();
    if(server.hasArg("prp")) PRatePitch = server.arg("prp").toFloat();
    if(server.hasArg("irp")) IRatePitch = server.arg("irp").toFloat();
    if(server.hasArg("drp")) DRatePitch = server.arg("drp").toFloat();
    if(server.hasArg("prr")) PRateRoll = server.arg("prr").toFloat();
    if(server.hasArg("irr")) IRateRoll = server.arg("irr").toFloat();
    if(server.hasArg("drr")) DRateRoll = server.arg("drr").toFloat();
    
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", "OK");
  });

  server.begin();
  for (;;) { 
    server.handleClient(); 
    vTaskDelay(pdMS_TO_TICKS(2)); 
  } 
}

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22); Wire.setClock(400000);

  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1B); Wire.write(0x08); Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x10); Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x05); Wire.endTransmission(true);
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1D); Wire.write(0x05); Wire.endTransmission(true);

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  
  mot1.attach(p12, 1000, 2000); mot1.setPeriodHertz(250); // Front Right
  mot2.attach(p27, 1000, 2000); mot2.setPeriodHertz(250); // Front Left
  mot3.attach(p13, 1000, 2000); mot3.setPeriodHertz(250); // Rear Right
  mot4.attach(p14, 1000, 2000); mot4.setPeriodHertz(250); // Rear Left

  mot1.writeMicroseconds(1000); mot2.writeMicroseconds(1000);
  mot3.writeMicroseconds(1000); mot4.writeMicroseconds(1000);
  delay(3000); 

  xTaskCreatePinnedToCore(webTask, "WebTask", 8192, NULL, 1, NULL, 0); 
  LoopTimer = micros();
}


void loop() {
  if (sysState == ARMED) {
    if (millis() - lastContactMs > 3000) sysState = DISARMED; 
    if (abs(pitch - pitch_acc_offset) > 60.0f || abs(roll - roll_acc_offset) > 60.0f) sysState = DISARMED; 
  }

  Wire.beginTransmission(MPU_ADDR); Wire.write(0x3B); Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true);
  int16_t ax = Wire.read()<<8 | Wire.read(); int16_t ay = Wire.read()<<8 | Wire.read(); int16_t az = Wire.read()<<8 | Wire.read();
  Wire.read(); Wire.read(); 
  int16_t rgx = Wire.read()<<8 | Wire.read(); int16_t rgy = Wire.read()<<8 | Wire.read();

  gyro_rate_pitch = ((float)rgx - gyro_x_offset) / GYRO_SENS;
  gyro_rate_roll  = ((float)rgy - gyro_y_offset) / GYRO_SENS;
  float acc_p  = atan2f((float)ay, sqrtf((float)ax*ax + (float)az*az)) * 57.2958f;
  float acc_r  = atan2f(-(float)ax, (float)az) * 57.2958f;

  float alpha = (sysState == ARMED) ? ALPHA_ARMED : ALPHA_DISARMED;
  if (!cf_init) { pitch = acc_p; roll = acc_r; cf_init = true; } 
  else {
    pitch = alpha * (pitch + gyro_rate_pitch * DT) + (1.0f - alpha) * acc_p;
    roll  = alpha * (roll  + gyro_rate_roll * DT)  + (1.0f - alpha) * acc_r;
  }
  
  float cal_pitch = pitch - pitch_acc_offset;
  float cal_roll  = roll - roll_acc_offset;

  if (sysState == CALIBRATING) {
    if (calib_warmup_count < 150) {
      calib_warmup_count++;
    } else {
      acc_pitch_sum += acc_p; acc_roll_sum += acc_r;
      gx_sum += rgx; gy_sum += rgy;
      calib_sample_count++;

      if (calib_sample_count >= 500) {
        pitch_acc_offset = acc_pitch_sum / 500.0f;
        roll_acc_offset  = acc_roll_sum  / 500.0f;
        gyro_x_offset    = (float)gx_sum / 500.0f;
        gyro_y_offset    = (float)gy_sum / 500.0f;
        
        pitch = pitch_acc_offset; 
        roll = roll_acc_offset;
        cf_init = true;

        acc_pitch_sum = 0; acc_roll_sum = 0; gx_sum = 0; gy_sum = 0;
        calib_sample_count = 0; calib_warmup_count = 0;
        sysState = DISARMED;
      }
    }
    mot1.writeMicroseconds(1000); mot2.writeMicroseconds(1000);
    mot3.writeMicroseconds(1000); mot4.writeMicroseconds(1000);
  }
  else if (sysState == ARMED) {
    float err_ap = 0.0f - cal_pitch; 
    prev_iterm_angle_p += IAnglePitch * (err_ap + prev_err_angle_p) * (DT / 2.0f);
    prev_iterm_angle_p = constrain(prev_iterm_angle_p, -400.0f, 400.0f);
    float dterm_ap = DAnglePitch * ((err_ap - prev_err_angle_p) / DT);
    float desired_rate_pitch = constrain((PAnglePitch * err_ap) + prev_iterm_angle_p + dterm_ap, -400.0f, 400.0f);
    prev_err_angle_p = err_ap;

    float err_ar = 0.0f - cal_roll; 
    prev_iterm_angle_r += IAngleRoll * (err_ar + prev_err_angle_r) * (DT / 2.0f);
    prev_iterm_angle_r = constrain(prev_iterm_angle_r, -400.0f, 400.0f);
    float dterm_ar = DAngleRoll * ((err_ar - prev_err_angle_r) / DT);
    float desired_rate_roll = constrain((PAngleRoll * err_ar) + prev_iterm_angle_r + dterm_ar, -400.0f, 400.0f);
    prev_err_angle_r = err_ar;


    float err_rp = desired_rate_pitch - gyro_rate_pitch;
    prev_iterm_rate_p += IRatePitch * (err_rp + prev_err_rate_p) * (DT / 2.0f);
    prev_iterm_rate_p = constrain(prev_iterm_rate_p, -400.0f, 400.0f);
    float dterm_rp = DRatePitch * ((err_rp - prev_err_rate_p) / DT);
    float pid_pitch = constrain((PRatePitch * err_rp) + prev_iterm_rate_p + dterm_rp, -400.0f, 400.0f);
    prev_err_rate_p = err_rp;

    float err_rr = desired_rate_roll - gyro_rate_roll;
    prev_iterm_rate_r += IRateRoll * (err_rr + prev_err_rate_r) * (DT / 2.0f);
    prev_iterm_rate_r = constrain(prev_iterm_rate_r, -400.0f, 400.0f);
    float dterm_rr = DRateRoll * ((err_rr - prev_err_rate_r) / DT);
    float pid_roll = constrain((PRateRoll * err_rr) + prev_iterm_rate_r + dterm_rr, -400.0f, 400.0f);
    prev_err_rate_r = err_rr;


    int m12 = target_pwm + pid_pitch - pid_roll; // Front Right
    int m27 = target_pwm + pid_pitch + pid_roll; // Front Left 
    int m13 = target_pwm - pid_pitch - pid_roll; // Rear Right 
    int m14 = target_pwm - pid_pitch + pid_roll; // Rear Left

    mot1.writeMicroseconds(constrain(m12, 1000, max_allowed_pwm));
    mot2.writeMicroseconds(constrain(m27, 1000, max_allowed_pwm));
    mot3.writeMicroseconds(constrain(m13, 1000, max_allowed_pwm));
    mot4.writeMicroseconds(constrain(m14, 1000, max_allowed_pwm));
  } 
  else {
    prev_err_angle_p = 0; prev_iterm_angle_p = 0; prev_err_angle_r = 0; prev_iterm_angle_r = 0;
    prev_err_rate_p = 0;  prev_iterm_rate_p = 0;  prev_err_rate_r = 0;  prev_iterm_rate_r = 0;
    
    mot1.writeMicroseconds(1000); mot2.writeMicroseconds(1000);
    mot3.writeMicroseconds(1000); mot4.writeMicroseconds(1000);
  }

  while (micros() - LoopTimer < 4000) { yield(); } 
  LoopTimer = micros();
}
