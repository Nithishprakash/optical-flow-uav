/*
 * flight_controller_esp.ino
 * 
 * Custom ESP32 Dual-Core Flight Controller for Companion Computer Architecture.
 * 
 * ARCHITECTURE & OPTICAL FLOW INTEGRATION:
 * Designed to operate without a traditional RF RC transmitter. A smartphone 
 * is physically mounted to the UAV chassis to serve as a companion computer, 
 * utilizing its camera for optical flow navigation and UDP telemetry relay.
 * 
 * The mounted smartphone calculates flight trajectories and streams real-time 
 * attitude/thrust setpoints (Pitch, Roll, PWM) directly to this ESP32 via a 
 * hardwired USB Serial (UART) connection for zero-latency control.
 * 
 * FIRMWARE HIGHLIGHTS:
 * - Core 0: Asynchronous Hardware Serial parser using zero-allocation char 
 *           buffers to prevent heap fragmentation.
 * - Core 1: Strict 250Hz Cascaded PID flight loop (Angle + Rate).
 * - Safety: 3-second serial watchdog failsafe and extreme-angle auto-disarm.
 */


#include <ESP32Servo.h>
#include <Wire.h>

const int MPU_ADDR = 0x68;
Servo mot1, mot2,mot3, mot4;

const int p12 = 12; // Front Right
const int p27 = 27; // Front Left  
const int p13 = 13; // Rear Right  
const int p14 = 14; // Rear Left   

enum State { DISARMED, CALIBRATING, ARMED };
volatile State sysState = DISARMED;

volatile int target_pwm =1100;       
volatile int max_allowed_pwm = 1400;  
volatile unsigned long lastContactMs = 0;
uint32_t LoopTimer;

// Target Angles (Controlled by the Smartphone via USB)
volatile float target_pitch = 0.0f;
volatile float target_roll =0.0f;

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

volatile float PAngleRoll =2.0, IAngleRoll = 0.5, DAngleRoll = 0.007;
volatile float PAnglePitch =2.0, IAnglePitch = 0.5, DAnglePitch = 0.007;

volatile float PRateRoll = 0.625, IRateRoll =2.1, DRateRoll = 0.0088;
volatile float PRatePitch= 0.625, IRatePitch = 2.1, DRatePitch = 0.0088;

float prev_err_angle_p = 0, prev_iterm_angle_p = 0;
float prev_err_angle_r = 0, prev_iterm_angle_r = 0;
float prev_err_rate_p = 0, prev_iterm_rate_p = 0;
float prev_err_rate_r = 0, prev_iterm_rate_r = 0;

void serialTask(void *pvParameters) {
  char buf[64];
  int len = 0;
  
  for (;;) { 
    while (Serial.available()) {
      char c = Serial.read();
      if (c =='\n') {
        buf[len] ='\0';
        if (strcmp(buf, "ARM") ==0) {
          if(sysState != CALIBRATING) sysState = ARMED;
          Serial.println("ACK_ARM");
        } 
        else if (strcmp(buf, "DISARM") == 0) {
          sysState = DISARMED;
          Serial.println("ACK_DISARM");
        } 
        else if (strcmp(buf, "CAL") == 0) {
          if(sysState != ARMED) { cf_init = false; sysState = CALIBRATING; }
          Serial.println("ACK_CAL");
        } 
        // Movement Command: M,pitch,roll,throttle
        else if (buf[0] == 'M' && buf[1] == ',') {
          float p, r; int t;
          if (sscanf(buf, "M,%f,%f,%d", &p, &r, &t) == 3) {
            target_pitch = p;
            target_roll = r;
            target_pwm = constrain(t, 1000, max_allowed_pwm);
            lastContactMs = millis(); 
          }
        }
        len = 0; 
      } 
      else if (len < 63 && c != '\r') {
        buf[len++] = c;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2)); // Prevent task watchdog trigger
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
  
  mot1.attach(p12, 1000, 2000); mot1.setPeriodHertz(500);
  mot2.attach(p27, 1000, 2000); mot2.setPeriodHertz(500);
  mot3.attach(p13, 1000, 2000); mot3.setPeriodHertz(500);
  mot4.attach(p14, 1000, 2000); mot4.setPeriodHertz(500);

  mot1.writeMicroseconds(1000); mot2.writeMicroseconds(1000);
  mot3.writeMicroseconds(1000); mot4.writeMicroseconds(1000);
  delay(3000); 

  xTaskCreatePinnedToCore(serialTask, "SerialTask", 4096, NULL, 1, NULL, 0); 
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
        pitch_acc_offset = acc_pitch_sum/500.0f;
        roll_acc_offset  = acc_roll_sum/500.0f;
        gyro_x_offset    = (float)gx_sum/500.0f;
        gyro_y_offset    = (float)gy_sum/500.0f;
        
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
    float err_ap = target_pitch - cal_pitch; 
    prev_iterm_angle_p += IAnglePitch * (err_ap + prev_err_angle_p) * (DT / 2.0f);
    prev_iterm_angle_p = constrain(prev_iterm_angle_p, -400.0f, 400.0f);
    float dterm_ap = DAnglePitch * ((err_ap - prev_err_angle_p) / DT);
    float desired_rate_pitch = constrain((PAnglePitch * err_ap) + prev_iterm_angle_p + dterm_ap, -400.0f, 400.0f);
    prev_err_angle_p = err_ap;

    float err_ar = target_roll - cal_roll; 
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
    target_pitch = 0.0f;  target_roll = 0.0f; // Reset targets on disarm
    
    mot1.writeMicroseconds(1000); mot2.writeMicroseconds(1000);
    mot3.writeMicroseconds(1000); mot4.writeMicroseconds(1000);
  }


  while (micros() - LoopTimer < 4000) { yield(); } 
  LoopTimer = micros();
}
