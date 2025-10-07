#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "MPU9250.h"

MPU9250 mpu;
// struct Vec3 {
//     float x, y, z;
// };
TaskHandle_t udpTaskHandle;
void udpTask(void *pvParameters);  // Forward declaration

// class VelocityEstimator {
// private:
//     Vec3 velocity = {0, 0, 0};
//     const float dt = 0.0025f; // fixed time step in seconds

// public:
//     // Call this every 2.5 ms
//     Vec3 updateVelocity(MPU9250 &mpu) {
//         // Quaternion from MPU
//         float qw = mpu.getQuaternionW();
//         float qx = mpu.getQuaternionX();
//         float qy = mpu.getQuaternionY();
//         float qz = mpu.getQuaternionZ();

//         // Gravity vector in sensor frame (your tuned calculation)
//         float gX = 2*(qx*qz - qw*qy);
//         float gY = 2*(qw*qx + qy*qz);
//         float gZ = qw*qw - qx*qx - qy*qy + qz*qz;

//         // Adjust accelerometer readings with gravity removed
//         float accelY = mpu.getAccX() + gX ;
//         float accelX = mpu.getAccY() - gY +0.01;
//         float accelZ = mpu.getAccZ();
//         // Serial.println(accelY);
//         // Serial.println(accelX);
//         // Serial.println(accelZ);
//         // Integrate acceleration to get velocity
//         velocity.x = accelX;
//         velocity.y = accelY;
//         velocity.z = accelZ;

//         return velocity;
//     }

//     void reset() {
//         velocity = {0,0,0};
//     }
// };

// VelocityEstimator velEstimator;

// MovingAverageFilter filterX;
// MovingAverageFilter filterY;
// MovingAverageFilter filterZ;
// MovingAverageFilter filterfb;
// MovingAverageFilter filterrl;
// MovingAverageFilter filterad;

static const int escPin1 = 11; //rotor 1
static const int escPin2 = 12; //rotor 2
static const int escPin3 = 13; //rotor 3
static const int escPin4 = 14; //rotor 4
static const int failsafe_timeout = 500; //Rotor goes off after 2 seconds of no UDP comm
static const int THROTTLE_LIMIT = 600; //Max rotor throttle aloud
//PID Parameters-------------------------------------------------------------------------------------
const float INTEGRAL_LIMIT = 20.0f;
const float OUTPUT_LIMIT = 90.0f;
// found rollX_offset = 18
int rollX_offset = 0; //offset due to weight imbalance
volatile int pitchY_offset = 0; //offset due to weight imbalance
volatile int yaw_bias = 0; // found 52

// PID tuning
// float kpX = 2.2, kdX = 0.3, kiX = 0.01;
// float kpY = 2.4, kdY = 0.8, kiY = 0.01;
// null -----------------------------------------------------------------------------
// volatile float kpX = 0, kiX = 0, kdX = 0;
// volatile float kpY = 0, kiY = 0, kdY = 0;
// volatile float kpZ = 0, kiZ = 0, kdZ = 0;  // set based on tuning
//-------------------------------------------------------------------------------------
volatile float kpX = 1.5, kiX = .0, kdX = 0.75;
volatile float kpY = 1.5, kiY = .0, kdY = 0.75;
volatile float kpZ = 1.5, kiZ = 0, kdZ = 0.75;  // set based on tuning
volatile float brake_scalefb = 0 ;
volatile float brake_scalelr = 0 ;
volatile bool altitude_hold = false;  // Set to true when you want to hold altitude
int throttle_adjust = 0 ;
// volatile float velX_debug = 0.0;
// volatile float velY_debug = 0.0;
// Global or static variables to maintain velocity over time
volatile int Xvbrake = 1;
volatile int Yvbrake = 1;

// float delta = 0.1;
// State variables for rollX
float integralX = 0;
float previous_errorX = 0;




// State variables for pitchY
float integralY = 0;
float previous_errorY = 0;

int correctionX = 0;
int correctionY = 0;
int rev_correctionX = 0;
int rev_correctionY = 0;

// PID state pf altitude controller--------------------------------
float alt_integral = 0.0f;
float alt_prev_error = 0.0f;

// Altitude PID gains-----------------------------------------------
// float alt_kp = 100.0f;
// float alt_ki = 0.5f;
// float alt_kd = 30.0f;

static unsigned long lastTime = 0;
static unsigned long currentTime = 0;

float targetYawRateZ = 0.0f;
float integralZ = 0.0f, previous_errorZ = 0.0f;

int yaw_offset =0;
int brake_offset_fb = 0;
int brake_offset_lr = 0;
float ramped_fb = 0.0;
float ramped_lr = 0.0;
// float derivativeX = 0;
//PWM Generator LEDC Parameters------------------------------------------------------------------------- 
unsigned long lastUpdate = 0;
// unsigned long pidInterval = 1;  // 200 Hz
uint32_t lastPwmWrite = 0 ;
volatile unsigned long lastUdpReceiveTime = 0;  // Track last time UDP packet was received
const int resolution = 14; // 14-bit resolution
const int freq = 400;       // 50 Hz for ESCs
uint32_t max_duty = (1 << resolution); // 16384
uint32_t period_us = 2500; // 2.5 ms period for 400 Hz
// Converts pulse width in µs (1000-2000) to LEDC duty cycle
uint32_t usToDuty(uint32_t pulse_us) {
  return (pulse_us * max_duty) / period_us;
}
// -----------------------------------------------------------------------------------------
//rotor biases--------------------------------------------------------------------
volatile uint16_t r1_off=0 ,r2_off=0 ,r3_off=0 ,r4_off=0 ;
volatile int r1y_off=0 ,r2y_off=0 ,r3y_off=0 ,r4y_off=0 ;
volatile int r1x_off=0 ,r2x_off=0 ,r3x_off=0 ,r4x_off=0 ;


//Comm Parameter transfer---------------------------------------------------
volatile uint16_t r = 0;
volatile uint16_t kpx_transfer = 0;
volatile uint16_t kdx_transfer = 0;
volatile uint16_t kix_transfer = 0;
volatile uint16_t kpy_transfer = 0;
volatile uint16_t kdy_transfer = 0;
volatile uint16_t kiy_transfer = 0;
volatile uint16_t kpz_transfer = 0;
volatile uint16_t kdz_transfer = 0;
volatile uint16_t kiz_transfer = 0;
volatile uint16_t fwd_bwd = 1;
volatile uint16_t left_right = 1;
volatile uint16_t asc_desc = 1;
volatile float fb = 1;
volatile float lr = 1;
volatile float ad = 1;
// int act = 0;
//Gyro variables---------------------------------------------------------------------------------------
float rollX = 0.0, pitchY = 0.0, yawZ = 0.0;
float raw_rollX = 0.0, raw_pitchY = 0.0, raw_yawZ = 0.0;
float raw_gyroX = 0.0, raw_gyroY = 0.0, raw_gyroZ = 0.0;
float gyroX = 0.0, gyroY = 0.0, gyroZ = 0.0;
float accelX = 0.0, accelY = 0.0, accelZ = 0.0;
float alpha = 0.1;
float targetrollX= 0 ;
float targetpitchY = 0  ;
float targetyawZ = -73 ;
// Replace with your Wi-Fi credentials-----------------------------------------------------------------
const char* ssid = "Hive";
const char* password = "12345678";

// PC IP and port (Receiving Data from PC)
IPAddress pcIP(192, 168, 4, 2);   // PC IP address
const uint16_t pcPort = 10001;      // PC Port

// ESP32 IP and port (Sending data to PC)
IPAddress espIP(192, 168, 4, 3);  // ESP32 IP address
const uint16_t espPort = 10002;     // ESP32 Port

WiFiUDP udp;
uint8_t rotorCmd[26]= {0};

//---------------------------------------------------------------------------------------------------------

// MPU9250Setting mySetting;
// mySetting.accel_fs_sel = ACCEL_FS_SEL::A2G;  // High sensitivity: ±2G

void setup() {
  Serial.begin(115200);
  Serial.println("Calibrating ESC ...");
  // Serial.println("Attaching LEDC to pin...");
  ledcAttach(escPin1, freq, resolution);
  ledcAttach(escPin2, freq, resolution);
  ledcAttach(escPin3, freq, resolution);
  ledcAttach(escPin4, freq, resolution);
  delay(2000);
  // Set throttle to maximum (full throttle) for calibration
  ledcWrite(escPin1, usToDuty(2000));
  ledcWrite(escPin2, usToDuty(2000));
  ledcWrite(escPin3, usToDuty(2000));
  ledcWrite(escPin4, usToDuty(2000));
  Serial.println("Full throttle for calibration...");
  delay(2000);  // Wait for ESC to register maximum throttle

  // Set throttle to minimum (no throttle) to complete calibration
  ledcWrite(escPin1, usToDuty(1000));
  ledcWrite(escPin2, usToDuty(1000));
  ledcWrite(escPin3, usToDuty(1000));
  ledcWrite(escPin4, usToDuty(1000));
  Serial.println("Throttle set to minimum...");
  delay(2000);

  Serial.println("ESC Calibrated!");
  pinMode(LED_BUILTIN, OUTPUT);
  delay(1000);

  Wire.begin();
  Wire.setClock(400000);  // 400 kHz = 4x faster I²C

  delay(2000);
  
  // Add this before WiFi.begin()
  IPAddress local_IP(192, 168, 4, 3);
  IPAddress gateway(192, 168, 4, 1);    // Usually your router's IP
  IPAddress subnet(255, 255, 255, 0);
  IPAddress primaryDNS(8, 8, 8, 8);     // Optional
  IPAddress secondaryDNS(8, 8, 4, 4);   // Optional

  // Configure static IP
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected. IP address: " + WiFi.localIP().toString());

  udp.begin(espPort);
  Serial.println("Listening on UDP port " + String(espPort));

   MPU9250Setting mySetting;
  mySetting.accel_fs_sel = ACCEL_FS_SEL::A2G;  // Change only accel sensitivity

  if (!mpu.setup(0x68, mySetting)) {  // change to your own address
      while (1) {
          digitalWrite(LED_BUILTIN, HIGH);
          Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
          delay(1000);
          digitalWrite(LED_BUILTIN, LOW);
          delay(1000);
      }
  }
  Serial.println("MPU connection success!!");
  delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
  // // calibrate anytime you want to
    // Serial.println("Accel Gyro calibration will start in 5sec.");
    // Serial.println("Please leave the device still on the flat plane.");
    // mpu.verbose(true);
    // delay(5000);
    mpu.calibrateAccelGyro();

   // Create UDP task on Core 0
  xTaskCreatePinnedToCore(
    udpTask,
    "UDP Task",
    12288,
    NULL,
    1,
    &udpTaskHandle,
    0  // Core 0
  );
}

void loop() {
  readGyro();
  // Vec3 velocity = velEstimator.updateVelocity(mpu);
  // lastTime = micros();  
    // stabilization();
    // Failsafe: shut down if no signal for 2 seconds
    if (millis() - lastUdpReceiveTime > failsafe_timeout) {
      // Serial.println(millis() - lastUdpReceiveTime);
      ledcWrite( escPin1, usToDuty(1000));
      ledcWrite( escPin4, usToDuty(1000));
      ledcWrite( escPin3, usToDuty(1000));
      ledcWrite( escPin2, usToDuty(1000));
      return;
      // Serial.println("Failsafe triggered — no UDP for 2 sec!");
      // delay(20);  // Just to reduce UART spamming
      // return;     // Exit loop early to skip motor write
    }
    // Serial.print(r1_off);
    // Serial.print("  ");
    // Serial.print(r2_off);
    // Serial.print("  ");
    // Serial.print(r3_off);
    // Serial.print("  ");
    // Serial.println(r4_off);
    // Serial.print("  ");
    // Serial.println(correctionY);
    
    if (micros() - lastPwmWrite > 2500){
      
      
      updatePID();
      // Serial.print("  Vx: "); Serial.print(velocity.x);
      // Serial.print("  Vy: "); Serial.println(velocity.y);
      // velX_debug = velocity.x;
      // velY_debug = velocity.y;  
      // Serial.print("  Vxbrake: "); Serial.print(Xvbrake);
      // Serial.print("  Vybrake: "); Serial.println(Yvbrake);
      // updateTargetTilt();
      // fb = filterfb.update(fb);
      // lr = filterrl.update(lr);
      // ad = filterad.update(ad);
      // ramped_fb = rampValue(ramped_fb, fb, 0.1);  // faster ramp
      // ramped_lr = rampValue(ramped_lr, lr, 0.1);
      // brake_offset_fb = constrain(int(ramped_fb*brake_scalefb),-20,20);  // scale to -100 to 100
      // brake_offset_lr = constrain(int(ramped_lr*brake_scalelr),-20,20);
      
      // computeBrakeOffsets(rollX,pitchY,brake_scaleX,brake_scaleY);
      // Serial.print(brake_offset_x);
      // Serial.print(" --- ");
      // Serial.println(brake_offset_y);
      // Serial.print("CorrectX = ");
      // Serial.println(correctionX);
      // Serial.print("  CorrectY = ");
      // Serial.println(correctionY);
      // === Setting Offsets ===
      // Y-axis correction
      if (correctionY > 0){
          r1y_off = -rev_correctionY;
          r2y_off = -rev_correctionY;
          r3y_off = correctionY;
          r4y_off = correctionY;
      }else if(correctionY < 0){
          r1y_off = -correctionY;
          r2y_off = -correctionY;
          r3y_off = rev_correctionY;
          r4y_off = rev_correctionY;
          // Serial.println(r4y_off);
      }else {
          r1y_off = 0;
          r2y_off = 0;
          r3y_off = 0;
          r4y_off = 0;
      }

      // X-axis correction
      if (correctionX > 0){
          r1x_off = -correctionX;
          r2x_off = rev_correctionX+rollX_offset;
          r3x_off = -correctionX+rollX_offset;
          r4x_off = rev_correctionX;
      }else if(correctionX < 0){
          r1x_off = -correctionX;
          r2x_off = rev_correctionX+rollX_offset;
          r3x_off = -correctionX+rollX_offset;
          r4x_off = rev_correctionX;
      }else {
          r1x_off = 0;
          r2x_off = 0+rollX_offset;
          r3x_off = 0+rollX_offset;
          r4x_off = 0;
      }
      // Serial.print(Xvbrake);
      // Serial.print("   ");
      // Serial.println(Yvbrake);

      r1_off = constrain(r + throttle_adjust + r1x_off + r1y_off + constrain(int(-accelX*Xvbrake + accelY*Yvbrake), -100, 100) - yaw_offset + pitchY_offset, 0, THROTTLE_LIMIT);
      r2_off = constrain(r + throttle_adjust + r2x_off + r2y_off + constrain(int(accelX*Xvbrake + accelY*Yvbrake), -100, 100) + yaw_offset + pitchY_offset, 0, THROTTLE_LIMIT);
      r3_off = constrain(r + throttle_adjust + r3x_off + r3y_off + constrain(int(-accelX*Xvbrake - accelY*Yvbrake), -100, 100) + yaw_offset, 0, THROTTLE_LIMIT);
      r4_off = constrain(r + throttle_adjust + r4x_off + r4y_off + constrain(int(accelX*Xvbrake - accelY*Yvbrake), -100, 100) - yaw_offset, 0, THROTTLE_LIMIT);
      // Serial.println(pitchY_offset);
      
      // Serial.println(millis() - lastPwmWrite);
      lastPwmWrite = micros();  
      ledcWrite( escPin1, usToDuty(1000 + r1_off));
      ledcWrite( escPin2, usToDuty(1000 + r2_off));
      ledcWrite( escPin3, usToDuty(1000 + r3_off));
      ledcWrite( escPin4, usToDuty(1000 + r4_off));
      
      // Serial.print(targetrollX);
      // Serial.print("     ");
      // Serial.print(targetpitchY);
      // Serial.print("     ");
      // Serial.println(targetyawZ);
      // Serial.println(ad);
    }
  
    // for X Y gains
  // Serial.println("kpX - kdX - kiX - kpY - kdY - kiY - kpZ - kdZ - kiZ");
  // Serial.print(kpX);
  // Serial.print(" ");
  // Serial.print(kdX);
  // Serial.print(" ");
  // Serial.print(kiX);
  // Serial.print(" ");

  // Serial.print(kpY);
  // Serial.print(" ");
  // Serial.print(kdY);
  // Serial.print(" ");
  // Serial.print(kiY);

  // Serial.print(kpZ);
  // Serial.print(" ");
  // Serial.print(kdZ);
  // Serial.print(" ");
  // Serial.println(kiZ);

  
  // Serial.println("-----------------------Wifi missed ----------------");
  // delayMicroseconds(10);
  // Serial.println(micros() - lastTime);
}

void udpTask(void *pvParameters){
  // udp.begin(espPort);
  while (true) {
    int packetSize = udp.parsePacket();
    if (packetSize == 26) {
      lastUdpReceiveTime = millis();
      udp.read(rotorCmd, 26);

    memcpy((void*)&r, rotorCmd, 2);
    memcpy((void*)&kpx_transfer, rotorCmd + 2, 2);
    memcpy((void*)&kdx_transfer, rotorCmd + 4, 2);
    memcpy((void*)&kix_transfer, rotorCmd + 6, 2);
    memcpy((void*)&kpy_transfer, rotorCmd + 8, 2);
    memcpy((void*)&kdy_transfer, rotorCmd + 10, 2);
    memcpy((void*)&kiy_transfer, rotorCmd + 12, 2);
    memcpy((void*)&kpz_transfer, rotorCmd + 14, 2);
    memcpy((void*)&kdz_transfer, rotorCmd + 16, 2);
    memcpy((void*)&kiz_transfer, rotorCmd + 18, 2);
    memcpy((void*)&fwd_bwd, rotorCmd + 20, 2);
    memcpy((void*)&left_right, rotorCmd + 22, 2);
    memcpy((void*)&asc_desc, rotorCmd + 24, 2);
    //visual motion detection----------------------------------------------------------
    fb = -((int)fwd_bwd-2)-1;
    lr = (int)left_right-1;
    ad = (int)asc_desc-1;

    // kp kd ki tunning parameters-----------------------------------------------------
    // kpX = (float)kpx_transfer / 100;
    // kdX = (float)kdx_transfer / 100;
    // kiX = (float)kix_transfer / 100;
    // kpY = (float)kpy_transfer / 100;
    // kdY = (float)kdy_transfer / 100;
    // kiY = (float)kiy_transfer / 100;
    // kpZ = (float)kpz_transfer / 100;
    // kdZ = (float)kdz_transfer / 100;
    // kiZ = (float)kiz_transfer / 100;

    // Linear drift braking parameter tunning ----------------------------------------------
    // Xvbrake = (float)kdx_transfer/10;
    // Yvbrake = (float)kix_transfer/10;
    // ---------Offset tuning --------------------------------------------------------------
    pitchY_offset = (float)kdx_transfer/10;
    // pitchY_offset

    // Teleop command parameter tunning ----------------------------------------------------
    // updateTargetTilt(constrain((float)kpx_transfer/10-1 ,-1 , 1), constrain((float)kdx_transfer/10-1,-1,1), 0);
    // Serial.print(kpx_transfer);
    //   Serial.print("     ");
    //   Serial.println(kdx_transfer);
    // applyAltitudeBraking(constrain(kix_transfer, 0 , 1));

      // float l= (float)brake_offset_fb;
      // float k = (float)brake_offset_lr;
      uint8_t response[20];
      memcpy(response,     (const void*)&r1_off, 2);
      memcpy(response + 2, (const void*)&r2_off, 2);
      memcpy(response + 4, (const void*)&r3_off, 2);
      memcpy(response + 6, (const void*)&r4_off, 2);
      memcpy(response + 8,  &rollX, 4);
      memcpy(response + 12, &pitchY, 4);
      // memcpy(response + 8,  &l, 4);
      // memcpy(response + 12, &k, 4);
      // memcpy(response + 8, (const void*)&velX_debug, 4);
      // memcpy(response + 12, (const void*)&velY_debug, 4);
      memcpy(response + 16, &yawZ, 4);

      udp.beginPacket(pcIP, pcPort);
      udp.write(response, 20);
      udp.endPacket();
    }
      delay(1);  // Prevent WDT
  }

}
