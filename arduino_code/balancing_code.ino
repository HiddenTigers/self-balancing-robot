#include <Wire.h>
#include <Adafruit_INA219.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_VL53L0X.h"
#include <math.h>
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <movingAvg.h>

#define SDA_PIN 8  // ESP32-S3 SDA
#define SCL_PIN 9  // ESP32-S3 SCL
#define LOX1_ADDRESS 0x30
#define LOX2_ADDRESS 0x31
int sensor1,sensor2;

#define SHT_LOX1 10
#define SHT_LOX2 11

Adafruit_MPU6050 mpu;
Adafruit_INA219 ina219;
Adafruit_VL53L0X lox1 = Adafruit_VL53L0X();
Adafruit_VL53L0X lox2 = Adafruit_VL53L0X();

const char* ssid = "XXXXXXXXXXXXXXXX";
const char* password = "XXXXXXXXXXXXXXXX";

AsyncWebServer server(80);
AsyncWebSocket ws("/ws"); // WebSocket on URL /ws

int motor1Pin1 = 5; 
int motor1Pin2 = 6; 
int enable1Pin = 4; 
int motor2Pin1 = 7; 
int motor2Pin2 = 15; 
int enable2Pin = 16; 

int hallPin1 = 17;

// Setting PWM properties
const int freq = 30000;
const int resolution = 8;

const int pwmChannelMotor1 = 0;
const int pwmChannelMotor2 = 1;

int dutyCycleMotor1 = 200;
int dutyCycleMotor2 = 200;

volatile int pulseCount1 = 0; // Counts the pulses
unsigned long lastTime = 0;
unsigned long lastTimempu = 0;
unsigned long lastTimetorque = 0;

float motorSpeed1 = 0.0; // Motor speed in RPM

float angleX = 0, angleY = 0;
float prevTime = 0;  // Time tracking for integration

VL53L0X_RangingMeasurementData_t measure1;
VL53L0X_RangingMeasurementData_t measure2;

float kp = 15;
float ki = 0;
float kd = 10;

float torquetotal;
int pwm;
int pwm2;

float offset = 0;

movingAvg ay_AVG(3);
movingAvg az_AVG(3);
movingAvg gx_AVG(3);

float calculateTorque(float theta, float omega) {
    float torque = kp * theta + kd * omega;

    Serial.print("CALCULATED TORQUE: ");
    Serial.println(torque);

    return torque;
}

int calculatePWM(float torque) {
    float onemotor = torque/2;

    Serial.print("calculated one motor torque: ");
    Serial.println(onemotor);

    if (onemotor>0) {
      pwm = (int)round(pow(onemotor/0.00000384259, 0.5));
      
      digitalWrite(motor1Pin1, HIGH);
      digitalWrite(motor1Pin2, LOW); 

      digitalWrite(motor2Pin1, HIGH);
      digitalWrite(motor2Pin2, LOW); 

      }
    else if (onemotor<0) {
      
      pwm = (int)round(pow((onemotor * -1)/0.00000384259, 0.5));


      digitalWrite(motor1Pin1, LOW);
      digitalWrite(motor1Pin2, HIGH); 

      digitalWrite(motor2Pin1, LOW);
      digitalWrite(motor2Pin2, HIGH); 
      
    }
    else pwm = 0;

    pwm = constrain(pwm, -255, 255);

    Serial.print("PWM: ");
    Serial.println(pwm);

    return pwm;
}

void IRAM_ATTR pulseInterrupt1() {
  pulseCount1++; // Increment the pulse count when the encoder gives a pulse
}

void setID() {
  // all reset
  digitalWrite(SHT_LOX1, LOW);    
  digitalWrite(SHT_LOX2, LOW);
  delay(10);
  // all unreset
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  // activating LOX1 and reseting LOX2
  digitalWrite(SHT_LOX1, HIGH);
  digitalWrite(SHT_LOX2, LOW);

  // initing LOX1
  if(!lox1.begin(LOX1_ADDRESS)) {
    Serial.println(F("Failed to boot first VL53L0X"));
    while(1);
  }
  delay(10);

  // activating LOX2
  digitalWrite(SHT_LOX2, HIGH);
  delay(10);

  //initing LOX2
  if(!lox2.begin(LOX2_ADDRESS)) {
    Serial.println(F("Failed to boot second VL53L0X"));
    while(1);
  }
}

void read_dual_sensors() {
  
  lox1.rangingTest(&measure1, false); // pass in 'true' to get debug data printout!
  lox2.rangingTest(&measure2, false); // pass in 'true' to get debug data printout!

  // print sensor one reading
  Serial.print("1: ");
  if(measure1.RangeStatus != 4) {     // if not out of range
    sensor1 = measure1.RangeMilliMeter;    
    Serial.print(sensor1);
    Serial.print("mm");    
  } else {
    Serial.print("Out of range");
  }
  
  Serial.print(" ");

  // print sensor two reading
  Serial.print("2: ");
  if(measure2.RangeStatus != 4) {
    sensor2 = measure2.RangeMilliMeter;
    Serial.print(sensor2);
    Serial.print("mm");
  } else {
    Serial.print("Out of range");
  }
  
  Serial.println();
}

void setupMotorPins() {

  pinMode(hallPin1, INPUT_PULLUP);  // Set hallPin as input
  attachInterrupt(digitalPinToInterrupt(hallPin1), pulseInterrupt1, RISING);  // Trigger an interrupt on the rising edge

  Serial.println("Motor Speed Monitoring Started");
  // sets the pins as outputs:
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);

  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);

  // configure LEDC PWM for both motors (using ledcAttach)
  
  ledcAttach(enable1Pin, freq, resolution);
  ledcAttach(enable2Pin, freq, resolution);

}

void setupSensors() {
  if (! ina219.begin())
  {
    Serial.println("Failed to find INA219 chip");
    while (1) 
    {
      delay(10);
    }
  }

  if (!mpu.begin()) {
    Serial.println("MPU6050 not detected!");
    while (1) delay(10);
  }

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  pinMode(SHT_LOX1, OUTPUT);
  pinMode(SHT_LOX2, OUTPUT);

  Serial.println("Shutdown pins inited...");

  digitalWrite(SHT_LOX1, LOW);
  digitalWrite(SHT_LOX2, LOW);

  Serial.println("Both in reset mode...(pins are low)");
  
  
  Serial.println("Starting...");
  setID();

  ay_AVG.begin();
  az_AVG.begin();
  gx_AVG.begin();

}


void setup() {
  Serial.begin(115200);

  Wire.begin(8, 9);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! I am ESP32.");
  });

  server.begin();
  Serial.println("HTTP server started");

  ElegantOTA.begin(&server);

  setupMotorPins();
  setupSensors();
}

void loop() {

  ElegantOTA.loop();

  unsigned long currentTime = millis();

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = ay_AVG.reading(a.acceleration.y);
  float az = az_AVG.reading(a.acceleration.z);
  
  float gx = -1 * gx_AVG.reading(g.gyro.x);  // Angular velocity (rad/s) 
  float gy = -1 * g.gyro.y;
  float gz = -1 * g.gyro.z;

  float dt = (currentTime - lastTimempu)/1000;
  lastTimempu = currentTime;

  float accelAngleX = (atan2(ay, az) * 180 / PI);
  float convertedaccelAngleX = (accelAngleX >= 0.0) ? (180.0 - accelAngleX) : -(accelAngleX + 180.0); // converted cuz upside down
  float accelAngleY = atan2(-ax, sqrt(ay * ay + az * az)) * 180 / PI;

  float gyroAngleX = angleX + gx * dt * 180 / PI;
  float gyroAngleY = angleY + gy * dt * 180 / PI;

  angleX = 0.9 * gyroAngleX + 0.1 * convertedaccelAngleX; //DEGREES from complementary filter
  angleY = 0.9 * gyroAngleY + 0.1 * accelAngleY;

  float RAD_angleX = angleX/180*PI; // converted complementary filter angle

  // Printing in format for serial plotter

  Serial.print("GyroAngleX:");
  Serial.print(gyroAngleX);
  Serial.print(",");
  Serial.print("AccelerometerAngleX:");
  Serial.print(convertedaccelAngleX);
  Serial.print(",");
  Serial.print("CombinedAngle:");
  Serial.println(angleX);

  //Print results
  Serial.print("Angle X: "); Serial.print(angleX); Serial.print("° | ");

 
  if (currentTime - lastTimetorque >= 5) { 
    Serial.print("Time difference: ");
    Serial.println(currentTime - lastTimetorque);
    lastTimetorque = currentTime;

    torquetotal = calculateTorque (RAD_angleX, gx);
    pwm2 = calculatePWM (torquetotal);

    if (abs(angleX) > 45) {
      pwm2 = 0; 
    }

    ledcWrite(enable1Pin, pwm2);  
    ledcWrite(enable2Pin, pwm2);   
  }
}