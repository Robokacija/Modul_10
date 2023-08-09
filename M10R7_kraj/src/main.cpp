#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <PID_v1.h>

Adafruit_MPU6050 mpu;

#define SDA_PIN 18
#define SCL_PIN 19
#define MOTOR_PIN 14

double Setpoint, Input, Output;

double Kp=3.5;
double Ki=0.0;
double Kd=2.0;

sensors_event_t a, g, temp;

// Define the PID controller
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);  // You might need to adjust these PID parameters (Kp, Ki, Kd)

void setup(void) {
    Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);
  pinMode(MOTOR_PIN,OUTPUT);
  analogWrite(MOTOR_PIN,0);

  while (!Serial)
    delay(10);

  Serial.println("Adafruit MPU6050 test!");



  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
  Serial.println("MPU6050 Found!");
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  // rest of your setup code...
  // ...

  // Initialize PID Controller
  Setpoint = 0;
  // Turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);  // Ensure Output is within PWM signal range.
}

void loop() {
  /* Get new sensor events with the readings */
  
  mpu.getEvent(&a, &g, &temp);

  Input = (atan(a.acceleration.y / sqrt(pow(a.acceleration.x, 2) + pow(a.acceleration.z, 2))) * 180 / PI) ;

  // PID computation and Motor control
  bool calculated = myPID.Compute();

  analogWrite(MOTOR_PIN, (int)Output);

  /* Print out the values */
  Serial.print("Roll: ");
  Serial.print(Input);
  Serial.print(", Motor Output: ");
  Serial.print (Output);
    Serial.print(", ax :");
  Serial.print(a.acceleration.x);
    Serial.print(", ay :");
  Serial.print(a.acceleration.y);
    Serial.print(", PID :");
  Serial.println(calculated);

// Serial.print(", OUTPUT : ");
//   Serial.print(Output);
//   Serial.print(", erP : ");
//   Serial.print(errorP);
//   Serial.print(", erI : ");
//   Serial.print(errorI);
//   Serial.print(", erD : ");
//   Serial.println(errorD);

  delay(1);
}