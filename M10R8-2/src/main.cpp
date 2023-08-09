#include <Arduino.h>
#include <Adafruit_MPU6050.h> //
#include <Adafruit_Sensor.h> //
#include <Wire.h> //I2C komunikacija prema MPU6050
#include <PID_v1.h> //PID regulacija s ugrađdženim errorima
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

const char* ssid = "ESP32_AP";
const char* password = "12345678";

double SetPoint;
double Input; //kut
double Output; //napon prema motorima (0-255) 

double Kp=2.5; //proporcionalna snaga prema kutu - mjeri grešku i što je veća trenutna greška - daje jači napon na motore - proporcionalno
double Ki=1.0; //integral - zbraja grešku tijekom nekog vremena
double Kd=0.0;  //diferencijalni - uspoređuje 2 greške 

AsyncWebServer server(80);

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<body>
  Kp: <input type="range" min="0" max="2" step="0.1" id="Kp" value="1.5"><span id="KpVal">1.5</span><br><br>
  Ki: <input type="range" min="0" max="2" step="0.1" id="Ki"><br>
  Kd: <input type="range" min="0" max="2" step="0.1" id="Kd"><br>
  SetPoint: <input type="range" min="-30" max="30" step="1" id="SetPoint"><br>
  <script>
  var slider_kp = document.getElementById("Kp");

  var val_kp = document.getElementById("KpVal");

  var slider_ki = document.getElementById("Ki");
  var slider_kd = document.getElementById("Kd");
  var slider_sp = document.getElementById("SetPoint");
  slider_kp.oninput = function() {
    fetch("/setKp?value=" + this.value);
    val_kp.textContent = this.value;
  }
  slider_ki.oninput = function() {fetch("/setKi?value=" + this.value);}
  slider_kd.oninput = function() {fetch("/setKd?value=" + this.value);}
  slider_sp.oninput = function() {fetch("/setSetPoint?value=" + this.value);}
  </script>
</body></html>
)rawliteral";

Adafruit_MPU6050 mpu;

#define SDA_PIN 18
#define SCL_PIN 19
#define MOTOR_PIN 12

sensors_event_t a, g, temp;

// Define the PID controller
PID myPID(&Input, &Output, &SetPoint, Kp, Ki, Kd, DIRECT);  // You might need to adjust these PID parameters (Kp, Ki, Kd)

void setup(void) {
    Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);

  WiFi.softAP(ssid, password);  //dodati
  Serial.println();
  Serial.print("IP address: ");   //dodati
  Serial.println(WiFi.softAPIP());  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  server.on("/setKp", HTTP_GET, [](AsyncWebServerRequest *request){
    String message;
    if (request->hasParam("value")) {
      Kp = request->getParam("value")->value().toFloat();
      message = "Kp changed to: " + String(Kp);
      myPID.SetTunings(Kp,Ki,Kd);
    } else {
      message = "No message sent";
    }
    request->send(200, "text/plain", message);
  });
  server.on("/setKi", HTTP_GET, [](AsyncWebServerRequest *request){
    String message;
    if (request->hasParam("value")) {
      Ki = request->getParam("value")->value().toFloat();
      message = "Ki changed to: " + String(Ki);
      myPID.SetTunings(Kp,Ki,Kd);
    } else {
      message = "No message sent";
    }
    request->send(200, "text/plain", message);
  });
  server.on("/setKd", HTTP_GET, [](AsyncWebServerRequest *request){
    String message;
    if (request->hasParam("value")) {
      Kd = request->getParam("value")->value().toFloat();
      message = "Kd changed to: " + String(Kd);
      myPID.SetTunings(Kp,Ki,Kd);
    } else {
      message = "No message sent";
    }
    request->send(200, "text/plain", message);
  });

server.on("/setSetPoint", HTTP_GET, [](AsyncWebServerRequest *request){
    String message;
    if (request->hasParam("value")) {
      SetPoint = request->getParam("value")->value().toFloat();
      message = "Kd changed to: " + String(Kd);
      myPID.SetTunings(Kp,Ki,Kd);
    } else {
      message = "No message sent";
    }
    request->send(200, "text/plain", message);
  });


  
  server.begin();


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
  SetPoint = 0;
  // Turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);  // Ensure Output is within PWM signal range.
}

void loop() {
  /* Get new sensor events with the readings */
  
  mpu.getEvent(&a, &g, &temp);

  Input = (atan(a.acceleration.y / a.acceleration.z)) * 180 / PI ;

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