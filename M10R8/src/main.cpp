#include <WiFi.h>
#include <ESPAsyncWebServer.h>

const char* ssid = "ESP32_AP";
const char* password = "12345678";

float Kp = 0.0f, Ki = 0.0f, Kd = 0.0f;

AsyncWebServer server(80);

const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<body>
  Kp: <input type="range" min="0" max="10" step="0.1" id="Kp"><br>
  Ki: <input type="range" min="0" max="10" step="0.1" id="Ki"><br>
  Kd: <input type="range" min="0" max="10" step="0.1" id="Kd"><br>
  <script>
  var slider_kp = document.getElementById("Kp");
  var slider_ki = document.getElementById("Ki");
  var slider_kd = document.getElementById("Kd");
  slider_kp.oninput = function() {fetch("/setKp?value=" + this.value);}
  slider_ki.oninput = function() {fetch("/setKi?value=" + this.value);}
  slider_kd.oninput = function() {fetch("/setKd?value=" + this.value);}
  </script>
</body></html>
)rawliteral";

void setup(){
  Serial.begin(115200);
  WiFi.softAP(ssid, password);
  Serial.println();
  Serial.print("IP address: ");
  Serial.println(WiFi.softAPIP());
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });
  server.on("/setKp", HTTP_GET, [](AsyncWebServerRequest *request){
    String message;
    if (request->hasParam("value")) {
      Kp = request->getParam("value")->value().toFloat();
      message = "Kp changed to: " + String(Kp);
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
    } else {
      message = "No message sent";
    }
    request->send(200, "text/plain", message);
  });
  server.begin();
}

void loop(){}
