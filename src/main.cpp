#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>

//#define SERVO_PIN 18       // GPIO pin connected to servo signal
//#define SERVO_CH  0        // LEDC channel
//#define SERVO_FREQ 50      // 50 Hz = 20ms period
//#define SERVO_RES 16       // 16-bit resolution
bool state= 0;
int ticks =0;


class Servo {
  public:
    int SERVO_PIN_L; // GPIO pin connected to servo signal L
    int SERVO_PIN_R; // GPIO pin connected to servo signal L
    int SERVO_CH_L=0 ;        // LEDC channel
    int SERVO_CH_R=1 ;        // LEDC channel
    int SERVO_FREQ=50;     // 50 Hz = 20ms period
    int SERVO_RES=16;       // 16-bit resolution

  uint32_t pulseToDuty(uint16_t microseconds) {
  const uint32_t maxDuty = (1 << SERVO_RES) - 1; // 65535 for 16-bit
  const uint32_t period = 1000000 / SERVO_FREQ;  // 20,000 us
  return (uint32_t)((microseconds * (uint64_t)maxDuty) / period);
  }
   void setup_n_attach(int pin_l,int pin_r) {
    SERVO_PIN_L = pin_l;
    SERVO_PIN_R = pin_r;
    ledcSetup(SERVO_CH_L, SERVO_FREQ, SERVO_RES);
    ledcAttachPin(SERVO_PIN_L, SERVO_CH_L);
    
    ledcSetup(SERVO_CH_R, SERVO_FREQ, SERVO_RES);
    ledcAttachPin(SERVO_PIN_R, SERVO_CH_R);

   }
    void go_straight(){
    ledcWrite(SERVO_CH_L, pulseToDuty(2000));
    ledcWrite(SERVO_CH_R, pulseToDuty(1000));
   }
   void turn_right(){
    ledcWrite(SERVO_CH_L, pulseToDuty(2000));
    ledcWrite(SERVO_CH_R, pulseToDuty(1600));
   }
   void turn_left(){
    ledcWrite(SERVO_CH_L, pulseToDuty(1400));
    ledcWrite(SERVO_CH_R, pulseToDuty(1000));
   }
   void go_backwards (){
    ledcWrite(SERVO_CH_L, pulseToDuty(1000));
    ledcWrite(SERVO_CH_R, pulseToDuty(2000));
   }
   void stop(){
    ledcWrite(SERVO_CH_L, pulseToDuty(1500));
    ledcWrite(SERVO_CH_R, pulseToDuty(1500));
   }
};
// Helper to convert microseconds to LEDC duty value
//Creating servo class robocik
Servo robocik;
//
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

//String header;

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    String msg = String((char*)data).substring(0, len);

    if (msg == "FORWARD_START") {
      robocik.go_straight();
      Serial.println("GPIO 26 ON (button pressed)");
    }
    else if (msg == "FORWARD_STOP") {
      robocik.stop();
      Serial.println("GPIO 26 OFF (button released)");
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
             AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_CONNECT) {
    Serial.println("WebSocket client connected");
  } else if (type == WS_EVT_DISCONNECT) {
    Serial.println("WebSocket client disconnected");
  } else if (type == WS_EVT_DATA) {
    handleWebSocketMessage(arg, data, len);
  }
}


void setup() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("dupajasiu","pierdzistasiu");
  Serial.begin(9600);
  IPAddress myIP = WiFi.softAPIP();
  
  //Serial.println(maxDuty);
  robocik.setup_n_attach(25,26);
  Serial.println("Enter command: w=forward, s=stop, r=reverse");
  //ledcSetup(SERVO_CH, SERVO_FREQ, SERVO_RES);
  //ledcAttachPin(SERVO_PIN, SERVO_CH);
  Serial.println(myIP);

  ws.onEvent(onEvent);
  server.addHandler(&ws);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
button { font-size: 24px; padding: 20px; margin: 10px; }
</style>
</head>
<body>
<h2>ESP32 WebSocket Control</h2>
<button id="forward">Hold Forward</button>

<script>
  var gateway = `ws://${window.location.hostname}/ws`;
  var websocket;

  window.addEventListener('load', onLoad);

  function initWebSocket() {
    websocket = new WebSocket(gateway);
  }

  function onLoad() {
    initWebSocket();
    const btn = document.getElementById("forward");
    btn.addEventListener("mousedown", () => { websocket.send("FORWARD_START"); });
    btn.addEventListener("mouseup",   () => { websocket.send("FORWARD_STOP"); });
    btn.addEventListener("touchstart",() => { websocket.send("FORWARD_START"); });
    btn.addEventListener("touchend",  () => { websocket.send("FORWARD_STOP"); });
  }
</script>
</body>
</html>
)rawliteral");
  });

  server.begin();
}

void loop() {
  ws.cleanupClients();  // keep WebSocket alive
}