#include <Arduino.h>

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
Servo robocik;

void setup() {
  
  Serial.begin(9600);
  //Serial.println(maxDuty);
  robocik.setup_n_attach(25,26);
  Serial.println("Enter command: w=forward, s=stop, r=reverse");
  //ledcSetup(SERVO_CH, SERVO_FREQ, SERVO_RES);
  //ledcAttachPin(SERVO_PIN, SERVO_CH);
}

void loop() {
   if (Serial.available() > 0) {
    
    char cmd = Serial.read();   // read one character
    if (cmd == 'w') {
      
      //Serial.println("Forward!");
      robocik.go_straight();
      // trigger forward action
    }
    else if (cmd == 'd') { 
      //Serial.println("Right!");
      robocik.turn_right();
      // trigger reverse action
    }
    else if (cmd == 'a') {
      //Serial.println("Right!");
      robocik.turn_left();
      // trigger reverse action
    }
    else if (cmd == 's') {
      //Serial.println("Backwards!");
      robocik.go_backwards();
    
  }
  ticks = 0;
   state=0;
  }
  else if (Serial.available() == 0 && state == 0 && ticks == 2000 ){
      ticks = 0;
      state=1;
      //Serial.println("Stop!");
      robocik.stop();
  }
  else if (Serial.available() == 0 ){ 
  //Serial.println("YOMAMA!");
   ticks ++;
  }
  
      // trigger stop action  
  
}
  ///for (int x = 2000; x > 1500;x= x-100 ){ledcWrite(SERVO_CH, pulseToDuty(x));
 // delay(1500);};// Full speed CW (2000 µs)
  //robocik.stop(2000);

  // Stop (1500 µs)
  //ledcWrite(SERVO_CH, pulseToDuty(1500));
  //delay(5000);

  // Full speed CCW (1000 µs)
  //for (int x = 1500; x > 1000;x= x-100 ){ledcWrite(SERVO_CH, pulseToDuty(x));
  //delay(1500);};// Full speed CW (2000 µs)
