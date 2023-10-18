#include <Servo.h>
#include <HardwareSerial.h>


struct Msg{
  unsigned short pw_servo;
  unsigned short pw_esc;
};

union BytesToMsg {
  byte bytes[4];
  Msg msg;
};

BytesToMsg msg_buffer;

//rate of serial communication
const long BAUD_RATE = 250000;

bool stale = true;
long time_last_msg;
long stale_time_limit = 200; //time in milliseconds before stopping vehicle when no signal
bool process = false;

//servo variables
//pulse widths in microseconds
const unsigned short S_PW_MAX = 1980;
const unsigned short S_PW_NEUTRAL = 1500;
const unsigned short S_PW_MIN = 1000;
const unsigned short S_PW_MAX_STEP = 5;
unsigned short steering_pw = S_PW_NEUTRAL;

const unsigned short M_PW_MAX = 1980;
const unsigned short M_PW_NEUTRAL = 1500;
const unsigned short M_PW_MIN = 1000;
const unsigned short M_PW_MAX_STEP = 5;
unsigned short motor_pw = M_PW_NEUTRAL;

const int STEERING_PIN = 5;
const int ESC_PIN = 6;

Servo steering;
Servo esc;

long last_t;

void setup() {
  Serial.begin(250000);
  Serial.setTimeout(0.1);

  //configure PWM drivers
  steering.attach(STEERING_PIN);
  esc.attach(ESC_PIN);

  //wait for a serial connection
  while(Serial.available() <=0){
    delay(100);
  }
  
  last_t = millis();
  time_last_msg = millis();

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
}


void loop() {
  //parse any waiting serial messages
  while(Serial.available()){
    uint8_t msg_size;
    Serial.readBytes(&msg_size,sizeof(msg_size));
    if(msg_size == sizeof(msg_buffer)){
      Serial.readBytes(msg_buffer.bytes,sizeof(msg_buffer));
      time_last_msg = millis();
//      process = true;
      digitalWrite(LED_BUILTIN, HIGH);
    }
  }

  Serial.flush();

  //check our timers for when no data is received
  long t = millis();
  //if(t > last_t + 1000){
  //  Serial.print("Tick. Stale = ");
   // Serial.println(stale);
   // last_t = t;
  //}
  if(stale){
    //write a neutral signal to esc
    motor_pw = M_PW_NEUTRAL; //esc can go to neutral immediately, steering servo cannot
    esc.writeMicroseconds(M_PW_NEUTRAL);

    //reset the msg data
    msg_buffer.msg.pw_servo = S_PW_NEUTRAL;
    msg_buffer.msg.pw_esc = M_PW_NEUTRAL;
  }

  //update our stale timer
  stale = t - time_last_msg > stale_time_limit;

  //if message was received, use it to determine pwm values (clamped to prevent instant changes in ESC and servo)
//  if(process){
    process = false;
    digitalWrite(LED_BUILTIN, LOW); 
    //write values to steering and esc
    unsigned short target_s_pw = (msg_buffer.msg.pw_servo <= S_PW_MAX && msg_buffer.msg.pw_servo >= S_PW_MIN) ? msg_buffer.msg.pw_servo : steering_pw;
    target_s_pw = constrain(target_s_pw, S_PW_MIN, S_PW_MAX);
    steering_pw = constrain(target_s_pw, steering_pw-S_PW_MAX_STEP, steering_pw+S_PW_MAX_STEP);
    steering.writeMicroseconds(steering_pw);
    
    unsigned short target_m_pw = (msg_buffer.msg.pw_esc <= M_PW_MAX && msg_buffer.msg.pw_esc >= M_PW_MIN) ? msg_buffer.msg.pw_esc : motor_pw;
    target_m_pw = constrain(target_m_pw, M_PW_MIN, M_PW_MAX);
    motor_pw = constrain(target_m_pw, motor_pw-M_PW_MAX_STEP,motor_pw+M_PW_MAX_STEP);
    esc.writeMicroseconds(motor_pw);
//  }

  delay(10);
}
