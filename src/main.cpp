// yes this is silly ... but I want to use ESTLCAM as controller and don't want to connect my China Spindle/vfd
// as it is noisy as hell ... so I came up with the idea to "read" the PWM signal of estlecam and convert it
// into the appropiat RS485 command. The data are send via optocoupler so there is no more electrical connection
// between estlcam and the VFD. 
// most of this is based on https://github.com/GilchristT/SpindleTalker2/

#include <Arduino.h>
#include <SoftwareSerial.h>
#include <Crc16.h>


#define RUN_PIN 14 // D5
#define PULSE_PIN 15
#define SS_RX 4
#define SS_TX 5
#define SS_DE 13
#define SS_RE 12

#define PULSE_US 126
#define RPM_MIN 8000
#define RPM_MAX 24000

#define VFD_ID 0x01
    
#define FUNC_READ 0x01
#define FUNC_WRITE 0x02
#define WRITE_CTRL_DATA 0x03
#define READ_CTRL_DATA 0x04
#define WRITE_FREQ_DATA 0x05

#define REG_START 0x08
#define REG_OUTPUT_FREQ 0x01
#define REG_OUTPUT_AMPS 0x02
#define REG_CURRENT_RPM = 0x03

#define VAL_RUN_FWD 0x01
#define VAL_RUN_BWD 0x11
#define VAL_RUN_STOP 0x08


uint32_t pw;  
int32_t rpm_filtered;  
uint32_t spindle_rpm;
bool spindle_running;
bool run_pin_state;
uint32_t last_updated;
uint32_t last_measured;

SoftwareSerial myserial(SS_RX,SS_TX); // RX, TX
Crc16 crc; 

void query(uint8_t cmd, uint8_t len, uint8_t* data);
char toHexa(int i);
void set_rpm(uint16_t v);
void set_state(bool s);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("setup!");
  delay(1000);

  pw = 0;  
  rpm_filtered = 0;  
  spindle_rpm = 0;
  spindle_running = false;
  run_pin_state = false;
  last_updated = millis();
  last_measured = millis();

  pinMode(PULSE_PIN,INPUT);
  pinMode(RUN_PIN,INPUT);
  pinMode(SS_DE,OUTPUT);
  pinMode(SS_RE,OUTPUT);
  myserial.begin(9600);
  digitalWrite(SS_DE,LOW); 
  digitalWrite(SS_RE,LOW); 
  Serial.println("end setup");
}

void loop() {
  if(millis()-last_measured>100){ // 100 measurements / sec
    last_measured = millis();
    pw = pulseIn(PULSE_PIN, HIGH, PULSE_US*2);
    if(pw==0){ 
      if(digitalRead(PULSE_PIN)==HIGH){
        set_rpm(RPM_MAX);
      } else {
        set_rpm(RPM_MIN);
      }
    } else {
      // 0 = 8000
      // 140-ish = 240000
      // x = 8000+(24000-8000)/140*pw_filtered
      int32_t v = RPM_MIN + ((RPM_MAX-RPM_MIN)/PULSE_US)*pw; // 8000 + x*126
      rpm_filtered = rpm_filtered + (v-rpm_filtered)/16; // 126/16=8
      Serial.print(v);
      Serial.print(" / ");
      Serial.println(rpm_filtered);
      set_rpm(rpm_filtered);
    }
  }
  if(digitalRead(RUN_PIN)!=run_pin_state){
    run_pin_state=!run_pin_state;
    set_state(run_pin_state);
  }
  /*
  Serial.println("an");
  set_state(false);
  delay(10000);
  Serial.println("8000");
  set_rpm(8000);
  delay(10000);
  Serial.println("10000");
  set_rpm(10000);
  delay(10000);
  Serial.println("aus");
  set_state(true);
  delay(10000);
  */
}

void set_state(bool s){
  // low active!!
  spindle_running = !s;
  if(s){
    Serial.println("turn off");
    uint8_t stop[1] = {VAL_RUN_STOP};
    query(WRITE_CTRL_DATA,1,stop);
  } else {
    Serial.println("turn on");
    uint8_t d=0x00;
    query(READ_CTRL_DATA,1,&d); // I'm not sure why this is needed but it seems to be
    last_updated = 0; // force update
    set_rpm(6000); // 600.00 is a dummy .. 
    d = VAL_RUN_FWD;
    query(WRITE_CTRL_DATA,1,&d);
  }
}

void set_rpm(uint16_t v) {
  if(millis()-last_updated<1000){
    Serial.println("ignore");
    return;
  }
  last_updated = millis();
  delay(1);

  if(!spindle_running){
    Serial.println("not running");
    return;
  }
  //Serial.println("t2"); delay(1000);

  if(abs(spindle_rpm-v)<250){
    Serial.print("Ignore new speed ");
    Serial.print(v);
    Serial.print(", due to small difference to "); 
    Serial.println(spindle_rpm);
    return;
  }
  //Serial.println("t3"); delay(1000);

  Serial.print("Set new speed "); 
  Serial.println(v,DEC);
  spindle_rpm = v;

  uint16_t freq = ((uint32_t)v)*5/3; // v/24000max_rpm*400Hz*100 (spindle expects freq in centi Herz) // https://github.com/GilchristT/SpindleTalker2/blob/d160041235267ffd40e0f1bd7a7cff793b35eb8d/SpindleClass.cs#L84
  uint16_t freq_LSBF = (freq&0xff)<<8 | (freq>>8);
  query(WRITE_FREQ_DATA,2,(uint8_t*)&freq_LSBF);
}

char toHexa(int i) {
  if(i<10) return 0x30 | i; 
  return 0x41 + i - 10; 
}


void query(uint8_t cmd, uint8_t len, uint8_t* data){
  uint8_t out[len+5]; // ID, CMD, LEN, CRC_LOW, CRC_HIGH
  out[0] = VFD_ID;
  out[1] = cmd;
  out[2] = len;
  for(uint8_t i=0; i<len; i++){
    out[3+i] = data[i];
  }
  uint16_t crc_value = crc.Modbus(out,0,3+len);
  out[3+len] = crc_value & 0xff;
  out[4+len] = crc_value >> 8;

  digitalWrite(SS_DE,HIGH); 
  delay(50);
  Serial.print("[Serial] ");

  for(uint8_t i=0; i<len+5; i++){
    ////////////////// SERIAL DEBUG //////////////////////
    // high nibble
    Serial.print("0x");
    Serial.print((char)toHexa(out[i]>>4));
    // low nibble
    Serial.print((char)toHexa(out[i]&0x0f));
    Serial.print((char)' ');
    ////////////////// SERIAL DEBUG //////////////////////
    
    myserial.print((char)out[i]);
  }

  Serial.println(' ');
  delay(100);
  digitalWrite(SS_DE,LOW); 
}
