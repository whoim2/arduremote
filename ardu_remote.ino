/*
 ARDU_REMOTE
 usb / ppm simple remote
 whoim@mail.ru
*/

#include <EEPROM.h>
#include <Joystick.h>
#include "PPMEncoder.h"

//uncomment this for see states in port monitor, 9600bps
//usb joystic not work, comment for usb mode
//#define SERIAL_DEBUG

#define TRTL_PIN          A2
#define YAW_PIN           A3
#define PITCH_PIN         A1
#define ROLL_PIN          A0
#define SW1_PIN           6
#define SW2_PIN           4
#define SW3_1_PIN         7
#define SW3_2_PIN         8
#define SW4_PIN           16  //comment to disable
#define SW5_PIN           14  //comment to disable
#define AUX1_PIN          A10 //comment to disable

//
#define BUZZER_PIN        3
#define LED_PIN           2
#define SW_CALIBRATE_PIN  5

//PPM output, comment to disable
#define PPM_PIN 9

//non-use beep timer millis, comment to disable
#define NTIM 30000

bool started = false;

//calibrate vars
int rollmax, rollmin, rollzero;
int pitchmax, pitchmin, pitchzero;
int yawmax, yawmin, yawzero;
int throttlemax, throttlemin;
#ifdef AUX1_PIN
int aux1max,aux1min;
#endif

#ifdef NTIM
uint32_t prevtimer = millis();
int16_t prev_ch_crc;
#endif

//********* FUNCTIONS ***********
void EEPROM_int_write(int addr, int num) {
  byte raw[2];
  (int&)raw = num;
  for(byte i = 0; i < 2; i++) EEPROM.write(addr+i, raw[i]);
}
int EEPROM_int_read(int addr) {    
  byte raw[2];
  for(byte i = 0; i < 2; i++) raw[i] = EEPROM.read(addr+i);
  int &num = (int&)raw;
  return num;
}

void beep(uint16_t time) {
  digitalWrite(LED_PIN, LOW);
  tone(BUZZER_PIN, 1000);
  delay(time);
  noTone(BUZZER_PIN);
  digitalWrite(LED_PIN, HIGH);
}

//********* SETUP ***********
void setup() {
  
  //serial
  #ifdef SERIAL_DEBUG //if debug
    Serial.begin(9600);
  #else
    Joystick.begin(false);
    Joystick.setThrottle(0);
    Joystick.setRudder(0);
    Joystick.setXAxis(0);
    Joystick.setYAxis(0);
    Joystick.setXAxis(0);
    Joystick.setXAxisRotation(0);
    Joystick.setYAxisRotation(0);
    Joystick.setZAxisRotation(0);
    Joystick.setHatSwitch(1,0);
    Joystick.setHatSwitch(2,0);
  #endif

  //ppm
  #ifdef PPM_PIN
    ppmEncoder.begin(PPM_PIN);
  #endif
  
  //pins
  pinMode(SW1_PIN, INPUT_PULLUP);
  pinMode(SW2_PIN, INPUT_PULLUP);
  pinMode(SW3_1_PIN, INPUT_PULLUP);
  pinMode(SW3_2_PIN, INPUT_PULLUP);
  #ifdef SW4_PIN
  pinMode(SW4_PIN, INPUT_PULLUP);
  #endif
  #ifdef SW5_PIN
  pinMode(SW5_PIN, INPUT_PULLUP);
  #endif
  pinMode(SW_CALIBRATE_PIN, INPUT_PULLUP);
  
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  //start led
  digitalWrite(LED_PIN, HIGH);
  delay(100);
  //calibration at button
  if(digitalRead(SW_CALIBRATE_PIN) == LOW) {
    delay(500);
      beep(1000);
      #ifdef SERIAL_DEBUG
        Serial.println(F("Calibrate starting"));
      #endif
      //null & min position
      rollzero = round((analogRead(ROLL_PIN) + 0.5) / 10) * 10;
      rollmin = rollzero;
      EEPROM_int_write(2, rollzero);
      EEPROM_int_write(14, rollmin);
      
      pitchzero = round((analogRead(PITCH_PIN) + 0.5) / 10) * 10;
      pitchmin = pitchzero;
      EEPROM_int_write(6, pitchzero);
      EEPROM_int_write(22, pitchmin);
      
      yawzero = round((analogRead(YAW_PIN) + 0.5) / 10) * 10;
      yawmin = yawzero;
      EEPROM_int_write(10, yawzero);
      EEPROM_int_write(30, yawmin);
      
      throttlemin = analogRead(TRTL_PIN);
      EEPROM_int_write(38, throttlemin);

      #ifdef AUX1_PIN
      aux1min = analogRead(AUX1_PIN);
      EEPROM_int_write(46, aux1min);
      #endif
    
      while(1) { //min and max positions
        #ifdef SERIAL_DEBUG
          Serial.println(F("New calibrate data"));
        #endif
        //roll
        if(analogRead(ROLL_PIN) < rollmin) {
           rollmin = analogRead(ROLL_PIN);
           EEPROM_int_write(14, rollmin);
           beep(20);
        }
        if(analogRead(ROLL_PIN) > rollmax) {
           rollmax = analogRead(ROLL_PIN);
           EEPROM_int_write(18, rollmax);
           beep(20);
        }
        //pitch
        if(analogRead(PITCH_PIN) < pitchmin) {
           pitchmin = analogRead(PITCH_PIN);
           EEPROM_int_write(22, pitchmin);
           beep(20);
        }
        if(analogRead(PITCH_PIN) > pitchmax) {
           pitchmax = analogRead(PITCH_PIN);
           EEPROM_int_write(26, pitchmax);
           beep(20);
        }
        //yaw
        if(analogRead(YAW_PIN) < yawmin) {
           yawmin = analogRead(YAW_PIN);
           EEPROM_int_write(30, yawmin);
           beep(20);
        }
        if(analogRead(YAW_PIN) > yawmax) {
           yawmax = analogRead(YAW_PIN);
           EEPROM_int_write(34, yawmax);
           beep(20);
        }
        //throttle
        if(analogRead(TRTL_PIN) < throttlemin) {
           throttlemin = analogRead(TRTL_PIN);
           EEPROM_int_write(38, throttlemin);
           beep(20);
        }
        if(analogRead(TRTL_PIN) > throttlemax) {
           throttlemax = analogRead(TRTL_PIN);
           EEPROM_int_write(42, throttlemax);
           beep(20);
        }
        //aux1
        #ifdef AUX1_PIN
        if(analogRead(AUX1_PIN) < aux1min) {
           aux1min = analogRead(AUX1_PIN);
           EEPROM_int_write(46, aux1min);
           beep(20);
        }
        if(analogRead(AUX1_PIN) > aux1max) {
           aux1max = analogRead(AUX1_PIN);
           EEPROM_int_write(50, aux1max);
           beep(20);
        }
        #endif

        //debug
        #ifdef SERIAL_DEBUG
          Serial.print("R: " + String(rollmin) + " < " + String(rollzero) + " > " + String(rollmax));
          Serial.print("\tP: " + String(pitchmin) + " < " + String(pitchzero) + " > " + String(pitchmax));
          Serial.print("\tT: " + String(throttlemin) + " <> " + String(throttlemax));
          Serial.print("\tY: " + String(yawmin) + " < " + String(yawzero) + " > " + String(yawmax));
          #ifdef AUX1_PIN
          Serial.print("\tA1: " + String(aux1min) + " <> " + String(aux1max));
          #endif
          Serial.println();
        #endif

        delay(200);
        
      }//while 1
  }//calibration

  //read calibration data
  rollzero =  EEPROM_int_read(2);
  pitchzero = EEPROM_int_read(6);
  yawzero =  EEPROM_int_read(10);
  rollmin =   EEPROM_int_read(14);
  rollmax =   EEPROM_int_read(18);
  pitchmin =  EEPROM_int_read(22);
  pitchmax =  EEPROM_int_read(26);
  yawmin =   EEPROM_int_read(30);
  yawmax =   EEPROM_int_read(34);
  throttlemin =  EEPROM_int_read(38);
  throttlemax =  EEPROM_int_read(42);
  #ifdef AUX1_PIN
  aux1min =  EEPROM_int_read(46);
  aux1max =  EEPROM_int_read(50);
  #endif

  #ifdef SERIAL_DEBUG
    beep(100);
    delay(3000);
    Serial.print("R: " + String(rollmin) + " < " + String(rollzero) + " > " + String(rollmax));
    Serial.print("\tP: " + String(pitchmin) + " < " + String(pitchzero) + " > " + String(pitchmax));
    Serial.print("\tT: " + String(throttlemin) + " <> " + String(throttlemax));
    Serial.print("\tY: " + String(yawmin) + " < " + String(yawzero) + " > " + String(yawmax));
    #ifdef AUX1_PIN
    Serial.print("\tA1: " + String(aux1min) + " <> " + String(aux1max));
    #endif
    delay(3000);
  #endif
}

//********* LOOP ***********
void loop() {
  #ifdef NTIM
    int16_t ch_crc = 0; 
  #endif
  
  //roll
  int8_t roll = 0;
  if( round((analogRead(ROLL_PIN) + 0.5) / 10) * 10 != rollzero )  roll = map(analogRead(ROLL_PIN),rollmin,rollmax,-127,127);
  #ifdef NTIM
    ch_crc += roll; 
  #endif
  //pitch
  int8_t pitch = 0;
  if( round((analogRead(PITCH_PIN) + 0.5) / 10) * 10 != pitchzero)  pitch = map(analogRead(PITCH_PIN),pitchmin,pitchmax,-127,127);
  #ifdef NTIM
    ch_crc += pitch; 
  #endif
  //yaw
  int8_t yaw = 0;
  if( round((analogRead(YAW_PIN) + 0.5) / 10) * 10 != yawzero) yaw = map(analogRead(YAW_PIN),yawmin,yawmax,-127,127);
  #ifdef NTIM
    ch_crc += yaw; 
  #endif
  //throttle
  uint8_t throttle = map(analogRead(TRTL_PIN),throttlemin,throttlemax,0,255);
  #ifdef NTIM
    ch_crc += throttle; 
  #endif
  //sw1
  uint8_t sw1 = 0;
  if(digitalRead(SW1_PIN) == LOW) sw1 = 255;
  #ifdef NTIM
    ch_crc += sw1+20; 
  #endif
  //sw2
  uint8_t sw2 = 0;
  if(digitalRead(SW2_PIN) == LOW) sw2 = 255;
  #ifdef NTIM
    ch_crc += sw2+30; 
  #endif
  //sw3
  uint8_t sw3 = 0;
  if(digitalRead(SW3_1_PIN) == HIGH && digitalRead(SW3_2_PIN) == HIGH) sw3 = 127;
  else if (digitalRead(SW3_1_PIN) == LOW && digitalRead(SW3_2_PIN) == HIGH) sw3 = 255;
  #ifdef NTIM
    ch_crc += sw3+40; 
  #endif
  //sw4
  #ifdef SW4_PIN
    uint8_t sw4 = 0;
    if(digitalRead(SW4_PIN) == LOW) sw4 = 255;
    #ifdef NTIM
      ch_crc += sw4+50; 
    #endif
  #endif
  //sw5
  #ifdef SW5_PIN
    uint8_t sw5 = 0;
    if(digitalRead(SW5_PIN) == LOW) sw5 = 255;
    #ifdef NTIM
      ch_crc += sw5+60; 
    #endif
  #endif
  //aux1
  #ifdef AUX1_PIN
    uint8_t aux1 = map(analogRead(AUX1_PIN),aux1min,aux1max,0,255);
    #ifdef NTIM
      ch_crc += aux1; 
    #endif
  #endif
  //startup protection
  if(!started) {
      if(throttle<10 && sw1==0 && sw2==0 && sw3==0) {
        started = true;
        //start short beep
        beep(200);
      }
      else {
        beep(50);
        delay(300);
      }
  } else {
    #ifdef SERIAL_DEBUG
      /* //uncomment for see raw rounded values
       * Serial.print("R_raw: " + String(round((analogRead(ROLL_PIN) + 0.5) / 10) * 10));
      Serial.print("\tP_raw: " + String(round((analogRead(PITCH_PIN) + 0.5) / 10) * 10));
      Serial.print("\tT_raw: " + String(map(analogRead(TRTL_PIN),throttlemin,throttlemax,0,255)));
      Serial.print("\tY_raw: " + String(round((analogRead(YAW_PIN) + 0.5) / 10) * 10));
      #ifdef AUX1_PIN
      Serial.print("\tA1_raw: " + String(map(analogRead(AUX1_PIN),aux1min,aux1max,0,255)));
      #endif
      Serial.println();
      */
      Serial.print("R: " + String(roll));
      Serial.print("\tP: " + String(pitch));
      Serial.print("\tT: " + String(throttle));
      Serial.print("\tY: " + String(yaw));
      Serial.print("\tS1: " + String(sw1));
      Serial.print("\tS2: " + String(sw2));
      Serial.print("\tS3: " + String(sw3));
      #ifdef SW4_PIN
      Serial.print("\tS4: " + String(sw4));
      #endif
      #ifdef SW5_PIN
      Serial.print("\tS5: " + String(sw5));
      #endif
      #ifdef AUX1_PIN
      Serial.print("\tA1: " + String(aux1));
      #endif
      Serial.println();
      
      delay(200);
    #else
      //sticks
      Joystick.setXAxis(roll);
      Joystick.setYAxis(pitch);
      Joystick.setZAxis(throttle-127);
      Joystick.setXAxisRotation((yaw+127)*1.411);
      
      //for pc/games (use setbutton)
      /*
      Joystick.setButton(0,sw1);
      Joystick.setButton(1,sw2);
      Joystick.setYAxisRotation(sw3);
      #ifdef SW4_PIN
      Joystick.setButton(2,sw4);
      #endif
      #ifdef SW5_PIN
      Joystick.setButton(3,sw5);
      #endif
      #ifdef AUX1_PIN
      Joystick.setZAxisRotation((aux1+127)*1.411);
      #endif
      */
      //for open.hd (use axis)
      Joystick.setYAxisRotation(sw1*1.411);
      Joystick.setZAxisRotation(sw2*1.411);
      Joystick.setThrottle(sw3);
      #ifdef SW4_PIN
      Joystick.setHatSwitch(1,sw4*1.411);
      #endif
      #ifdef SW5_PIN
      Joystick.setHatSwitch(2,sw5*1.411);
      #endif
      #ifdef AUX1_PIN
      Joystick.setRudder(aux1);
      #endif
      Joystick.sendState();
    #endif

    //ppm
    #ifdef PPM_PIN
      //axis
      ppmEncoder.setChannel(0, 1000+(roll+127)*3.937);
      ppmEncoder.setChannel(1, 1000+(pitch+127)*3.937);
      ppmEncoder.setChannel(2, 1000+throttle*3.922);
      ppmEncoder.setChannel(3, 1000+(yaw+127)*3.937);
      //sw
      ppmEncoder.setChannel(4, 1000+sw1*3.922);
      ppmEncoder.setChannel(5, 1000+sw2*3.922);
      ppmEncoder.setChannel(6, 1000+sw3*3.922);
      #ifdef SW4_PIN
      ppmEncoder.setChannel(7, 1000+sw4*3.922);
      #endif
      #ifdef SW5_PIN
      ppmEncoder.setChannel(8, 1000+sw5*3.922);
      #endif
      //aux
      #ifdef AUX1_PIN
      ppmEncoder.setChannel(9, 1000+aux1*3.922);
      #endif
    #endif
  }
  #ifdef NTIM
  if((prevtimer + NTIM) < millis()) {
      prevtimer = millis();
      if(ch_crc == prev_ch_crc) beep(10);
      prev_ch_crc = ch_crc;
  }
  #endif
}
