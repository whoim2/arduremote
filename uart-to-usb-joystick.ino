#include <JoystickMHX.h>

//#define SERIAL_DEBUG
#define JOY_UPDATE_RATE 15 //ms

//Joystic settings
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,
  32, 2,                  // Button Count, Hat Switch Count
  true, true, true,     // X and Y, Z Axis
  true, true, true,   // Rx, Ry,  Rz
  true, true, true, true,  // slider, dial, rudder and throttle
  false, false, false);  // accelerator, brake, and steering

byte data[18];
uint32_t joyTime = 0;
uint8_t pos=0;

void setup() {
  #ifdef SERIAL_DEBUG
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("Started");
  #else
  Joystick.begin();
  //range
  Joystick.setXAxisRange(-127, 127);
  Joystick.setYAxisRange(-127, 127);
  Joystick.setZAxisRange(-127, 127);
  Joystick.setRxAxisRange(-127, 127);
  Joystick.setRyAxisRange(-127, 127);
  Joystick.setRzAxisRange(-127, 127);
  Joystick.setSliderRange(0, 255);
  Joystick.setDialRange(0, 255);
  Joystick.setRudderRange(0, 255);
  Joystick.setThrottleRange(0, 255);
  //default
  Joystick.setRudder(127);
  Joystick.setAccelerator(127);
  Joystick.setBrake(127);
  Joystick.setSteering(127);
  #endif
  Serial1.begin(38400); //jdy-41 baud
  while (!Serial1) {}
  Serial.println("uart started");
}

void loop() {
  if (Serial1.available()) {
   data[pos] = Serial1.read();
   pos++;
   if(pos > 17) pos=0;

   //sync
   if(pos > 1) {
    if((uint8_t)data[pos-2] == 0x15 && (uint8_t)data[pos-1] == 0x37 && (uint8_t)data[pos] == 0xFA) { pos=2; }
   }
  }
  if (millis() > joyTime) {
   #ifdef SERIAL_DEBUG
   for (uint8_t i = 0; i<18; i++) {
     Serial.print(i);
     Serial.print(":");
     if(i==3 || i==4 || i==6) Serial.print((int8_t)data[i]);
     else Serial.print((uint8_t)data[i]);
     Serial.print("\t");
   }
   Serial.println();
   #else
   Joystick.setXAxis    ((int8_t)data[3]);
   Joystick.setYAxis    ((int8_t)data[4]);
   Joystick.setZAxis    ((uint8_t)data[5]-127);
   Joystick.setRxAxis   ((int8_t)data[6]);
   Joystick.setRyAxis   ((uint8_t)data[7]);
   Joystick.setRzAxis   ((uint8_t)data[8]);
   Joystick.setSlider   ((uint8_t)data[9]);
   Joystick.setRudder   ((uint8_t)data[10]);
   Joystick.setThrottle ((uint8_t)data[11]);
   Joystick.setDial     ((uint8_t)data[12]);
   Joystick.sendState();
   joyTime = millis() + JOY_UPDATE_RATE;
   #endif
  }
}
