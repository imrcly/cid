#include "Adafruit_TinyUSB.h"

#define LASERDEBUG

boolean laserNewData = false; //is there a new message
int16_t laserErrorNumber = 0;
char laserData[32] = {0};
float laserTemperature = 0;
float laserVoltage = 0;
float distance = 0;
bool laserpointer = false;

String laserError() {
  String errorText = "";
  switch (laserErrorNumber) {
    case 1: // :Er01!  VBAT too low, should be >=2.0V
#ifdef LASERDEBUG
      Serial.println(F("Laser: BAT too low"));
#endif
      break;
    case 2: // :Er02!  Internal error, don't care
#ifdef LASERDEBUG
      Serial.println(F("Laser: Hardware error"));
#endif
      break;
    case 3: // :Er03!  Module Temperature is too low(<-20c)
#ifdef LASERDEBUG
      Serial.println(F("Laser: Temperature low"));
#endif
      break;
    case 4: // :Er04!  Module Temperature is too high (>40c)
#ifdef LASERDEBUG
      Serial.println(F("Laser: Temperature high"));
#endif
      break;
    case 5: // :Er05!  Target out of measure range
#ifdef LASERDEBUG
      Serial.println(F("Laser: out of range"));
#endif
      break;
    case 6: // :Er06!  Invalid measure result
#ifdef LASERDEBUG
      Serial.println(F("Laser: Invalid measure result"));
#endif
      break;
    case 7: // :Er07!  Background light is too strong
#ifdef LASERDEBUG
      Serial.println(F("Laser: Background light is too strong"));
#endif
      break;
    case 8: // :Er08!  Laser signal is too weak
#ifdef LASERDEBUG
      Serial.println(F("Laser: signal is too weak"));
#endif
      break;
    case 9: // :Er09!  Laser signal is too strong
#ifdef LASERDEBUG
      Serial.println(F("Laser: signal is too strong"));
#endif
      break;
    case 10: // :Er10!  Hardware Fault 1
#ifdef LASERDEBUG
      Serial.println(F("Laser: Hardware error"));
#endif
      break;
    case 11: // :Er11!  Hardware fault 2
#ifdef LASERDEBUG
      Serial.println(F("Laser: Hardware error"));
#endif
      break;
    case 12: // :Er12!  Hardware fault 3
#ifdef LASERDEBUG
      Serial.println(F("Laser: Hardware error"));
#endif
      break;
    case 13: // :Er13!  Hardware fault 4
#ifdef LASERDEBUG
      Serial.println(F("Laser: Hardware error"));
#endif
      break;
    case 14: // :Er14!  Hardware fault 5
#ifdef LASERDEBUG
      Serial.println(F("Laser: Hardware error"));
#endif
      break;
    case 15: // :Er15!  Laser signal is not stable
#ifdef LASERDEBUG
      Serial.println(F("Laser: beam is not stable"));
#endif
      break;
    case 16: // :Er16!  Hardware fault 6
#ifdef LASERDEBUG
      Serial.println(F("Laser: Hardware errore"));
#endif
      break;
    case 17: // :Er17!  Hardware fault 7
#ifdef LASERDEBUG
      Serial.println(F("Laser: Hardware error"));
#endif
      break;
    default: // no error
#ifdef LASERDEBUG
      Serial.println(F("Laser: No Error"));
#endif
      break;
  }
  return (errorText);
} // end laserError

void controlLaser(int laserState) { //sends the control commands to the laser.
  switch (laserState) {
    case 1: //O open - turn on laser   O,OK!
      Serial1.println(F("O"));
      break;
    case 2: //C close - turn off laser  C,OK!
      Serial1.println(F("C"));
      break;
    case 3: // F fast   F: 3.308m,1385
      Serial1.println(F("F"));
      break;
    case 4: // D slow Automatic   D: 3.308m,1385
      Serial1.println(F("D"));
      break;
    case 5: // M slow   M: 3.308m,1385
      Serial1.println(F("M"));
      break;
    case 6: // S temperature and power  S: 26.0'C,3.2V
      Serial1.println(F("S"));
      break;
    case 7: // V module version and serial    V:11047400032,47843
      Serial1.println(F("V"));
      break;
    case 8: // X close the module and pull PWR_ON pin
      Serial1.println(F("X"));
      break;
    default: // do nothing
      break;
  }
} //end controlLaser

void parselaserserial() { //listens to the laser serial port for data
  boolean laserNewData = false;
  if (Serial1.available() > 0 ) { //Determine whether there is data to read on the serial
    Serial1.readBytesUntil('\n', laserData, 32);
    laserNewData = true;
  }
  if (laserNewData) {
    for (int i = 0; i < 32; i++) {
#ifdef LASERDEBUG
      Serial.print(laserData[i]);
#endif
    }
#ifdef LASERDEBUG
    Serial.println();
#endif
  }
}

void parselaserdata() { //makes the laser data usable.
  if (String(laserData[2]) == "E")  { // get the error number if something goes wrong
    char tempdata[2] = {laserData[4], laserData[5]};
    laserErrorNumber = atoi(tempdata);
#ifdef LASERDEBUG
    Serial.print(F("laserErrorNumber = "));
    Serial.println(laserErrorNumber, 3);
#endif
  }
  if (String(laserData[8]) == "C") { // Get the temperature and voltage of the laser module

    char tempdata[5] = {laserData[3], laserData[4], laserData[5], laserData[6], laserData[7]};
    laserTemperature = atof(tempdata);

    char tempdata2[3] = {laserData[10], laserData[11], laserData[12]};
    laserVoltage = atof(tempdata2);
#ifdef LASERDEBUG
    Serial.print(F("laserTemperature = "));
    Serial.print(laserTemperature, 3);
    Serial.println(F(" C"));
    Serial.print(F("laserVoltage = "));
    Serial.print(laserVoltage, 3);
    Serial.println(F(" V"));
#endif
  }
  if (String(laserData[8]) == "m") {//fast read and auto read
    char tempdata[5] = {laserData[3], laserData[4], laserData[5], laserData[6], laserData[7]};
    distance = atof(tempdata);
#ifdef LASERDEBUG
    Serial.print(F("Distance F or D= "));
    Serial.print(distance, 3);
    Serial.println(F(" M"));
#endif
  }
  if (String(laserData[7]) == "m") { //slow read distance most accurate
    char tempdata[5] = {laserData[2], laserData[3], laserData[4], laserData[5], laserData[6]};
    distance = atof(tempdata);
#ifdef LASERDEBUG
    Serial.print(F("Distance M= "));
    Serial.print(distance, 3);
    Serial.println(F(" M"));
#endif
  }
  if (String(laserData[0]) == "O")  { // The laser turned off
#ifdef LASERDEBUG
    Serial.println(F("Laser On"));
#endif
    laserpointer = true;
  }
  if (String(laserData[0]) == "C")  { // The laser turned on
#ifdef LASERDEBUG
    Serial.println(F("Laser Off"));
#endif
    laserpointer = false;
  }
  for (int i = 0; i < 32; i++) {
    laserData[i] = 0;
  }
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  delay(1000);
  Serial.begin(115200); //usb port on the NRF52
  while (!Serial);
  Serial1.begin(19200); //start laser serial
  while (!Serial1);

  for (int i = 0; i < 3; i++) { //Blink the LED 3 times to indicate program started
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
#ifdef LASERDEBUG
  Serial.println(F("tada"));
#endif
} // end setup

void loop() {
  if (Serial.available() > 0) { //allows commands to be sent from console
    Serial1.write(Serial.read());
  }
  parselaserserial();
  parselaserdata();
  laserError();
  delay(500);
} // end loop
