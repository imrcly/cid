#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
// libraries
#include "bmm150.h"
#include "bmm150_defs.h"
#include "LSM6DS3.h"


LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A
BMM150 bmm = BMM150();
bmm150_mag_data compass1valueoffset;

U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* clock=*/ 4, /* data=*/ 5, /* reset=*/ U8X8_PIN_NONE);   // All Boards without Reset of the Display

//display
String txtline1 = "";
String txtline2 = "";
String txtline3 = "";
String txtline4 = "";
String txtline5 = "";
String txtline6 = "";
String txtline7 = "";


//compass
float AcX, AcY, AcZ, gX, gY, gZ;
int uniAngle, dezAngle;
int axis, angle, angleX, angleY;
int angleRefX = 0;
int angleRefY = 0;

//pins
int button1 = 1;
// int scl = 4;
// int sda = 5;
// int rx = 6;
// int tx = 7;

//sensors
float compass1;

void u8g2_prepare(void) {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0); // 0 normal; 1 90; 2 180; 3 270
}

void u8g2_string() {
  //separate setCursor(), print() accepts all Print.h arguments
  u8g2.setCursor(0, 0);
  u8g2.print( txtline1 );
  u8g2.setCursor( 0, 9);
  u8g2.print(txtline2 );
  u8g2.setCursor(0, 18);
  u8g2.print(txtline3);
  u8g2.setCursor( 0, 27);
  u8g2.print(txtline4);
  u8g2.setCursor(0, 36);
  u8g2.print(txtline5);
  u8g2.setCursor(0, 45);
  u8g2.print(txtline6);
  u8g2.setCursor( 0, 54);
  u8g2.print(txtline7);
}

void draw(void) {
  u8g2_prepare();
  u8g2_string();
}

void setup()
{
  u8g2.begin();
  pinMode(button1, INPUT_PULLUP);
  if (bmm.initialize() == BMM150_E_ID_NOT_CONFORM) {
    txtline1 = "Compass1 error";
    while (1);
  } else {
    txtline1 = "Compass1 connected";
  }
  if (myIMU.begin() != 0) {
    txtline2 = "LSM6DS3 error";
  } else {
    txtline2 = "LSM6DS3 connected";
  }
}

/**
   @brief Do figure-8 calibration for a limited time to get offset values of x/y/z axis.
   @param timeout - seconds of calibration period.
*/
void bmm150calibrate(uint32_t timeout)
{
  int16_t value_x_min = 0;
  int16_t value_x_max = 0;
  int16_t value_y_min = 0;
  int16_t value_y_max = 0;
  int16_t value_z_min = 0;
  int16_t value_z_max = 0;
  uint32_t timeStart = 0;

  bmm.read_mag_data();
  value_x_min = bmm.raw_mag_data.raw_datax;
  value_x_max = bmm.raw_mag_data.raw_datax;
  value_y_min = bmm.raw_mag_data.raw_datay;
  value_y_max = bmm.raw_mag_data.raw_datay;
  value_z_min = bmm.raw_mag_data.raw_dataz;
  value_z_max = bmm.raw_mag_data.raw_dataz;
  delay(100);

  timeStart = millis();

  while ((millis() - timeStart) < timeout)
  {
    bmm.read_mag_data();

    /* Update x-Axis max/min value */
    if (value_x_min > bmm.raw_mag_data.raw_datax)
    {
      value_x_min = bmm.raw_mag_data.raw_datax;
      // Serial.print("Update value_x_min: ");
      // Serial.println(value_x_min);

    }
    else if (value_x_max < bmm.raw_mag_data.raw_datax)
    {
      value_x_max = bmm.raw_mag_data.raw_datax;
      // Serial.print("update value_x_max: ");
      // Serial.println(value_x_max);
    }

    /* Update y-Axis max/min value */
    if (value_y_min > bmm.raw_mag_data.raw_datay)
    {
      value_y_min = bmm.raw_mag_data.raw_datay;
      // Serial.print("Update value_y_min: ");
      // Serial.println(value_y_min);

    }
    else if (value_y_max < bmm.raw_mag_data.raw_datay)
    {
      value_y_max = bmm.raw_mag_data.raw_datay;
      // Serial.print("update value_y_max: ");
      // Serial.println(value_y_max);
    }

    /* Update z-Axis max/min value */
    if (value_z_min > bmm.raw_mag_data.raw_dataz)
    {
      value_z_min = bmm.raw_mag_data.raw_dataz;
      // Serial.print("Update value_z_min: ");
      // Serial.println(value_z_min);

    }
    else if (value_z_max < bmm.raw_mag_data.raw_dataz)
    {
      value_z_max = bmm.raw_mag_data.raw_dataz;
      // Serial.print("update value_z_max: ");
      // Serial.println(value_z_max);
    }

    delay(100);

  }
  compass1valueoffset.x = value_x_min + (value_x_max - value_x_min) / 2;
  compass1valueoffset.y = value_y_min + (value_y_max - value_y_min) / 2;
  compass1valueoffset.z = value_z_min + (value_z_max - value_z_min) / 2;
}

void loop()
{
  //button1
  if (digitalRead(button1) == LOW ) {
    txtline3 = "Start figure-8 ";
    txtline4 = "calibration after 3s";
    delay(3000);
    bmm150calibrate(10000);
    txtline3 = "Calibrate done..";
    txtline4 = "";
  }
  //end button1

  // screen loop
  u8g2.clearBuffer();
  draw();
  u8g2.sendBuffer();
  //end screen loop

  compass1 = compass1read();
  getangle();
  txtline6 = "Angle: " + String(angle);
  delay(100);
}

float compass1read() {
  bmm150_mag_data value;
  bmm.read_mag_data();

  value.x = bmm.raw_mag_data.raw_datax - compass1valueoffset.x;
  value.y = bmm.raw_mag_data.raw_datay - compass1valueoffset.y;
  value.z = bmm.raw_mag_data.raw_dataz - compass1valueoffset.z;

  float xyHeading = atan2(value.x, value.y);
  float zxHeading = atan2(value.z, value.x);
  float heading = xyHeading;

  if (heading < 0)
    heading += 2 * PI;
  if (heading > 2 * PI)
    heading -= 2 * PI;
  float headingDegrees = heading * 180 / M_PI;
  float xyHeadingDegrees = xyHeading * 180 / M_PI;
  float zxHeadingDegrees = zxHeading * 180 / M_PI;

  txtline7 =  "Heading: " + String(headingDegrees);
}

void getangle() {

  //Accelerometer
  AcX = myIMU.readFloatAccelX();
  AcY = myIMU.readFloatAccelY();
  AcZ = myIMU.readFloatAccelZ();

  //Gyroscope
  gX = myIMU.readFloatGyroX();
  gY = myIMU.readFloatGyroY();
  gZ = myIMU.readFloatGyroZ();

  angleX = atan(AcZ / AcX);
  angleY = atan(AcZ / AcY);

 // if ((abs(gY) > 5) && (abs(gX) < 2) && (axis == 0)) {
    angle = angleX;
 // }

 /* if ((abs(gX) > 5) && (abs(gY) < 2) && (axis == 0)) {
    angle = angleY;
  } */
  uniAngle = abs(int(angle)) % 10;
  dezAngle = abs(int(angle)) / 10;
  //angleRefX = angleX;
  //angleRefY = angleY;
}



/*
  Universal 8bit Graphics Library (https://github.com/olikraus/u8g2/)
  Copyright (c) 2016, olikraus@gmail.com
  All rights reserved.

  Redistribution and use in source and binary forms, with or without modification,
  are permitted provided that the following conditions are met:

    Redistributions of source code must retain the above copyright notice, this list
    of conditions and the following disclaimer.

    Redistributions in binary form must reproduce the above copyright notice, this
    list of conditions and the following disclaimer in the documentation and/or other
    materials provided with the distribution.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
  CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
  STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
  ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
