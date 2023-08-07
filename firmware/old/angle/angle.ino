/*****************************************************************************/
//  HighLevelExample.ino
//  Hardware:      Grove - 6-Axis Accelerometer&Gyroscope
//	Arduino IDE:   Arduino-1.65
//	Author:	       Lambor
//	Date: 	       Oct,2015
//	Version:       v1.0
//
//  Modified by:
//  Data:
//  Description:
//
//	by www.seeedstudio.com
//
//  This library is free software; you can redistribute it and/or
//  modify it under the terms of the GNU Lesser General Public
//  License as published by the Free Software Foundation; either
//  version 2.1 of the License, or (at your option) any later version.
//
//  This library is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
//  Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public
//  License along with this library; if not, write to the Free Software
//  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
//
/*******************************************************************************/

#include "LSM6DS3.h"
#include "Wire.h"

//Create a instance of class LSM6DS3
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

float AcX, AcY, AcZ, gX, gY, gZ;
int uniAngle, dezAngle;
int axis, angle, angleX, angleY;
int angleRefX = 0;
int angleRefY = 0;


void setup() {
  if (myIMU.begin() != 0) {
    Serial.println("Device error");
  } else {
    Serial.println("Device OK!");
  }
}

void loop() {

  getangle();
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

  angleX = atan(AcX / AcZ) * 180 / PI - angleRefX;
  angleY = atan(AcY / AcZ) * 180 / PI - angleRefY;

  if ((abs(gY) > 5) && (abs(gX) < 2) && (axis == 0)) {
    angle = angleX;
  }

  if ((abs(gX) > 5) && (abs(gY) < 2) && (axis == 0)) {
    angle = angleY;
  }
  uniAngle = abs(int(angle)) % 10;
  dezAngle = abs(int(angle)) / 10;
  angleRefX = angleX;
  angleRefY = angleY;
}
