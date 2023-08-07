/*****************************************************************************/
//  LowLevelExample.ino
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
#include "SPI.h"

uint16_t errorsAndWarnings = 0;

//Create instance of LSM6DS3Core
LSM6DS3Core myIMUcore(I2C_MODE, 0x6A);    //I2C device address 0x6A
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

void setup() {
  //Init Serial port
  SerialUSB.begin(9600);
  while (!SerialUSB);

  //Call .beginCore() to configure the IMU
  if (myIMUcore.beginCore() != 0) {
    SerialUSB.println("Device Error.");
  } else {
    SerialUSB.println("Device OK.");
  }
  if (myIMU.begin() != 0) {
    SerialUSB.println("Device error");
  } else {
    SerialUSB.println("Device OK!");
  }
  uint8_t dataToWrite = 0;  //Temporary variable

  //Setup the accelerometer******************************
  dataToWrite = 0; //Start Fresh!
  dataToWrite |= LSM6DS3_ACC_GYRO_BW_XL_100Hz;
  dataToWrite |= LSM6DS3_ACC_GYRO_FS_XL_8g;
  dataToWrite |= LSM6DS3_ACC_GYRO_ODR_XL_104Hz;

  //Now, write the patched together data
  errorsAndWarnings += myIMUcore.writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

  //Set the ODR bit
  errorsAndWarnings += myIMUcore.readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
  dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);

}

void loop() {
  int16_t temp;
  //Get all parameters
  SerialUSB.print("\nAccelerometer Counts:\n");

  //Acelerometer axis X
  if (myIMUcore.readRegisterInt16(&temp, LSM6DS3_ACC_GYRO_OUTX_L_XL) != 0) {
    errorsAndWarnings++;
  }
  SerialUSB.print(" X = ");
  SerialUSB.println(temp);

  //Acelerometer axis Y
  if (myIMUcore.readRegisterInt16(&temp, LSM6DS3_ACC_GYRO_OUTY_L_XL) != 0) {
    errorsAndWarnings++;
  }
  SerialUSB.print(" Y = ");
  SerialUSB.println(temp);

  //Acelerometer axis Z
  if (myIMUcore.readRegisterInt16(&temp, LSM6DS3_ACC_GYRO_OUTZ_L_XL) != 0) {
    errorsAndWarnings++;
  }
  SerialUSB.print(" Z = ");
  SerialUSB.println(temp);

  SerialUSB.println();
  SerialUSB.print("Total reported Errors and Warnings: ");
  SerialUSB.println(errorsAndWarnings);

  //Accelerometer
  SerialUSB.print("\nAccelerometer:\n");
  SerialUSB.print(" X1 = ");
  SerialUSB.println(myIMU.readFloatAccelX(), 4);
  SerialUSB.print(" Y1 = ");
  SerialUSB.println(myIMU.readFloatAccelY(), 4);
  SerialUSB.print(" Z1 = ");
  SerialUSB.println(myIMU.readFloatAccelZ(), 4);

  //Gyroscope
  SerialUSB.print("\nGyroscope:\n");
  SerialUSB.print(" X1 = ");
  SerialUSB.println(myIMU.readFloatGyroX(), 4);
  SerialUSB.print(" Y1 = ");
  SerialUSB.println(myIMU.readFloatGyroY(), 4);
  SerialUSB.print(" Z1 = ");
  SerialUSB.println(myIMU.readFloatGyroZ(), 4);

  //Thermometer
  SerialUSB.print("\nThermometer:\n");
  SerialUSB.print(" Degrees C1 = ");
  SerialUSB.println(myIMU.readTempC(), 4);
  SerialUSB.print(" Degrees F1 = ");
  SerialUSB.println(myIMU.readTempF(), 4);

  delay(1000);
}
