#include <Arduino.h>
#include <Wire.h>
// libraries
#include "bmm150.h"
#include "bmm150_defs.h"

#define BMMDEBUG
BMM150 bmm = BMM150();
bmm150_mag_data value_offset;

void bmm150calibrate(uint32_t timeout) {
  /**
     @brief Do figure-8 calibration for a limited time to get offset values of x/y/z axis.
     @param timeout - seconds of calibration period.
  */

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
#ifdef BMMDEBUG
      SerialUSB.print(F("Update value_x_min: "));
      SerialUSB.println(value_x_min);
#endif
    }
    else if (value_x_max < bmm.raw_mag_data.raw_datax)
    {
      value_x_max = bmm.raw_mag_data.raw_datax;
#ifdef BMMDEBUG
      SerialUSB.print(F("update value_x_max: "));
      SerialUSB.println(value_x_max);
#endif
    }

    /* Update y-Axis max/min value */
    if (value_y_min > bmm.raw_mag_data.raw_datay)
    {
      value_y_min = bmm.raw_mag_data.raw_datay;
#ifdef BMMDEBUG
      SerialUSB.print(F("Update value_y_min: "));
      SerialUSB.println(value_y_min);
#endif
    }
    else if (value_y_max < bmm.raw_mag_data.raw_datay)
    {
      value_y_max = bmm.raw_mag_data.raw_datay;
#ifdef BMMDEBUG
      SerialUSB.print(F("update value_y_max: "));
      SerialUSB.println(value_y_max);
#endif
    }

    /* Update z-Axis max/min value */
    if (value_z_min > bmm.raw_mag_data.raw_dataz)
    {
      value_z_min = bmm.raw_mag_data.raw_dataz;
#ifdef BMMDEBUG
      SerialUSB.print(F("Update value_z_min: "));
      SerialUSB.println(value_z_min);
#endif
    }
    else if (value_z_max < bmm.raw_mag_data.raw_dataz)
    {
      value_z_max = bmm.raw_mag_data.raw_dataz;
#ifdef BMMDEBUG
      SerialUSB.print(F("update value_z_max: "));
      SerialUSB.println(value_z_max);
#endif
    }
  }

  value_offset.x = value_x_min + (value_x_max - value_x_min) / 2;
  value_offset.y = value_y_min + (value_y_max - value_y_min) / 2;
  value_offset.z = value_z_min + (value_z_max - value_z_min) / 2;
#ifdef BMMDEBUG
  SerialUSB.print(F("\n\rCalibrate done.."));
#endif
}

float heading() {
  /**
    @brief Do figure-8 calibration for a limited time to get offset values of x/y/z axis.
    @param none
  */
  bmm150_mag_data value;
  bmm.read_mag_data();

  value.x = bmm.raw_mag_data.raw_datax - value_offset.x;
  value.y = bmm.raw_mag_data.raw_datay - value_offset.y;
  value.z = bmm.raw_mag_data.raw_dataz - value_offset.z;

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
#ifdef BMMDEBUG
  SerialUSB.print(F("Heading: "));
  SerialUSB.println(headingDegrees);
#endif
  return (headingDegrees);

}

void setup() {
  SerialUSB.begin(9600);

  if (bmm.initialize() == BMM150_E_ID_NOT_CONFORM) {
    SerialUSB.println(F("BMM150 ID can not read!"));
    while (1);
  } else {
    SerialUSB.println(F("Initialize done!"));
  }

  SerialUSB.println(F("Start figure-8 calibration after 3 seconds."));
  delay(3000);
  bmm150calibrate(10000);
}

void loop() {
  heading();
  delay(4000);
}
