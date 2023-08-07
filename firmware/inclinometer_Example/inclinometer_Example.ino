/*
   This program is free software: you can redistribute it and/or modify it
   under the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   This program is distributed in the hope that it will be useful, but WITHOUT
   ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
   more details.

   You should have received a copy of the GNU General Public License along
   with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Wire.h>
#include <Adafruit_ADXL345_U.h>
#include <EEPROM.h>

#define LPF_ALPHA (500000) // Low pass filter time constant for filtering acceleration micros 

/** Time interval in microseconds since the of the last loop() call. This is
   used to calculate accurate alpha values for low pass filters.  Note that
   this is a 16bit value, allowing for a maximum of approx 65 milliseconds
   between subsequent loop() calls.
*/
uint16_t delta_time;

float accelerometer_calibration(6) = {1, 2, 3, 4, 5, 6}; // initialize known impossible value

struct orientation_t {
  float xaxis[3];
  float yaxis[3];
  float zaxis[3];
} orientation;

Adafruit_ADXL345_Unified adxl345;  //ADXL345 accelerometer access class

void accelerometer_calibrate() {  // calibrate the accelerometer
  int16_t xUp = -32768;
  int16_t xDown = 32767;
  int16_t yUp = -32768;
  int16_t yDown = 32767;
  int16_t zUp = -32768;
  int16_t zDown = 32767;
  uint32_t calitimer = 0;

  while (docalibration) {
    while (calitimer < millis() + 2000) {
      xUp = max(adxl345.getX(), xUp); // standing on the table (X axis pointing up)
      //beep
    }
    while ((calitimer > millis() + 2000) && (calitimer < millis() + 4000)) {
      xDown = min(adxl345.getX(), xDown); // standing upside-down on the table (X axis pointing down)
      //beep
    }
    while ((calitimer > millis() + 4000) && (calitimer < millis() + 6000)) {
      yUp = max(adxl345.getY(), yUp); // sideways on the table (Y axis pointing up)
      //beep
    }
    while ((calitimer > millis() + 6000) && (calitimer < millis() + 8000)) {
      yDown = min(adxl345.getY(), yDown); // sideways upside-down on the table (Y axis pointing down)
      //beep
    }
    while ((calitimer > millis() + 8000) && (calitimer < millis() + 10000)) {
      zUp = max(adxl345.getZ(), zUp); // flat on the table (Z axis pointing up)
      //beep
    }
    while ((calitimer > millis() + 10000) && (calitimer < millis() + 12000)) {
      zDown = min(adxl345.getZ(), zDown); // upside down on the table (Z axis pointing down)
      //beep
    }
    while (calitimer > millis() + 12000) {
      docalibration = false;
      //beepbeep
    }
  }

  accelerometer_calibration(0) = (2 / (xUp - xDown));  // slope = 2 / (max_reading - min_reading)
  accelerometer_calibration(1) = (1 - (accelerometer_calibration(0) * xDown)); // intercept = 1 - slope * min_reading
  accelerometer_calibration(2) = (2 / (yUp - yDown));
  accelerometer_calibration(3) = (1 - (accelerometer_calibration(2) * yDown));
  accelerometer_calibration(4) = (2 / (zUp - zDown));
  accelerometer_calibration(5) = (1 - (accelerometer_calibration(4) * zDown));

} // end accelerometer_calibrate

/** vector math 
 RadAnglex = acos(Ax / sqrt( pow(Ax,2) + pow(Ay,2) + pow(Az,2)) )
 Ax = 9.8 * sin(xslope)
*/

float rad2deg(float angle) {//Convert angle from radians to degrees.
  return 180.0 * (angle / 3.1415926);
} // end rad2deg

float vlen(float vec[3]) { // sqrt of x^2 + y^2 + z^2
  return sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
} // end vlen

float* vnormalize(float vec[3], float vec_out[3]) { //give it an axis and it divides it by the vlen
  float m = vlen(vec);
  vec_out[0] = vec[0] / m;
  vec_out[1] = vec[1] / m;
  vec_out[2] = vec[2] / m;
  return vec_out;
} // end vnormalize

float vdot(float vec_a[3], float vec_b[3]) {
  return vec_a[0] * vec_b[0] + vec_a[1] * vec_b[1] + vec_a[2] * vec_b[2];
} // end vdot

void vprint(float xyzPrint[3]) { //SerialUSB output the x,y,z values
  SerialUSB.print(F("["));
  SerialUSB.print(xyzPrint[0]);
  SerialUSB.print(F(", "));
  SerialUSB.print(xyzPrint[1]);
  SerialUSB.print(F(", "));
  SerialUSB.print(xyzPrint[2]);
  SerialUSB.println(F("]"));
} // end vprint

void read_accelerometer(float output[3]) { // Read the accelerometer X, Y and Z values and store them in 'output'. calibrated and filtered.
  // The filtered accelerometer values.  This is 'static', so it remembers
  // its values between `read_accelerometer' calls.
  static float filter[3];

  // Step 1: read the raw values from the accelerometer
  output[0] = adxl345.getX();
  output[1] = adxl345.getY();
  output[2] = adxl345.getZ();

  // Step 2: calibrate the values
  output[0] = output[0] * accelerometer_calibration(0) + accelerometer_calibration(1);
  output[1] = output[1] * accelerometer_calibration(2) + accelerometer_calibration(3);
  output[2] = output[2] * accelerometer_calibration(4) + accelerometer_calibration(5);

  // Step 3: calculate the filter alpha value and update the filter.
  float alpha = float(delta_time) / (LPF_ALPHA + float(delta_time));
  filter[0] = filter[0] * (1 - alpha) + output[0] * alpha;
  filter[1] = filter[1] * (1 - alpha) + output[1] * alpha;
  filter[2] = filter[2] * (1 - alpha) + output[2] * alpha;

  // Step 4: produce the final calibrated and filtered values.
  output[0] = filter[0];
  output[1] = filter[1];
  output[2] = filter[2];
} // end read_accelerometer

float calculate_pitch(float cal[3]) {
  float down[3] = {0, 0, 1};
  float pitch_dir[3] = { cal[0], 0, cal[2] };
  vnormalize(pitch_dir, pitch_dir);
  float pitch = vdot(down, pitch_dir);
  float angle = rad2deg(acos(pitch));
  if (pitch_dir[0] > 0)
    return angle;                   // up
  else
    return -angle;                  // down
} // end calculate_pitch

void on_running(float cal[3]) {
  // 'cal' is in world coordinates, transform it to local coordinates, to
  // calculate the calibrated roll and pitch.  Note that the vdot() calls
  // together make a matrix -- vector multiplication.
  float ncal[3];
  ncal[0] = vdot(orientation.xaxis, cal);
  ncal[1] = vdot(orientation.yaxis, cal);
  ncal[2] = vdot(orientation.zaxis, cal);

  float pitch = calculate_pitch(ncal);
  float gforce = vlen(cal);

  if (gforce < 1.01 ) {
    if (abs(pitch) > max_pitch) {
      max_pitch = abs(pitch);
    }
  }
} // end on_running

void update_timer()
{
  uint32_t now = micros();

  // Calculate `delta_time`, taking care of roll overs which happen
  // approximately every 70 minutes
  if (now < last_loop_start) { // roll over
    delta_time = (0xFFFFFFFF - last_loop_start) + now;
  } else {
    delta_time = now - last_loop_start;
  }
  last_loop_start = now;
} // end update_timer

void setup() {
  SerialUSB.begin(115200);
  adxl345.begin();
  adxl345.setRange(ADXL345_RANGE_4_G);
  adxl345.setDataRate(ADXL345_DATARATE_50_HZ);
} // end setup

void loop() {
  update_timer()
  float acceleration[3];
  read_accelerometer(acceleration);
  on_running(acceleration);
} // end loop
