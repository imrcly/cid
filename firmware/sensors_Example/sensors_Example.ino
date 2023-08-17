/*
   You should have received a copy of the GNU General Public License along
   with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Wire.h>
#include <Adafruit_FXOS8700.h>

#define LPF_ALPHA (500000) // Low pass filter time constant for filtering acceleration micros 
uint16_t delta_time;
uint32_t last_loop_start = 0;
float accelerometer_calibration[6] = {0};

Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

void accelerometer_calibrate() {  // calibrate the accelerometer
  sensors_event_t aevent, mevent;
  accelmag.getEvent(&aevent, &mevent);

  int16_t xUp = -32768;
  int16_t xDown = 32767;
  int16_t yUp = -32768;
  int16_t yDown = 32767;
  int16_t zUp = -32768;
  int16_t zDown = 32767;
  uint32_t calitimer = 0;
  bool docalibration = true;
  while (docalibration) {
    while (calitimer < millis() + 2000) {
      xUp = max(aevent.acceleration.x, xUp); // standing on the table (X axis pointing up)
      //beep
    }
    while ((calitimer > millis() + 2000) && (calitimer < millis() + 4000)) {
      xDown = min(aevent.acceleration.x, xDown); // standing upside-down on the table (X axis pointing down)
      //beep
    }
    while ((calitimer > millis() + 4000) && (calitimer < millis() + 6000)) {
      yUp = max(aevent.acceleration.y, yUp); // sideways on the table (Y axis pointing up)
      //beep
    }
    while ((calitimer > millis() + 6000) && (calitimer < millis() + 8000)) {
      yDown = min(aevent.acceleration.y, yDown); // sideways upside-down on the table (Y axis pointing down)
      //beep
    }
    while ((calitimer > millis() + 8000) && (calitimer < millis() + 10000)) {
      zUp = max(aevent.acceleration.z, zUp); // flat on the table (Z axis pointing up)
      //beep
    }
    while ((calitimer > millis() + 10000) && (calitimer < millis() + 12000)) {
      zDown = min(aevent.acceleration.z, zDown); // upside down on the table (Z axis pointing down)
      //beep
    }
    while (calitimer > millis() + 12000) {
      docalibration = false;
      //beepbeep
    }
  }

  accelerometer_calibration[0] = (2 / (xUp - xDown));  // slope = 2 / (max_reading - min_reading)
  accelerometer_calibration[1] = (1 - (accelerometer_calibration[0] * xDown)); // intercept = 1 - slope * min_reading
  accelerometer_calibration[2] = (2 / (yUp - yDown));
  accelerometer_calibration[3] = (1 - (accelerometer_calibration[2] * yDown));
  accelerometer_calibration[4] = (2 / (zUp - zDown));
  accelerometer_calibration[5] = (1 - (accelerometer_calibration[4] * zDown));

} // end accelerometer_calibrate

/** vector math
  RadAnglex = acos(Ax / sqrt( pow(Ax,2) + pow(Ay,2) + pow(Az,2)) )
  Ax = 9.8 * sin(xslope)
*/

float rad2deg(float angle) {//Convert angle from radians to degrees.
  return 180.0 * (angle / 3.1415926);
} // end rad2deg

float gravityVector(float value[3]) {
  float stuff = value[0] * value[0] + value[1] * value[1] + value[2] * value[2];
  Serial.print(stuff);
  float devisor = sqrt(stuff);
  Serial.print(" ");
  Serial.print(devisor);
  Serial.print(" ");
  float answer=devisor / value[0]; //x=0 y=1 z=2
  Serial.println(answer);
  return answer;
}

void read_accelerometer(float output[3]) { // Read the accelerometer X, Y and Z values and store them in 'output'. calibrated and filtered.
  // The filtered accelerometer values.  This is 'static', so it remembers
  // its values between `read_accelerometer' calls.
  static float filter[3];
  sensors_event_t aevent, mevent;
  accelmag.getEvent(&aevent, &mevent);

  // Step 1: read the raw values from the accelerometer
  output[0] = aevent.acceleration.x;
  output[1] = aevent.acceleration.y;
  output[2] = aevent.acceleration.z;

  Serial.print("RawA ");
  Serial.print(output[0]);
  Serial.print(" ");
  Serial.print(output[1]);
  Serial.print(" ");
  Serial.println(output[2]);
/*
  // Step 2: calibrate the values
  output[0] = output[0] * accelerometer_calibration[0] + accelerometer_calibration[1];
  output[1] = output[1] * accelerometer_calibration[2] + accelerometer_calibration[3];
  output[2] = output[2] * accelerometer_calibration[4] + accelerometer_calibration[5];

  // Step 3: calculate the filter alpha value and update the filter.
  float alpha = float(delta_time) / (LPF_ALPHA + float(delta_time));
  filter[0] = filter[0] * (1 - alpha) + output[0] * alpha;
  filter[1] = filter[1] * (1 - alpha) + output[1] * alpha;
  filter[2] = filter[2] * (1 - alpha) + output[2] * alpha;

  // Step 4: produce the final calibrated and filtered values.
  output[0] = filter[0];
  output[1] = filter[1];
  output[2] = filter[2];
  */
} // end read_accelerometer

float calculate_incline(float axcellvalue[3]) {
  float angle = gravityVector(axcellvalue);
  Serial.print(F("Arads: "));
  Serial.print(acos(angle));
  Serial.print(F(" Adeg: "));
  Serial.println(rad2deg(angle));
  return angle;
} // end calculate_incline

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

void magnetic() {
  sensors_event_t aevent, mevent;
  accelmag.getEvent(&aevent, &mevent);

  /* Display the mag results (mag data is in uTesla) */
  Serial.print("Mag X: ");
  Serial.print(mevent.magnetic.x, 1);
  Serial.print(" Y: ");
  Serial.print(mevent.magnetic.y, 1);
  Serial.print(" Z: ");
  Serial.print(mevent.magnetic.z, 1);
  Serial.println(" uT");
}
void FXOS8700Details(void) {
  /* Initialise the sensor */
  if (!accelmag.begin()) {//FXOS8700
    Serial.println("Ooops, no FXOS8700 detected ... Check your wiring!");
  } else Serial.println("FXOS8700 works");

  // accelmag.setAccelRange(ACCEL_RANGE_8G);//(optional, default is 2G)
  // accelmag.setSensorMode(ACCEL_ONLY_MODE);//(optional, default is hybrid mode)
  // accelmag.setMagOversamplingRatio(MAG_OSR_7);//(optional, default is 7)
  // accelmag.setOutputDataRate(ODR_400HZ); //(optional, default is 100Hz)

  sensor_t accel, mag;
  accelmag.getSensor(&accel, &mag);

  Serial.print("ACCELEROMETER Sensor:       ");
  Serial.println(accel.name);
  Serial.print("Driver Ver:   ");
  Serial.println(accel.version);
  Serial.print("Unique ID:    0x");
  Serial.println(accel.sensor_id, HEX);
  Serial.print("Min Delay:    ");
  Serial.print(accel.min_delay);
  Serial.println(" s");
  Serial.print("Max Value:    ");
  Serial.print(accel.max_value, 4);
  Serial.println(" m/s^2");
  Serial.print("Min Value:    ");
  Serial.print(accel.min_value, 4);
  Serial.println(" m/s^2");
  Serial.print("Resolution:   ");
  Serial.print(accel.resolution, 8);
  Serial.println(" m/s^2");
  Serial.print("MAGNETOMETER Sensor:       ");
  Serial.println(mag.name);
  Serial.print("Driver Ver:   ");
  Serial.println(mag.version);
  Serial.print("Unique ID:    0x");
  Serial.println(mag.sensor_id, HEX);
  Serial.print("Min Delay:    ");
  Serial.print(accel.min_delay);
  Serial.println(" s");
  Serial.print("Max Value:    ");
  Serial.print(mag.max_value);
  Serial.println(" uT");
  Serial.print("Min Value:    ");
  Serial.print(mag.min_value);
  Serial.println(" uT");
  Serial.print("Resolution:   ");
  Serial.print(mag.resolution);
  Serial.println(" uT");

}

void setup() {
  delay(1000);
  Serial.begin(115200);
  while (!Serial);

  FXOS8700Details(); //initialize sensor
} // end setup

void loop() {
  float acceleration[3];
  update_timer();
  read_accelerometer(acceleration);
  Serial.print("RawRA ");
  Serial.print(acceleration[0]);
  Serial.print(" ");
  Serial.print(acceleration[1]);
  Serial.print(" ");
  Serial.println(acceleration[2]);
  calculate_incline(acceleration);
  //magnetic();
  delay(3000);
} // end loop
