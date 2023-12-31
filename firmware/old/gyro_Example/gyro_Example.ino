#include <Adafruit_FXAS21002C.h>
#include <Adafruit_FXOS8700.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_FXAS21002C gyro = Adafruit_FXAS21002C(0x0021002C);
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

void displaySensorDetails(void) {
  sensor_t accel, mag, gyro;
  gyro.getSensor(&gyro);
  Serial.print("Sensor:       ");
  Serial.println(gyro.name);
  Serial.print("Driver Ver:   ");
  Serial.println(gyro.version);
  Serial.print("Unique ID:    0x");
  Serial.println(gyro.sensor_id, HEX);
  Serial.print("Max Value:    ");
  Serial.print(gyro.max_value);
  Serial.println(" rad/s");
  Serial.print("Min Value:    ");
  Serial.print(gyro.min_value);
  Serial.println(" rad/s");
  Serial.print("Resolution:   ");
  Serial.print(gyro.resolution);
  Serial.println(" rad/s");
  accelmag.getSensor(&accel, &mag);
  Serial.println("ACCELEROMETER");
  Serial.print("Sensor:       ");
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
  Serial.println("MAGNETOMETER");
  Serial.print("Sensor:       ");
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
  delay(500);
}

void setup(void) {
  Serial.begin(115200);
  while (!Serial);
  if (!gyro.begin()) { //FXAS21002C ... check your connections
    Serial.println(F("Ooops, no FXAS21002C detected ... Check your wiring!"));
  }
  // gyro.setRange(GYRO_RANGE_2000DPS);  /* Set gyro range. (optional, default is 250 dps) */

  if (!accelmag.begin()) { // detecting the FXOS8700 ... check your connections */
    Serial.println(F("Ooops, no FXOS8700 detected ... Check your wiring!"));
  }
  // accelmag.setAccelRange(ACCEL_RANGE_8G);  /* Set accelerometer range (optional, default is 2G) */
  // accelmag.setSensorMode(ACCEL_ONLY_MODE);  /* Set the sensor mode (optional, default is hybrid mode) */
  // accelmag.setMagOversamplingRatio(MAG_OSR_7);  /* Set the magnetometer's oversampling ratio (optional, default is 7) */
  // accelmag.setOutputDataRate(ODR_400HZ);  /* Set the output data rate (optional, default is 100Hz) */
  displaySensorDetails();  /* Display some basic information on this sensor */
}

void loop(void) {
  /* Get a new sensor event */
  sensors_event_t event;
  gyro.getEvent(&event);
  /* Display the results (speed is measured in rad/s) */
  Serial.print("Gyro X: ");
  Serial.print(event.gyro.x);
  Serial.print("  Y: ");
  Serial.print(event.gyro.y);
  Serial.print("  Z: ");
  Serial.print(event.gyro.z);
  Serial.println("  rad/s ");
  sensors_event_t aevent, mevent;  /* Get a new sensor event */
  accelmag.getEvent(&aevent, &mevent); /* Display the accel results (acceleration is measured in m/s^2) */
  Serial.print("Accel X: ");
  Serial.print(aevent.acceleration.x, 4);
  Serial.print("  Y: ");
  Serial.print(aevent.acceleration.y, 4);
  Serial.print("  Z: ");
  Serial.print(aevent.acceleration.z, 4);
  Serial.println("  m/s^2");
  /* Display the mag results (mag data is in uTesla) */
  Serial.print("Mag X: ");
  Serial.print(mevent.magnetic.x, 1);
  Serial.print("  Y: ");
  Serial.print(mevent.magnetic.y, 1);
  Serial.print("  Z: ");
  Serial.print(mevent.magnetic.z, 1);
  Serial.println("  uT");
  delay(500);
}
