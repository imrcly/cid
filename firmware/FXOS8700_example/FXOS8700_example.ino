#include <Adafruit_FXOS8700.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_FXOS8700 accelmag = Adafruit_FXOS8700(0x8700A, 0x8700B);

void accelereration() {
  sensors_event_t aevent, mevent;
  accelmag.getEvent(&aevent, &mevent);

  /* Display the accel results (acceleration is measured in m/s^2) */
  Serial.print("Accel X: ");
  Serial.print(aevent.acceleration.x);
  Serial.print(" Y: ");
  Serial.print(aevent.acceleration.y);
  Serial.print(" Z: ");
  Serial.print(aevent.acceleration.z);
  Serial.println(" m/s^2");
}

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

void setup(void) {
  delay(1000);
  Serial.begin(115200);
  while (!Serial);  /* Wait for the Serial Monitor */

  FXOS8700Details();
}

void loop() {
  accelereration();
  magnetic();
  delay(500);
}
