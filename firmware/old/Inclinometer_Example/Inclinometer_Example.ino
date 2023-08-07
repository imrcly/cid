#include <Adafruit_SSD1306.h>


#include "MPU9250.h"
#include "Math.h"

#define OLED_RESET 4
Adafruit_SSD1306 display(OLED_RESET);


// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire,0x68);
int status;
double ax,ay,az;
double angle;

void setup() {
  // serial to display data
  Serial.begin(115200);
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextSize(2);
  display.setTextColor(WHITE);
  
  
  while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  // setting the accelerometer full scale range to +/-2G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_4G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(19);
}

void loop() {
  // read the sensor
  IMU.readSensor();

  // display the data
  ax=(map(IMU.getAccelX_mss()*10000,-17113,-114648,0,-981)/100.0);
  ay=(map(IMU.getAccelY_mss()*10000,-21107,-122134,0,-981)/100.0);
  az=(map(IMU.getAccelZ_mss()*10000,4750,-90780,0,-981)/100.0);
  ax=constrain(ax,-9.81,9.81);
  angle=(asin((float)(ax/9.81))*180/3.141);
  Serial.println(angle);

  display.clearDisplay();
  display.setCursor(0,0);
  display.println(angle);
  display.println("deg");
  display.display();

  delay(300);
  
  
  
}
