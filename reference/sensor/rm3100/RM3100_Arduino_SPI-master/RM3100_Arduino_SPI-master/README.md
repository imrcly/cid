# RM3100_Arduino_SPI
A Simple Arduino Library for RM3100 using SPI communication

```
# The connection to Teensy 3.2
RM3100 --> Teensy

MOSI   --> Pin 11
MISO   --> Pin 12
SSN    --> GPIO PIN(10 for example)
DRDY   --> GPIO PIN(2 for example)
VCC    --> 3.3V
GND    --> GND
SCK    --> Pin 13

```

![alt text](https://github.com/Ahmed-Dakrory/RM3100_Arduino_SPI/blob/master/teensy.png)


# Example

```
#include <RM3100.h>

RM3100 rm3100(10,2);  // 10 is CS, 2 is DRDY
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
rm3100.begin();
delay(50);
}

void loop() {
  // put your main code here, to run repeatedly:
//rm3100.readMag();
float head = rm3100.readHeading();
if(head!=NULL)
  Serial.println(head);

}

```


