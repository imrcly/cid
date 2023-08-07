#include <RM3100.h>

RM3100 rm3100(10,9);  // 10 is CS, 9 is DRDY
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
