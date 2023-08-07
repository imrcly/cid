/**
* @file         RM3100.cpp
*
* @brief        Sample interface for RM3100.
*
* @authors      A7med Dakrory
* @date         04/13/2020
*
*/
#if (ARDUINO >= 100)
#include <Arduino.h>
#else
#include <WProgram.h>
#endif


#include "RM3100.h"

void RM3100::begin(){



    
  SPI.begin();
  SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  pinMode(CS, OUTPUT); 
  pinMode(DRDY,INPUT); 
    //Init to default
    for (int i = 0; i < 3; i++)
    {
        current_ccount[i] = DEFAULTCCOUNT;
        current_gain[i] = DEFAULTGAIN;
    }




           digitalWrite(CS,HIGH);
		   
		   delay(30);

 digitalWrite(CS,LOW);
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    SPI.transfer(0x00);
    digitalWrite(CS,HIGH); 
	
delay(30);

 digitalWrite(CS,LOW);
    SPI.transfer(0x04);
    SPI.transfer(0x01);
    SPI.transfer(0x90);
    SPI.transfer(0x01);
    SPI.transfer(0x90);
    SPI.transfer(0x01);
    SPI.transfer(0x90);
    digitalWrite(CS,HIGH); 
     


delay(50);

/*
digitalWrite(CS,HIGH);
  digitalWrite(CS,LOW);
    SPI.transfer(0x01);
    SPI.transfer(0x04);
    digitalWrite(CS,HIGH);
	*/
   
digitalWrite(CS,HIGH);
  digitalWrite(CS,LOW);
    SPI.transfer(0x01);
    SPI.transfer(CMM);
    digitalWrite(CS,HIGH);
   
 // TMRC
 digitalWrite(CS,LOW);
    //set sample rate
    SPI.transfer(RM3100_TMRC_REG);
    SPI.transfer(0x96); //about 1Hz
    digitalWrite(CS,HIGH);

delay(20);
    digitalWrite(CS,LOW);
    SPI.transfer(0x8B);
    int data = SPI.transfer(0);
    digitalWrite(CS,HIGH);
    Serial.print("Data TMRC: ");
    Serial.println(data);
}


int RM3100::readMag(float * MagValues){
	

	
    digitalWrite(CS,LOW);
    SPI.transfer(RM3100_POLL_REG);
    SPI.transfer(0b01110000);
    digitalWrite(CS,HIGH);
	
    state_drdy =digitalRead(DRDY);


 
if(state_drdy){
    digitalWrite(CS,LOW);
    SPI.transfer(0x24);
    for(int i=0;i<9;i++){
    mag[i] = SPI.transfer(0);
    }
    digitalWrite(CS,HIGH);



     int measurement = 0;
    int index = 0;
    for (int j = 0; j < 9; j += 3) {
        if (mag[j] & 0x80)
            measurement = 0xFF;
        measurement <<= 24; //left shift 24-bit
        measurement |= (mag[j+2] | (mag[j+1] | (mag[j] << 8)) << 8);
        count[index] = measurement;
        measurement = 0;
        index++;
    }

    for (int k = 0; k < 3; k++) {
    MagValues[k] = (float)count[k]*13.0/2.0;
    }
   
}

 return state_drdy;
}

float RM3100::readHeading(){
	float MagValues[3];
	int state = readMag(MagValues);
	if(state!=0){
		float heading = -((atan2(MagValues[1],MagValues[0]) * 180) / 3.14)+180;
		return heading;
	}else{
		return NULL;
	}
	
}

//constructor
RM3100::RM3100(int cs,int drdy):CS(cs),DRDY(drdy){
	
}



