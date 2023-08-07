/**
* @file         RM3100.h
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
#ifndef RM3100_h
#define RM3100_h

#include <SPI.h>
//RM3100 Registers 
#define RM3100_POLL_REG     0x00 /**< RW; Poll for a single measurement*/
#define RM3100_CMM_REG      0x01 /**< RW; Initiate continuous measurement mode*/
#define RM3100_CCXLSB_REG   0x04 /**< RW; Cycle count X-axis */
#define RM3100_CCXMSB_REG   0x05
#define RM3100_CCYLSB_REG   0x06 /**< RW; Cycle count Y-axis */
#define RM3100_CCYMSB_REG   0x07
#define RM3100_CCZLSB_REG   0x08 /**< RW; Cycle count Z-axis */
#define RM3100_CCZMSB_REG   0x09
#define RM3100_TMRC_REG     0x0B /**< RW; Set data rate in continuous measurement mode*/
#define RM3100_MX_REG       0x24 /**< RW; Measurement Result X-axis, Signed 24-bit */
#define RM3100_BIST_REG     0x33 /**< RW; Built-in self test */
#define RM3100_STATUS_REG   0x34 /**< R; Status of DRDY */
#define RM3100_REVID_REG    0x36 /**< R; Revision ID, default 0x22 */

//RM3100 Default values
#define DEFAULTCCOUNT 200 //200
#define DEFAULTGAIN 75    //200 Cycle Count, Gain 75

//Other Settings
#define UPPERCYCLECOUNT 400
#define LOWERCYCLECOUNT 30


class RM3100 {
    public:
    	void begin();    
    	RM3100(int cs,int drdy);//constructor
		int readMag(float * MagValues);
		float readHeading();

    
private:
    	int current_gain[3];
    	int current_ccount[3];
    
	int DRDY;
	int CS ;
	int CMM = 0x79;
    	int BIST = 0b00000000;
    
	int mag[9];
	int count[3];
	int state_drdy = 0;

};
#endif /* _RM3100_H_ */