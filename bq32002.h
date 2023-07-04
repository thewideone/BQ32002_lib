/*
 * bq32002.h
 *
 *  Created on: 6 Sep 2022
 *      Author: Szymon Kajda
 *
 *  Higher-level code based on DS3231
 *  RTC library by Mirosław Kardaś from Atnel
 */

#ifndef BQ32002_LIB_BQ32002_H_
#define BQ32002_LIB_BQ32002_H_

#include <stdint.h>

#define BQ32002_ADDR 0b11010000 // = 0xD0

// Registers
#define BQ32002_REG_SECONDS 0x00
#define BQ32002_REG_MINUTES 0x01
#define BQ32002_REG_CENT_HOURS 0x02
#define BQ32002_REG_DAY 0x03
#define BQ32002_REG_DATE 0x04
#define BQ32002_REG_MONTH 0x05
#define BQ32002_REG_YEARS 0x06
#define BQ32002_REG_CAL_CFG1 0x07
#define BQ32002_REG_CFG2 0x09

#define BQ32002_STOP_BIT 7		// oscillator stop bit
#define BQ32002_OF_BIT 7		// oscillator fail flag
#define BQ32002_CENT_EN_BIT 7	// century enable bit
#define BQ32002_CENT_BIT 6		// century bit
#define BQ32002_CAL_S_BIT 5		// IRQ pin logic state control
#define BQ32002_CAL_FT_BIT 6	// frequency test (on IRQ pin) enable
#define BQ32002_CAL_OUT_BIT 7	// calibration sign
								// (0 - slowing, 1 - speeding the RTC)

// Special function registers
#define BQ32002_REG_SF_KEY_1 0x20
#define BQ32002_REG_SF_KEY_2 0x21
#define BQ32002_REG_SFR 0x22

#define BQ32002_SF_KEY_1 0x5E
#define BQ32002_SF_KEY_2 0xC7
#define BQ32002_FTF_BIT 0	// force calibration to 1Hz
// The default 512-Hz calibration signal does not
// include the effect of the ppm adjustment

// Software configuration
#define TIME_AS_STRING 1
#define DATE_AS_STRING 1

#ifdef DATE_AS_STRING
#define DATE_AS_STRING_LONG
#endif

#define TIME_SEPARATOR ':'
#define DATE_SEPARATOR '.'

#define BQ32002_HANDLE_CENTURIES

#ifdef BQ32002_HANDLE_CENTURIES
#define CURRENT_MILLENNIUM_DIGIT	'2'
#define CURRENT_CENTURY_DIGIT		'0'
#endif

#define BQ32002_CALIBRATION

// Data structures
typedef union {
	uint8_t bytes[7];
	struct {
		uint8_t ss;
		uint8_t mm;
		uint8_t hh;
		uint8_t dayofweek;	// value (decimal 1 to 7) from DAY register
		uint8_t day;		// value from DATE register
		uint8_t month;
		uint8_t year;
#ifdef TIME_AS_STRING
		char time[9];	// e.g. 13:59:53\0
#endif
#ifdef DATE_AS_STRING
#ifdef DATE_AS_STRING_LONG
		char date[11];	// e.g. 22.02.2022\0
#else
		char date[9];	// e.g. 22.02.22\0
#endif
#endif
	};
} datetime_t;

// Functions
void BQ32002_init( void );
void BQ32002_getDateTime( datetime_t * dt );
void BQ32002_setTime( uint8_t hh, uint8_t mm, uint8_t ss );
void BQ32002_setDate( uint8_t year, uint8_t month, uint8_t day, uint8_t dayofweek );

// Enable/disable oscillator without changing stored seconds
void BQ32002_enableOsc( void );
void BQ32002_disableOsc( void );

// Set/clear oscillator fail flag
uint8_t BQ32002_getOFFlag( void );
void BQ32002_setOFFlag( void );
void BQ32002_clearOFFlag( void );


#ifdef BQ32002_HANDLE_CENTURIES

// Enable/disable century timekeeping feature
void BQ32002_enableCenturyTimekeeping( void );
void BQ32002_disableCenturyTimekeeping( void );
// Returns value of the CENT bit in CENT_HOURS register
uint8_t BQ32002_getCentury( void );

#endif // BQ32002_HANDLE_CENTURIES


// Set IRQ pin state (high or low)
// (changes are applied if frequency test is disabled)
void BQ32002_writeIRQ( uint8_t state );

#ifdef BQ32002_CALIBRATION

// Enable/disable 1Hz/512Hz square wave output on IRQ pin
void BQ32002_enableFreqTest( void );
void BQ32002_disableFreqTest( void );

// 0 	 - calibration will slow the RTC
// non-0 - calibration will speed the RTC
void BQ32002_setCalSign( uint8_t sign );
// Meaning: see table on page 16 in the data sheet
// Valid values: 0-31 (decimal)
void BQ32002_setCalValue( uint8_t val );

// 0 	 - normal 512Hz calibration
// non-0 - 1Hz calibration
void BQ32002_setCalFreq( uint8_t mode );

#endif // BQ32002_CALIBRATION

void uint8ToStr( char str[], uint8_t num, uint8_t len );
uint8_t dec2bcd( uint8_t dec );
uint8_t bcd2dec( uint8_t bcd );

#endif /* BQ32002_LIB_BQ32002_H_ */
