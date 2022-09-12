/*
 * bq32002.c
 *
 *  Created on: 6 Sep 2022
 *      Author: Szymon Kajda
 */

// Uses I2C_TWI library

#include <avr/io.h>
#include <util/atomic.h>	// used for access to the SFR register

#include "bq32002.h"
#include "../I2C_TWI/i2c_twi.h"

#include "../uart_lib/uart.h"

void BQ32002_init( void ){
	i2cSetBitrate();
	// Force start its oscillator?
}

void BQ32002_getDateTime( datetime_t * dt ){
	uint8_t i;
	uint8_t buf[7];
	TWI_read_buf( BQ32002_ADDR, BQ32002_REG_SECONDS, 7, buf );
	for( i=0; i<7; i++ ) dt->bytes[i] = bcd2dec( buf[i] );

#if TIME_AS_STRING == 1 || DATE_AS_STRING == 1
	char s[2];
#endif
#if TIME_AS_STRING == 1
	uint8ToStr( s,dt->hh,2 );
	dt->time[0] = s[0];
	dt->time[1] = s[1];
	dt->time[2] = TIME_SEPARATOR;
	uint8ToStr( s,dt->mm,2 );
	dt->time[3] = s[0];
	dt->time[4] = s[1];
	dt->time[5] = TIME_SEPARATOR;
	uint8ToStr( s,dt->ss,2 );
	dt->time[6] = s[0];
	dt->time[7] = s[1];
#endif
#if DATE_AS_STRING == 1
	uint8ToStr( s,dt->day,2 );
	dt->date[0] = s[0];
	dt->date[1] = s[1];
	dt->date[2] = DATE_SEPARATOR;
	uint8ToStr( s,dt->month,2 );
	dt->date[3] = s[0];
	dt->date[4] = s[1];
	dt->date[5] = DATE_SEPARATOR;
	uint8ToStr( s,dt->year,2 );
	dt->date[6] = s[0];
	dt->date[7] = s[1];
#endif
}
void BQ32002_setTime( uint8_t hh, uint8_t mm, uint8_t ss ){
	uint8_t buf[3];
	buf[0] = dec2bcd(ss);
	buf[1] = dec2bcd(mm);
	buf[2] = dec2bcd(hh);
	TWI_write_buf( BQ32002_ADDR, BQ32002_REG_SECONDS, 3, buf );
}

void BQ32002_setDate( uint8_t year, uint8_t month, uint8_t day, uint8_t dayofweek ){
	uint8_t buf[4];
	buf[0] = dayofweek;
	buf[1] = dec2bcd(day);
	buf[2] = dec2bcd(month);
	buf[3] = dec2bcd(year);
	TWI_write_buf( BQ32002_ADDR, 0x03, 4, buf );
}

void BQ32002_enableOsc( void ){
	uint8_t buf[1];
	TWI_read_buf( BQ32002_ADDR, BQ32002_REG_SECONDS, 1, buf );
	buf[0] &= ~(1<<BQ32002_STOP_BIT);
	TWI_write_buf( BQ32002_ADDR, BQ32002_REG_SECONDS, 1, buf );
}

void BQ32002_disableOsc( void ){
	uint8_t buf[1];
	TWI_read_buf( BQ32002_ADDR, BQ32002_REG_SECONDS, 1, buf );
	buf[0] |= (1<<BQ32002_STOP_BIT);
	TWI_write_buf( BQ32002_ADDR, BQ32002_REG_SECONDS, 1, buf );
}

uint8_t BQ32002_getOFFlag( void ){
	uint8_t buf[1];
	TWI_read_buf( BQ32002_ADDR, BQ32002_REG_MINUTES, 1, buf );

	if( bit_is_set( buf[0], BQ32002_OF_BIT ) )
		return 1;
	return 0;
}

void BQ32002_setOFFlag( void ){
	uint8_t buf[1];
	TWI_read_buf( BQ32002_ADDR, BQ32002_REG_MINUTES, 1, buf );
	buf[0] |= (1<<BQ32002_OF_BIT);
	TWI_write_buf( BQ32002_ADDR, BQ32002_REG_MINUTES, 1, buf );
}

void BQ32002_clearOFFlag( void ){
	uint8_t buf[1];
	TWI_read_buf( BQ32002_ADDR, BQ32002_REG_MINUTES, 1, buf );
	buf[0] &= ~(1<<BQ32002_OF_BIT);
	TWI_write_buf( BQ32002_ADDR, BQ32002_REG_MINUTES, 1, buf );
}

#ifdef BQ32002_HANDLE_CENTURIES

void BQ32002_enableCenturyTimekeeping( void ){
	uint8_t buf[1];
	TWI_read_buf( BQ32002_ADDR, BQ32002_REG_CENT_HOURS, 1, buf );
	buf[0] |= (1<<BQ32002_CENT_EN_BIT);
	TWI_write_buf( BQ32002_ADDR, BQ32002_REG_CENT_HOURS, 1, buf );
}

void BQ32002_disableCenturyTimekeeping( void ){
	uint8_t buf[1];
	TWI_read_buf( BQ32002_ADDR, BQ32002_REG_CENT_HOURS, 1, buf );
	buf[0] &= ~(1<<BQ32002_CENT_EN_BIT);
	TWI_write_buf( BQ32002_ADDR, BQ32002_REG_CENT_HOURS, 1, buf );
}

uint8_t BQ32002_getCentury( void ){
	uint8_t buf[1];
	TWI_read_buf( BQ32002_ADDR, BQ32002_REG_CENT_HOURS, 1, buf );

	if( bit_is_set( buf[0], BQ32002_CENT_BIT ) )
		return 1;
	return 0;
}

#endif // BQ32002_HANDLE_CENTURIES

void BQ32002_writeIRQ( uint8_t state ){
	uint8_t buf[1];
	TWI_read_buf( BQ32002_ADDR, BQ32002_REG_CAL_CFG1, 1, buf );
	if( state )
		buf[0] |= (1<<BQ32002_CAL_OUT_BIT);
	else
		buf[0] &= ~(1<<BQ32002_CAL_OUT_BIT);
	TWI_write_buf( BQ32002_ADDR, BQ32002_REG_CAL_CFG1, 1, buf );
}

#ifdef BQ32002_CALIBRATION

void BQ32002_enableFreqTest( void ){
	uint8_t buf[1];
	TWI_read_buf( BQ32002_ADDR, BQ32002_REG_CAL_CFG1, 1, buf );
	buf[0] |= (1<<BQ32002_CAL_FT_BIT);
	TWI_write_buf( BQ32002_ADDR, BQ32002_REG_CAL_CFG1, 1, buf );
}

void BQ32002_disableFreqTest( void ){
	uint8_t buf[1];
	TWI_read_buf( BQ32002_ADDR, BQ32002_REG_CAL_CFG1, 1, buf );
	buf[0] &= ~(1<<BQ32002_CAL_FT_BIT);
	TWI_write_buf( BQ32002_ADDR, BQ32002_REG_CAL_CFG1, 1, buf );
}

void BQ32002_setCalSign( uint8_t sign ){
	uint8_t buf[1];
	TWI_read_buf( BQ32002_ADDR, BQ32002_REG_CAL_CFG1, 1, buf );
	if( sign )
		buf[0] |= (1<<BQ32002_CAL_S_BIT);
	else
		buf[0] &= ~(1<<BQ32002_CAL_S_BIT);
	TWI_write_buf( BQ32002_ADDR, BQ32002_REG_CAL_CFG1, 1, buf );
}

void BQ32002_setCalValue( uint8_t val ){
	uint8_t buf[1];
	TWI_read_buf( BQ32002_ADDR, BQ32002_REG_CAL_CFG1, 1, buf );
	buf[0] &= 0b11100000;	// clear previous CAL value
	buf[0] |= val;
	TWI_write_buf( BQ32002_ADDR, BQ32002_REG_CAL_CFG1, 1, buf );
}

void BQ32002_setCalFreq( uint8_t mode ){
	uint8_t buf[3];
	buf[0] = BQ32002_SF_KEY_1;	// value for BQ32002_REG_SF_KEY_1
	buf[1] = BQ32002_SF_KEY_2;	// value for BQ32002_REG_SF_KEY_2

	// Value for BQ32002_REG_SFR
	// we're only interested in the FTF bit,
	// the rest should be written as 0's
	if( mode )
		buf[2] = (1<<BQ32002_FTF_BIT);
	else
		buf[2] = 0x00;

	// Write data to 3 succeeding registers:
	// SF_KEY_1, SF_KEY_2 and SFR
	// It should be done as quick as possible I guess
	ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
		TWI_write_buf( BQ32002_ADDR, BQ32002_REG_SF_KEY_1, 3, buf );
	}
}

#endif // BQ32002_CALIBRATION

void uint8ToStr( char str[], uint8_t num, uint8_t len ){
	uint8_t rem;
	for( uint8_t i=0; i<len; i++ ){
		rem = num % 10;
		num /= 10;
		str[len-(i+1)] = rem + '0';
	}
}

uint8_t dec2bcd( uint8_t dec ){
	return ((dec / 10)<<4) | ( dec % 10 );
}
uint8_t bcd2dec( uint8_t bcd ){
	return ((((bcd) >> 4) & 0x0F) * 10) + ((bcd) & 0x0F);
}
