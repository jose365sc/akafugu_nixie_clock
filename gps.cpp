/*
 * GPS support for The Akafugu Nixie Clock
 * (C) 2012 William B Phelps
 *
 * This program is free software; you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 */

#include "global.h"

#ifdef HAVE_GPS

#include <avr/interrupt.h>
#include <string.h>
#include <util/delay.h>
#include "gps.h"
#include "time.h"
#include "pitches.h"

#include <WireRtcLib.h>

unsigned long tGPSupdate;

// we double buffer: read into one line and leave one for the main program
volatile char gpsBuffer1[GPSBUFFERSIZE];
volatile char gpsBuffer2[GPSBUFFERSIZE];
// our index into filling the current line
volatile uint8_t gpsBufferPtr;
// pointers to the double buffers
volatile char *gpsNextBuffer;
volatile char *gpsLastBuffer;
volatile uint8_t gpsDataReady_;

extern int8_t g_gps_enabled;
extern int8_t g_TZ_hour;
extern int8_t g_TZ_minute;

int8_t g_gps_updating;  // for signalling GPS update on some displays
// debugging counters 
int8_t g_gps_cks_errors;  // gps checksum error counter
int8_t g_gps_parse_errors;  // gps parse error counter
int8_t g_gps_time_errors;  // gps time error counter
int8_t g_DST_mode;  // DST off, on, auto?
int8_t g_DST_offset;  // DST offset in Hours
int8_t g_DST_updated;  // DST update flag = allow update only once per day

//volatile uint8_t gpsEnabled = 0;
#define gpsTimeoutLimit 5  // 5 seconds until we display the "no gps" message
uint16_t gpsTimeout;  // how long since we received valid GPS data?

extern WireRtcLib rtc;

void setRTCTime(time_t t)
{
    tmElements_t tm;    
    breakTime(t, &tm);
    rtc.setTime_s(tm.Hour, tm.Minute, tm.Second);   
}

void debugwrite(char *s)
{
#if 0
  Serial.write(s); 
#endif
}

void debugwritechar(char c)
{
#if 0
  Serial.write(c);
#endif
}

void GPSread(void) 
{
  if (!g_gps_enabled)
    return;
  
  int cc = 0;
  while ((cc = Serial.read()) >= 0) {
    char c = cc;
    debugwritechar(c); // debug
    
		if (c == '$') {
			gpsNextBuffer[gpsBufferPtr] = 0;
			gpsBufferPtr = 0;
     
		} else if (c == '\n') {  // newline marks end of sentence
      debugwrite("read as:"); // debug
      debugwrite(gpsNextBuffer); // debug
      debugwrite("\r\n"); // debug
			gpsNextBuffer[gpsBufferPtr] = 0;  // terminate string
			if (gpsNextBuffer == gpsBuffer1) {  // switch buffers
				gpsNextBuffer = gpsBuffer2;
				gpsLastBuffer = gpsBuffer1;
			} else {
				gpsNextBuffer = gpsBuffer1;
				gpsLastBuffer = gpsBuffer2;
			}
			gpsBufferPtr = 0;
			gpsDataReady_ = true;  // signal data ready
		}
		gpsNextBuffer[gpsBufferPtr++] = c;  // add char to current buffer, then increment index
		if (gpsBufferPtr >= GPSBUFFERSIZE) { // if buffer full
			gpsBufferPtr = GPSBUFFERSIZE-1;  // decrement index to make room (overrun)
    }
	}
}

uint8_t gpsDataReady(void) {
	return gpsDataReady_;
}

char *gpsNMEA(void) {
  gpsDataReady_ = false;
  return (char *)gpsLastBuffer;
}

uint32_t parsedecimal(char *str) {
  uint32_t d = 0;
  while (str[0] != 0) {
   if ((str[0] > '9') || (str[0] < '0'))
     return d;  // no more digits
	 d = (d*10) + (str[0] - '0');
   str++;
  }
  return d;
}
const char hex[17] = "0123456789ABCDEF";
uint8_t atoh(char x) {
  return (strchr(hex, x) - hex);
}
uint32_t hex2i(char *str, uint8_t len) {
  uint32_t d = 0;
	for (uint8_t i=0; i<len; i++) {
	 d = (d*10) + (strchr(hex, str[i]) - hex);
	}
	return d;
}

//  225446       Time of fix 22:54:46 UTC
//  A            Navigation receiver warning A = OK, V = warning
//  4916.45,N    Latitude 49 deg. 16.45 min North
//  12311.12,W   Longitude 123 deg. 11.12 min West
//  000.5        Speed over ground, Knots
//  054.7        Course Made Good, True
//  191194       Date of fix  19 November 1994
//  020.3,E      Magnetic variation 20.3 deg East
//  *68          mandatory checksum

//$GPRMC,225446.000,A,4916.45,N,12311.12,W,000.5,054.7,191194,020.3,E*68\r\n
// 0         1         2         3         4         5         6         7
// 0123456789012345678901234567890123456789012345678901234567890123456789012
//    0     1       2    3    4     5    6   7     8      9     10  11 12
void parseGPSdata(char *gpsBuffer) {  
	time_t tNow;
	tmElements_t tm;
	uint8_t gpsCheck1, gpsCheck2;  // checksums
//	char gpsTime[10];  // time including fraction hhmmss.fff
	char gpsFixStat;  // fix status
//	char gpsLat[7];  // ddmm.ff  (with decimal point)
//	char gpsLatH;  // hemisphere 
//	char gpsLong[8];  // dddmm.ff  (with decimal point)
//	char gpsLongH;  // hemisphere 
//	char gpsSpeed[5];  // speed over ground
//	char gpsCourse[5];  // Course
//	char gpsDate[6];  // Date
//	char gpsMagV[5];  // Magnetic variation 
//	char gpsMagD;  // Mag var E/W
//	char gpsCKS[2];  // Checksum without asterisk
	char *ptr;
  uint32_t tmp;
	if ( strncmp( gpsBuffer, "$GPRMC,", 7 ) == 0 ) {  
  
    //Serial.println("parseGPSData");
  //Serial.println(gpsBuffer);  

		//beep(1000, 1);
		//Calculate checksum from the received data
		ptr = &gpsBuffer[1];  // start at the "G"
		gpsCheck1 = 0;  // init collector
		 /* Loop through entire string, XORing each character to the next */
		while (*ptr != '*') // count all the bytes up to the asterisk
		{
			gpsCheck1 ^= *ptr;
			ptr++;
			if (ptr>(gpsBuffer+GPSBUFFERSIZE)) goto GPSerror1;  // extra sanity check, can't hurt...
		}
		// now get the checksum from the string itself, which is in hex
    gpsCheck2 = atoh(*(ptr+1)) * 16 + atoh(*(ptr+2));
		if (gpsCheck1 == gpsCheck2) {  // if checksums match, process the data
			//beep(1000, 1);
			ptr = strtok(gpsBuffer, ",*\r");  // parse $GPRMC
			if (ptr == NULL) goto GPSerror1;
			ptr = strtok(NULL, ",*\r");  // Time including fraction hhmmss.fff
			if (ptr == NULL) goto GPSerror1;
			if ((strlen(ptr) < 6) || (strlen(ptr) > 10)) goto GPSerror1;  // check time length
//			strncpy(gpsTime, ptr, 10);  // copy time string hhmmss
			tmp = parsedecimal(ptr);   // parse integer portion
			tm.Hour = tmp / 10000;
			tm.Minute = (tmp / 100) % 100;
			tm.Second = tmp % 100;
			ptr = strtok(NULL, ",*\r");  // Status
			if (ptr == NULL) goto GPSerror1;
			gpsFixStat = ptr[0];
			if (gpsFixStat == 'A') {  // if data valid, parse time & date
				gpsTimeout = 0;  // reset gps timeout counter
				ptr = strtok(NULL, ",*\r");  // Latitude including fraction
				if (ptr == NULL) goto GPSerror1;
//				strncpy(gpsLat, ptr, 7);  // copy Latitude ddmm.ff
				ptr = strtok(NULL, ",*\r");  // Latitude N/S
				if (ptr == NULL) goto GPSerror1;
//				gpsLatH = ptr[0];
				ptr = strtok(NULL, ",*\r");  // Longitude including fraction hhmm.ff
				if (ptr == NULL) goto GPSerror1;
//				strncpy(gpsLong, ptr, 7);
				ptr = strtok(NULL, ",*\r");  // Longitude Hemisphere
				if (ptr == NULL) goto GPSerror1;
//				gpsLongH = ptr[0];
				ptr = strtok(NULL, ",*\r");  // Ground speed 000.5
				if (ptr == NULL) goto GPSerror1;
//				strncpy(gpsSpeed, ptr, 5);
				ptr = strtok(NULL, ",*\r");  // Track angle (course) 054.7
				if (ptr == NULL) goto GPSerror1;
//				strncpy(gpsCourse, ptr, 5);
				ptr = strtok(NULL, ",*\r");  // Date ddmmyy
				if (ptr == NULL) goto GPSerror1;
//				strncpy(gpsDate, ptr, 6);
				if (strlen(ptr) != 6) goto GPSerror1;  // check date length
				tmp = parsedecimal(ptr); 
				tm.Day = tmp / 10000;
				tm.Month = (tmp / 100) % 100;
				tm.Year = tmp % 100;
				ptr = strtok(NULL, "*\r");  // magnetic variation & dir
				if (ptr == NULL) goto GPSerror1;
				ptr = strtok(NULL, ",*\r");  // Checksum
				if (ptr == NULL) goto GPSerror1;
//				strncpy(gpsCKS, ptr, 2);  // save checksum chars
				
				tm.Year = y2kYearToTm(tm.Year);  // convert yy year to (yyyy-1970) (add 30)
				tNow = makeTime(&tm);  // convert to time_t
				
				if ((tGPSupdate>0) && (abs(tNow-tGPSupdate)>SECS_PER_DAY))  goto GPSerror2;  // GPS time jumped more than 1 day

				if ((tm.Second == 0) || ((tNow - tGPSupdate)>=60)) {  // update RTC once/minute or if it's been 60 seconds
					//beep(1000, 1);  // debugging
					g_gps_updating = true;
					tGPSupdate = tNow;  // remember time of this update
					tNow = tNow + (long)(g_TZ_hour + g_DST_offset) * SECS_PER_HOUR;  // add time zone hour offset & DST offset
					if (g_TZ_hour < 0)  // add or subtract time zone minute offset
						tNow = tNow - (long)g_TZ_minute * SECS_PER_HOUR;
					else
						tNow = tNow + (long)g_TZ_minute * SECS_PER_HOUR;
					setRTCTime(tNow);  // set RTC from adjusted GPS time & date
				}
				else {
					g_gps_updating = false;
				}
			} // if fix status is A
		} // if checksums match
		else { // checksums do not match
			g_gps_cks_errors++;  // increment error count
		}
		return;
GPSerror1:
		g_gps_parse_errors++;  // increment error count
		goto GPSerror2a;
GPSerror2:
		g_gps_time_errors++;  // increment error count
GPSerror2a:
		//beep(2093,1);  // error signal - I'm leaving this in for now /wm
		//flash_display(200);  // flash display to show GPS error
		strcpy(gpsBuffer, "");  // wipe GPS buffer
	}  // if "$GPRMC"
}


void gps_init(uint8_t gps) {
	switch (gps) {
		case(0):  // no GPS
			break;
		case(48):
      Serial.begin(4800, SERIAL_8N1);
			break;
		case(96):
      Serial.begin(9600, SERIAL_8N1);
			break;
	}
	tGPSupdate = 0;  // reset GPS last update time
	gpsDataReady_ = false;
  gpsBufferPtr = 0;
  gpsNextBuffer = gpsBuffer1;
  gpsLastBuffer = gpsBuffer2;
  debugwrite("serial initialized\r\n");
}

#endif // HAVE_GPS
