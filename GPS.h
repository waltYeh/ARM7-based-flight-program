#ifndef GPS_H
#define GPS_H
#include "global.h"
//#define notCONFIG_GPS 1
#if OUTDOOR
extern void gps_init(void);

extern __irq void gps_int_handler(void);

extern void get_gps_data(void);
extern float char_to_float(char *a);
#endif
/*****received_type define******/
#define LAT		0//in unit of min, minus means S
#define LON		1//in unit of min, minus means W
#define ALT		2//in unit of m
#define VEL		3//in unit of m/s
#define AZM		4//Azimuth, in unit of degrees
#define SAT		5//number of nevigating satellites
#define STATUS	6//
/*******frame define*******/
#define GPGGA		0
#define GPRMC		1
#define OTH_FRM		2
#endif
