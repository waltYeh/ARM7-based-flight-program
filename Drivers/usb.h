//*----------------------------------------------------------------------------
//*      ATMEL Microcontroller Software Support  -  ROUSSET  -
//*----------------------------------------------------------------------------
//* The software is delivered "AS IS" without warranty or condition of any
//* kind, either express, implied or statutory. This includes without
//* limitation any warranty or condition with respect to merchantability or
//* fitness for any particular purpose, or against the infringements of
//* intellectual property rights of others.
//*----------------------------------------------------------------------------
//* File Name           : cdc_enumerate.h
//* Object              : Handle CDC enumeration
//*
//* 1.0 Apr 20 200 	: ODi Creation
//*----------------------------------------------------------------------------
#ifndef CDC_ENUMERATE_H
#define CDC_ENUMERATE_H
#include "at91sam7s256.h"
#include "../Library/lib_AT91SAM7S256.h"

#define AT91C_EP_OUT 1
#define AT91C_EP_OUT_SIZE 0x40
#define AT91C_EP_IN  2
#define AT91C_EP_IN_SIZE 0x40
#define MSG_SIZE 				1000
//struct _AT91S_CDC 	pCDC;


typedef struct _AT91S_CDC
{
	// Private members
	AT91PS_UDP pUdp;
	unsigned char currentConfiguration;
	unsigned char currentConnection;
	unsigned int  currentRcvBank;
	// Public Methods:
	unsigned char (*IsConfigured)(struct _AT91S_CDC *pCdc);
	unsigned int  (*Write) (struct _AT91S_CDC *pCdc, const char *pData, unsigned int length);
	unsigned int  (*Read)  (struct _AT91S_CDC *pCdc, char *pData, unsigned int length);
} AT91S_CDC, *AT91PS_CDC;

//* external function description

AT91PS_CDC AT91F_CDC_Open(AT91PS_CDC pCdc, AT91PS_UDP pUdp);
void USB_init(void);
extern void USB_int_init(void);
char USB_wait_connect(int cnt_down);
void USB_read_Raspberry(void);
void USB_write_Raspberry(unsigned int time_stamp, unsigned char descriptor);
//int USB_armed(void);
int USB_check(void);
__irq void usb_int_handler(void);
void USB_read_Process(void);

#endif // CDC_ENUMERATE_H

