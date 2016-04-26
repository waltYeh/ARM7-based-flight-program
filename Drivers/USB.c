//*----------------------------------------------------------------------------
//*      ATMEL Microcontroller Software Support  -  ROUSSET  -
//*----------------------------------------------------------------------------
//* The software is delivered "AS IS" without warranty or condition of any
//* kind, either express, implied or statutory. This includes without
//* limitation any warranty or condition with respect to merchantability or
//* fitness for any particular purpose, or against the infringements of
//* intellectual property rights of others.
//*----------------------------------------------------------------------------
//* File Name           : cdc_enumerate.c
//* Object              : Handle CDC enumeration
//*
//* 1.0 Apr 20 200 	: ODi Creation
//* 1.1 14/Sep/2004 JPP : Minor change
//* 1.1 15/12/2004  JPP : suppress warning
//* 1.2 30-Jun-2006 JPP : Set in AT91C_UDP_EPTYPE_INT_IN
//*----------------------------------------------------------------------------
//#include "project.h"
#include "USB.h"
#include "Pins.h"
#include "../Main/global.h"
#include "../Main/timer.h"
#include "at91sam7s256.h"
#include "PIO.h"
#include "string.h"
typedef unsigned char  uchar;
typedef unsigned short ushort;
typedef unsigned int   uint;
#define USB_STATUS_SUCCESS      0
#define USB_STATUS_LOCKED       1
#define USB_STATUS_ABORTED      2
#define USB_STATUS_RESET        3
#define UDP_CLEAREPFLAGS(bEndpoint, dFlags) { \
	CLEAR(pUdp->UDP_CSR[bEndpoint], dFlags);}// \
//    while (!ISCLEARED(pUdp->UDP_CSR[bEndpoint], dFlags));}
#define UDP_SETEPFLAGS(bEndpoint, dFlags) { \
	SET(pUdp->UDP_CSR[bEndpoint], dFlags);\
    while (ISCLEARED(pUdp->UDP_CSR[bEndpoint], dFlags));}
        
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#define SET(register, flags)        ((register) = (register) | (flags))
#define CLEAR(register, flags)      ((register) &= ~(flags))

// Poll the status of flags in a register
#define ISSET(register, flags)      (((register) & (flags)) == (flags))
#define ISCLEARED(register, flags)  (((register) & (flags)) == 0)
unsigned char data_from_Raspberry[64];
void USB_written_Callback(void);
void USB_read_Callback(void);

const char devDescriptor[] = {
	/* Device descriptor */
	0x12,   // bLength
	0x01,   // bDescriptorType
	0x10,   // bcdUSBL
	0x01,   //
	0x02,   // bDeviceClass:    CDC class code
	0x00,   // bDeviceSubclass: CDC class sub code
	0x00,   // bDeviceProtocol: CDC Device protocol
	0x08,   // bMaxPacketSize0
	0xEB,   // idVendorL
	0x03,   //
	// mtmt 0x24,   // idProductL
	0x25,   // idProductL
	0x61,   //
	0x10,   // bcdDeviceL
	0x01,   //
	0x00,   // iManufacturer    // 0x01
	0x00,   // iProduct
	0x00,   // SerialNumber
	0x01    // bNumConfigs
};

const char cfgDescriptor[] = {
	/* ============== CONFIGURATION 1 =========== */
	/* Configuration 1 descriptor */
	0x09,   // CbLength
	0x02,   // CbDescriptorType
	0x43,   // CwTotalLength 2 EP + Control
	0x00,
	0x02,   // CbNumInterfaces
	0x01,   // CbConfigurationValue
	0x00,   // CiConfiguration
	0xC0,   // CbmAttributes 0xA0
	0x00,   // CMaxPower

	/* Communication Class Interface Descriptor Requirement */
	0x09, // bLength
	0x04, // bDescriptorType
	0x00, // bInterfaceNumber
	0x00, // bAlternateSetting
	0x01, // bNumEndpoints
	0x02, // bInterfaceClass
	0x02, // bInterfaceSubclass
	0x00, // bInterfaceProtocol
	0x00, // iInterface

	/* Header Functional Descriptor */
	0x05, // bFunction Length
	0x24, // bDescriptor type: CS_INTERFACE
	0x00, // bDescriptor subtype: Header Func Desc
	0x10, // bcdCDC:1.1
	0x01,

	/* ACM Functional Descriptor */
	0x04, // bFunctionLength
	0x24, // bDescriptor Type: CS_INTERFACE
	0x02, // bDescriptor Subtype: ACM Func Desc
	0x00, // bmCapabilities

	/* Union Functional Descriptor */
	0x05, // bFunctionLength
	0x24, // bDescriptorType: CS_INTERFACE
	0x06, // bDescriptor Subtype: Union Func Desc
	0x00, // bMasterInterface: Communication Class Interface
	0x01, // bSlaveInterface0: Data Class Interface

	/* Call Management Functional Descriptor */
	0x05, // bFunctionLength
	0x24, // bDescriptor Type: CS_INTERFACE
	0x01, // bDescriptor Subtype: Call Management Func Desc
	0x00, // bmCapabilities: D1 + D0
	0x01, // bDataInterface: Data Class Interface 1

	/* Endpoint 1 descriptor */
	0x07,   // bLength
	0x05,   // bDescriptorType
	0x83,   // bEndpointAddress, Endpoint 03 - IN
	0x03,   // bmAttributes      INT
	0x08,   // wMaxPacketSize
	0x00,
	0xFF,   // bInterval

	/* Data Class Interface Descriptor Requirement */
	0x09, // bLength
	0x04, // bDescriptorType
	0x01, // bInterfaceNumber
	0x00, // bAlternateSetting
	0x02, // bNumEndpoints
	0x0A, // bInterfaceClass
	0x00, // bInterfaceSubclass
	0x00, // bInterfaceProtocol
	0x00, // iInterface

	/* First alternate setting */
	/* Endpoint 1 descriptor */
	0x07,   // bLength
	0x05,   // bDescriptorType
	0x01,   // bEndpointAddress, Endpoint 01 - OUT
	0x02,   // bmAttributes      BULK
	AT91C_EP_OUT_SIZE,   // wMaxPacketSize
	0x00,
	0x00,   // bInterval

	/* Endpoint 2 descriptor */
	0x07,   // bLength
	0x05,   // bDescriptorType
	0x82,   // bEndpointAddress, Endpoint 02 - IN
	0x02,   // bmAttributes      BULK
	AT91C_EP_IN_SIZE,   // wMaxPacketSize
	0x00,
	0x00    // bInterval
};

/* USB standard request code */
#define STD_GET_STATUS_ZERO           0x0080
#define STD_GET_STATUS_INTERFACE      0x0081
#define STD_GET_STATUS_ENDPOINT       0x0082

#define STD_CLEAR_FEATURE_ZERO        0x0100
#define STD_CLEAR_FEATURE_INTERFACE   0x0101
#define STD_CLEAR_FEATURE_ENDPOINT    0x0102

#define STD_SET_FEATURE_ZERO          0x0300
#define STD_SET_FEATURE_INTERFACE     0x0301
#define STD_SET_FEATURE_ENDPOINT      0x0302

#define STD_SET_ADDRESS               0x0500
#define STD_GET_DESCRIPTOR            0x0680
#define STD_SET_DESCRIPTOR            0x0700
#define STD_GET_CONFIGURATION         0x0880
#define STD_SET_CONFIGURATION         0x0900
#define STD_GET_INTERFACE             0x0A81
#define STD_SET_INTERFACE             0x0B01
#define STD_SYNCH_FRAME               0x0C82

/* CDC Class Specific Request Code */
#define GET_LINE_CODING               0x21A1
#define SET_LINE_CODING               0x2021
#define SET_CONTROL_LINE_STATE        0x2221
typedef struct {
    char                    *pData;            
    unsigned int            dBytesRemaining;   //set to length at UDP_Write and UDP_Read, minused at Write_Payload and Get_Payload
    unsigned int            dBytesBuffered;    
												//for IN, set to 0 at UDP_Write
												//set to 0 when transfer finished, FIFO empty
												//added at Write_Payload when data given to FIFO
	
												//for OUT, set to 0 at UDP_Read
												//at Get_Payload, record data still not read in Get_Payload												
    unsigned int            dBytesTransferred; //set to 0 at UDP_Read and UDP_Write, added at IN ep handler
    unsigned int            wMaxPacketSize;    //!!!!!set only for EP_IN and EP_OUT
    unsigned int            dFlag;      //stores current bank 0, 1 in use, cleared in UDP_CLEAREPFLAGS       
    unsigned char           dNumFIFO;   //!!!!!not set yet, should be 2 for bulks       
    volatile unsigned int   dState;     //states as follows       
} S_usb_endpoint;

#define    endpointStateDisabled 0
#define    endpointStateIdle 1	//set at end of read or write in ep handler
#define    endpointStateWrite 2 //set at UDP_Write, the start of writing
#define    endpointStateRead 3	//set at UDP_Read, the start of reading
#define    endpointStateHalted 4 

typedef struct {
	unsigned int dwDTERRate;
	char bCharFormat;
	char bParityType;
	char bDataBits;
} AT91S_CDC_LINE_CODING, *AT91PS_CDC_LINE_CODING;

AT91S_CDC_LINE_CODING line = {
	115200, // baudrate
	0,      // 1 Stop Bit
	0,      // None Parity
	8};     // 8 Data bits

// mtmt uint currentReceiveBank = AT91C_UDP_RX_DATA_BK0;
S_usb_endpoint pEndpoint[4];


int ClearStock = 0;

static uchar AT91F_UDP_IsConfigured(AT91PS_CDC pCdc);
//static uint AT91F_UDP_Read(AT91PS_CDC pCdc, char *pData, uint length);
//static uint AT91F_UDP_Write(AT91PS_CDC pCdc, const char *pData, uint length);
static void AT91F_CDC_Enumerate(AT91PS_CDC pCdc);









//*********************** Functions for CONTROL EP0***********************



//---------------------------------------------------------------------
// AT91F_CDC_Open
//
//---------------------------------------------------------------------
AT91PS_CDC AT91F_CDC_Open(AT91PS_CDC pCdc, AT91PS_UDP pUdp)
{
	pCdc->pUdp = pUdp;
	pCdc->currentConfiguration = 0;
	pCdc->currentConnection    = 0;
	pCdc->currentRcvBank       = AT91C_UDP_RX_DATA_BK0;
	pCdc->IsConfigured = AT91F_UDP_IsConfigured;
//	pCdc->Write        = AT91F_UDP_Write;
//	pCdc->Read         = AT91F_UDP_Read;
	pEndpoint[0].wMaxPacketSize = AT91C_EP_IN_SIZE;
	pEndpoint[AT91C_EP_IN].wMaxPacketSize = AT91C_EP_IN_SIZE;
	pEndpoint[AT91C_EP_OUT].wMaxPacketSize = AT91C_EP_OUT_SIZE;
	pEndpoint[AT91C_EP_IN].dNumFIFO = 1;
	pEndpoint[AT91C_EP_OUT].dNumFIFO = 1;
	pEndpoint[AT91C_EP_IN].dState=endpointStateIdle;
	pEndpoint[AT91C_EP_OUT].dState=endpointStateIdle;
	pEndpoint[AT91C_EP_IN].dFlag=AT91C_UDP_RX_DATA_BK0;
	pEndpoint[AT91C_EP_OUT].dFlag=AT91C_UDP_RX_DATA_BK0;
	return pCdc;
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_UDP_IsConfigured
//* \brief Test if the device is configured and handle enumeration
//*----------------------------------------------------------------------------
static uchar AT91F_UDP_IsConfigured(AT91PS_CDC pCdc)
{
	AT91PS_UDP pUDP = pCdc->pUdp;
	AT91_REG isr = pUDP->UDP_ISR;

	if (isr & AT91C_UDP_ENDBUSRES) {
		pUDP->UDP_ICR = AT91C_UDP_ENDBUSRES;
		// reset all endpoints
		pUDP->UDP_RSTEP  = (unsigned int)-1;
		pUDP->UDP_RSTEP  = 0;
		// Enable the function
		pUDP->UDP_FADDR = AT91C_UDP_FEN;
		// Configure endpoint 0
		//enable and set as control EP
		pUDP->UDP_CSR[0] = (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_CTRL);
	}
	else if (isr & AT91C_UDP_EPINT0) {
		pUDP->UDP_ICR = AT91C_UDP_EPINT0;
		AT91F_CDC_Enumerate(pCdc);
	}
	return pCdc->currentConfiguration;
	//enumerate until SET_CONFIG(set EP types for each of them)
}
/*
//----------------------------------------------------------------------------
// \fn    AT91F_UDP_Read
// \brief Read available data from Endpoint OUT
//----------------------------------------------------------------------------
static uint AT91F_UDP_Read(AT91PS_CDC pCdc, char *pData, uint length)
{
	AT91PS_UDP pUdp = pCdc->pUdp;
	uint packetSize, nbBytesRcv = 0, currentReceiveBank = pCdc->currentRcvBank;

	while (length) {
		if ( !AT91F_UDP_IsConfigured(pCdc) )
			break;
		if ( pUdp->UDP_CSR[AT91C_EP_OUT] & currentReceiveBank ) {
			packetSize = MIN(pUdp->UDP_CSR[AT91C_EP_OUT] >> 16, length);
			length -= packetSize;
			if (packetSize < AT91C_EP_OUT_SIZE)
				length = 0;
			while(packetSize--)
				pData[nbBytesRcv++] = pUdp->UDP_FDR[AT91C_EP_OUT];
			//notify USB peripheral device that data 
			//have been read in the FIFO's Bank x
			pUdp->UDP_CSR[AT91C_EP_OUT] &= ~(currentReceiveBank);
			//exchange the Bank
			if (currentReceiveBank == AT91C_UDP_RX_DATA_BK0)
				currentReceiveBank = AT91C_UDP_RX_DATA_BK1;
			else
				currentReceiveBank = AT91C_UDP_RX_DATA_BK0;

		}
	}
	pCdc->currentRcvBank = currentReceiveBank;
	return nbBytesRcv;

}

//----------------------------------------------------------------------------
// \fn    AT91F_CDC_Write
// \brief Send through endpoint 2
//----------------------------------------------------------------------------
static uint AT91F_UDP_Write(AT91PS_CDC pCdc, const char *pData, uint length)
{
	AT91PS_UDP pUdp = pCdc->pUdp;
	uint cpt = 0;

	// Send the first packet
	cpt = MIN(length, AT91C_EP_IN_SIZE);
	length -= cpt;
	while (cpt--) pUdp->UDP_FDR[AT91C_EP_IN] = *pData++;
	pUdp->UDP_CSR[AT91C_EP_IN] |= AT91C_UDP_TXPKTRDY;

	while (length) {
		// Fill the second bank
		cpt = MIN(length, AT91C_EP_IN_SIZE);
		length -= cpt;
		while (cpt--) pUdp->UDP_FDR[AT91C_EP_IN] = *pData++;
		// Wait for the the first bank to be sent
		while ( !(pUdp->UDP_CSR[AT91C_EP_IN] & AT91C_UDP_TXCOMP) )
			if ( !AT91F_UDP_IsConfigured(pCdc) ) return length;
		pUdp->UDP_CSR[AT91C_EP_IN] &= ~(AT91C_UDP_TXCOMP);
		while (pUdp->UDP_CSR[AT91C_EP_IN] & AT91C_UDP_TXCOMP);
		pUdp->UDP_CSR[AT91C_EP_IN] |= AT91C_UDP_TXPKTRDY;
	}
	// Wait for the end of transfer
	while ( !(pUdp->UDP_CSR[AT91C_EP_IN] & AT91C_UDP_TXCOMP) )
		if ( !AT91F_UDP_IsConfigured(pCdc) ) return length;
	pUdp->UDP_CSR[AT91C_EP_IN] &= ~(AT91C_UDP_TXCOMP);
	while (pUdp->UDP_CSR[AT91C_EP_IN] & AT91C_UDP_TXCOMP);

	return length;
}
*/
//----------------------------------------------------------------------------
//AT91F_USB_SendData
//Send Data through the
//CONTROL EP
//----------------------------------------------------------------------------

static void AT91F_USB_SendData(AT91PS_UDP pUdp, const char *pData, uint length)
{
	uint cpt = 0;
	AT91_REG csr;

	do {
		cpt = MIN(length, 8);
		length -= cpt;

		while (cpt--)
			pUdp->UDP_FDR[0] = *pData++;

		if (pUdp->UDP_CSR[0] & AT91C_UDP_TXCOMP) {
			pUdp->UDP_CSR[0] &= ~(AT91C_UDP_TXCOMP);
			while (pUdp->UDP_CSR[0] & AT91C_UDP_TXCOMP);
		}

		pUdp->UDP_CSR[0] |= AT91C_UDP_TXPKTRDY;
		do {
			csr = pUdp->UDP_CSR[0];

			// Data IN stage has been stopped by a status OUT
			if (csr & AT91C_UDP_RX_DATA_BK0) {
				pUdp->UDP_CSR[0] &= ~(AT91C_UDP_RX_DATA_BK0);
				return;
			}

		} while ( !(csr & AT91C_UDP_TXCOMP) );

	} while (length);

	if (pUdp->UDP_CSR[0] & AT91C_UDP_TXCOMP) {
		pUdp->UDP_CSR[0] &= ~(AT91C_UDP_TXCOMP);
		while (pUdp->UDP_CSR[0] & AT91C_UDP_TXCOMP);
	}
}

//--------------------------------------------------------------------
//AT91F_USB_SendZlp
//Send zero length packet through the 
//CONTROL EP
//----------------------------------------------------------------------------
void AT91F_USB_SendZlp(AT91PS_UDP pUdp)
{
	pUdp->UDP_CSR[0] |= AT91C_UDP_TXPKTRDY;
	while ( !(pUdp->UDP_CSR[0] & AT91C_UDP_TXCOMP) );
	pUdp->UDP_CSR[0] &= ~(AT91C_UDP_TXCOMP);
	while (pUdp->UDP_CSR[0] & AT91C_UDP_TXCOMP);
}

//--------------------------------------------------------------------
//AT91F_USB_SendStall
//Stall the
//CONTROL EP
//--------------------------------------------------------------------
void AT91F_USB_SendStall(AT91PS_UDP pUdp)
{
	pUdp->UDP_CSR[0] |= AT91C_UDP_FORCESTALL;
	while ( !(pUdp->UDP_CSR[0] & AT91C_UDP_ISOERROR) );
	pUdp->UDP_CSR[0] &= ~(AT91C_UDP_FORCESTALL | AT91C_UDP_ISOERROR);
	while (pUdp->UDP_CSR[0] & (AT91C_UDP_FORCESTALL | AT91C_UDP_ISOERROR));
}

//*----------------------------------------------------------------------------
//* \fn    AT91F_CDC_Enumerate
//* \brief This function is a callback invoked when a SETUP packet is received
//*----------------------------------------------------------------------------
static void AT91F_CDC_Enumerate(AT91PS_CDC pCdc)
{
	AT91PS_UDP pUDP = pCdc->pUdp;
	uchar bmRequestType, bRequest;
	ushort wValue, wIndex, wLength, wStatus;
	typedef union myun{
		char b[2];
		short c;
	}un;
//	un un_Value,un_Index,un_Length;

	if ( !(pUDP->UDP_CSR[0] & AT91C_UDP_RXSETUP) )
		return;

	bmRequestType = pUDP->UDP_FDR[0];
	bRequest      = pUDP->UDP_FDR[0];
	wValue        = (pUDP->UDP_FDR[0] & 0xFF);
	wValue       |= (pUDP->UDP_FDR[0] << 8);
	wIndex        = (pUDP->UDP_FDR[0] & 0xFF);
	wIndex       |= (pUDP->UDP_FDR[0] << 8);
	wLength       = (pUDP->UDP_FDR[0] & 0xFF);
	wLength      |= (pUDP->UDP_FDR[0] << 8);
//	un_Value.c = wValue;
//	un_Index.c = wIndex;
//	un_Length.c = wLength;
/*	uart_write_char(bmRequestType);
	uart_write_char(bRequest);
	uart_write_char(un_Value.b[1]);
	uart_write_char(un_Value.b[0]);
	uart_write_char(un_Index.b[1]);
	uart_write_char(un_Index.b[0]);
	uart_write_char(un_Length.b[1]);
	uart_write_char(un_Length.b[0]);
	uart_write_char(0xFF);
	uart_write_char(0xFF);
	uart_write_char(0xFF);
	uart_write_char(0xFF);
	uart_write_char(0xFF);
	uart_write_char(0xFF);
*/
	if (bmRequestType & 0x80) {
		pUDP->UDP_CSR[0] |= AT91C_UDP_DIR;
		while ( !(pUDP->UDP_CSR[0] & AT91C_UDP_DIR) );
	}
	pUDP->UDP_CSR[0] &= ~AT91C_UDP_RXSETUP;
	while ( (pUDP->UDP_CSR[0]  & AT91C_UDP_RXSETUP)  );

	// Handle supported standard device request Cf Table 9-3 in USB specification Rev 1.1
	switch ((bRequest << 8) | bmRequestType) {
	case STD_GET_DESCRIPTOR:
		if (wValue == 0x100)       // Return Device Descriptor
			AT91F_USB_SendData(pUDP, devDescriptor, MIN(sizeof(devDescriptor), wLength));
		else if (wValue == 0x200)  // Return Configuration Descriptor
			AT91F_USB_SendData(pUDP, cfgDescriptor, MIN(sizeof(cfgDescriptor), wLength));
		else
			AT91F_USB_SendStall(pUDP);
		break;
	case STD_SET_ADDRESS:
		AT91F_USB_SendZlp(pUDP);
		pUDP->UDP_FADDR = (AT91C_UDP_FEN | wValue);
		pUDP->UDP_GLBSTATE  = (wValue) ? AT91C_UDP_FADDEN : 0;
		break;
	case STD_SET_CONFIGURATION:
		pCdc->currentConfiguration = wValue;
		AT91F_USB_SendZlp(pUDP);
		pUDP->UDP_GLBSTATE  = (wValue) ? AT91C_UDP_CONFG : AT91C_UDP_FADDEN;
		pUDP->UDP_CSR[1] = (wValue) ? (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_BULK_OUT) : 0;
		pUDP->UDP_CSR[2] = (wValue) ? (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_BULK_IN)  : 0;
		pUDP->UDP_CSR[3] = (wValue) ? (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_INT_IN)   : 0;
		break;
	case STD_GET_CONFIGURATION:
		AT91F_USB_SendData(pUDP, (char *) &(pCdc->currentConfiguration), sizeof(pCdc->currentConfiguration));
		break;
	case STD_GET_STATUS_ZERO:
		wStatus = 0;
		AT91F_USB_SendData(pUDP, (char *) &wStatus, sizeof(wStatus));
		break;
	case STD_GET_STATUS_INTERFACE:
		wStatus = 0;
		AT91F_USB_SendData(pUDP, (char *) &wStatus, sizeof(wStatus));
		break;
	case STD_GET_STATUS_ENDPOINT:
		wStatus = 0;
		wIndex &= 0x0F;
		if ((pUDP->UDP_GLBSTATE & AT91C_UDP_CONFG) && (wIndex <= 3)) {
			wStatus = (pUDP->UDP_CSR[wIndex] & AT91C_UDP_EPEDS) ? 0 : 1;
			AT91F_USB_SendData(pUDP, (char *) &wStatus, sizeof(wStatus));
		}
		else if ((pUDP->UDP_GLBSTATE & AT91C_UDP_FADDEN) && (wIndex == 0)) {
			wStatus = (pUDP->UDP_CSR[wIndex] & AT91C_UDP_EPEDS) ? 0 : 1;
			AT91F_USB_SendData(pUDP, (char *) &wStatus, sizeof(wStatus));
		}
		else
			AT91F_USB_SendStall(pUDP);
		break;
	case STD_SET_FEATURE_ZERO:
		AT91F_USB_SendStall(pUDP);
	    break;
	case STD_SET_FEATURE_INTERFACE:
		AT91F_USB_SendZlp(pUDP);
		break;
	case STD_SET_FEATURE_ENDPOINT:
		wIndex &= 0x0F;
		if ((wValue == 0) && wIndex && (wIndex <= 3)) {
			pUDP->UDP_CSR[wIndex] = 0;
			AT91F_USB_SendZlp(pUDP);
		}
		else
			AT91F_USB_SendStall(pUDP);
		break;
	case STD_CLEAR_FEATURE_ZERO:
		AT91F_USB_SendStall(pUDP);
	    break;
	case STD_CLEAR_FEATURE_INTERFACE:
		AT91F_USB_SendZlp(pUDP);
		break;
	case STD_CLEAR_FEATURE_ENDPOINT:
		wIndex &= 0x0F;
		if ((wValue == 0) && wIndex && (wIndex <= 3)) {
			if (wIndex == 1)
				pUDP->UDP_CSR[1] = (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_BULK_OUT);
			else if (wIndex == 2)
				pUDP->UDP_CSR[2] = (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_BULK_IN);
			else if (wIndex == 3)
				pUDP->UDP_CSR[3] = (AT91C_UDP_EPEDS | AT91C_UDP_EPTYPE_ISO_IN);
			AT91F_USB_SendZlp(pUDP);
		}
		else
			AT91F_USB_SendStall(pUDP);
		break;

	// handle CDC class requests
	case SET_LINE_CODING:
		//until a packet has received
		while ( !(pUDP->UDP_CSR[0] & AT91C_UDP_RX_DATA_BK0) );
		pUDP->UDP_CSR[0] &= ~(AT91C_UDP_RX_DATA_BK0);
		AT91F_USB_SendZlp(pUDP);
		break;
	case GET_LINE_CODING:
		AT91F_USB_SendData(pUDP, (char *) &line, MIN(sizeof(line), wLength));
		break;
	case SET_CONTROL_LINE_STATE:
		pCdc->currentConnection = wValue;
		AT91F_USB_SendZlp(pUDP);
		break;
	default:
		AT91F_USB_SendStall(pUDP);
	    break;
	}
}





//****************** Functions for EP IN and OUT**************************




static void UDP_ClearRXFlag(unsigned char bEndpoint)
{
    // Clear flag
	AT91S_UDP * pUdp = AT91C_BASE_UDP;

    UDP_CLEAREPFLAGS(bEndpoint, pEndpoint[bEndpoint].dFlag);

    // Swap banks
    if (pEndpoint[bEndpoint].dFlag == AT91C_UDP_RX_DATA_BK0) {

        if (pEndpoint[bEndpoint].dNumFIFO > 1) {

            // Swap bank if in dual-fifo mode
            pEndpoint[bEndpoint].dFlag = AT91C_UDP_RX_DATA_BK1;
        }
    }
    else {

        pEndpoint[bEndpoint].dFlag = AT91C_UDP_RX_DATA_BK0;
    }
}
static unsigned int UDP_WritePayload(unsigned char bEndpoint)
{
	AT91S_UDP * pUdp = AT91C_BASE_UDP;
    unsigned int   dBytes;
    unsigned int   dCtr;

    // Get the number of bytes to send
    dBytes = MIN(pEndpoint[bEndpoint].wMaxPacketSize, pEndpoint[bEndpoint].dBytesRemaining);

    // Transfer one packet in the FIFO buffer
    for (dCtr = 0; dCtr < dBytes; dCtr++) {

        pUdp->UDP_FDR[bEndpoint] = *(pEndpoint[bEndpoint].pData);
        pEndpoint[bEndpoint].pData++;
    }

    pEndpoint[bEndpoint].dBytesBuffered += dBytes;
    pEndpoint[bEndpoint].dBytesRemaining -= dBytes;

    return dBytes;
}

static unsigned int UDP_GetPayload(unsigned char bEndpoint,
                                   unsigned short wPacketSize)//num of bytes available in FIFO
{
	AT91S_UDP * pUdp = AT91C_BASE_UDP;
    unsigned int   dBytes;
    unsigned int   dCtr;
	char dummy;
    // Get number of bytes to retrieve
    dBytes = MIN(pEndpoint[bEndpoint].dBytesRemaining, wPacketSize);

    // Retrieve packet
    for (dCtr = 0; dCtr < dBytes; dCtr++) {

        *pEndpoint[bEndpoint].pData = (char) pUdp->UDP_FDR[bEndpoint];
        pEndpoint[bEndpoint].pData++;
    }

    pEndpoint[bEndpoint].dBytesRemaining -= dBytes;//bytes in FIFO still to be read
    pEndpoint[bEndpoint].dBytesTransferred += dBytes;//bytes already read
    pEndpoint[bEndpoint].dBytesBuffered += wPacketSize - dBytes;
	//bytes I'm not going to read due to sily argument at UDP_READ, still in FIFO
//	for (dCtr = 0; dCtr < pEndpoint[bEndpoint].dBytesBuffered; dCtr++) {
//        dummy = (char) pUdp->UDP_FDR[bEndpoint];
//    }
	dummy=dummy;
	pEndpoint[bEndpoint].dBytesBuffered = 0;
    return dBytes;//bytes read this time
}

static void UDP_EndpointHandler(unsigned char bEndpoint)
{
	AT91S_UDP * pUdp = AT91C_BASE_UDP;
	unsigned int dStatus = pUdp->UDP_CSR[bEndpoint];


    // Handle interrupts
    // IN packet sent
    if (ISSET(dStatus, AT91C_UDP_TXCOMP)) {
        // Check that endpoint was in Write state
        if (pEndpoint[bEndpoint].dState == endpointStateWrite) {

            // End of transfer ?
            if ((pEndpoint[bEndpoint].dBytesBuffered < pEndpoint[bEndpoint].wMaxPacketSize)
			//last time I didnt make FIFO full, which means i have no more to send
                ||
                (!ISCLEARED(dStatus, AT91C_UDP_EPTYPE)
                 && (pEndpoint[bEndpoint].dBytesRemaining == 0)
                 && (pEndpoint[bEndpoint].dBytesBuffered == pEndpoint[bEndpoint].wMaxPacketSize))) {
			//last time I make FIFO full, but there's nothing more to send this time

                pEndpoint[bEndpoint].dBytesTransferred += pEndpoint[bEndpoint].dBytesBuffered;//totally transfered
                pEndpoint[bEndpoint].dBytesBuffered = 0;

                // Disable interrupt if this is not a control endpoint
                if (!ISCLEARED(dStatus, AT91C_UDP_EPTYPE)) {

                    SET(pUdp->UDP_IDR, 1 << bEndpoint);
                }

                pEndpoint[bEndpoint].dState = endpointStateIdle;
				USB_written_Callback();
            }
            else {

                // Transfer remaining data
                pEndpoint[bEndpoint].dBytesTransferred += pEndpoint[bEndpoint].wMaxPacketSize;//full packet has been transfered
                pEndpoint[bEndpoint].dBytesBuffered -= pEndpoint[bEndpoint].wMaxPacketSize;//minus one full packet

                // Send next packet
                if (pEndpoint[bEndpoint].dNumFIFO == 1) {

                    // No double buffering
                    UDP_WritePayload(bEndpoint);
                    UDP_SETEPFLAGS(bEndpoint, AT91C_UDP_TXPKTRDY);
                }
                else {

                    // Double buffering
                    UDP_SETEPFLAGS(bEndpoint, AT91C_UDP_TXPKTRDY);
                    UDP_WritePayload(bEndpoint);
                }
            }
        }

        // Acknowledge interrupt
        UDP_CLEAREPFLAGS(bEndpoint, AT91C_UDP_TXCOMP);
    }
    // OUT packet received
    if (ISSET(dStatus, AT91C_UDP_RX_DATA_BK0)
        || ISSET(dStatus, AT91C_UDP_RX_DATA_BK1)) {
        // Check that the endpoint is in Read state
        if (pEndpoint[bEndpoint].dState != endpointStateRead) {

            // Endpoint is NOT in Read state
            if (ISCLEARED(dStatus, AT91C_UDP_EPTYPE)
                && ISCLEARED(dStatus, 0xFFFF0000)) {

                // Control endpoint, 0 bytes received
                // Acknowledge the data and finish the current transfer
                ClearStock = 1;
				UDP_ClearRXFlag(bEndpoint);

                pEndpoint[bEndpoint].dState = endpointStateIdle;
            }
            else if (ISSET(dStatus, AT91C_UDP_FORCESTALL)) {

                // Non-control endpoint
                // Discard stalled data
                ClearStock = 2;
				UDP_ClearRXFlag(bEndpoint);
            }
            else {
                // Non-control endpoint
                // Nak data
                SET(pUdp->UDP_IDR, 1 << bEndpoint);
            }
        }
        else {

            // Endpoint is in Read state
            // Retrieve data and store it into the current transfer buffer
            unsigned short wPacketSize = (unsigned short) (dStatus >> 16);//num of bytes available in FIFO
            UDP_GetPayload(bEndpoint, wPacketSize);//read all available data from FIFO
			//!!!
			ClearStock = 3;
            UDP_ClearRXFlag(bEndpoint);//change Bank for 2-bank EP
            if ((pEndpoint[bEndpoint].dBytesRemaining == 0)//no more data to read, close interrupt
                || (wPacketSize < pEndpoint[bEndpoint].wMaxPacketSize)) {//data read this time is not full in FIFO
                // Disable interrupt if this is not a control endpoint
                if (!ISCLEARED(dStatus, AT91C_UDP_EPTYPE)) {
                    SET(pUdp->UDP_IDR, 1 << bEndpoint);
                }
                pEndpoint[bEndpoint].dState = endpointStateIdle;
				USB_read_Callback();
            }
        }
    }
    // SETUP packet received
    if (ISSET(dStatus, AT91C_UDP_RXSETUP)) {
        // Set the DIR bit before clearing RXSETUP in Control IN sequence
        UDP_CLEAREPFLAGS(bEndpoint, AT91C_UDP_RXSETUP);
    }
    // STALL sent
    if (ISSET(dStatus, AT91C_UDP_STALLSENT)) {
        // Acknowledge the stall flag
        UDP_CLEAREPFLAGS(bEndpoint, AT91C_UDP_STALLSENT);

        // If the endpoint is not halted, clear the stall condition
        if (pEndpoint[bEndpoint].dState != endpointStateHalted) {

            UDP_CLEAREPFLAGS(bEndpoint, AT91C_UDP_FORCESTALL);
        }
    }
	
}
__irq void usb_int_handler(void)
{
    unsigned int        dStatus;
    unsigned char       bEndpoint = 4;
	unsigned char i;
	
	AT91S_UDP * pUdp = AT91C_BASE_UDP;
    dStatus = pUdp->UDP_ISR & pUdp->UDP_IMR;
//	beep(ON);
    // Handle all UDP interrupts
    while (dStatus != 0) {
        // Start Of Frame (SOF)
        if (ISSET(dStatus, AT91C_UDP_SOFINT)) {
            // Acknowledge interrupt
            SET(pUdp->UDP_ICR, AT91C_UDP_SOFINT);
            CLEAR(dStatus, AT91C_UDP_SOFINT);
        }
        // Suspend
        if (dStatus == AT91C_UDP_RXSUSP) {
            SET(pUdp->UDP_ICR, AT91C_UDP_RXSUSP);

        }
        // Resume
        else if (ISSET(dStatus, AT91C_UDP_WAKEUP)
              || ISSET(dStatus, AT91C_UDP_RXRSM)) {
            
            SET(pUdp->UDP_ICR,
                AT91C_UDP_WAKEUP | AT91C_UDP_RXRSM | AT91C_UDP_RXSUSP);
        }
        // End of bus reset
        else if (ISSET(dStatus, AT91C_UDP_ENDBUSRES)) {
            SET(pUdp->UDP_ICR,
                AT91C_UDP_WAKEUP | AT91C_UDP_RXRSM | AT91C_UDP_RXSUSP);
            // Acknowledge end of bus reset interrupt
            SET(pUdp->UDP_ICR, AT91C_UDP_ENDBUSRES);
        }
        // Endpoint interrupts
        else {
            while (dStatus != 0) {
                // Get endpoint index
				for(i=0;i<4;i++){
					if(dStatus & (1<<i)){
						bEndpoint = i;
						break;
					}
				}
				if(bEndpoint<4){
					if(bEndpoint == 0){
						UDP_EndpointHandler(bEndpoint);
					}
					else{
                		UDP_EndpointHandler(bEndpoint);
					}
                	CLEAR(pUdp->UDP_CSR[bEndpoint],
                      AT91C_UDP_TXCOMP | AT91C_UDP_RX_DATA_BK0
                    | AT91C_UDP_RX_DATA_BK1 | AT91C_UDP_RXSETUP
                    | AT91C_UDP_STALLSENT);

                	CLEAR(dStatus, 1 << bEndpoint);
				}
            }
        }

        // Retrieve new interrupt status
        dStatus = pUdp->UDP_ISR & pUdp->UDP_IMR;
    }
//	beep(OFF);
	*AT91C_AIC_ICCR |= (1 << AT91C_ID_UDP);
	*AT91C_AIC_EOICR = 1;
}

char UDP_Write(unsigned char bEndpoint,
               const void    *pData,
               unsigned int  dLength)
{
    AT91S_UDP * pUdp = AT91C_BASE_UDP;

    // Check that the endpoint is in Idle state
    if (pEndpoint[bEndpoint].dState != endpointStateIdle) {

        return USB_STATUS_LOCKED;
    }

    // Setup the transfer descriptor
    pEndpoint[bEndpoint].pData = (char *) pData;
    pEndpoint[bEndpoint].dBytesRemaining = dLength;
    pEndpoint[bEndpoint].dBytesBuffered = 0;
    pEndpoint[bEndpoint].dBytesTransferred = 0;

    // Send one packet
    pEndpoint[bEndpoint].dState = endpointStateWrite;
    UDP_WritePayload(bEndpoint);
    UDP_SETEPFLAGS(bEndpoint, AT91C_UDP_TXPKTRDY);

    // If double buffering is enabled and there is data remaining,
    // prepare another packet
    if ((pEndpoint[bEndpoint].dNumFIFO > 1) && (pEndpoint[bEndpoint].dBytesRemaining > 0)) {

        UDP_WritePayload(bEndpoint);
    }

    // Enable interrupt on endpoint
    SET(pUdp->UDP_IER, 1 << bEndpoint);

    return USB_STATUS_SUCCESS;
}

char UDP_Read(unsigned char bEndpoint,
              void          *pData,
              unsigned int  dLength)
{
    AT91S_UDP * pUdp = AT91C_BASE_UDP;
	int status;
    //! Return if the endpoint is not in IDLE state
    if (pEndpoint[bEndpoint].dState != endpointStateIdle) {

        return USB_STATUS_LOCKED;
    }


    // Endpoint enters Read state
    pEndpoint[bEndpoint].dState = endpointStateRead;

    // Set the transfer descriptor
    pEndpoint[bEndpoint].pData = (char *) pData;
    pEndpoint[bEndpoint].dBytesRemaining = dLength;
    pEndpoint[bEndpoint].dBytesBuffered = 0;
    pEndpoint[bEndpoint].dBytesTransferred = 0;

    // Enable interrupt on endpoint
    SET(pUdp->UDP_IER, 1 << bEndpoint);
	
	status = pUdp->UDP_IMR;
	status = status;
    return USB_STATUS_SUCCESS;
}




//************** Exported ******************




void USB_init(void)
{
	AT91S_UDP * pUdp = AT91C_BASE_UDP;
	AT91S_AIC * pAIC = AT91C_BASE_AIC;
	pAIC->AIC_SVR[AT91C_ID_UDP] = (unsigned long)usb_int_handler;
    pAIC->AIC_SMR[AT91C_ID_UDP] = AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | 2  ;
    //* Clear the interrupt on the interrupt controller
    pAIC->AIC_ICCR |= (1 << AT91C_ID_UDP); 
	pAIC->AIC_IECR |= (1 << AT91C_ID_UDP);
	pUdp->UDP_IDR = 0xFFFFFFFF;
//    AT91S_UDP * pUdp = AT91C_BASE_UDP;
//	AT91S_AIC * pAIC = AT91C_BASE_AIC;
	// Set the PLL USB Divider
    AT91C_BASE_CKGR->CKGR_PLLR |= AT91C_CKGR_USBDIV_1 ;
	
    
    // Specific Chip USB Initialisation
    // Enables the 48MHz USB clock UDPCK and System Peripheral USB Clock
    AT91C_BASE_PMC->PMC_SCER = AT91C_PMC_UDP;
    AT91C_BASE_PMC->PMC_PCER = (1 << AT91C_ID_UDP);

    // Enable UDP PullUp (USB_DP_PUP) : enable & Clear of the corresponding PIO
    // Set in PIO mode and Configure in Output
//    AT91F_PIO_CfgOutput(AT91C_BASE_PIOA,USB_PUP);
    // Clear for set the Pul up resistor
//    AT91F_PIO_ClearOutput(AT91C_BASE_PIOA,USB_PUP);

    // CDC Open by structure initialization
    AT91F_CDC_Open(&pCDC, AT91C_BASE_UDP);
	
}
void USB_int_init(void)
{
	AT91S_UDP * pUdp = AT91C_BASE_UDP;
	AT91S_AIC * pAIC = AT91C_BASE_AIC;
//	pAIC->AIC_SVR[AT91C_ID_UDP] = (unsigned long)usb_int_handler;
    pAIC->AIC_SMR[AT91C_ID_UDP] = AT91C_AIC_SRCTYPE_INT_HIGH_LEVEL | 2  ;
    //* Clear the interrupt on the interrupt controller
    pAIC->AIC_ICCR |= (1 << AT91C_ID_UDP); 
	pAIC->AIC_IECR |= (1 << AT91C_ID_UDP);
	pUdp->UDP_IDR = 0xFFFFFFFF;
}

char USB_wait_connect(int cnt_down)
{
	while (!pCDC.IsConfigured(&pCDC)){
		cnt_down--;
		if(cnt_down==0){
		//	return 1;
		}
//		delay_ms(50);
	}
	return 0;	
}
//int USB_armed(void)
//{
//	return (*AT91C_PIOA_PDSR & USB_VBUS);
//}
int USB_check(void)
{
	if(myusb.connect_flag==PLUG_IN){
		myusb.connect_flag = PLUG_CLEAR;
		if(!USB_wait_connect(100)){
			myusb.connected = 1;
		}
	}
	else if(myusb.connect_flag==PLUG_OUT){
		myusb.connected = 0;
	}
	if(myusb.connected)
		return 1;
	else
		return 0;
}
void USB_write_Raspberry(unsigned int time_stamp, unsigned char descriptor)
{	
	const unsigned char in_head[3]={'>','*','>'};
	unsigned char check_length;//(0~49)
	unsigned short check_sum = 0;
	unsigned short i;
	unsigned char flight_mode = mode.FlightMode;
	unsigned char usb_data2send[64];
	usb_in_pin(OFF);
	switch(descriptor){
	case 's'://states
		check_length = 62;
		memcpy(usb_data2send, in_head, 3);//uchar
		memcpy(usb_data2send + 3, &descriptor, 1);//uchar
		memcpy(usb_data2send + 4, &time_stamp, 4);//uint
		memcpy(usb_data2send + 8, &check_length, 1);//uchar
		memcpy(usb_data2send + 9, &flight_mode, 1);//uchar
		memcpy(usb_data2send + 10, pos.x_est, 8);//int*2
		memcpy(usb_data2send + 18, pos.y_est, 8);
		memcpy(usb_data2send + 26, pos.z_est, 8);
		memcpy(usb_data2send + 34, cmd.rc, 8);//short the sticks, 0,1,2,3
		memcpy(usb_data2send + 42, cmd.rc + 5, 4);//short the tunings, 5,6
		memcpy(usb_data2send + 46, att.q, 16);//int*4
		
		break;
	case 'r'://rc commands
		check_length = 28;
		memcpy(usb_data2send, in_head, 3);//uchar
		memcpy(usb_data2send + 3, &descriptor, 1);//uchar
		memcpy(usb_data2send + 4, &time_stamp, 4);//uint
		memcpy(usb_data2send + 8, &check_length, 1);//uchar
		memcpy(usb_data2send + 9, &flight_mode, 1);//uchar
		memcpy(usb_data2send + 10, cmd.rc, 18);//int*2

		break;
		
	default:
		break;
	}
	for(i=0; i<check_length; i++){
		check_sum += usb_data2send[i];
	}
	memcpy(usb_data2send + 62, &check_sum, 2);
	UDP_Write(AT91C_EP_IN,usb_data2send,64);
}
void USB_written_Callback(void)
{
//	beep(OFF);
	usb_in_pin(ON);
}
void USB_read_Raspberry(void)
{
	AT91S_UDP * pUdp = AT91C_BASE_UDP;
	int status;
	status = pUdp->UDP_ISR;
	status = status;
	UDP_Read(AT91C_EP_OUT,data_from_Raspberry,64);
	//must give up the last byte
}
void USB_read_Callback(void)
{
	myusb.out_read_flag = 1;
}
void USB_read_Process(void)
{
	short i;
	unsigned int timestamp_out;
	unsigned char check_length_out;
	unsigned char out_head[3] = {'?','?','?'};
	unsigned char descriptor_out = '?';
	unsigned short check_sum_out = 0;
	unsigned short check_sum_self_cal = 0;
//	short data4log[9];
	memcpy(out_head, data_from_Raspberry, 3);
	if(out_head[0] == '<' && out_head[1] == '#' && out_head[2] == '<' ){
		memcpy(&descriptor_out, data_from_Raspberry + 3, 1);
	}
	if(descriptor_out != '?'){
		memcpy(&timestamp_out, data_from_Raspberry + 4, 4);
		memcpy(&check_length_out, data_from_Raspberry + 8, 1);
		memcpy(&check_sum_out, data_from_Raspberry + 29, 2);
	}
	for(i=0; i < check_length_out; i++){
		check_sum_self_cal += data_from_Raspberry[i];
	}
	if(check_sum_self_cal == check_sum_out){
		switch(descriptor_out){
		case 'i':{//idle linking 
		
			break;
		}
		case 'm'://rPi is the pilot at her manuel control
		{
			int _q0_sp, _q1_sp, _q2_sp, _q3_sp;
			int _thrust_force;
			memcpy(&_q0_sp, data_from_Raspberry + 9, 4);
			memcpy(&_q1_sp, data_from_Raspberry + 13, 4);
			memcpy(&_q2_sp, data_from_Raspberry + 17, 4);
			memcpy(&_q3_sp, data_from_Raspberry + 21, 4);
			memcpy(&_thrust_force, data_from_Raspberry + 25, 4);
			ctrl.rasp_q_sp[0] = _q0_sp;
			ctrl.rasp_q_sp[1] = _q1_sp;
			ctrl.rasp_q_sp[2] = _q2_sp;
			ctrl.rasp_q_sp[3] = _q3_sp;
			ctrl.rasp_thrust = _thrust_force;
			if(cmd.data2send == sendUSB){
				data2[0]=_q0_sp;
				data2[1]=_q1_sp;	
				data2[2]=_q2_sp;				
				data2[3]=_q3_sp;
				data2[4]=_thrust_force / 10;
				data2[5]=0;
				data2[6]=0;
				data2[7]=0;
				data2[8]=0;
			}
			break;
		}
		case 'v':
		{
			int _vel_x_sp, _vel_y_sp, _vel_z_sp;
			int _yaw_sp;
			memcpy(&_vel_x_sp, data_from_Raspberry + 9, 4);
			memcpy(&_vel_y_sp, data_from_Raspberry + 13, 4);
			memcpy(&_vel_z_sp, data_from_Raspberry + 17, 4);
			memcpy(&_yaw_sp, data_from_Raspberry + 21, 4);
			break;
		}
		case 'p':
		{
			int _pos_x_sp, _pos_y_sp, _pos_z_sp;
			int _yaw_sp;
			memcpy(&_pos_x_sp, data_from_Raspberry + 9, 4);
			memcpy(&_pos_y_sp, data_from_Raspberry + 13, 4);
			memcpy(&_pos_z_sp, data_from_Raspberry + 17, 4);
			memcpy(&_yaw_sp, data_from_Raspberry + 21, 4);
			break;
		}
		default:
			break;		
		}//end of switch
		memcpy(ctrl.rasp_pos_sp, data_from_Raspberry + 32, 12);
		memcpy(ctrl.rasp_vel_ff, data_from_Raspberry + 44, 12);
	//vel_sp is a local given and used variable in controller node
	//so the linker node gets vel_sp always as 0
	}//end of if check sum okay
}
