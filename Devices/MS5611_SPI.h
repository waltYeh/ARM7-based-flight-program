#ifndef SPIBARO_H
#define SPIBARO_H
void spi_init(void);
float get_altitude(void);
void MS5611_getTemperature(void);
void MS5611_getPressure(void);
//void SPI_B0_Strobe(unsigned char strobe);
//unsigned char SPI_B0_Send_byte(unsigned char byte);
//unsigned char SPI_B0_Receive_byte(void);
//unsigned short SPI_B0_Read_16bits(unsigned char addr);
void MS5611_RESET(void);
void MS5611_init(void);
unsigned long MS5611_SPI_read_ADC(void);
//void sendorder(unsigned char order, unsigned char cs);
//unsigned char SPI_getdata(void);
//void send0data(unsigned char cs);
//void SPIenable(void);
//void SPIdisable(void);
void data2tempeture(void);
void data2pressure(void);
#define MS5611_ADC     0x00
#define CMD_MS5611_RESET 0x1E
#define CMD_MS5611_PROM_Setup 0xA0
#define CMD_MS5611_PROM_C1 0xA2
#define CMD_MS5611_PROM_C2 0xA4
#define CMD_MS5611_PROM_C3 0xA6
#define CMD_MS5611_PROM_C4 0xA8
#define CMD_MS5611_PROM_C5 0xAA
#define CMD_MS5611_PROM_C6 0xAC
#define CMD_MS5611_PROM_CRC 0xAE
#define CMD_CONVERT_D1_OSR256 0x40   // 
#define CMD_CONVERT_D2_OSR256 0x50   //
#define CMD_CONVERT_D1_OSR512 0x42   // 
#define CMD_CONVERT_D2_OSR512 0x52   //
#define CMD_CONVERT_D1_OSR1024 0x44   // 
#define CMD_CONVERT_D2_OSR1024 0x54   //
#define CMD_CONVERT_D1_OSR4096 0x48   // Maximun resolution
#define CMD_CONVERT_D2_OSR4096 0x58   // Maximun resolution



#endif


