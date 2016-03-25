#ifndef SPI_H
#define SPI_H
void spi_init(void);
unsigned char spi_read_write(unsigned char order, unsigned char cs, unsigned int last_tx);
unsigned short spi_write_1_read_2(unsigned char data_w, unsigned char cs);
unsigned char spi_read_reg(unsigned char cs, unsigned char reg);
unsigned char spi_write_reg(unsigned char cs, unsigned char reg, unsigned char data);
__irq void spi_int_handler(void);
void spi_dma_refill(unsigned char switcher);
void spi_fast_init(void);
void spi_dma_decode(unsigned char switcher);

#define CS_BARO 0
#define CS_MPU 2

#define IS_LAST 1
#define NOT_LAST 0

#define SPI_IDLE 0
#define GYRO_ACC 1
#define GYRO 2
#define GYRO_PTRIG 3
#define GYRO_TPGET 4
#define GYRO_TTRIG 5

#define P2GET 1
#define T2GET 2

#define GYRO_TTRIG 5
#endif
