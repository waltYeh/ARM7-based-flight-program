#ifndef EEPROM_H
#define EEPROM_H

#define EEPROM_WRITE_ADDRESS 0xA0
#define EEPROM_READ_ADDRESS 0xA1
#define EEPROM_ADDRESS 0x50
#define PAGE_SIZE 64
#define PAGE_0 (0x0000 << 6)
#define PAGE_1 (0x0020 << 6)
#define PAGE_2 (0x0040 << 6)
#define PAGE_3 (0x0060 << 6)
#define PAGE_4 (0x0080 << 6)
#define PAGE_5 (0x00A0 << 6)
#define PAGE_6 (0x00C0 << 6)
#define PAGE_7 (0x00E0 << 6)

int eeprom_write(void *data, unsigned char len, unsigned short page, unsigned char word);
int eeprom_read(char *buffer, unsigned char len, unsigned short page, unsigned char word);


#endif
