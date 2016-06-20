#ifndef MAIN_H
#define	MAIN_H

// types of shields supported
#define SHIELD_SPARKFUN			1
#define SHIELD_SEEDSTUDIO		2

// SPI PINS (changes by shield type)
uint8_t P_CS;
uint8_t P_MISO;
uint8_t P_MOSI;
uint8_t P_SCK;
uint8_t P_SS;
//#define P_CS        2
//#define P_MISO      4
//#define P_MOSI      3
//#define P_SCK       5
//#define P_SS        4

void 			uart_init(void);
void 			spi_init(uint8_t shield);
void 			hardware_init(void);
inline uint8_t 	spi_putc(uint8_t data);
inline void 	OnLED1(void);
inline void 	OffLED1(void);
inline void 	toggleLED1(void);
inline void 	toggleLED2(void);
inline void 	uart_putchar(char c);
void 			uart_println(char *l);
void 			iprint(int n);
void 			puthexdigit(unsigned char c);
void 			puthexb(unsigned char c);
void 			puthex(unsigned char c);
void 			puthex2(unsigned int c);
unsigned char 	count_chr(unsigned char *msg, unsigned char c);
unsigned char  *get_next_val(unsigned char *msg, unsigned char c);
uint16_t 		a_to_uint16(char * ptr);
void 			parse_send(void);
void 			parse_version(void);
void 			parse_init(void);

#endif // MAIN_H
