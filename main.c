////////////////////////////////////////////////////////////////////////////////////
// can2usb Arduino/Genuino UNO R3 firmware
// version 1.1
// 1.1 - moved CAN GIE to happen after mcp_init() to avoid lockups with seedstudio shield
//
// (c) Georg Swoboda <cn@warp.at>, Robert Blaizer - All rights reserved.
////////////////////////////////////////////////////////////////////////////////////
#include <avr/io.h>
#include <util/delay.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <string.h>
#include "main.h"
#include "mcp_2515.h"


#define toggleLED1_DELAY_MS 5
#define BAUD 115200
#define USE_2X 1
#include <util/setbaud.h>
const char hexDigit[] = "0123456789ABCDEF";

#define _VERSION_STRING "can2usb-1.1"

/* CAN RX/TX Message handling */
#define  R_MSG_SIZE 96
volatile CANMesg_t r_msg[R_MSG_SIZE]; 	// can receive round robin buffer
volatile uint8_t   w_ptr;				// write pointer to RX RR buffer
volatile uint8_t   r_ptr;				// read pointer to RX RR buffer
static   CANMesg_t   tx_msg;		    // CAN message buffer used for serial TX
volatile CANMesg_t cmsg;

/* USART RX/TX Message handling */
#define  UART_MAXSTRLEN 32
volatile uint8_t uart_str_complete = 0;     // 1 .. String komplett empfangen
volatile uint8_t uart_str_count = 0;
volatile char uart_string[UART_MAXSTRLEN + 1] = "";
char buffer[UART_MAXSTRLEN + 1] = "";

//////////////////////////////////////////////////////////////////////
// Interrupt Service Routine for USART receive
//////////////////////////////////////////////////////////////////////
ISR(USART_RX_vect)
{
  unsigned char nextChar;
  
  nextChar = UDR0;
  if( uart_str_complete == 0 ) {	     
    if( nextChar != '\n' &&
        nextChar != '\r' &&
        uart_str_count < UART_MAXSTRLEN ) {
      uart_string[uart_str_count] = nextChar;
      uart_str_count++;
    }
    else {
      uart_string[uart_str_count] = '\0';
      uart_str_count = 0;
      uart_str_complete = 1;
    }
  }
}

//////////////////////////////////////////////////////////////////////
// Interrupt Service Routine for INT0 (MCP2515 receive)
//////////////////////////////////////////////////////////////////////
ISR(INT0_vect)
{
	uint8_t  status;
	uint8_t  addr;
	uint8_t  t;
	uint8_t  length;
	uint16_t id;

	toggleLED2();	
	// wrap around w_ptr
	if (w_ptr >= R_MSG_SIZE-1)	
			w_ptr=0;

	status = mcp2515_read_status(SPI_RX_STATUS);	
	if (status>>6 & 1) 
	{		
			addr = SPI_READ_RX;							
			PORT_CS &= ~(1<<P_CS);
			spi_putc(addr);			
			id  = (uint16_t) spi_putc(0xff) << 3;
			id |=            spi_putc(0xff) >> 5;	
			spi_putc(0xff);
			spi_putc(0xff);	
			length = spi_putc(0xff) & 0x0f;
			for (t=0;t<length;t++) {
				r_msg[w_ptr].data[t]=spi_putc(0xff);
			}
			r_msg[w_ptr].length = length;
			r_msg[w_ptr].id = id;			
			r_msg[w_ptr].used = 1;			
			PORT_CS |= (1<<P_CS);					
			mcp2515_bit_modify(CANINTF, (1<<RX0IF), 0);	
			w_ptr++;	
	}
	if (status>>7 & 1) 
	{			
			addr = SPI_READ_RX | 0x04;				
			PORT_CS &= ~(1<<P_CS);
			spi_putc(addr);			
			id  = (uint16_t) spi_putc(0xff) << 3;
			id |=            spi_putc(0xff) >> 5;	
			spi_putc(0xff);
			spi_putc(0xff);	
			length = spi_putc(0xff) & 0x0f;
			for (t=0;t<length;t++) {
				r_msg[w_ptr].data[t]=spi_putc(0xff);
			}
			r_msg[w_ptr].length = length;
			r_msg[w_ptr].id = id;
			r_msg[w_ptr].used = 1;
			PORT_CS |= (1<<P_CS);			
			mcp2515_bit_modify(CANINTF, (1<<RX1IF), 0);				
			w_ptr++;						
	}		
}

//////////////////////////////////////////////////////////////////////
// Initialize UART
//////////////////////////////////////////////////////////////////////
void uart_init(void) 
{
    UBRR0H = UBRRH_VALUE;
    UBRR0L = UBRRL_VALUE;

#if USE_2X
    UCSR0A |= _BV(U2X0);
#else
    UCSR0A &= ~(_BV(U2X0));
#endif
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); /* 8-bit data */
    UCSR0B = _BV(RXEN0) | _BV(TXEN0) |(1<<RXCIE0);   /* Enable RX and TX + RX Interrupt */  
}

//////////////////////////////////////////////////////////////////////
// Initialize SPI
//////////////////////////////////////////////////////////////////////
void spi_init(uint8_t shield)
{ 
  // assign P_<XXX> vars depending on shield type
  if (shield == 1)
  {
	P_CS   = SPARKFUN_CS;
	P_MISO = SPARKFUN_MISO;     
	P_MOSI = SPARKFUN_MOSI;
	P_SCK  = SPARKFUN_SCK;
	P_SS   = SPARKFUN_SS;
  }
  if (shield == 2)
  {
	P_CS   = SEEDSTUDIO_CS;
	P_MISO = SEEDSTUDIO_MISO;     
	P_MOSI = SEEDSTUDIO_MOSI;
	P_SCK  = SEEDSTUDIO_SCK;
	P_SS   = SEEDSTUDIO_SS;
  }
  // SPI pin assignments
  DDRB |= (1<<P_SCK)|(1<<P_MOSI)|(1<<P_CS)|(1<<P_SS);
  PORTB |= (1<<P_CS);
  PORTB |= (1<<P_SS);
  // SPI speed settings
  SPCR = (1<<SPE)|(1<<MSTR)|(0<<SPR1)|(0<<SPR0);
  SPSR |= (1<<SPI2X); 
}

//////////////////////////////////////////////////////////////////////
// send single byte via SPI
//////////////////////////////////////////////////////////////////////
inline uint8_t spi_putc( uint8_t data )
{    
			
    SPDR = data;          
    while( !( SPSR & (1<<SPIF) ) );  
    return SPDR;
}

//////////////////////////////////////////////////////////////////////
// initialize hardware
// set LED pins, INT input from CAN Shield (+Pullup)
// init SPI, UART, MCP2515
// enable CAN interrupts
//////////////////////////////////////////////////////////////////////
void hardware_init(void)
{	 
 	DDRD  |= _BV(DDD7); 	// LED1 on CAN shield (output)
 	DDRB  |= _BV(DDB0); 	// LED2 on CAN shield (output)
 	DDRD  &= ~_BV(DDD2); 	// MPC2515 INT CAN shield (input)
 	PORTD |= _BV(PORTD2); 	// turn On the Pull-up	
	uart_init();
	wdt_enable(WDTO_500MS);	
	sei();					//Enable Global Interrupt	
}


inline void OnLED1(void)
{	
	PORTD |= _BV(PORTD7);	
}
inline void OffLED1(void)
{	
	PORTD &= ~_BV(PORTD7);
}

//////////////////////////////////////////////////////////////////////
// toggle LED1
//////////////////////////////////////////////////////////////////////
inline void toggleLED1(void)
{
	PORTD ^= _BV(PORTD7);
}

//////////////////////////////////////////////////////////////////////
// toggleLED2 Pin
//////////////////////////////////////////////////////////////////////
inline void toggleLED2(void)
{	
	PORTB ^= _BV(PORTB0);	
}

//////////////////////////////////////////////////////////////////////
// send one char via UART
//////////////////////////////////////////////////////////////////////
inline void uart_putchar(char c) 
{
    // loop_until_bit_is_set(UCSR0A, UDRE0); /* Wait until data register empty. */    
    while( ( UCSR0A & ( 1 << UDRE0 ) ) == 0 ){}
    UDR0 = c;   
}

void uart_println(char *l)
{
	uint8_t i=0;

	while (l[i]!=0)
	  uart_putchar(l[i++]);
	uart_putchar('\r');
	uart_putchar('\n');	
}

void iprint(int n)
  { 
    if( n > 9 )
      { int a = n / 10;
        n -= 10 * a;
        iprint(a);
      }
    uart_putchar('0'+n);
  }

void puthexdigit(unsigned char c)
{	
    uart_putchar(hexDigit[c]);
}

void puthexb(unsigned char c)
{	
    puthexdigit(c & 0x0F);
}


void puthex(unsigned char c)
{	
    puthexdigit(c >> 4);
    puthexdigit(c & 0x0F);
}
 
void puthex2(unsigned int c)
{	
	puthexdigit((c >> 8) & 0x0F);	
    puthexdigit((c >> 4) & 0x0F);
    puthexdigit(c & 0x0F);
}

//////////////////////////////////////////////////////////////////////
// helper function - count number of "c" within "*msg"
//////////////////////////////////////////////////////////////////////
unsigned char count_chr(unsigned char *msg, unsigned char c)
{
    unsigned char i=0,cnt=0;
        
    while (msg[i] != 0)
    {          
       if (msg[i++]==c) cnt++;    
    }
    return(cnt);
}

//////////////////////////////////////////////////////////////////////
// get next value
//////////////////////////////////////////////////////////////////////
unsigned char *get_next_val(unsigned char *msg, unsigned char c)
{
    unsigned char i=0,cnt=0;
        
    while (msg[i] != 0)
    {          
       if (msg[i]==c) return(&msg[i]);
       i++;
    }
    return(0);
}

//////////////////////////////////////////////////////////////////////
// parse hex val ascii to uint16_t
//////////////////////////////////////////////////////////////////////
uint16_t a_to_uint16(char * ptr)
{
    uint16_t val = 0;
    char c;
    
    while ((c = *ptr++) && (c != ','))
    {
        if (c)         // C library function
        {
            val *= 16UL;
            val += (c > '9')? (c &~ 0x20) - 'A' + 10: (c - '0');

        }
    }
    return val;
}

//////////////////////////////////////////////////////////////////////
// parse dec val ascii to uint16_t
//////////////////////////////////////////////////////////////////////
uint16_t a_to_dec_uint16(char * ptr)
{
    uint16_t val = 0;
    char c;
    
    while ((c = *ptr++) && (c != ','))
    {
        if (c)         // C library function
        {
            val *= 16UL;
            val += (c > '9')? (c &~ 0x20) - 'A' + 10: (c - '0');

        }
    }
    return val;
}

//////////////////////////////////////////////////////////////////////
// parse "CAN send message" and send CAN frame
// format:
//          $S,<len>,<id>,<byte0>,...<byte7>    (hex values without 0x)
//////////////////////////////////////////////////////////////////////
void parse_send(void)
{
    unsigned char  c_count,i;     // , count
    char  *b;
    char *ptr;    
    unsigned int    can_id;
    unsigned char   can_len;    

    char buf[8];

    // check format        
    c_count=count_chr(&uart_string, ',');    
    if ((c_count < 3) || (c_count> 10))
    {                    
       // uart_println(S_FAIL);
        return;
    }
    // get CAN LEN    
    can_len = uart_string[3]-0x30;    
    if (can_len<1 || can_len>8 || can_len!=c_count-2)
    {            	
       // uart_println(S_FAIL);
        return;
    }    
    // get CAN ID
    b=get_next_val(uart_string,',')+3;    
    can_id = a_to_uint16(b);        
    if (can_id>2048 || can_id==0x0)
    {        
      //  uart_println(S_FAIL);
        return;
    }    
    // get CAN DATA Bytes
    for (i=0;i<can_len;i++)
    {       
    	b=get_next_val(b, ',')+1;
    	buf[i]=a_to_uint16(b);        
    }    
    cli();
    cmsg.id=can_id;
    cmsg.length=can_len;
    memcpy(&cmsg.data, &buf, 8);
    cmsg.used=1;
    sei();
    //uart_println(S_OK);                     
}

//////////////////////////////////////////////////////////////////////
// parse "VERSION" and show version information
// calls: 
// format:
//          $VER
//////////////////////////////////////////////////////////////////////
void parse_version(void)
{	
    sprintf(buffer, "$VER,%s", _VERSION_STRING);        
    uart_println(buffer);
}

//////////////////////////////////////////////////////////////////////
// parse "Init CAN" 
// format:
//          $I,<speed>,<shield-type>
//
//			<speed>  = 125,250,500,1000 (Kbit CAN speed)
//			<shield> = 1 (SparkFun)
//			<shield> = 2 (SeedStudio)
//////////////////////////////////////////////////////////////////////
void parse_init(void)
{
    unsigned char  c_count,i;     // , count
    char  *b;
    char *ptr;    
    uint16_t speed=0;
    uint8_t  shield=0;
  

    // check format        
    c_count=count_chr(&uart_string, ',');    
    if (c_count != 2)
    {                           
        return;
    }
 	// <speed>
    b=get_next_val(uart_string,',')+1;         
    speed = atoi(b);
    b=get_next_val(b,',')+1;        
    shield = atoi(b);

    // check for valid shield type and speed setting
    if (
    	((shield == SHIELD_SPARKFUN) || (shield == SHIELD_SEEDSTUDIO)) 
    	 &&
    	((speed == 125) || (speed == 250) || (speed == 500) || (speed == 1000))
       )
    {
    	spi_init(shield);
    	mcp2515_init(speed);   
      // enable INT0 (CANINT)
      EICRA |= _BV(ISC11);    // set INT0 
      EIMSK |= _BV(INT0);     // Turn on INT0
   	}
   	return;
}

//////////////////////////////////////////////////////////////////////
// main loop
// initialzed hardware (CAN, SPI, SERIAL)
// loop forever doing
//   * check incoming cmds from serial
//   * handle full TX Serial buffers, and send them via CAN
//   * handle full RX CAN buffers, and send them via Serial
//////////////////////////////////////////////////////////////////////
int main (void)
{	
	uint16_t idle=0;
	uint8_t i,j;
	uint8_t len_id;
	uint16_t crc;
	////////////////////////////////////
    // initalize RR buffer for CAN RX
    ////////////////////////////////////
	r_ptr=0;
	w_ptr=0;	
	for (i=0;i<R_MSG_SIZE-1;i++)
		r_msg[i].used=0;
	cmsg.used=0;
	tx_msg.used=0;	
  hardware_init();
  while(1) 
	{					
		wdt_reset();		
		if (uart_str_complete && cmsg.used==0)
		{							
				if (strstr(uart_string, "$VER")) parse_version();
				if (strstr(uart_string, "$S"))   parse_send();
				if (strstr(uart_string, "$I"))   parse_init();
				toggleLED1();
				uart_str_complete=0;
		}		
		cli();
		///////////////////////////////
		// handle full CAN TX Msg buffer
		///////////////////////////////
		if (cmsg.used)
		{
			   //OnLED1();  		
			   mcp2515_send_message(&cmsg);
			   //OffLED1();  		
			   cmsg.used=0;
	    }
		if (r_msg[r_ptr].used)
		{
			
			memcpy(&tx_msg, &r_msg[r_ptr], sizeof(CANMesg_t));
			r_msg[r_ptr].used=0;	
			sei();		
			 //if ((tx_msg.id != 0x400) && (tx_msg.id != 0x401))
			//if (tx_msg.id == 0x401)
			//{
				//////////////////////////////////////////////////////////////////////
			 	// send received CAN Frame via UART
			    // $F<len|id><id><d0><d1><d2><d3><d4><d5><d6><d7><crc><CR><LF>
				// <len|id> = 4bit LEN, 4 MSB of ID
				// <crc> = sum over <len>,<id>,<d0-7>
				//////////////////////////////////////////////////////////////////////
		        crc=0;
				uart_putchar('$');
				uart_putchar('F');			    
			    len_id = (tx_msg.length << 4) | ((tx_msg.id >> 8) & 0xF);			   								
				// len nibble + msb id nibble
				uart_putchar(len_id);
				// id lower byte
	    		uart_putchar(tx_msg.id  & 0xFF);
	    		crc += len_id;
	    		crc += tx_msg.id;
				// data x bytes
				for (j=0;j<tx_msg.length;j++)		
				{							
					uart_putchar(tx_msg.data[j]);
					crc +=tx_msg.data[j];
				}							
				// send crc		
				uart_putchar(crc&0xFF);
				uart_putchar('\r');
				uart_putchar('\n');	
			//}			
		}		
		sei();
		
		// wrap around r_ptr
		// increase idle counter
		cli();
		if (r_ptr++ >= R_MSG_SIZE-1)	
		{
		//	idle++;		
			r_ptr=0;
		}	
		sei();	
	}
}
