#include <avr/io.h>
#include <util/delay.h>
#include "main.h"
#include "mcp_2515.h"


//////////////////////////////////////////////////////////////////////
// Initialize MCP2514
// * setup CAN speed
//////////////////////////////////////////////////////////////////////
void mcp2515_init(uint16_t speed)
{	
    // MCP 2515 software reset, enter config mode    
    PORT_CS &= ~(1<<P_CS);
    spi_putc( SPI_RESET );
    _delay_ms(1);
    PORT_CS |= (1<<P_CS);       
    _delay_ms(10);

    // setup speed
    if (speed == 125)
    {
        mcp2515_write_register( CNF1, R_125_CNF1 );        
        mcp2515_write_register( CNF2, R_125_CNF2 );        
        mcp2515_write_register( CNF3, R_125_CNF3 );
    }
    if (speed == 250)
    {
        mcp2515_write_register( CNF1, R_250_CNF1 );        
        mcp2515_write_register( CNF2, R_250_CNF2 );        
        mcp2515_write_register( CNF3, R_250_CNF3 );
    }
    if (speed == 500)
    {
        mcp2515_write_register( CNF1, R_500_CNF1 );        
        mcp2515_write_register( CNF2, R_500_CNF2 );        
        mcp2515_write_register( CNF3, R_500_CNF3 );
    }
    if (speed == 1000)
    {
        mcp2515_write_register( CNF1, R_1000_CNF1 );        
        mcp2515_write_register( CNF2, R_1000_CNF2 );        
        mcp2515_write_register( CNF3, R_1000_CNF3 );
    }
    // activate RX1IE/RX0IE
    mcp2515_write_register( CANINTE, (1<<RX1IE)|(1<<RX0IE) );
    
    // clear RXB0
    mcp2515_write_register( RXB0CTRL, (1<<RXM1)|(1<<RXM0) );
    
    // clear RXB1
    mcp2515_write_register( RXB1CTRL, (1<<RXM1)|(1<<RXM0) );
    
    // clear RX Masks    
    mcp2515_write_register( RXM0SIDH, 0 );
    mcp2515_write_register( RXM0SIDL, 0 );
    mcp2515_write_register( RXM0EID8, 0 );
    mcp2515_write_register( RXM0EID0, 0 );
    
    mcp2515_write_register( RXM1SIDH, 0 );
    mcp2515_write_register( RXM1SIDL, 0 );
    mcp2515_write_register( RXM1EID8, 0 );
    mcp2515_write_register( RXM1EID0, 0 );
       
    mcp2515_write_register( BFPCTRL, 0 );   
    mcp2515_write_register( TXRTSCTRL, 0 );   

    // leave config mode 
    mcp2515_bit_modify( CANCTRL, 0xE0, 0);
}

//////////////////////////////////////////////////////////////////////
// Read MCP2515 status register
//////////////////////////////////////////////////////////////////////
uint8_t mcp2515_read_status(uint8_t type)
{
	uint8_t data;
		
	PORT_CS &= ~(1<<P_CS); 
	spi_putc(type);
	data = spi_putc(0xff);	
	PORT_CS |= (1<<P_CS);	
	return data;
}

//////////////////////////////////////////////////////////////////////
// Write MCP2515 register
//////////////////////////////////////////////////////////////////////
inline void mcp2515_write_register( uint8_t adress, uint8_t data )
{
    PORT_CS &= ~(1<<P_CS);    
    spi_putc(SPI_WRITE);
    spi_putc(adress);
    spi_putc(data);    
    PORT_CS |= (1<<P_CS);
}

//////////////////////////////////////////////////////////////////////
// Read MCP2515 register
//////////////////////////////////////////////////////////////////////
uint8_t mcp2515_read_register(uint8_t adress)
{
    uint8_t data;
        
    PORT_CS &= ~(1<<P_CS);    
    spi_putc(SPI_READ);
    spi_putc(adress);    
    data = spi_putc(0xff);      
    PORT_CS |= (1<<P_CS);   
    return data;
}

//////////////////////////////////////////////////////////////////////
// Modify one bit in MCP2515 registers
//////////////////////////////////////////////////////////////////////
void mcp2515_bit_modify(uint8_t adress, uint8_t mask, uint8_t data)
{    
    PORT_CS &= ~(1<<P_CS);   
    spi_putc(SPI_BIT_MODIFY);
    spi_putc(adress);
    spi_putc(mask);
    spi_putc(data);
    PORT_CS |= (1<<P_CS);
}

//////////////////////////////////////////////////////////////////////
// Send CAN message
//////////////////////////////////////////////////////////////////////
uint8_t mcp2515_send_message(CANMesg_t *p_message)
{
	uint8_t length = p_message->length;
    uint8_t i;
    
    mcp2515_write_register(TXB0SIDH, (uint8_t) (p_message->id>>3));
    mcp2515_write_register(TXB0SIDL, (uint8_t) (p_message->id<<5));    
 
    if (p_message->rtr)
    {
        mcp2515_write_register(TXB0DLC, (1<<RTR) | length);
    }
    else
    {        
        mcp2515_write_register(TXB0DLC, length);            
        for (i=0;i<length;i++) {
            mcp2515_write_register(TXB0D0 + i, p_message->data[i]);
        }
    }    
    PORT_CS &= ~(1<<P_CS);
    spi_putc(SPI_RTS | 0x01);
    PORT_CS |= (1<<P_CS);
    return(1);
}