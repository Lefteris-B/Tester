/* ************************************************************************
 *
 *   driver functions for HD44780 compatible character displays
 *   - 4 bit parallel interface
 *
 *   (c) 2015-2016 by Markus Reschke
 *
 * ************************************************************************ */

/*
 *  hints:
 *  - pin assignment for 4 bit parallel
 *    DB4    LCD_PORT Bit #0
 *    DB5    LCD_PORT Bit #1
 *    DB6    LCD_PORT Bit #2
 *    DB7    LCD_PORT Bit #3
 *    RS     LCD_RS
 *    R/W    Gnd
 *    E      LCD_EN1
 *  - write only since R/W  is hardwired to Gnd
 *  - max. clock for interface: 2 MHz
 */


/* local includes */
#include "config.h"           /* global configuration */

#ifdef LCD_HD44780


/*
 *  local constants
 */

/* source management */
#define LCD_DRIVER_C


/*
 *  include header files
 */

/* local includes */
#include "common.h"           /* common header file */
#include "variables.h"        /* global variables */
#include "functions.h"        /* external functions */
#include "HD44780.h"          /* HD44780 specifics */

/* fonts */
#include "font_HD44780_int.h" /* standard international */



/* ************************************************************************
 *   low level functions for 4 bit parallel interface
 * ************************************************************************ */


#ifdef LCD_PAR_4

/*
 *  create enable pulse
 *  - LCD needs a pulse to take in data for processing
 */

void LCD_EnablePulse(void)
{
   LCD_PORT |= (1 << LCD_EN1);     /* set enable bit */

   /* the LCD needs some time */
   /* if required adjust time according to LCD's datasheet */
   wait10us();

   LCD_PORT &= ~(1 << LCD_EN1);    /* unset enable bit */
}



/*
 *  set up interface bus
 *  - should be called at firmware startup
 */

void LCD_BusSetup(void)
{
  /* set port pins to output mode */
  LCD_DDR = LCD_DDR | 0x0F | (1 << LCD_RS) | (1 << LCD_EN1);

  /* LCD_EN1 should be low by default */
}



/*
 *  send a nibble to the LCD (4 bit mode)
 *
 *  requires:
 *  - nibble value to send
 */

void LCD_SendNibble(uint8_t Nibble)
{
  uint8_t           Data;

  Data = LCD_PORT;            /* get current bitmask */
  /* reset all 4 data lines */
  Data &= ~((1 << LCD_DB4) | (1 << LCD_DB5) | (1 << LCD_DB6) | (1 << LCD_DB7));

  #ifdef LCD_DB_STD
    /* standard pins: simply take nibble */
    Data |= Nibble;
  #else
    /* non-standard pins: set bits individually */
    if (Nibble & 0b00000001) Data |= (1 << LCD_DB4);
    if (Nibble & 0b00000010) Data |= (1 << LCD_DB5);
    if (Nibble & 0b00000100) Data |= (1 << LCD_DB6);
    if (Nibble & 0b00001000) Data |= (1 << LCD_DB7);
  #endif

  LCD_PORT = Data;            /* set nibble */

  /* give LCD some time */
  #if CPU_FREQ < 2000000
    _delay_us(5);
  #else
    wait5us();
  #endif

  LCD_EnablePulse();          /* trigger LCD */
}



/*
 *  send a byte (data or command) to the LCD
 *  - bit-bang 8 bits as two nibbles (MSB first, LSB last)
 *
 *  requires:
 *  - byte value to send
 */

void LCD_Send(uint8_t Byte)
{
  uint8_t           Nibble;

  /*
   *  send upper nibble (bits 4-7)
   */

  Nibble = (Byte >> 4) & 0x0F;          /* get upper nibble */
  LCD_SendNibble(Nibble);


  /*
   *  send lower nibble (bits 0-3)
   */

  Nibble = Byte & 0x0F;                 /* get lower nibble */
  LCD_SendNibble(Nibble);

  wait50us();            /* LCD needs some time for processing */


  /*
   *  clear data lines on port
   */  

  Nibble = ~((1 << LCD_DB4) | (1 << LCD_DB5) | (1 << LCD_DB6) | (1 << LCD_DB7));
  LCD_PORT &= Nibble;         /* clear port */
}



/*
 *  send a command to the LCD
 *
 *  requires:
 *  - byte value to send
 */
 
void LCD_Cmd(uint8_t Cmd)
{
  /* indicate command mode */
  LCD_PORT &= ~(1 << LCD_RS);    /* set RS to 0 */

  LCD_Send(Cmd);                 /* send command */
}



/*
 *  display a single character
 *
 *  requires:
 *  - Char: character to display
 */

void LCD_Char(unsigned char Char)
{
  uint8_t           *Table;        /* pointer to table */
  uint8_t           ID;            /* char ID */  

  /* get character ID from lookup table */
  Table = (uint8_t *)&FontTable;        /* start address */
  Table += Char;                        /* add offset for character */
  ID = pgm_read_byte(Table);            /* get ID number */
  if (ID == 0xff) return;               /* no character available */

  /* indicate data mode */
  LCD_PORT |= (1 << LCD_RS);       /* set RS to 1 */
 
  LCD_Send(ID);                    /* send character ID */

  /* update character position */
  UI.CharPos_X++;                  /* next character in current line */
  /* LCD's RAM address already points to next character */
}

#endif



/* ************************************************************************
 *   high level functions
 * ************************************************************************ */


/*
 *  clear the display 
 */ 

void LCD_Clear(void)
{
  LCD_Cmd(CMD_CLEAR_DISPLAY);      /* send clear command */
  MilliSleep(2);                   /* LCD needs some time for processing */

  /* reset character position */
  UI.CharPos_X = 1;
  UI.CharPos_Y = 1;
}



/*
 *  load a custom character into LCD module
 *
 *  requires:
 *  - pointer of fixed character data
 *  - ID for custom character (0-7)
 */

void LCD_CustomChar(uint8_t ID)
{
  uint8_t      i;                  /* counter */
  uint8_t      Byte;               /* data byte */
  uint8_t      *Table;             /* pointer to char data */

  /* set data start address */
  Table = (uint8_t *)&FontData;         /* start address */
  Table += (ID * 8);                    /* add offset for character */


  /*
   *  set CG RAM start address (for a 5x8 character)
   *  - lower 3 bits determine the row in a character
   *  - higher 3 bits determine the start (ID) of the character
   *  - so we have to shift the ID to the higher part
   *  - LCD module supports up to 8 custom characters (5x8 font)
   */

  LCD_Cmd(CMD_SET_CG_RAM_ADDR | (ID << 3));

  /* indicate data mode */
  LCD_PORT |= (1 << LCD_RS);       /* set RS to 1 */

  /* write custom character */
  for (i = 0; i < 8; i++)               /* 8 bytes */
  {
    Byte = pgm_read_byte(Table);        /* read byte */
    LCD_Send(Byte);                     /* send byte */

    Table++;                            /* next byte */
  }
}



/*
 *  initialize LCD
 *  - for 4bit mode
 */
 
void LCD_Init(void)
{
  /*
   *  first we have to send three times:
   *  - RS and R/W unset
   *  - DB4 and DB5 set
   */

  /* round #1 */
  MilliSleep(30);
  LCD_PORT = (LCD_PORT & 0xF0 & ~(1 << LCD_RS)) | 0x03;
  LCD_EnablePulse();

  /* round #2 */
  MilliSleep(5);
  LCD_EnablePulse();

  /* round #3 */
  MilliSleep(1);
  LCD_EnablePulse();


  /*
   *  set modes
   */

  /* init 4 bit mode  */
  MilliSleep(1);
  LCD_PORT = (LCD_PORT & 0xF0 & ~(1 << LCD_RS)) | 0x02;
  MilliSleep(1);
  LCD_EnablePulse();
  MilliSleep(1);

  /* function set: 4 bit interface / 2 rows / font 5x7 */
  LCD_Cmd(CMD_FUNCTION_SET | 0x08);

  /* display: display on / cursor off / no blinking */
  LCD_Cmd(CMD_DISPLAY_CONTROL | 0x04);

  /* entry mode: increment cursor position / no scrolling */    
  LCD_Cmd(CMD_ENTRY_MODE_SET | 0x02);


  /*
   *  load custom characters
   */

  /* todo: why not a simple loop? */

  /* custom symbols for components */
  LCD_CustomChar(LCD_CHAR_DIODE_AC);    /* diode symbol '|>|' */
  LCD_CustomChar(LCD_CHAR_DIODE_CA);    /* diode symbol '|<|' */
  LCD_CustomChar(LCD_CHAR_CAP);         /* capacitor symbol '||' */
  LCD_CustomChar(LCD_CHAR_RESISTOR_L);  /* resistor symbol '[' */
  LCD_CustomChar(LCD_CHAR_RESISTOR_R);  /* resistor symbol ']' */

  /* kyrillish LCD character set lacks omega and µ */
  LCD_CustomChar(LCD_CHAR_OMEGA);       /* Omega */
  LCD_CustomChar(LCD_CHAR_MICRO);       /* µ / micro */


  /* and clear display */
  LCD_Clear();

  /* update character maximums */
  UI.CharMax_X = LCD_CHAR_X;
  UI.CharMax_Y = LCD_CHAR_Y;
}



/*
 *  set LCD character position
 *  - since we can't read the LCD and don't use a RAM buffer
 *    we have to move page-wise in y direction
 *  - top left: 0/0
 *
 *  requires:
 *  - x:  horizontal position 
 *  - y:  vertical position
 */

void LCD_CharPos(uint8_t x, uint8_t y)
{
  uint8_t           Address = 0;        /* RAM address */

  /* update character position */
  UI.CharPos_X = x;
  UI.CharPos_Y = y;

  /*
   *  vertical position
   *
   *  Line 1: 0x00
   *  Line 2: 0x40
   *  Line 3: 0x14
   *  Line 4: 0x54
   */

  y--;                             /* for binary magic */

  /* line 3 or 4: base starts at 20 bytes */
  if (y & 0b00000010) Address = 0x14;

  /* line 2 or 4: base + 20 bytes */
  if (y & 0b00000001) Address += 0x40;


  /*
   *  horizontal position
   *
   *  line address + horizontal position
   */

  x--;                             /* address starts at 0 */
  Address += x;                    /* add x position */

  /* set RAM address */
  LCD_Cmd(CMD_SET_DD_RAM_ADDR | Address);    /* send command */
}



/*
 *  clear one single character line
 *
 *  requires:
 *  - Line: line number (1-4)
 *    special case line 0: clear remaining space in current line
 */ 

void LCD_ClearLine(uint8_t Line)
{
  uint8_t           n = 0;         /* counter */

  if (Line == 0)         /* special case: rest of current line */
  {
    n = UI.CharPos_X;         /* current character position */
    n--;                      /* starts at 0 */
  }
  else                   /* standard case: some line */
  {
    LCD_CharPos(1, Line);     /* go to beginning of line */
  }

  while (n < 20)              /* up to 20 bytes */
  {
    LCD_Char(' ');            /* send space */
    n++;                      /* next one */
  }
}



/*
 *  set cursor
 *
 *  required:
 *  - Mode: cursor mode
 *    0: cursor on
 *    1: cursor off
 */

void LCD_Cursor(uint8_t Mode)
{
  uint8_t           Command;

  LCD_CharPos(LCD_CHAR_X, LCD_CHAR_Y);       /* move to bottom right */

  /* default: cursor off */
  Command = CMD_DISPLAY_CONTROL | FLAG_DISPLAY_ON | FLAG_CURSOR_OFF;

  /* enable cursor if requested */
  if (Mode) Command |= FLAG_CURSOR_ON;

  LCD_Cmd(Command);           /* send command */
}



/* ************************************************************************
 *   clean-up of local constants
 * ************************************************************************ */

/* source management */
#undef LCD_DRIVER_C

#endif

/* ************************************************************************
 *   EOF
 * ************************************************************************ */
