/*!
 * lcd.h
 *
 *      Description: Helper file for LCD library. For Hitachi HD44780 parallel LCD
 *               in 4-bit mode. Assumes the following connections:
 *               P2.7 <-----> RS
 *               P2.6 <-----> E
 *                            R/W --->GND
 *                P4  <-----> DB
 *
 *          This module uses SysTick timer for delays.
 *
 *      Author: Rohan Malipeddi
 */

#ifndef LCD_H_
#define LCD_H_

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#include <msp.h>

#define LCD_DB_PORT         P4
#define LCD_RS_PORT         P5
#define LCD_EN_PORT         P6    // my pin 5.6 is damaged
#define LCD_RS_MASK         BIT7
#define LCD_EN_MASK         BIT7  // same as above

#define CTRL_MODE           0
#define DATA_MODE           1
#define LINE1_OFFSET        0x0
#define LINE2_OFFSET        0x40

/* Instruction masks */
#define CLEAR_DISPLAY_MASK  0x01
#define RETURN_HOME_MASK    0x02
#define ENTRY_MODE_MASK     0x04
// TODO complete values for instruction masks
#define DISPLAY_CTRL_MASK   0x08
#define CURSOR_SHIFT_MASK   0x10
#define FUNCTION_SET_MASK   0x20
#define SET_CGRAM_MASK      0x40
#define SET_CURSOR_MASK     0x80

/* Field masks for instructions:
 * DL   = 1: 8 bits, DL = 0: 4 bits
 * N    = 1: 2 lines, N = 0: 1 line
 * S/C  = 1: Display shift
 * S/C  = 0: Cursor move
 * F    = 1: 5 x 10 dots, F = 0: 5 x 8 dots
 * R/L  = 1: Shift to the right
 * R/L  = 0: Shift to the left
 * D    = 1: Display On, D = 0: Display Off
 * C    = 1: Cursor On, D = 0: Cursor Off
 * I/D  = 1: Increment
 * I/D  = 0: Decrement
 * B    = 1: Cursor blink On, D = 0: Cursor blink Off
 * S    = 1: Accompanies display shift
 * BF   = 1: Internally operating
 * BF   = 0: Instructions acceptable
 */
#define DL_FLAG_MASK        0x10    // 4-bit mode
#define N_FLAG_MASK         0x08    // 2 lines
#define SC_FLAG_MASK        0x08    // shifting the display
// TODO complete values for instruction field masks
#define F_FLAG_MASK         0x4
#define RL_FLAG_MASK        0x4
#define D_FLAG_MASK         0x4
#define C_FLAG_MASK         0x2
#define ID_FLAG_MASK        0x2
#define B_FLAG_MASK         0x1
#define S_FLAG_MASK         0x1

/*!
 *
 *  \brief This function configures the selected pins for an LCD
 *
 *  This function configures the selected pins as output pins to interface
 *      with a Hitachi HD44780 LCD in 8-bit mode. Also initializes sysTickDelay
 *      library based on system clock frequency.
 *
 *  \param clkFreq is the frequency of the system clock (MCLK) in Hz
 *
 *  Modified bits of \b P2DIR register and \b P4DIR register, and bits of
 *      \b P2SEL and \b P4SEL registers.
 *
 *  \return None
 */
extern void configLCD(uint32_t clkFreq);

/*!
 *  \brief This function initializes LCD
 *
 *  This function generates initialization sequence for LCD for 8-bit mode.
 *      Delays set by worst-case 2.7 V
 *
 *  \return None
 */
extern void initLCD(void);

/*!
 *  \brief This function prints character to current cursor position
 *
 *  This function prints ASCII character to current cursor position on LCD.
 *
 *  \param character is the character to display on LCD
 *
 *  \return None
 */
extern void printChar(char character);

/*!
 * puts the cursor at the start of the first line
 */
extern void firstLine(void);

/*!
 * puts the cursor at the start of the second line
 */
extern void secondLine(void);

/*!
 * prints character in 4 bit mode
 */
extern void putch(const char *string);

/*!
 * clears the display
 */
extern void clearDisplay(void);
//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif /* LCD_H_ */
