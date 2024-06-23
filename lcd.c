/*!
 * lcd.c
 *
 *      Description: Helper file for LCD library. For Hitachi HD44780 parallel LCD
 *               in 4-bit mode. Assumes the following connections:
 *               P5.7 <-----> RS
 *               P6.7 <-----> E
 *                            R/W --->GND
 *                P4  <-----> DB
 *
 *          This module uses SysTick timer for delays.
 *
 *      Author: Rohan Malipeddi
 */

#include <msp.h>

#include "lcd.h"
#include "sysTickDelays.h"

#define NONHOME_MASK        0xFC

#define LONG_INSTR_DELAY    2000
#define SHORT_INSTR_DELAY   50

void configLCD(uint32_t clkFreq) {
    // configure pins as GPIO
    LCD_DB_PORT->SEL0 = 0;
    LCD_DB_PORT->SEL1 = 0;
    LCD_RS_PORT->SEL0 &= ~LCD_RS_MASK;
    LCD_RS_PORT->SEL1 &= ~LCD_RS_MASK;
    LCD_EN_PORT->SEL0 &= ~LCD_EN_MASK;
    LCD_EN_PORT->SEL1 &= ~LCD_EN_MASK;
    // initialize En output to Low
    LCD_EN_PORT->OUT &= ~LCD_EN_MASK;
    // set pins as outputs
    LCD_DB_PORT->DIR = 0xFF;
    LCD_RS_PORT->DIR |= LCD_RS_MASK;
    LCD_EN_PORT->DIR |= LCD_EN_MASK;

    initDelayTimer(clkFreq);
}

/*!
 * Delay method based on instruction execution time.
 *   Execution times from Table 6 of HD44780 data sheet, with buffer.
 *
 * \param mode RS mode selection
 * \param instruction Instruction/data to write to LCD
 *
 * \return None
 */
void instructionDelay(uint8_t mode, uint8_t instruction) {
    // if instruction is Return Home or Clear Display, use long delay for
    //  instruction execution; otherwise, use short delay
    if ((mode == DATA_MODE) || (instruction & NONHOME_MASK)) {
        delayMicroSec(SHORT_INSTR_DELAY);
    }
    else {
        delayMicroSec(LONG_INSTR_DELAY);
    }
}

/*!
 * Function to write instruction/data to LCD.
 *
 * \param mode          Write mode: 0 - control, 1 - data
 * \param instruction   Instruction/data to write to LCD
 *
 * \return None
 */
void writeInstruction(uint8_t mode, uint8_t instruction) {
    // set 8-bit data on LCD DB port
    LCD_DB_PORT->OUT = instruction;


    //  set RS for data or control instruction mode
    //   use bit-masking to avoid affecting other pins of port
    if (mode == DATA_MODE) {
        LCD_RS_PORT->OUT |= LCD_RS_MASK;
    } else {
        LCD_RS_PORT->OUT &= ~LCD_RS_MASK;
    }
    // pulse E to execute instruction on LCD
    // set Enable signal high
    //      use bit-masking to avoid affecting other pins of port
    LCD_EN_PORT->OUT |= LCD_EN_MASK;
    delayMicroSec(1);
    // set Enable signal low
    // use bit-masking to avoid affecting other pins of port
    LCD_EN_PORT->OUT &= ~LCD_EN_MASK;
    // delay to allow instruction execution to complete
    instructionDelay(mode, instruction);
}

/*!
 * Function to write instruction/data to LCD in 4 bit mode
 *
 * \param mode          Write mode: 0 - control, 1 - data
 * \param instruction   Instruction/data to write to LCD
 *
 * \return None
 */
void write4bitInstruction(uint8_t mode, uint8_t instruction) {
    // TODO set 4-bit data on LCD DB port
    LCD_DB_PORT->OUT = (instruction & 0xF0) + (LCD_DB_PORT->OUT & 0x0F);

    // TODO set RS for data or control instruction mode
    //      use bit-masking to avoid affecting other pins of port
    if (mode == DATA_MODE) {
        LCD_RS_PORT->OUT |= LCD_RS_MASK;
    } else {
        LCD_RS_PORT->OUT &= ~LCD_RS_MASK;
    }
    // pulse E to execute instruction on LCD
    // TODO set Enable signal high
    //      use bit-masking to avoid affecting other pins of port
    LCD_EN_PORT->OUT |= LCD_EN_MASK;
    delayMicroSec(1);
    // TODO set Enable signal low
    //      use bit-masking to avoid affecting other pins of port
    LCD_EN_PORT->OUT &= ~LCD_EN_MASK;
    // delay to allow instruction execution to complete
    delayMicroSec(5);

    LCD_DB_PORT->OUT = (LCD_DB_PORT->OUT & 0x0F) + (instruction << 4);

    // TODO set RS for data or control instruction mode
    //      use bit-masking to avoid affecting other pins of port
    if (mode == DATA_MODE) {
        LCD_RS_PORT->OUT |= LCD_RS_MASK;
    } else {
        LCD_RS_PORT->OUT &= ~LCD_RS_MASK;
    }
    // pulse E to execute instruction on LCD
    // TODO set Enable signal high
    //      use bit-masking to avoid affecting other pins of port
    LCD_EN_PORT->OUT |= LCD_EN_MASK;
    delayMicroSec(1);
    // TODO set Enable signal low
    //      use bit-masking to avoid affecting other pins of port
    LCD_EN_PORT->OUT &= ~LCD_EN_MASK;
    // delay to allow instruction execution to complete
    instructionDelay(mode, instruction);
}

/*!
 * Function to write command instruction to LCD.
 *
 * \param command Command instruction to write to LCD
 *
 * \return None
 */
void commandInstruction(uint8_t command) {
    write4bitInstruction(CTRL_MODE, command);
}

/*!
 * Function to write data instruction to LCD. Writes ASCII value to current
 *  cursor location.
 *
 * \param data ASCII value/data to write to LCD
 *
 * \return None
 */
void dataInstruction(uint8_t data) {
    writeInstruction(DATA_MODE, data);
}

void initLCD(void) {
    // follows initialization sequence described for 4-bit data mode in
    //  Figure 23 of HD447780 data sheet
    delayMilliSec(40);
    writeInstruction(CTRL_MODE, FUNCTION_SET_MASK | DL_FLAG_MASK);
    delayMilliSec(5);
    writeInstruction(CTRL_MODE, FUNCTION_SET_MASK | DL_FLAG_MASK);
    delayMicroSec(150);
    writeInstruction(CTRL_MODE, FUNCTION_SET_MASK | DL_FLAG_MASK);
    delayMicroSec(150);
    writeInstruction(CTRL_MODE, FUNCTION_SET_MASK);
    delayMicroSec(SHORT_INSTR_DELAY);
    commandInstruction(FUNCTION_SET_MASK | N_FLAG_MASK | F_FLAG_MASK);
    delayMicroSec(SHORT_INSTR_DELAY);
    commandInstruction(DISPLAY_CTRL_MASK);
    delayMicroSec(SHORT_INSTR_DELAY);
    commandInstruction(CLEAR_DISPLAY_MASK);
    delayMicroSec(SHORT_INSTR_DELAY);
    commandInstruction(ENTRY_MODE_MASK | ID_FLAG_MASK);
    delayMicroSec(LONG_INSTR_DELAY);

    // after initialization and configuration, turn display ON
    commandInstruction(DISPLAY_CTRL_MASK | D_FLAG_MASK);
}

void firstLine(void) {
    write4bitInstruction(CTRL_MODE, SET_CURSOR_MASK);
}

void secondLine(void) {
    write4bitInstruction(CTRL_MODE, SET_CURSOR_MASK | LINE2_OFFSET);
}

void printChar(char character) {
    // print ASCII \b character to current cursor position
    dataInstruction(character);
}

void putch(const char *string) {
    // print ASCII \b character to current cursor position
    while(*string != 0) {
        write4bitInstruction(DATA_MODE, *string++);
    }

}

void clearDisplay() {
    // clear the LCD display and return cursor to home position
    commandInstruction(CLEAR_DISPLAY_MASK);
}
