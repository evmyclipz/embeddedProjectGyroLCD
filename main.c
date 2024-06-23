/*! \file */
/******************************************************************************
 * File Name: ece230_2122s_project5MalipeddiR.c
 * Author : Rohan Malipeddi
 * Last Modified: 04/28/24
 *
 * Description: Using UART and I2C serial communication and parallel to display
 * gyro value onto the LCD upon prompt in terminals.
 *
 * An external HF crystal between HFXIN & HFXOUT is required for MCLK,SMCLK.
 *
 *                                 ____  ___
 *                                   |    |
 *               MSP432P411x        10k  10k     MPU6050
 *             ------------------    |    |    -----------
 *         /|\|     P1.6/UCB0SDA |<--|----|-->| SDA
 *          | |                  |   |        |
 *          --|RST               |   |        |
 *            |     P1.7/UCB0SCL |<--|------->| SCL
 *            |                  |            |
 *            |                  |
 *            |                  |
 *            |     P1.3/UCA0TXD |----> PC (echo)
 *            |     P1.2/UCA0RXD |<---- PC
 *            |                  |
 *            |                  |
 *            |                  |       --------
 *            |             P5.7 |----->| RS
 *            |                  |      |
 *            |             P6.7 |----->| En
 *            |                  |      |
 *            |                  |  8   |
 *            |              P4  |--\-->| DB
 *            |                  |      |
 *            |                  |       --------
 *            |             PJ.2 |------
 *            |                  |     |
 *            |                  |    HFXT @ 48MHz
 *            |                  |     |
 *            |             PJ.3 |------
 *
*******************************************************************************/


#include "msp.h"

/*Standard includes */
#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include "csHFXT.h"
#include "lcd.h"
#include "sysTickDelays.h"


#define NUM_OF_REC_BYTES        6       // number of bytes to receive from sensor read
/* update peripheral address - 0b110100X */
#define GY521_ADDRESS           0x69    // I2C address of GY-521 sensor
/* update register addresses   */
#define ACCEL_BASE_ADDR         0x3B    // base address of accelerometer data registers
#define SAMPLE_RATE_DIV_ADDR    0x19    // address of the sample rate divider register which is 8 bits
#define PWR_MGMT_ADDR           0x6B    // address of power management register
#define CLK_FREQUENCY           48000000    // MCLK using 48MHz HFXT

//prompts in the program
const char prompt[] = "\n\rOptions:\n\r (P)rint sensor values\n\r display (X)-axis values\n\r display (Y)-axis values\n\r display (Z)-axis values\n\r (2)g sensor range\n\r (4)g sensor range\n\r (8)g sensor range\n\r 1(6)g sensor range\n\rSelection: ";
const char invalid[] = "\n\rInvalid value\n\r";

//interrupt variables
uint8_t RXData[NUM_OF_REC_BYTES] = {0, 0, 0, 0, 0, 0};
uint8_t RXDataPointer, TXDataPointer, input;
volatile int16_t accel_x, accel_y, accel_z;

//global variables
bool p = false;
bool validity = false;
float prekky = 4096;
int g = 8;
char pref;

/*!
 * This function prints \a message over UART. Assumes configuration
 *  of eUSCI_A0 for UART transmit.
 *
 *  \param message Character string message to print
 *  \param msgLength Length of message to print
 *
 * Modified \b eUSCI_A0 TXBUF register.
 *
 * \return None
 */
void printMessage(const char* message, int msgLength);

/*!
 * This function configures the CS register for right clock
 */
void configUART();

/*!
 * This function configures EUSCI_B0 register to I2C mode
 */
void configI2C();

// function declaration
void delay5(void);

/*!
 * func declaration for precision setting
 */
void setPrecision();

/*!
 * func declaration for printing output
 */
void pushingP();

/**
 * main.c
 */
void main(void)
{

	WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;		// stop watchdog timer

	/*  configs:
	 *      -> HFXT clock config
	 *      -> LCD config with CLK_frequency
	 *      -> Initialization of LCD
	 *      -> UART (both)
	 *          -> UART pins configuration
	 *              ->Terminal pins: 1.3 and 1.2
 	 *      -> Note: Terminal uses EUSCI_A0 and gyro uses EUSCI_B0
 	 *      -> I2C config (gyro only)
 	 *          -> Gyro pins: 1.6 and 1.7
	 */
	configHFXT();
	configLCD(CLK_FREQUENCY);
	initLCD();
	configUART();
	configI2C();

	// Enable global interrupt
	__enable_irq();

	// print prompt to terminal over UART
	printMessage(prompt, (sizeof(prompt)/sizeof(prompt[0])));

	// Start the actual program
	while(1)
	{

	    delay5();
	    // Ensure stop condition got sent
        while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);

        /* Read register values from sensor by sending register address and restart
         *
         *  format for Write-Restart-Read operation
         *  _______________________________________________________________________
         *  |       | Periph | <Register |       | Periph |               |       |
         *  | Start |  Addr  |  Address> | Start |  Addr  | <6 Byte Read> | Stop  |
         *  |_______|____W___|___________|_______|____R___|_______________|_______|
         *
         *
         *  Initiated with start condition - completion handled in ISR
         */
        // change to transmitter mode (Write
        delayMilliSec(4000);
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR;
        // send I2C start condition with address frame and W bit
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

        // wait for sensor data to be received
        while (RXDataPointer < NUM_OF_REC_BYTES) ;
        /* combine bytes to form 16-bit accel_ values  */
        accel_x = ((uint16_t) RXData[0] << 8) + (RXData[1]);
        accel_y = ((uint16_t) RXData[2] << 8) + (RXData[3]);
        accel_z = ((uint16_t) RXData[4] << 8) + (RXData[5]);

        RXDataPointer = 0;

        delay5();
        clearDisplay();
        setPrecision();

        if(validity && input != 'a') {
            printMessage(invalid, (sizeof(invalid)/sizeof(invalid[0])));
            validity = false;
            printMessage(prompt, (sizeof(prompt)/sizeof(prompt[0])));
        }
        else {
            switch (input) {
                case 'P':
                case 'p':
                    pushingP();
                    break;
                case 'X':
                case 'x':
                case 'Y':
                case 'y':
                case 'Z':
                case 'z':
                case '2':
                case '4':
                case '8':
                case '6':
                    printMessage(prompt, (sizeof(prompt)/sizeof(prompt[0])));
                    input = NULL;
                    break;
                default:
                    break;
            }
        }

	}


}

/*
 * Function that print both to the terminal and the LCD display once p is entered
 */
void pushingP(void){
    char gBuffer[16], dataBuffer[16];;
    float valueX = 0; float valueY = 0; float valueZ = 0;


    //code to print display
    if(g==16) sprintf(gBuffer, "Accelerometer%dg", g);
    else sprintf(gBuffer, "Accelerometer %dg", g);

    //code to print to LCD depending on axis selected
    firstLine();
    putch(gBuffer);
    secondLine();
    switch (pref) {
    case 'x':
        valueX = accel_x / prekky;
        sprintf(dataBuffer, "X: %4.3f g",valueX);
        putch(dataBuffer);
        break;
    case 'y':
        valueY = accel_y / prekky;
        sprintf(dataBuffer, "Y: %4.3f g",valueY);
        putch(dataBuffer);
        break;
    case 'z':
        valueZ = accel_z / prekky;
        sprintf(dataBuffer, "Z: %4.3f g",valueZ);
        putch(dataBuffer);
        break;
    default:
        break;
    }
    // print only once to terminal
    if(p) {
        char termBuffer[55];
        char termBuffer2[48];
        sprintf(termBuffer2, "\n\n\r\tAccel_X: %d \tAccel_Y: %d \tAccel_Z: %d", accel_x, accel_y, accel_z);
        sprintf(termBuffer, "\n\r\tAccel_X_g: %4.3f g\tAccel_Y: %4.3f g\tAccel_Z: %4.3f g", valueX, valueY, valueZ);
        printMessage(termBuffer2, (sizeof(termBuffer2)/sizeof(termBuffer2[0])));
        printMessage(termBuffer, (sizeof(termBuffer)/sizeof(termBuffer[0])));
        printMessage(prompt, (sizeof(prompt)/sizeof(prompt[0])));
        p = false;
    }
}

/*
 * Sets the precision depending on the g value specified
 */
void setPrecision() {
    if(g == 2) prekky = 16384;
    else if(g == 4) prekky = 8192;
    else if(g == 8) prekky = 4096;
    else if(g == 16) prekky = 2048;
}

// for terminal printing
void printMessage(const char* message, int msgLength)
{
    int i;
    for (i = 0; i < msgLength; i++) {
        // Check if the TX buffer is empty first
        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG));

        // Send next character of message
        //  Note that writing to TX buffer clears the flag
        EUSCI_A0->TXBUF = message[i];
    }
}


void configUART()
{
    /* Configure MCLK/SMCLK source to DCO, with DCO = 12MHz */
//    CS->KEY = CS_KEY_VAL;                   // Unlock CS module for register access
//    CS->CTL0 = 0;                           // Reset tuning parameters
//    CS->CTL0 = CS_CTL0_DCORSEL_3;           // Set DCO to 12MHz (nominal, center of 8-16MHz range)
//    CS->CTL1 = CS_CTL1_SELA_2 |             // Select ACLK = REFO
//            CS_CTL1_SELS_3 |                // SMCLK = DCO
//            CS_CTL1_SELM_3;                 // MCLK = DCO
//    CS->KEY = 0;                            // Lock CS module from unintended accesses

    /* Configure UART pins */
    P1->SEL0 |= BIT2 | BIT3;                // set 2-UART pins as secondary function
    P1->SEL1 &= ~(BIT2 | BIT3);

    /* Configure UART
     *  Asynchronous UART mode, 8O1 (8-bit data, even parity, 1 stop bit),
     *  LSB first, SMCLK clock source
     */
    EUSCI_A0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Put eUSCI in reset
    EUSCI_A0->CTLW0 = EUSCI_A_CTLW0_SWRST | 0xC080; // Remain eUSCI in reset
    // complete configuration of UART in eUSCI_A0 control register

    /* Baud Rate calculation
     * Refer to Section 24.3.10 of Technical Reference manual
     * BRCLK = 48MHz, Baud rate = 38400
     *
     * calculate N = 48MHz/38400 = 1250.000 and determine values for UCBRx = INT(N/16) = 78 - 0x4E, UCBRFx = INT((N/16-INT(N/16))*16) = 2
     * , and UCBRSx = 0 because fractional component of N is 0.0000
     *          values used in next two TODOs
     *
     */
    // set clock prescaler in eUSCI_A0 baud rate control register
    EUSCI_A0->BRW = 0x4E;
    // configure baud clock modulation in eUSCI_A0 modulation control register
    EUSCI_A0->MCTLW = 0x21;

    EUSCI_A0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;    // Initialize eUSCI
    EUSCI_A0->IFG &= ~EUSCI_A_IFG_RXIFG;        // Clear eUSCI RX interrupt flag
    EUSCI_A0->IE |= EUSCI_A_IE_RXIE;            // Enable USCI_A0 RX interrupt

    // Enable eUSCIA0 interrupt in NVIC module
    NVIC->ISER[0] = (1 << EUSCIA0_IRQn );
}

void configI2C()
{
    /* Configure UART pins */
    P1->SEL0 |= BIT6 | BIT7;                // set I2C pins as secondary function
    P1->SEL1 &= ~(BIT6 | BIT7);

    // Initialize data variable
    RXDataPointer = 0;
    TXDataPointer = 0;

    /* Configure eUSCI_B0 for I2C mode
     *  I2C master mode, synchronous, 7-bit address, SMCLK clock source,
     *  transmit mode, with automatic STOP condition generation
     */
    EUSCI_B0->CTLW0 |= EUSCI_A_CTLW0_SWRST; // Software reset enabled
    EUSCI_B0->CTLW0 = EUSCI_A_CTLW0_SWRST | // Remain eUSCI in reset mode
            EUSCI_B_CTLW0_MODE_3 |          // I2C mode
            EUSCI_B_CTLW0_MST |             // Master mode
            EUSCI_B_CTLW0_SYNC |            // Sync mode
            EUSCI_B_CTLW0_TR |              // Transmitter mode
            EUSCI_B_CTLW0_SSEL__SMCLK;      // SMCLK

    /* I2C clock calculation
     * Refer to Section 26.3.6 of Technical Reference manual
     * BRCLK = 48MHz, I2C bit clock rate = 100kbps
     *
     * UCBBRW
    */
    // configure eUSCI_B0 bit rate control for 100 kbps
    EUSCI_B0->BRW = 0x1E0;

    /* Configure I2C to communicate with GY-521 */
    EUSCI_B0->I2CSA = GY521_ADDRESS;            // I2C peripheral address
    EUSCI_B0->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;    // Release eUSCI from reset

    /* Initialize GY-521 by writing to Power Management Register
     *
     *  format for Write operations
     *  _________________________________________________________________
     *  |       |          |                 |                  |       |
     *  | Start |  Addr  W | <Register Addr> | <Value to write> | Stop  |
     *  |_______|__________|_________________|__________________|_______|
     */
    // Ensure stop condition not pending
    while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);
    do {
        // Send I2C start condition and address frame with W
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
        // wait for TX buffer to be ready
        while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
        // load 1st data byte into TX buffer
        EUSCI_B0->TXBUF = PWR_MGMT_ADDR;            // send register address
        // wait for ACK/NACK after address frame
        while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTT);
    } while(EUSCI_B0->IFG & EUSCI_B_IFG_NACKIFG);   // resend address frame if ACK not received
    // wait for TX buffer to be ready
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
    // load 2nd data byte into TX buffer
    EUSCI_B0->TXBUF = 0;                // write value to register
    // wait for 2nd data byte to begin transmit
    while (!(EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0));
    // Send I2C stop condition
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

    // Ensure stop condition got sent
    while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);
    // ensure flags are cleared before enabling interrupts
    EUSCI_B0->IFG &= ~(EUSCI_B_IFG_TXIFG0 | EUSCI_B_IFG_RXIFG0 | EUSCI_B_IFG_NACKIFG);

    EUSCI_B0->IE |= EUSCI_A_IE_RXIE |       // Enable receive interrupt
            EUSCI_A_IE_TXIE |               // Enable transmit interrupt
            EUSCI_B_IE_NACKIE;              // Enable NACK interrupt

    // Enable eUSCIB0 interrupt in NVIC module
    NVIC->ISER[0] = (1 << EUSCIB0_IRQn);

}

/*!
 * \brief This function just delays for 5 milli seconds
 *
 * \return none
 */
void delay5(void)
{
    SysTick->CTRL |= 0x5;
    SysTick->LOAD |= 0x3A981;
    while(!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
}

// UART interrupt service routine
void EUSCIA0_IRQHandler(void)
{
    // Check if receive flag is set (value ready in RX buffer)
    if (EUSCI_A0->IFG & EUSCI_A_IFG_RXIFG)
    {
        // Note that reading RX buffer clears the flag
        input = EUSCI_A0->RXBUF;

        // Echo character back to screen, otherwise user will not be able to
        //  verify what was typed
        while(!(EUSCI_A0->IFG & EUSCI_A_IFG_TXIFG)); // Wait for TX buffer ready
        EUSCI_A0->TXBUF = input;                 // Echo character to terminal

        // Reset if invalid value length
//        if (digitsReceived > RGB_DIGITS) {
//            digitsReceived = INVALID_VALUE;
//            return;
//        }
        // Convert ASCII character to appropriate hexadecimal value for current
        //  to interept user input data and put them in variables used

        switch (input) {
        case '2':
            g = 2;
            break;
        case '4':
            g = 4;
            break;
        case '8':
            g = 8;
            break;
        case '6':
            g = 16;
            break;
        case 'P':
        case 'p':
            p = true;
            break;
        case 'X':
        case 'x':
            pref = 'x';
            break;
        case 'Y':
        case 'y':
            pref = 'y';
            break;
        case 'Z':
        case 'z':
            pref = 'z';
            break;
        default:
            validity = true;
            break;
        }
    }
}

// I2C interrupt service routine
void EUSCIB0_IRQHandler(void)
{
    // Handle if ACK not received for address frame
    if (EUSCI_B0->IFG & EUSCI_B_IFG_NACKIFG) {
        EUSCI_B0->IFG &= ~ EUSCI_B_IFG_NACKIFG;

        // resend I2C start condition and address frame
        EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
        TXDataPointer = 0;
        RXDataPointer = 0;
    }
    // When TX buffer is ready, load next byte or Restart for Read
    if (EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) {
        if (TXDataPointer == 0) {
            // load 1st data byte into TX buffer (writing to buffer clears the flag)
            EUSCI_B0->TXBUF = ACCEL_BASE_ADDR;      // send register address
            TXDataPointer = 1;
        } else {
            // change to receiver mode (Read)
            EUSCI_B0->CTLW0 &= ~EUSCI_B_CTLW0_TR;
            // send Restart and address frame with R bit
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTT;
            TXDataPointer = 0;
            RXDataPointer = 0;
            // need to clear flag since not writing to buffer
            EUSCI_B0->IFG &= ~ EUSCI_B_IFG_TXIFG0;
        }
    }
    // When new byte is received, read value from RX buffer
    if (EUSCI_B0->IFG & EUSCI_B_IFG_RXIFG0) {
        // Get RX data
        if (RXDataPointer < NUM_OF_REC_BYTES) {
            // reading the buffer clears the flag
            RXData[RXDataPointer++] = EUSCI_B0->RXBUF;
        }
        else {  // in case of glitch, avoid array out-of-bounds error
            EUSCI_B0->IFG &= ~ EUSCI_B_IFG_RXIFG0;
        }

        // check if last byte being received - if so, initiate STOP (and NACK)
        if (RXDataPointer == (NUM_OF_REC_BYTES-1)) {
            // Send I2C stop condition
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
        }
    }
}

