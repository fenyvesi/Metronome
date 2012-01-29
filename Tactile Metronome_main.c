// Wayne and Layne present:
// Tactile Metronome, Firmware revision 1.14
// Copyright (c) 2009 Adam W Wolf and Matthew L Beckler (Wayne and Layne)
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along
// with this program; if not, write to the Free Software Foundation, Inc.,
// 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
//
//
// For pictures, instructions, and software, please visit:
// http://wayneandlayne.com/metronome/
//
// PIC16F685 Version 1.14
// Last Updated: Tuesday, October 20, 2009
//
// Description: Moving call to button initialization code to be second. This
//              was probably causing weird behavior in selecting a beep pitch
//              on startup.
//
// 
// 
//
// Hardware pinout plans:
//   Infrastructure:
//              1               Vdd
//              20              Vss
//              3               OSC2 (used for external crystal)
//              2               OSC1 (ditto)
//   Seven-segment displays:
//              19              RA0 digit 3 enable
//              18              RA1 digit 2 enable
//              17              RA2 digit 1 enable
//              16              RC0 segment A
//              15              RC1 segment B
//              14              RC2 segment C
//              7               RC3 segment D
//              6               RC4 segment E
//              8               RC6 segment F
//              9               RC7 segment G
//   Piezo:
//              5               CCP1 pwm output
//              13              AN10 analog input
//   Buttons:
//              11              RB6 Up   Button (internal pullup)
//              10              RB7 Down Button (internal pullup)
//   Free Pins:
//              4               RA3 (!MCLR) (input only, no pullup available, only used for programming)
//              12              RB5 (AN11)  (used for LED debug, tied to decimal point on production pcb)

#include <htc.h>
#include <stdlib.h> // Gives us the abs function

// These two macros set or clear a bit in a register
#define bitset(var,bitno) ((var) |= 1UL << (bitno))
#define bitclr(var,bitno) ((var) &= ~(1UL << (bitno)))

// These two macros check if a bit is set or cleared
#define bitisset(var,bitno) ((var) & 1UL << (bitno))
#define bitisclr(var,bitno) (!((var) & 1UL << (bitno)))

// Set the config bits:
__CONFIG(
        HS &                    // External high-speed mode
        WDTDIS &                // Watchdog disable
        PWRTEN &                // Provides 64ms power-up timer
        MCLREN &                // MCLR external - Later, change to: Pin 4 is IO, MCLR internally pulled up
        UNPROTECT &             // No code or EEPROM read protection
        BORDIS &                // Brown-out-reset disabled
        IESODIS &               // Internal External Switchover mode is disabled
        FCMDIS                  // Fail-Safe Clock Monitor is disabled
        );

// Frequency of oscillator in hertz (Fosc), used by the compiler's delay functions.
// NOTE: This is the raw external oscillator frequency (Fosc), not the Tcy (instr. period).
// You also set the Processor Frequency in the simulator settings to this (Fosc).
#define _XTAL_FREQ              8000000 // 8 MHz

// With 8 MHz clock (Fosc = 8 MHz), Fosc/4 = 2 Mhz; with no prescaler,
// it takes (0.0001/(1/2000000)) = 200 ticks to make 0.1 ms
// Use 200-2=198 since writing the reset value disables timer increments for 2 cycles.
// This timer works by pre-setting a desired value in the interrupt, and getting the 
// next interrupt once the timer has counted up to 256 and rolled-over. That is why we
// subtract our desired number of counts from the rollover count, 256.
#define TMR0_RESET_VAL          256-198


// Pin definitions for the debug LED output
// This pin is actually tied to the decimal point on the seven-segment displays
// on the production PCBs. We could still use it for debug output.
#define LED_PIN                 RB5
#define LED_TRIS                TRISB5

// Pin definitions for PWM output
#define PWM_TRIS                TRISC5

// Pin definitions for ADC input
#define ADC_TRIS                TRISB4

// Pin definitions for seven-segment displays
#define DIGIT_TRIS              TRISA
#define DIGIT_PORT              PORTA
#define SEVEN_TRIS              TRISC
#define SEVEN_PORT              PORTC

// Pin definitions for input buttons
#define BTN_UP_VAL              RB6
#define BTN_DN_VAL              RB7
#define BTN_UP_TRIS             TRISB6
#define BTN_DN_TRIS             TRISB7
#define BTN_UP_WPU              WPUB6
#define BTN_DN_WPU              WPUB7

// How long to wait between button repeats, in ms:
#define BUTTON_REPEAT_WAIT_MS	100

// How long each beep lasts, in milliseconds.
// How long should this be? It doesn't affect the frequency (pitch) of the beep.
#define BEEP_TIME_MS            20


// Data definitions for seven-segment displays. (common anode, so 0 means on).
// Bit 5 isn't used, so the value in that column doesn't matter.
#define SEG_A               0b11111110
#define SEG_B               0b11111101
#define SEG_C               0b11111011
#define SEG_D               0b11110111
#define SEG_E               0b11101111
#define SEG_F               0b10111111
#define SEG_G               0b01111111
// If we don't want to have hex digits A-F we can remove them to save space.
#define DIGIT_A             10
#define DIGIT_B             11
#define DIGIT_C             12
#define DIGIT_D             13
#define DIGIT_E             14
#define DIGIT_F             15
#define DIGIT_N             16
#define DIGIT_O             17
#define DIGIT_P             18
#define DIGIT_T             19
#define DIGIT_EQUALS        20
#define DIGIT_BLANK         21
const char digit_data[] =
{
    SEG_A & SEG_B & SEG_C & SEG_D & SEG_E & SEG_F,          // 0
    SEG_B & SEG_C,                                          // 1
    SEG_A & SEG_B &         SEG_D & SEG_E &         SEG_G,  // 2
    SEG_A & SEG_B & SEG_C & SEG_D &                 SEG_G,  // 3
    SEG_B & SEG_C &                 SEG_F & SEG_G,          // 4
    SEG_A &         SEG_C & SEG_D &         SEG_F & SEG_G,  // 5
    SEG_A &         SEG_C & SEG_D & SEG_E & SEG_F & SEG_G,  // 6
    SEG_A & SEG_B & SEG_C,                                  // 7
    SEG_A & SEG_B & SEG_C & SEG_D & SEG_E & SEG_F & SEG_G,  // 8
    SEG_A & SEG_B & SEG_C & SEG_D &         SEG_F & SEG_G,  // 9
    SEG_A & SEG_B & SEG_C &         SEG_E & SEG_F & SEG_G,  // A
    SEG_C & SEG_D & SEG_E & SEG_F & SEG_G,                  // b
    SEG_D & SEG_E &         SEG_G,                          // c
    SEG_B & SEG_C & SEG_D & SEG_E &         SEG_G,          // d
    SEG_A &                 SEG_D & SEG_E & SEG_F & SEG_G,  // E
    SEG_A &                         SEG_E & SEG_F & SEG_G,  // F
    SEG_C &         SEG_E &         SEG_G,                  // n
    SEG_C & SEG_D & SEG_E &         SEG_G,                  // o
    SEG_A & SEG_B &                 SEG_E & SEG_F & SEG_G,  // P
    SEG_D & SEG_E & SEG_F & SEG_G,                          // t
    SEG_D &                 SEG_G,                          // =
    0xFF                                                    // all off
};


#define PIEZO_THRESHOLD         200     // What is the voltage threshold of each tap, out of 1024.
#define PIEZO_DEBOUNCE_MS       30      // How many ms to wait after each piezo tap for debounce.
#define TAP_TIMEOUT_MS          2000    // When inputting taps, how long to wait before assuming the user is finished

#define MAX_PATTERN_INTERVALS   12      // How long the pattern can be. Num taps = num intervals + 1.
#define MINIMUM_REPETITIONS     3       // How many repetitions of the pattern are required.
#define NUM_RAW_INTERVALS       MAX_PATTERN_INTERVALS * MINIMUM_REPETITIONS
#define MIN_INTERVAL_MS         10      // This is used when scaling intervals from button presses

// These global variables are used to read in and do pattern recognition on the input taps:
unsigned int    intervals[NUM_RAW_INTERVALS];   // When we are reading in taps, store the intervals in this array.
unsigned char   num_raw_intervals;              // After the taps are all read in, this is the total number of intervals.
unsigned int*   scaled_intervals;               // This will point inside 'intervals', at the second measure. See website.
unsigned char   pattern_length;                 // The pattern recognition code sets this value to the most likely pattern length.

// These global variables are used during regular output:
unsigned long   previous_ms;                    // The time of the previous beep, in ms since startup.
unsigned int    interval_ms;                    // How long to wait for the next beep, in ms.

// Display variables - Basic idea: display one digit for a bit, blank the displays for a bit, 
// then move on to the next digit. Wash, rinse, repeat.
// CUS means "hundred micro seconds", from a roman-numeral perspective.
#define DISP_COUNT_CUS          1;      // How many 100us to display the digit.
#define DISP_WAIT_DIM_CUS       40;     // During normal (dim) output, how long to wait between digits.
#define DISP_WAIT_LIT_CUS       3;      // During beep (bright) output, how long to wait between digits.
                                        // Probably shouldn't go below 3.

volatile unsigned char disp_flags;      // The bottom two bits are used for the current digit
#define DISP_FLAG_WAIT  2               // This is 1 if we are in a wait state, 0 if we are in a display state
#define DISP_FLAG_LIT   3               // This is 1 if we want to display bright digits, 0 for dim
#define DISP_DIGIT_MASK 0b00000011      // Used to mask off the bottom two bits, the current digit to display
volatile unsigned char disp_count;      // Counter for both disp and wait times - reset to a value, count down to zero
volatile unsigned char disp_digits[3];  // The three digit values - stores the actual pin output data
                                        // disp_digits[0] is digit 3, which is the right-most digit
                                        // disp_digits[1] is digit 2, which is the middle digit
                                        // disp_digits[2] is digit 1, which is the left-most digit

// Button handling variables - The interrupt sets the flags to 1, the main loop sets them to 0 to ACK the flag
volatile unsigned char button_flags = 0;
#define BTN_UP_FLAG             0           // 0 means a new pending button press
#define BTN_DN_FLAG             1           // 0 means a new pending button press
#define BTN_FLAG_MASK           0b00000011  // Mask for the button flags
#define BTN_UP_PREV_STATE       2           // 0 means pressed, 1 means released
#define BTN_DN_PREV_STATE       3           // 0 means pressed, 1 means released
// these two store the past 8 samples - lsb is most recent
volatile unsigned char button_up_hist = 0;
volatile unsigned long button_up_last_repeat;
volatile unsigned char button_dn_hist = 0;
volatile unsigned long button_dn_last_repeat;
signed int scale_factor = 0;                // Using the buttons, we can scale the original intervals by this value (in bpm)

// This stores the number of milliseconds since startup, but be sure to use the
// millis() accessor function to avoid breakage due to multi-byte reads and writes with interrupts going on.
volatile unsigned long hidden_millis = 0;
// In order to have better control over the LED output multiplexing, the interrupt handler fires every 100 us.
// partial_ms is decremented each time, and only when 10 interrupts have passed, do we increment hidden_millis.
volatile unsigned char partial_ms = 10;     // this actually counts down, to make for a nicer (== 0) check

void interrupt my_isr(void)
{
    // Timer 0 is the only interrupt source, but we check the flags anyway, because it's a good habit.
    if (T0IE && T0IF)
    {
        partial_ms--;
        TMR0 = TMR0_RESET_VAL;
        if (partial_ms == 0)
        {
            // we have counted up 10 interrupts of 100 us, and now tick off another millisecond
            partial_ms = 10;
            hidden_millis++;

            // The button handling code doesn't need to run every millisecond, so we look only
            // at the two least-significant digits of hidden_millis. When these are both zero,
            // every fourth millisecond, we deal with the buttons.
            if (!(hidden_millis & 0b00000011))
            {
                // record the current button state in our history variables
                button_up_hist <<= 1;
                button_dn_hist <<= 1;
                if (BTN_UP_VAL)
                    bitset(button_up_hist, 0);
                if (BTN_DN_VAL)
                    bitset(button_dn_hist, 0);

                if (button_up_hist == 0)
                {
                    if (bitisset(button_flags, BTN_UP_PREV_STATE))
                    {
                        // just detected a new button press
                        bitset(button_flags, BTN_UP_FLAG);
                        bitclr(button_flags, BTN_UP_PREV_STATE);
                        button_up_last_repeat = hidden_millis;
                    }
                    else
                    {
                        // we are in a long press situation
                        if (hidden_millis - button_up_last_repeat > BUTTON_REPEAT_WAIT_MS)
                        {
                            bitset(button_flags, BTN_UP_FLAG);
                            button_up_last_repeat = hidden_millis;
                        }
                    }
                }

                if ( (button_up_hist == 0xFF) && bitisclr(button_flags, BTN_UP_PREV_STATE) )
                {
                    // just detected a button release
                    bitset(button_flags, BTN_UP_PREV_STATE);
                }
                // else button_up_hist == 0xff, but the bit was set means we are in a long release

                if (button_dn_hist == 0)
                {
	                if (bitisset(button_flags, BTN_DN_PREV_STATE))
	                {
	                    // just detected a new button press
	                    bitset(button_flags, BTN_DN_FLAG);
	                    bitclr(button_flags, BTN_DN_PREV_STATE);
	                    button_dn_last_repeat = hidden_millis;
	                }
	                else
	                {
		                // we are in a long press situation
		                if (hidden_millis - button_dn_last_repeat > BUTTON_REPEAT_WAIT_MS)
		                {
			                bitset(button_flags, BTN_DN_FLAG);
			                button_dn_last_repeat = hidden_millis;
			            } 
		            }
		        }  
                
                if ( (button_dn_hist == 0xFF) && bitisclr(button_flags, BTN_DN_PREV_STATE) )
                {
                    // just detected a button release
                    bitset(button_flags, BTN_DN_PREV_STATE);
                }
                // else button_dn_hist == 0xff, but the bit was set means we are in a long release
            }

        }

        // Every 100 us, update the seven-segment displays
        disp_count--;
        if (disp_count == 0)
        {
            if (bitisset(disp_flags, DISP_FLAG_WAIT))
            { // we were in a waiting state, now we display the next digit
                disp_count = DISP_COUNT_CUS;
                disp_flags++;
                if ((disp_flags & DISP_DIGIT_MASK) == 3) // rollover prevention
                    disp_flags &= ~DISP_DIGIT_MASK; // clear the bottom two bits
                SEVEN_PORT &= 0b00100000; // turn off all segments (bit 5 is the PWM output, so we leave it alone)
                SEVEN_PORT |= disp_digits[disp_flags & DISP_DIGIT_MASK];
                bitclr(disp_flags, DISP_FLAG_WAIT); // switching to disp mode
                bitset(DIGIT_PORT, disp_flags & DISP_DIGIT_MASK); // turn on the digit
            }
            else
            { // we were in a displaying state, now we wait
                bitclr(DIGIT_PORT, disp_flags & DISP_DIGIT_MASK); // turn off the digit
                // How long should we wait?
                // If we're in a bright mode, don't wait too long,
                // if we're in a dim mode, wait a while longer to make it less bright.
                if (bitisset(disp_flags, DISP_FLAG_LIT))
                {
                    // currently beeping = lit
                    disp_count = DISP_WAIT_LIT_CUS;
                }
                else
                {
                    // not currently beeping = dim
                    disp_count = DISP_WAIT_DIM_CUS;
                }
                bitset(disp_flags, DISP_FLAG_WAIT);
            }
        }
        T0IF = 0; // clear interrupt flag
    }
}

// This function is very similar to the millis() function on the Arduino.
// Be sure to use this function instead of directly accessing the hidden_millis variable.
unsigned long millis()
{
    unsigned long m;
    di(); // disable interrupts
    m = hidden_millis;
    ei(); // enable interrupts
    return m;
}

// This function is very similar to the delay(ms) function on the Arduino.
// It does a proper delay of the specified number of milliseconds.
void delay(unsigned long ms)
{
    unsigned long start = millis();
    while (millis() - start <= ms);
}

// This function has the same name and functionality as the Arduino function,
// but all the code was rewritten to work with the pic ADC hardware.
unsigned int analogRead(void)
{
    // acquisition time of 5us; this function is a built-in for the HITECH PICC compiler.
    __delay_us(5);
    GODONE = 1;                     // start the ADC conversion
    while (GODONE == 1);            // Wait for it to finish
    return (ADRESH << 8) + ADRESL;
}

// This function enables the PWM output to the piezo for BEEP_TIME_MS milliseconds,
// producing an audible beep.
void beep(void)
{
    // enable output
    PWM_TRIS = 0;
    bitset(disp_flags, DISP_FLAG_LIT);  // Set the lit flag, so the digits flash with the beep.

    // wait
    delay(BEEP_TIME_MS);

    // disable output
    PWM_TRIS = 1;
    bitclr(disp_flags, DISP_FLAG_LIT);

    // wait
    delay(1);
    // we have to turn the pin to input mode, and wait one ms for it to drain
    // the voltage through the 10k resistor, otherwise taps are detected after output beeps.
    // Using the scope, it looks like we need at least 500us here.
}

// Initialize the Analog-to-Digital converter to sample from AN10
void init_adc(void)
{
    ADC_TRIS = 1;               // Set pin as input
    ANSEL = 0;                  // Configure all pins as digital,
    ANSELH = 0;
    ANS10 = 1;                  // except for AN10
    ADCON1 = 0b01010000;        // Fosc/16 = 8M / 16 -> Tad = 2us
    ADCON0 = 0b10101001;        // Right-justified, Vref = Vdd, Channel AN10, Enable ADC
}

// Initialize Timer0 to generate 0.1 ms interrupts for use with millis()
void init_timer0(void)
{
    PSA = 1;                    // Prescaler is assigned to the WDT
    TMR0 = TMR0_RESET_VAL;      // This write disables timer increment for two cycles (see above #define)
    T0IE = 1;                   // Enable Timer0 interrupt
    GIE = 1;                    // Enable global interrupts
    T0IF = 0;                   // Clear interrupt flag
    partial_ms = 0;             // Our 0.1 ms interrupt tracks partial ms
    hidden_millis = 0;          // Start out at zero
    T0CS = 0;                   // TMR0 Click Source is internal instruction cycle clock, Fosc/4. (This starts the timer.)
}

// Initialize Timer2 and PWM output
void init_timer2_pwm(void)
{
    // Our piezo has a resonant frequency of 4 khz, so we tune our square wave to that frequency:
    // PWM period       = (PR2 + 1) * 4 * Tosc   * TMR2_prescaler
    // 1/4000 = 0.00025 = (124 + 1) * 4 * 1/8MHz * 4
    #define     PR2_THIRD  		124
    // This resonant frequency should be the loudest. We set it to be the default.
    
    // If you hold down the DOWN button, then we instead use a frequency a major third below (the root),
    // which is (1/1.259921) * 4 khz = 3174.802
    // PWM period             = (PR2 + 1) * 4 * Tosc   * TMR2_prescaler
    // 1/3174.802 = 0.0003149 = (156 + 1) * 4 * 1/8MHz * 4
    #define     PR2_ROOT    	156

    // If you hold down the UP button, then we instead use a frequency a perfect fifth above the root,
    // which is 1.189207 * 4 khz = 4756.828
    // PWM period             = (PR2 + 1) * 4 * Tosc   * TMR2_prescaler
    // 1/4756.828 = 0.0002102 = (104 + 1) * 4 * 1/8MHz * 4
    #define     PR2_FIFTH       104

    PWM_TRIS = 1;               // Disable output for now
    CCP1CON = 0b00001100;       // Single output (P1A/CCP1), DC LSb = 00, ECCP Mode is PWM (P1A Active-high)

    if (!BTN_DN_VAL)             // See above
    {
        PR2 = PR2_ROOT;
    }
    else if (!BTN_UP_VAL)
    {
        PR2 = PR2_FIFTH;
    }
    else
    {
        PR2 = PR2_THIRD;
    } 
    CCPR1L = PR2 >> 1;          // Duty cycle of 50%
    TMR2IF = 0;                 // Clear interrupt flag
    T2CON = 0b00000101;         // Prescaler is 4, Timer2 is on, 1:1 Postscaler
}

// Initialize the seven-segment displays
void init_display(void)
{
    DIGIT_TRIS &= 0b11111000;                       // clear lower three bits = outputs
    DIGIT_PORT &= 0b11111000;                       // set output values to low = digits off
    SEVEN_TRIS = 0b00100000;                        // All are outputs, except for CCP1 output on RC5 (for now)
    SEVEN_PORT = digit_data[DIGIT_BLANK];
    disp_flags = 0;                                 // Start out displaying digit 0
    disp_count = DISP_COUNT_CUS;
    disp_digits[0] = digit_data[DIGIT_BLANK];       // This is digit 3, which is the right-most digit.
    disp_digits[1] = digit_data[DIGIT_BLANK];
    disp_digits[2] = digit_data[DIGIT_BLANK];       // This is digit 1, which is the left-most digit.
}

// Initialize the two button pins
void init_buttons(void)
{
    BTN_UP_TRIS = 1;
    BTN_DN_TRIS = 1;

    // Globally enable weak pullups:
    RABPU = 0;

    // Enable weak pullups on the button inputs:
    BTN_UP_WPU = 1;
    BTN_DN_WPU = 1;
}

// This function reads in a sequence of taps. Up to (NUM_RAW_INTERVALS+1) taps can be read in.
// Stores interval counts (in milliseconds) into the intervals[] array.
void read_in_taps(void)
{
    // For the first tap, we simply record when it occured.
    while (analogRead() <= PIEZO_THRESHOLD);
    previous_ms = millis();
    // We set this flag to cause the display to flash, indicating that the tap was recognized.
    bitset(disp_flags, DISP_FLAG_LIT);
    // To prevent repeat taps, we delay for a few ms here to debounce the piezo
    delay(PIEZO_DEBOUNCE_MS);
    bitclr(disp_flags, DISP_FLAG_LIT);

    // second tap
    while (analogRead() <= PIEZO_THRESHOLD);
    intervals[0] = millis() - previous_ms;
    previous_ms = millis();
    bitset(disp_flags, DISP_FLAG_LIT);
    delay(PIEZO_DEBOUNCE_MS);
    bitclr(disp_flags, DISP_FLAG_LIT);

    // rest of the taps, keep listening until one of these happens:
    // * We go TAP_TIMEOUT_MS without another tap (timeout)
    // * Run out of space in the array (too many taps)
    // * The user pushes one of the up/down buttons
    num_raw_intervals = 1;
    button_flags &= ~BTN_FLAG_MASK; // clear any pending button flags
    while ( (millis() - previous_ms < TAP_TIMEOUT_MS) &&
            (num_raw_intervals < NUM_RAW_INTERVALS) &&
            (bitisclr(button_flags, BTN_UP_FLAG)) &&
            (bitisclr(button_flags, BTN_DN_FLAG)) )
    {
        if (analogRead() >= PIEZO_THRESHOLD)
        {
            intervals[num_raw_intervals] = millis() - previous_ms;
            previous_ms = millis();
            bitset(disp_flags, DISP_FLAG_LIT);
            delay(PIEZO_DEBOUNCE_MS);
            bitclr(disp_flags, DISP_FLAG_LIT);
            num_raw_intervals++;
        }
    }
}

// This function is called after we have read in the taps. It considers all possible pattern lengths,
// and finds the pattern length that is the most likely. Please see the website for more in-depth
// documentation and explanations of the pattern recognition ideas.
void do_pattern_recognition()
{
    // intervals[] has valid data in 0 through (num_raw_intervals - 1).
    //what if this is a pattern, not a metronome?
    //lets try to find the pattern length.
    unsigned int temp;
    unsigned int first_beat, second_beat;
    unsigned int lowest_cost = 0xFFFF;
    // If the user didn't provide enough taps to make a pattern, then we just go with straight taps:
    pattern_length = 1;

    // We are currently requiring at least three copies of the pattern to be tapped in.
    // We start considering a pattern length of 1, and go up to gpl_stop
    unsigned char gpl_stop = 1 + ( (1 + num_raw_intervals) / ((unsigned int) MINIMUM_REPETITIONS) );
    for (unsigned int guessed_pattern_length = 1; guessed_pattern_length < gpl_stop; guessed_pattern_length++)
    {
        // For each possible pattern length, we need to find the cost of that pattern length
        temp = 0;

        // We look at each beat in the first measure of the guessed pattern:
        for (unsigned int beat = 0; beat < guessed_pattern_length; beat++)
        {
            //for any pattern size, we want to compare the intervals between n and n+1, 
            //  and then 2n, 2n+1, and then 3n, 3n+1 until we run out of space...
            unsigned int measure_stop = ((num_raw_intervals - 1)/guessed_pattern_length) - 1;
            for (unsigned int measure = 0; measure < measure_stop; measure++)
            {
                // We now have to compare these two:
                first_beat = (measure * guessed_pattern_length) + beat;
                second_beat = first_beat + guessed_pattern_length;
                // for now, we simply take the cost to be the difference between beats
                temp += abs(intervals[first_beat] - intervals[second_beat]);
            }
        }

        // We want to consider shorter patterns as more likely than longer patterns,
        // so we weigh our cost (temp) by the pattern length:
        temp *= guessed_pattern_length;
        if (temp < lowest_cost)
        {
            pattern_length = guessed_pattern_length;
            lowest_cost = temp;
        } 
    }

    // Now that we know our pattern is 'pattern_length' intervals long,
    // we go back through and find the mean interval for each beat:
    unsigned int number_of_measures = num_raw_intervals / pattern_length;
    for (unsigned int beat = 0; beat < pattern_length; beat++)
    {
        temp = 0;
        for (unsigned int measure = 0; measure < number_of_measures; measure++)
        {
            temp += intervals[beat + (pattern_length * measure)];
        }
        // We are simply averaging the delay for each 'beat' across all measures,
        // so we can just overwrite the original intervals as we go along
        intervals[beat] = temp / number_of_measures;
    }
}

// This function updates the BPM output display based on the provided interval (in ms).
void update_bpm(unsigned int interval)
{
    // updates the three values in disp_digits
    unsigned int bpm = 60000U / interval;
    disp_digits[0] = digit_data[bpm % 10];       // This is digit 3, which is the right-most digit.
    bpm = bpm / 10U;
    disp_digits[1] = digit_data[bpm % 10];       // This is digit 2, which is the middle digit.
    bpm = bpm / 10U;
    disp_digits[2] = digit_data[bpm % 10];       // This is digit 1, which is the left-most digit.
    // leading zero blanking
    if (disp_digits[2] == digit_data[0])
    {
        disp_digits[2] = digit_data[DIGIT_BLANK];
        if (disp_digits[1] == digit_data[0])
        {
            disp_digits[1] = digit_data[DIGIT_BLANK];
        }
    }
}

// This function adds up the total length of the pattern
unsigned long get_pattern_length_in_millis()
{
    unsigned long pattern_length_in_millis = 0;
    for (unsigned int i = 0; i < pattern_length; i++)
    {
        pattern_length_in_millis += intervals[i];
    }
    return pattern_length_in_millis;
}

// This function figures out how long we need to delay in order to start the beeps in time with the input taps.
unsigned long delay_to_stay_in_rhythm(unsigned long last_tap_time)
{
    unsigned long pattern_length_in_millis = get_pattern_length_in_millis();

    return pattern_length_in_millis - ((millis()-last_tap_time) % pattern_length_in_millis);
}

// This function tries to scale the intervals, based on the value in scale_factor.
// If the intervals get too short, it returns 1, otherwise 0.
unsigned char scale_intervals(void)
{
    unsigned int total_pattern_length_ms = 0;
    for (unsigned char i = 0; i < pattern_length; i++)
    {
        scaled_intervals[i] = 60000U / (unsigned int)( (60000U / intervals[i]) + scale_factor );
        // if any of the pattern's intervals have gotten too short,
        if (scaled_intervals[i] < MIN_INTERVAL_MS)
        {
            // the user tried to go too fast, so we need to decrease the scale_factor
            return 1;
        }
        total_pattern_length_ms += scaled_intervals[i];
    }
    update_bpm(total_pattern_length_ms);
    return 0;
}

void main(void)
{
    di(); // disable interrupts
    init_adc(); // do this first to set all other pins to digital, not analog, mode
    init_buttons(); // do this second to get the buttons working before init_timer2_pwm()
    init_timer2_pwm();
    init_display();
    init_timer0();
    ei(); // enable interrupts

	// this is the main loop
    while (1)
    {
        // Here, we display "TAP" so the user knows what to do
        disp_digits[2] = digit_data[DIGIT_T];       // This is digit 1, which is the left-most digit.
        disp_digits[1] = digit_data[DIGIT_A];
        disp_digits[0] = digit_data[DIGIT_P];       // This is digit 3, which is the right-most digit.

        read_in_taps();
        //at this point, previous_ms is the millis() value of when the last tap occured.
        do_pattern_recognition();
        // This is a pointer. We use the second 'measure' to store our scaled values.
        // Check out the website documentation for more details about this business.
        scaled_intervals = intervals + pattern_length; 
        scale_factor = 0; // how 

        // We use the scaled_intervals array to allow us to tweak the intervals without losing any information:
        unsigned int total_pattern_length_ms = 0;
        for (unsigned char i = 0; i < pattern_length; i++)
        {
            scaled_intervals[i] = intervals[i];
            total_pattern_length_ms += intervals[i];
        }
        // Regardless of whether or not pattern_length is 1, this next line sets the right thing:
        update_bpm(total_pattern_length_ms);
        button_flags &= ~BTN_FLAG_MASK; // clear any pending button flags

        // Let's take care of the delay in starting right here:
        delay(delay_to_stay_in_rhythm(previous_ms));
        unsigned char state = 0;
        interval_ms = intervals[state];
        previous_ms = millis();
        // we loop until the user taps the piezo
        while (analogRead() <= PIEZO_THRESHOLD)
        {
            if (millis() - previous_ms > interval_ms)
            {
                interval_ms = scaled_intervals[state];
                previous_ms = millis();

                state = state++;
                if (state >= pattern_length)
                    state = 0;

                beep();
            }

            if (button_flags & BTN_FLAG_MASK)
            {
                // if any button flags are active
                if (bitisset(button_flags, BTN_UP_FLAG))
                    scale_factor += pattern_length;
                if (bitisset(button_flags, BTN_DN_FLAG))
                    scale_factor -= pattern_length;
                button_flags &= ~BTN_FLAG_MASK; // clear button flags
                while (scale_intervals() == 1)
                {
	                // fix the scale factor
            		scale_factor -= pattern_length;
            	}
            }
        }

        // beep twice quickly to say "I understand the reset tap"
        delay(200);
        beep();
        delay(30);
        beep();
        // the user tapped us, so reset to "waiting for input taps"
    }
}

