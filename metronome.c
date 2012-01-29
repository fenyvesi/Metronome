// Metronome
// The tempo and beat number are selected by 4 switches. Tempo: 20-180, beat:2-16
// The beat is audible and visible.
// The 1st beat is 880HZ and red/green led, the other beats are 440Hz and green.
// The tempo and beat are stored in EEPROM, new run starts from last set values.
// PIC 16f690

//============================================================================
//#define DEBUG

#include <16F690.h>
//#device *=16
#use delay(clock=8000000)
#ifdef DEBUG
   #use rs232(BAUD=9600,XMIT=PIN_A0, RCV=PIN_A1)         //for debug
#endif
#FUSES INTRC_IO                    //Internal RC Osc, NO clock out
//#FUSES HS                        //High speed Oscillator, XTAL
#FUSES NOFCMEN                    //Fail-safe clock monitor disabled
#FUSES PUT                       //Power Up Timer
#FUSES NOWDT                    //No Watch Dog Timer
#FUSES NOPROTECT                //Code not protected from reading
#FUSES BROWNOUT                 //Reset when brownout detected
#FUSES MCLR                     //Master Clear pin enabled
#FUSES NOCPD                    //No EE protection
#FUSES IESO                     //Internal External Switch Over mode enabled

//*****************    Hardware setup
#define LCD_TYPE 2            // 2lines * 16 char

// PIC PIN functions
// pickit ICSP PINs
//  P1 Vpp:       RA3   !!!!!!
//  P2 VDD        PIC 1
//  P3 GND        PIC20
//  P4 DAT:       RA0, xmit
//  P5 Clock:     RA1, rcv
//  P6 T1G:       RA4, idle
 
// PIC PIN functions
//                      RA_0              PIC19 XMIT
//                      RA_1              PIC18 RCV 
#define FIRST_BEAT_LED  PIN_A2       //   PIC17 LED out for 1st beat
//                      RA_3              PIC 4 VPP
#define SOUND_OUT       PIN_A4     //     PIC 3 - A pwm output for sound and LED beats

//   LCD PINs
#define LCD_ENABLE_PIN  PIN_C0        // PIC16-LCD PIN6 
#define LCD_RS_PIN      PIN_C1        // PIC15-LCD PIN4
#define LCD_RW_PIN      PIN_C2        // PIC14 - empty, (LCD PIN5:Ground - only write)
//                      PIN_C3           PIC 7 - empty
//   data1              PIN_C4           PIC 6 -LCD PIN11
//   data2              PIN_C5           PIC 5 -LCD PIN12
//   data3              PIN_C6           PIC 8 -LCD PIN13
//   data4              PIN_C7           PIC 9 -LCD PIN14
//
//    INPUT PINs
//                  PIN_B4             //PIC 13 - SW1 in - Tempo DOWN
//                  PIN_B5             //PIC 12 - SW2 in - Tempo UP
//                  PIN_B6             //PIC 11 - SW3 in - Beat UP
//                  PIN_B7             //PIC 10 - SW4 in - Beat DOWN
// 

#define tempo_up     PIN_B5      //switch connections
#define tempo_down   PIN_B4
#define beat_up      PIN_B6
#define beat_down    PIN_B7

#define tempo_min 20       // intervals for tempo and beat
#define tempo_max 180
#define beat_min 2
#define beat_max 16

// Includes
#include<lcd_mod.c>   // for LCD write

// global variables
volatile int tempo, beat;
volatile boolean rhytm_changed=TRUE;      // set by switches, interrupt B, tested in main
volatile long int sound, silence, cycle;  // sound and silence in ms, sound+silene =cycle
volatile boolean period_high = TRUE;      // the generated pwm for sound
volatile long int time_spent=0;           // time spent within the cycle
volatile boolean sound_active = FALSE;    // to generate sound
volatile int beat_no;                     // the actual beat number

//===========================================================================
void enable_int_RB() 
{
   clear_interrupt(INT_RB);  //
   enable_interrupts(INT_RB);   
   enable_interrupts(GLOBAL);
}
void pic_init()
{
int temp;
setup_oscillator( OSC_8MHZ );               // own 8MHz Osc
//   PINs init
temp = input_b();                         // switch to input

//       set_tris_c az lcd_initben
}
#INT_RB                                   // setting TEMPO, BEAT by 4 switches
void setting_rhytm()
{
  delay_ms(15);            // to reduce noise, debouncing

  if (!input(tempo_up))
   if (tempo < tempo_max)
      tempo++;

  if (!input(tempo_down))
   if (tempo > tempo_min)
      tempo--;

  if (!input(beat_up))
   if (beat < beat_max)
      beat++;
  if (!input(beat_down))
   if (beat > beat_min)
      beat--;   

   rhytm_changed=TRUE;        // to initiate recalculating timing
}
void calc_timing()                        // sound and silence in ms, sound+silene =cycle
{
cycle =(60000/tempo);
sound = cycle/3;
silence= cycle-sound;
time_spent=0;
#ifdef DEBUG
//   printf("\r\n cycle: %6lu \r\n", cycle);
#endif
}
#INT_TIMER2    //
void generate_sound()                     // generating half period high or low for sound
{
   PERIOD_HIGH = !PERIOD_HIGH;
   if (PERIOD_HIGH)
      output_high(SOUND_OUT);
   else      
      output_low(SOUND_OUT);
}      
#INT_TIMER1    //
void generate_rhytm()                     // generating sound and silence periods, setting frequencies
{
time_spent=time_spent +33;    // increase with time slice
if (time_spent > cycle)       // cycle is over
   {
   time_spent=0;
   beat_no++;                 // next beat
   if (beat_no > beat)        // if beat is over, 1st beat
      beat_no=1;
   }
#ifdef DEBUG
//   printf("\r\n cycle, time_spent, beat: %6lu %6lu %2u \r\n", cycle, time_spent, beat_no);
#endif

if (time_spent<sound)         // sound period
   {
      if (!sound_active)      // there was no sound
         {
         sound_active = TRUE;
         if (beat_no == 1)
            {
            setup_timer_2(T2_DIV_BY_4, 100, 2);    // 880 Hz, half period = ~ 0.6 ms
            output_high(FIRST_BEAT_LED);
            }
         else
            {
            setup_timer_2(T2_DIV_BY_4, 200, 2);    // 440 Hz, half period = ~ 1.2 ms
            output_low(FIRST_BEAT_LED);           //
            }
         enable_interrupts(INT_TIMER2);            // sound generation
         }
   }
else                          // silence period
   {
   output_low(FIRST_BEAT_LED);           //
   disable_interrupts(INT_TIMER2); 
   sound_active = FALSE;
   }
}
void print_data()                         // print tempo, beat
{
   lcd_gotoxy(7,1);
   printf(lcd_putc,"%3u",tempo);
   lcd_gotoxy(7,2);
   printf(lcd_putc,"%2u",beat);
}
void setting_beat_time_slice()            // T1, 33ms
{
setup_timer_1 ( T1_INTERNAL | T1_DIV_BY_1); // 2.5*104.8576ms/8 =  32.768 ms
}
void read_stored_data(int tempo, int beat)
{
tempo=read_eeprom (254);                  // last stored values
beat=read_eeprom (255);
if (tempo > 200)                          // the data aren't stored - 1st run
   {
   tempo=90;
   beat=4;
   }
}
void write_stored_data(int tempo, int beat)
{
   write_eeprom (254,tempo);                 // storing initial values 1st run.
   delay_ms(10);
   write_eeprom (255,beat);
   delay_ms(10);
}
//===========================================================================
void main()                 
{
//******************************************************************************
//Declarations

//INIT
delay_ms(100);
pic_init();
lcd_init();
enable_int_RB();                          // enable setting tempo and beat with switches
lcd_putc("\f");                           // print names, the data is printed in print_data
lcd_gotoxy(1,1);
printf(lcd_putc,"TEMPO:");
lcd_gotoxy(1,2);
printf(lcd_putc,"BEAT:");

read_stored_data(tempo, beat);            // read stored last values for tempo, beat

rhytm_changed=TRUE;                       // to enter into settings
//******************************************************************************
while (TRUE)
   {
      if (rhytm_changed )                 // runs at start or generated by RB interrupt changing tempo and beat
         {
         disable_interrupts(INT_RB);      // no change in tempo beat
         disable_interrupts(INT_TIMER1);  // no beating
         disable_interrupts(INT_TIMER2);  // no sound
         print_data();                    // tempo, beat
         write_stored_data(tempo,beat);   // storing values
         calc_timing();                   // calculating sound and silence time
         rhytm_changed=FALSE;             // not to enter again without a change
         time_spent=0;                    // beginning of cycle
         beat_no=1;                       // 1st beat is coming
         setting_beat_time_slice();       // time slice =33 ms
         clear_interrupt(INT_TIMER1); 
         enable_interrupts(INT_TIMER1);   // to start beating cycle
         enable_interrupts(INT_RB);       // enable changing tempo, beat again
         }
   }
}
//******************************************************************************
