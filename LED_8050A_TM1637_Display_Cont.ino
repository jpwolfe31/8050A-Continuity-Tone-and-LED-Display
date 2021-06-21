/*
LED_8050A_TM1637_Display_Cont.ino

LED display implementation for Fluke 8050A multimeter
- for TM1637 display 
- with Continuity tone

Original code created 2021.06.4
Michael Damkier
Hamburg, Germany
(michael@vondervotteimittiss.com)

Modified 2021.06.11
John Wolfe (jpwolfe31@yahoo.com) 
for use with TM1637 six digit display 
and to add continuity tone using display output
*/

/*
PIN ASSIGNMENTS
*/
#include <Arduino.h>
#include <TM1637TinyDisplay6.h>       // Include 6-Digit Display Class Header

// Define TM1637 Digital Pins
#define CLK 12
#define DIO 13

// 8050A strobe lines
#define PIN_ST0 A0
#define PIN_ST1 A1
#define PIN_ST2 A2
#define PIN_ST3 A3
#define PIN_ST4 A4

// 8050A scancode lines
#define PIN_Z 2
#define PIN_Y 3
#define PIN_X 4
#define PIN_W 5
// Note - lines below are reversed from Strobe_8050A test software
// which uses PIN_HV 7 and PIN_DP 6
#define PIN_HV 6
#define PIN_DP 7

// Continuity additions
//  When using D0 and D1, the serial port is not available for upload to the Uno, 
//    so these lines need to be disconnected when uploading the sketch
#define PIN_Fb_VOLT 0  // for check not in voltage mode
#define PIN_Fc_MA 1  // for check not in mA mode
#define PIN_RNGa 8 // for check in 200 ohm range
#define PIN_RNGb 9 // for check in 200 ohm range
#define PIN_RNGc 10 // for check in 200 ohm range
#define PIN_TONE 11 // for tone generation
//  Continuity testing works by confirming the meter is in resistance mode in the 200 ohm range.  
//  Once this is determined, the format for measurement is:
//  1 _ _._ _  for open circuit and as follows for normal measurements:
//  1 5 3.1 6  
//  _ 9 8.2 6
//  _ 0 8.4 5 
//  _ 0 0.9 6
//  Therefore, for example, less than 10 ohms can be checked by confirming digit [ 0 ] is not a 1
//    and digit [ 1 ] is 0.  Less than 100 ohms can be checked by confirming digit [ 0 ] is not a 1.
//  The decimal point does not move in this range.  
//  Ohms being selected is confirmed when VOLT and MA are both 0 volts on IC pins 6(Fb) and 19(Fc)
//  R200 range selection is confirmed by decoding IC pins 3(RNGa), 4(RNGb) and 5(RNGc) as follows when 
//    the range buttons are selected:
//      RNGa-c       3 4 5   (IC pins)
//      200          5 5 0   (5 = 5 volts, 0 = 0 volts)
//      2k           0 5 5
//      20k          5 0 5
//      200k         0 0 5
//      2m           5 5 5
//      20m          0 0 0
//      200 and 2k   0 5 0
//      2m and 20m   5 0 0
//  R200 is confirmed by finding 5 5 0 on pins 3, 4 and 5 through inputs D8, D9 and D10
//  The continuity tone is generated with a tone on pin 11.
/*
CONSTANTS
*/

// strobe lines
#define NUM_STROBES 5
#define ST0 0
#define ST1 1
#define ST2 2
#define ST3 3
#define ST4 4

// scancode lines
#define NUM_SCANCODES 6
#define SC0 0
#define SC1 1
#define SC2 2
#define SC3 3
#define SC4 4
#define SC5 5

#define SCANCODE_Z 0
#define SCANCODE_Y 1
#define SCANCODE_X 2
#define SCANCODE_W 3
#define SCANCODE_HV 4
#define SCANCODE_DP 5

#define SCANCODE_DB SCANCODE_Y
#define SCANCODE_1 SCANCODE_Z
#define SCANCODE_MINUS SCANCODE_W
#define SCANCODE_PLUS SCANCODE_X
#define SCANCODE_REL SCANCODE_DP

// pin arrays
static const uint8_t StrobePins[ NUM_STROBES ] = {
  PIN_ST0,
  PIN_ST1,
  PIN_ST2,
  PIN_ST3,
  PIN_ST4
};

static const uint8_t ScanCodePins[ NUM_SCANCODES ] = {
  PIN_Z, // 2^0
  PIN_Y, // 2^1
  PIN_X, // 2^2
  PIN_W, // 2^3
  PIN_HV,
  PIN_DP
};

// Pin Change Interrupts
// Enable pin change interrupts in the PCICR Pin Change Interrupt Control Register
// by setting the appropriate PCIEn Pin Change Interrupt Enable bit(s)
//   PCIE0 enables Arduino pins 8-13, mask bits PCINT0-5
//   PCIE1 enables Arduino pins A0-A5, mask bits PCINT8-13
//   PCIE2 enables Arduino pins 0-7, mask bits PCINT16-23
// Set the desired pin(s) in the appropriate PCMSKn Pin Change Mask Register
//   PCMSK0 handles bits PCINT0-5
//   PCMSK1 handles bits PCINT8-13
//   PCMSK2 handles bits PCINT16-23
// for example, enable a pin change interrupt on Arduino pin 7
// PCICR = _BV( PCIE2 );
// PCMSK2 = _BV( PCINT23 );
//
//------------------------------------------------------------------
// Arduino pins for the 8050A strobe lines are defined in PIN_ST0-4.
// Adjust the following values, accordingly.
//------------------------------------------------------------------
//
// the control register interrupt enable bit
#define STROBE_INTERRUPT_ENABLE _BV( PCIE1 )
// pin change mask register
#define STROBE_INTERRUPT_MASK_REGISTER PCMSK1
// The ISR is called on the vector PCINTn_vect, corresponding PCIEn.
#define STROBE_INTERRUPT_VECTOR PCINT1_vect
// This array maps the Arduino pins used for the strobes to
// the appropriate bit-mask for setting the PCMSKn register.
static const uint8_t StrobePin2PCINT[ NUM_STROBES ] = {
  _BV( PCINT8 ),
  _BV( PCINT9 ),
  _BV( PCINT10 ),
  _BV( PCINT11 ),
  _BV( PCINT12 )
};

// number of digits in the display
#define NUM_DIGITS 6

/*
TM1637 segment mapping DP, G, F, E, D, C, B, A and numbers
*/
#define LED_SEGMENT_A B00000001
#define LED_SEGMENT_B B00000010
#define LED_SEGMENT_C B00000100
#define LED_SEGMENT_D B00001000
#define LED_SEGMENT_E B00010000
#define LED_SEGMENT_F B00100000
#define LED_SEGMENT_G B01000000
#define LED_SEGMENT_DP B10000000
#define LED_SEGMENT_ALL B01111111
#define LED_SEGMENT_NONE B00000000
#define LED_SEGMENT_N0 B00111111
#define LED_SEGMENT_N1 B00000110
#define LED_SEGMENT_N2 B01011011
#define LED_SEGMENT_N3 B01001111
#define LED_SEGMENT_N4 B01100110
#define LED_SEGMENT_N5 B01101101
#define LED_SEGMENT_N6 B01111101
#define LED_SEGMENT_N7 B00000111
#define LED_SEGMENT_N8 B01111111
#define LED_SEGMENT_N9 B01101111

// for TM1637 display
TM1637TinyDisplay6 display(CLK, DIO); // 6-Digit Display Class

/*
GLOBALS
*/

// strobe position (ISR)
volatile uint8_t st;

// 8050A scancode buffer (ISR)
volatile uint8_t scanCodes[ NUM_STROBES ][ NUM_SCANCODES ];

// update display flag (ISR)
volatile boolean displayUpdate = false;

// TM1637 digit data
uint8_t digits[ NUM_DIGITS ] = {
  LED_SEGMENT_NONE,
  LED_SEGMENT_NONE,
  LED_SEGMENT_NONE,
  LED_SEGMENT_NONE,
  LED_SEGMENT_NONE,
  LED_SEGMENT_NONE
};

uint8_t digits_segments[ 10 ] = {
LED_SEGMENT_N0, 
LED_SEGMENT_N1, 
LED_SEGMENT_N2,
LED_SEGMENT_N3,
LED_SEGMENT_N4,
LED_SEGMENT_N5,
LED_SEGMENT_N6,
LED_SEGMENT_N7,
LED_SEGMENT_N8,
LED_SEGMENT_N9,
}; 

// not used - low-battery indicator
// boolean loBatt = false;

/*
FUNCTIONS
*/

// pin change interrupt service routine
// - reads the scancode values from the 8050A
// - sets the displayUpdate flag when all strobes are read
ISR( STROBE_INTERRUPT_VECTOR )
{
  if ( digitalRead( StrobePins[ st ] ) == LOW )
  {
    // that was a falling transition
    // (In the 4054/55/56 LCD drivers datasheet, it shows
    // the data being latched on the strobe falling edge.)
    // now, read the scan code values
    for ( uint8_t i = SC0; i < NUM_SCANCODES; i++ )
    {
      scanCodes[ st ][ i ] = digitalRead( ScanCodePins[ i ] );
    }
    // next time, next strobe
    st++;
    if ( st >= NUM_STROBES )
    {
      // scancodes for all strobes are read
      st = ST0; // reset strobe counter
      // now is the time to update the display
      displayUpdate = true;
    }
    // enable the appropriate strobe pin
    STROBE_INTERRUPT_MASK_REGISTER = StrobePin2PCINT[ st ];
  }
}

void formatDigits()
{
  // clear digits
  for ( uint8_t i = 0; i < NUM_DIGITS; i++ )
  {
    digits[ i ] = 0;
  }
   // convert BCD coding in digits 1 to 4 to LCD segment coding
  uint8_t code;
  uint8_t mask;
  // obtain BCD codes from scancodes
  for ( uint8_t i = ST1; i <= ST4; i++ )
  {
    code = SCANCODE_W; // MSB
    mask = B00001000;
    while ( mask )
    {
      if ( scanCodes[ i ][ code ] )
        digits[ i ] |= mask;
      code--;
      mask >>= 1;
    }
     // Convert BCD coding to LCD segment coding
     if (digits[ i ] <= 0x9) {
     digits[ i ] = digits_segments [digits[ i ]]; 
     }
     else digits[ i ] = LED_SEGMENT_NONE; // for blank case 0xf
    }    
  // set digit 0 (minus, 1, indicator BT)
  // In the LED driver this digit uses no segment decode.
  if ( scanCodes[ ST0 ][ SCANCODE_MINUS ] && ! scanCodes[ ST0 ][ SCANCODE_PLUS ] )
  {
  	// The 8050A LCD displays both minus and plus signs.
  	// The plus sign is made up of the minus sign and the vertical plus segments.
  	// So, to correctly display minus, check that it is not on as a part of plus.  
    digits[ 0 ] |= LED_SEGMENT_G;
  }
  if ( scanCodes[ ST0 ][ SCANCODE_1 ] )
  {
    digits[ 0 ] |= (LED_SEGMENT_B | LED_SEGMENT_C);
  }
  //   if ( loBatt ) // not  used
  //   {
  //     digits[ 0 ] |= LED_SEGMENT_A;
  //   }
  //
  // Note - the following code was moved down from above so as not to interfere 
  //   with revised BCD to LED segment coding
  // set decimal points 
  // In the 8050A LCD display the dp is to the left of the digit, i.e., strobe 1-4.
  // In the LED display, the dp is to the right, so map to digits 0-3.
  for ( uint8_t i = ST1; i <= ST4; i++ )
  {
    if ( scanCodes[ i ][ SCANCODE_DP ] )
    {
      digits[ i - 1 ] |= LED_SEGMENT_DP;
     }
  }  
  // set digit 5 (indicators dB, HV, REL)
  // In the LED driver this digit 5 uses no segment decode.
  if ( scanCodes[ ST0 ][ SCANCODE_DB ] )
  {
    digits[ 5 ] |= LED_SEGMENT_A;
  }
  if ( scanCodes[ ST0 ][ SCANCODE_HV ] )
  {
    digits[ 5 ] |= LED_SEGMENT_G;
  }
  if ( scanCodes[ ST0 ][ SCANCODE_REL ] )
  {
    digits[ 5 ] |= LED_SEGMENT_D;
  }  
  // Continuity
  //   Determine if in ohm mode (volt and ma not selected) and 200 ohm range selected
  //   If so, tone should be played if < 10 ohms or <100 ohms as desired.
  bool volt_test = false; 
  bool ma_test = false; 
  bool r200_test = false;
  bool digit0_test = false;
  bool digit1_test = true; // set to false if a value less than 100 is desired
  if( digitalRead(PIN_Fb_VOLT) == LOW) volt_test = true; 
  if( digitalRead(PIN_Fc_MA) == LOW) ma_test = true; 
  if( (digitalRead(PIN_RNGa) == HIGH) && 
      (digitalRead(PIN_RNGb) == HIGH) && 
      (digitalRead(PIN_RNGc) == LOW) ) r200_test = true; 
  if( !(scanCodes[ ST0 ][ SCANCODE_1 ])) digit0_test = true;
  if( volt_test && ma_test && r200_test && digit0_test && digit1_test)
    {
    // modify tone pitch as desired
    int freq = 2000;
    tone(PIN_TONE, freq); 
    }
    else {
      noTone(PIN_TONE);
    }
  return;
}

void writeDigits()
{
display.setSegments(digits);
}

void updateDisplay()
{
  // displayUpdate flag is set in the ISR
  if ( displayUpdate )
  {
    displayUpdate = false;
    formatDigits();
    writeDigits();
  }
}  

// The Arduino setup and loop
void setup(void)
{
  //Serial.begin(9600); // for debugging with serial monitor 
  // for TM1637 display
  display.setBrightness(BRIGHT_HIGH);
  display.clear();
  // disable ADC (to reduce power)
  ADCSRA = 0; 
  // initialize strobe and scancode pins
  for ( uint8_t i = ST0; i < NUM_STROBES; i++ )
  {
    pinMode( StrobePins[ i ], INPUT );
  }
  for ( uint8_t i = 0; i < NUM_SCANCODES; i++ )
  {
    pinMode( ScanCodePins[ i ], INPUT );
  }
  // enable pin change interrupts
  cli();
  st = ST0; // initialize the strobe counter
  PCICR = STROBE_INTERRUPT_ENABLE;
  STROBE_INTERRUPT_MASK_REGISTER = StrobePin2PCINT[ st ];
  sei();
  // for continuity testing
  pinMode( PIN_Fb_VOLT, INPUT ); // note - no pull up resistors on inputs
  pinMode( PIN_Fc_MA, INPUT );  
  pinMode( PIN_RNGa, INPUT );
  pinMode( PIN_RNGb, INPUT );
  pinMode( PIN_RNGc, INPUT );
}

void loop(void)
{
  updateDisplay();
}


/*  
Connections
   
The 328P and 8050A ICs use GND as their +5 VCC and -5 as their GND
  
  328P IC   8050A IC        other board connects

1   RST     NC              connects to 8050A IC pin 40 VCC(gnd) through 10k resistor
2   D0      6   Fb (volt)
3   D1      19  Fc (mA)
4   D2      37  Z
5   D3      36  Y
6   D4      35  X
7   VCC     40  VCC (gnd)   also connects to LED VCC
8   GND     20  GND (-5)    also connects to LED GND and passive buzzer
9   XTAL    NC              connects to GND through 20pf cap
10  XTAL    NC              connects to GND through 20pf cap
11  D5      34  W
12  D6      23  HV
13  D7      22  DP
14  D8      3   RNGa
15  D9      4   RNGb
16  D10     5   RNGc
17  D11     NC              connects to passive buzzer through 100 ohm resistor
18  D12     NC              connects to LED CLK
19  D13     NC              connects to LED DIO
20  VCC     40  VCC (gnd)
21  AREF    NC
22  GND     20  GND (-5)
23  A0      15  ST0
24  A1      14  ST1
25  A2      13  ST2
26  A3      12  ST3
27  A4      11  ST4
28  A5      NC

LED Display

VCC    connects to pin 40 of 8050A IC
GND    connects to pin 20 of 8050A IC
DIO    connects to pin 19 of 328P
CLK    connects to pin 18 of 328P

*/
