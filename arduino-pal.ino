// AV data for display on CRT screen
// code for Arduino Uno (16 MHz)
//
// Author: Chris Petrich, 2025
// License: MIT
//
// Hardware:
// Connect yellow RCA connector (AV input) of PAL CRT with
//   shield to GND and center pin to a point that is
//   connected to:
//      PIN_SYNC via 1 kOhm, and
//      PIN_VIDEO via 470 Ohm, and
//      possibly GND to 75 Ohm.
//   The latter 75 Ohm resistor can be omitted for brighter whites,
//   i.e. only two resistors are needed.
//
// Software:
// The sketch draws a gray frame around a black "text field" onto which
// a field of white blocks is rendered.
// The field is held in RAM and has dimensions x=86, y=57.
// This demonstration sketch lets a small white dot zap around to take 
// out each white dot it encounters along the way.
// Sending any serial data will reset the display field.
//
// The display is flicker-free on my screen when compiled with Arduino IDE
// avg-g++ 7.3.0-atmel3.6.1-arduino7.
//
// This script generates fields of "fake progressive" monochrome PAL video
// as defined here: https://www.batsocks.co.uk/readme/video_timing.htm
//
// Each half-frame consists of the following sequence of scanlines (64 us):
// 1: broad sync, broad sync
// 2: broad sync, broad sync
// 3: broad sync, short sync
// 4: short sync, short sync
// 5: short sync, short sync
// 6-309: image data (display area starts at line 23.5)
// 310-312: short sync, short sync

#define PIN_VIDEO (8)
#define PIN_SYNC (11)

// SYNC        Video=L Sync=L
// BLACK/BLANK Video=L Sync=H
// GRAY        Video=H Sync=L
// WHITE       Video=H Sync=H

#define SET_SYNC (PORTB = 0x00)
#define SET_GRAY (PORTB = 0x01)
#define SET_BLACK (PORTB = 0x08) // used for both black and blank
#define SET_WHITE (PORTB = 0x09)

// All operations are performed inside the TIMER1_COMPA and TIMER1_COMPB ISRs (interrupt handlers).
// Both A and B ISR have the same start-up delay, so we can completely ignore that and pretend they act instantaneous.
//    However, since the line sync happens every 64 us only if the processor had exited the ISR already, the start-up
//    and wrap-up times of the ISR are missing for data transmission and the text safe area does not extent as far to
//    the right as it should.
// TIMER1_COMPA is configured to clear the timer on match and used to ensure that the sync signal appears either every
//    32 us or every 64 us, precisely.
// TIMER1_COMPB is used to terminate the sync pulse (0 V), possibly followed by the transfer of picture information.
// The horizontal sync pulse at the start of a scanline is preceeded by a short front-porch blanking period at the
//    end of the previous scanline. This fron-porch is placed at the beginning of the TIMER1_COMPA period, adusting the
//    lengths of the preceeding and following sync frames accordingly. This allows us to fill the screen with a gray frame
//    all the way to the right hand side edge of the screen.

#define TICKS_LINE (1024)
#define TICKS_HALF_LINE (512)
#define TICKS_FRONT_PORCH (26) // 1.65 us
#define TICKS_BACK_PORCH (91) // 5.7 us -- if this is too short then the output will be too dark
#define TICKS_GAP (75) // 4.7 us (75) gap between broad sync pulses
#define TICKS_HSYNC (75) // 4.7 us start of display scanline
#define TICKS_PULSE (38) // 2.35 us sync pulse in short sync
#define TICKS_ISR_ENTER (23)
#define TICKS_ISR_EXIT (19)

// used by TIMER1_COMPA
void line_32us_broad_sync();
void line_32us_short_sync();
void line_32us_short_sync_with_front_porch();
void line_64us_blank();
void line_64us_image();
void (*line_jump_pad)() = line_32us_broad_sync;

// used by TIMER1_COMPB
void line_blank();
void line_back_porch();
void (*compb_action)() = line_blank;

// we use 0-based count of half scanlines, i.e. of length 32 us
int16_t half_line_no = 0; // needs to be signed

// this is where the frame is updated during a
//   sync period:
void loop_action();

// frame counter, not currently used.
int16_t half_frame_counter = 0;

// cycle-accurate delay, similar to _delay_us() but without floats
#define DELAY_CYCLES(cnt) __builtin_avr_delay_cycles(cnt)

// shorthand to remove repetitive code
#define SEND_SIX_BITS {          \
      PORTB = 8 | (val & 0x01);  \
      val >>= 1;                 \
      DELAY_CYCLES(dcycles);     \
      PORTB = 8 | (val & 0x01);  \
      val >>= 1;                 \
      DELAY_CYCLES(dcycles);     \
      PORTB = 8 | (val & 0x01);  \
      val >>= 1;                 \
      DELAY_CYCLES(dcycles);     \
      PORTB = 8 | (val & 0x01);  \
      val >>= 1;                 \
      DELAY_CYCLES(dcycles);     \
      PORTB = 8 | (val & 0x01);  \
      val >>= 1;                 \
      DELAY_CYCLES(dcycles);     \
      PORTB = 8 | (val & 0x01);  \
      val >>= 1;                 \
      DELAY_CYCLES(dcycles); }

#define ROUND_UP(a, b) (((a) + ((b)-1)) / (b))

const int visible_lines = 230; // Corresponding to PAL standard. Could be adjusted.
const uint8_t n_rows = 64;
const int divisor = ROUND_UP(visible_lines, n_rows);
const int dcycles = 2; // choose either 2 or 3
const int screen_x = dcycles == 3 ? 76 : 86; // empirical constants: beyond that we're using too much time to exit the ISR
const int screen_y = visible_lines / divisor; // NB: this is rounded down, so we're usually using less than visible_lines
const uint8_t n_cols = ROUND_UP(screen_x, 8) * 8; // e.g. 88 bits, i.e. 11 bytes, large enough to fit screen_x

volatile uint8_t field[n_rows * (n_cols / 8) + 4]; // use 4 bytes extra to be sure we access only this array in multi-byte manner

// black magic variable declaration:
//   current_line is a temporary buffer declared here and as volatile to make make gcc emit code with consistent timing
volatile int16_t current_line;

void line_blank() {
  SET_BLACK;
}

void line_back_porch() {
  SET_BLACK;
  // this is to adjust the brightness. Don't make this too short.
  _delay_us(TICKS_BACK_PORCH*0.0625);  
  // withou wasting time continue straight to draw the picture, start with a gray frame:
  SET_GRAY;
    
  if ((half_line_no >= 102) && (half_line_no < 102+screen_y*divisor*2)) {
    
    current_line = (half_line_no >> 1) - 51;
    volatile uint8_t *now = &field[(current_line / divisor) * (n_cols / 8)];
    uint16_t val = *((uint16_t*)now);
    _delay_us(1.5);

    // unrolled loop for consistent speed
    {
      PORTB = 8 | (val & 0x01);
      val >>= 1;
      now += 2;
      DELAY_CYCLES(dcycles-1);
      SEND_SIX_BITS;
      PORTB = 8 | (val & 0x01);
      val >>= 1;
      DELAY_CYCLES(dcycles);
      //
      PORTB = 8 | (val & 0x01);
      val >>= 1;
      DELAY_CYCLES(dcycles);
      SEND_SIX_BITS;
      PORTB = 8 | (val & 0x01);

      val = *((uint16_t*)now);
      PORTB = 8 | (val & 0x01);
      val >>= 1;
      now += 2;
      DELAY_CYCLES(dcycles-1);
      SEND_SIX_BITS;
      PORTB = 8 | (val & 0x01);
      val >>= 1;
      DELAY_CYCLES(dcycles);
      //
      PORTB = 8 | (val & 0x01);
      val >>= 1;
      DELAY_CYCLES(dcycles);
      SEND_SIX_BITS;
      PORTB = 8 | (val & 0x01);

      val = *((uint16_t*)now);
      PORTB = 8 | (val & 0x01);
      val >>= 1;
      now += 2;
      DELAY_CYCLES(dcycles-1);
      SEND_SIX_BITS;
      PORTB = 8 | (val & 0x01);
      val >>= 1;
      DELAY_CYCLES(dcycles);
      //
      PORTB = 8 | (val & 0x01);
      val >>= 1;
      DELAY_CYCLES(dcycles);
      SEND_SIX_BITS;
      PORTB = 8 | (val & 0x01);

      val = *((uint16_t*)now);
      PORTB = 8 | (val & 0x01);
      val >>= 1;
      now += 2;
      DELAY_CYCLES(dcycles-1);
      SEND_SIX_BITS;
      PORTB = 8 | (val & 0x01);
      val >>= 1;
      DELAY_CYCLES(dcycles);
      //
      PORTB = 8 | (val & 0x01);
      val >>= 1;
      DELAY_CYCLES(dcycles);
      SEND_SIX_BITS;
      PORTB = 8 | (val & 0x01);

      val = *((uint16_t*)now);
      PORTB = 8 | (val & 0x01);
      val >>= 1;
      now += 2;
      DELAY_CYCLES(dcycles-1);
      SEND_SIX_BITS;
      PORTB = 8 | (val & 0x01);
      val >>= 1;
      DELAY_CYCLES(dcycles);
      //
      PORTB = 8 | (val & 0x01);
      val >>= 1;
      DELAY_CYCLES(dcycles);
      PORTB = 8 | (val & 0x01);
      val >>= 1;
      DELAY_CYCLES(dcycles);
      PORTB = 8 | (val & 0x01);
      val >>= 1;
      DELAY_CYCLES(dcycles);
      PORTB = 8 | (val & 0x01);
      val >>= 1;
      DELAY_CYCLES(dcycles);

      if (dcycles <= 2) {
        PORTB = 8 | (val & 0x01);
        val >>= 1;
        DELAY_CYCLES(dcycles);
        PORTB = 8 | (val & 0x01);
        val >>= 1;
        DELAY_CYCLES(dcycles);
        PORTB = 8 | (val & 0x01);
        val >>= 1;
        DELAY_CYCLES(dcycles);
        PORTB = 8 | (val & 0x01);
        val >>= 1;

        val = *((uint16_t*)now);
        SEND_SIX_BITS;
      }
      DELAY_CYCLES(2);
    }
    SET_GRAY;
  } else if (half_line_no > 600) {
    // test only while we're in the border
    // below the text safe area since this
    // test introduces jitter
      if (UCSR0A & (1<<RXC0)) {
        UDR0 = UDR0;
        // NB: init_field takes longer than the time
        //   available, so the picture will be unstable
        //   during this time
        init_field();
      }
  }
}

void line_32us_broad_sync() {
  SET_SYNC;
  OCR1B = TICKS_HALF_LINE - TICKS_GAP; // set to 4.7 us (75)
  compb_action = &line_blank;
  OCR1A = TICKS_HALF_LINE; // 32 us

  if (half_line_no >= 4)
    line_jump_pad = &line_32us_short_sync;

  // we process the animation
  //   during an arbitrarily chosen half_line_no:
  if (half_line_no == 1)
    loop_action();  
  half_line_no += 1; // line number of next call
}

void line_32us_short_sync_with_front_porch() {
  SET_BLACK; // front porch
  _delay_us(TICKS_FRONT_PORCH * 0.0625);
  SET_SYNC;
  OCR1A = TICKS_HALF_LINE + TICKS_FRONT_PORCH; // 32 us plus front porch that belonged to the previous scanline
  OCR1B = TICKS_PULSE;
  compb_action = &line_blank;

  line_jump_pad = &line_32us_short_sync;
  half_line_no += 1; // line number of next call
}

void line_32us_short_sync() {
  SET_SYNC;
  OCR1A = TICKS_HALF_LINE; // 32 us.
  OCR1B = TICKS_PULSE;
  compb_action = &line_blank;

  if (half_line_no == 9) {
    // make THIS current half-scanline shorter so the front porch appears at the start of the 64 us intervals rather than at the end
    OCR1A = TICKS_HALF_LINE - TICKS_FRONT_PORCH;
    line_jump_pad = &line_64us_blank;
  }
  else if (half_line_no >= 623) {
    line_jump_pad = &line_32us_broad_sync;
    half_line_no = -1; // will be incremented to 0 below
    half_frame_counter += 1; // for future use
  }
  half_line_no += 1; // line number of next call
}

void line_64us_blank() {
  SET_BLACK; // front porch
  _delay_us(TICKS_FRONT_PORCH * 0.0625);
  SET_SYNC;
  OCR1A = TICKS_LINE; // 64 us
  OCR1B = TICKS_FRONT_PORCH + TICKS_HSYNC;
  compb_action = &line_blank;
  if (half_line_no >= 44)
    line_jump_pad = &line_64us_image;

  half_line_no += 2; // half-line number of next call
}

void line_64us_image() {
  // the front porch is actually at the end of a line
  //   but we send it at the start of a line that starts
  //   early by TICKS_FRONT_PORCH because it is short
  //   compared to the time required to end the ISR.
  SET_BLACK; // front porch
  _delay_us(TICKS_FRONT_PORCH * 0.0625);
  SET_SYNC; 
  OCR1A = TICKS_LINE; // 64 us
  OCR1B = TICKS_FRONT_PORCH + TICKS_HSYNC;
  compb_action = &line_back_porch;

  if (half_line_no >= 616)
    line_jump_pad = &line_32us_short_sync_with_front_porch;

  half_line_no += 2; // half-line number of next call
}

// entering the ISR takes 7 cycles, i.e. 0.4375 us
// plus 16 cycles on entry (=> total 1.4 us), and 19 cycles on exit (1.2 us)
ISR(TIMER1_COMPA_vect)
{
  line_jump_pad();
}

ISR(TIMER1_COMPB_vect)
{
  compb_action();
}

void setup() {
  Serial.begin(115200);
  Serial.print("screen_x = "); // e.g. 86
  Serial.println(screen_x, DEC);
  Serial.print("screen_y = "); // e.g. 57
  Serial.println(screen_y, DEC);
  Serial.flush();

  UCSR0B = (1 << RXEN0) | (1 << TXEN0); // disable UART Rx Tx interrupt (and hence Serial...) but enable receive to poll UART registers instead.
  while (UCSR0A & (1<<RXC0)) UDR0 = UDR0; // clear receive FIFO

  pinMode(PIN_VIDEO, OUTPUT);
  pinMode(PIN_SYNC, OUTPUT);  
  digitalWrite(PIN_VIDEO, LOW);
  digitalWrite(PIN_SYNC,LOW);

  cli();
  TIMSK0 = 0; // disable Timer0 (used for millis() etc)
  TIMSK2 = 0; // disable Timer2 (used for tone())

  // use 16-bit Timer1
  TCCR1A = 0;
  TCCR1B = 0; // stop timer
  TCNT1 = 0; // set counter to 0
  OCR1A = 100; // set counter value on interrupt. Use anything >> 0. Will be adjusted by IRQ handler.
  OCR1B = 0xFFFF; // set OCR1B > OCR1A
  TCCR1B |= (1 << WGM12); // CTC mode (i.e. clear on OCR1A interrupt)

  TIFR1 = 0xFF; // clear all IRQ flags
  TCCR1B |= 0x01; // set prescaler to 1 (16 MHz) ==> start timer
  TIMSK1 = (1 << OCIE1A) | (1 << OCIE1B); // enable IRQ on Compare A and B
  sei();

  init_field();
}

void init_field() {
  // clear field
  for (int y=0; y<n_rows; y++)
    for (int x=0; x<n_cols; x+=8)
      field[y * (n_cols/8) + x / 8] = 0;
  // draw frame
  for (int y=0; y<n_rows; y+=screen_y-1)
    for (int x=0; x<n_cols; x++)
      field[y * (n_cols / 8) + x / 8] |= (1 << (x & 0x07));
  for (int y=0; y<n_rows; y+=1)
    for (int x=0; x<n_cols; x+=screen_x-1)
      field[y * (n_cols / 8) + x / 8] |= (1 << (x & 0x07));  
  // draw center block
  for (int y=screen_y/2-4; y<screen_y/2+4; y+=1)
    for (int x=42-6; x<42+6; x+=1)
      field[y * (n_cols / 8) + x / 8] |= (1 << (x & 0x07));
}

void loop() {
	// do nothing. This is to avoid flickering since the
  // timer ISR cannot interrupt commands that take multiple
  // cycles to complete.
}

void loop_action() {
  // this is called at a suitable time during a sync frame,
  //   so keep it short.
  static int x = 0;
  static int dydx_10th = 10;
  static int y = 10;
  static int y_10th = y*10;
  static int dir = 0;
  
	field[y * (n_cols / 8) + (x / 8)] &= 0xFF - (1 << (x & 0x7));
	if (dir == 0) {
	  x += 1;
	  if (x >= screen_x-1)
		dir = 1-dir;
	} else {
	  x -= 1;
	  if (x < 1)
		dir = 1-dir;
	}
	y_10th += dydx_10th;
	if (y_10th < 0) {
	  y_10th -= 2* dydx_10th;
	  dydx_10th *= -1;
	}
	if (y_10th > 10*(screen_y-1)) {
	  y_10th -= 2* dydx_10th;
	  dydx_10th *= -1;
	  if ((x < screen_x / 8)) x += 1;
	}
	y = (y_10th+5) / 10;
	field[y * (n_cols / 8) + (x / 8)] |= (1 << (x & 0x7));
}
