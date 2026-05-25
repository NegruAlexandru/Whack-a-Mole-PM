#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdint.h>

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#define SET_BIT(port, bit)   ((port) |=  (1 << (bit)))
#define CLR_BIT(port, bit)   ((port) &= ~(1 << (bit)))
#define READ_BIT(port, bit)  (((port) >> (bit)) & 0x01)

#define LED_PORT  PORTD
#define LED_DDR   DDRD
static const uint8_t led_bits[4] = {2, 3, 4, 5};

#define BTN_PIN   PINC
#define BTN_PORT  PORTC
#define BTN_DDR   DDRC
static const uint8_t btn_bits[4] = {0, 1, 2, 3};

#define BUZZ_PORT PORTD
#define BUZZ_DDR  DDRD
#define BUZZ_BIT  6

#define SR_DATA_PORT  PORTD
#define SR_DATA_DDR   DDRD
#define SR_DATA_BIT   7
#define SR_LATCH_PORT PORTB
#define SR_LATCH_DDR  DDRB
#define SR_LATCH_BIT  0
#define SR_CLK_PORT   PORTB
#define SR_CLK_DDR    DDRB
#define SR_CLK_BIT    1

#define DIG_PORT  PORTB
#define DIG_DDR   DDRB
static const uint8_t dig_bits[4] = {2, 3, 4, 5};

// =====================================================================
//          MILLIS via Timer0 CTC, 1 ms tick
// =====================================================================
volatile uint32_t millis_count = 0;

ISR(TIMER0_COMPA_vect) {
  millis_count++;
}

static void timer0_init(void) {
  TCCR0A = (1 << WGM01);
  TCCR0B = (1 << CS01) | (1 << CS00); // prescaler 64 -> 16MHz/64/250 = 1kHz
  OCR0A  = 249;
  TIMSK0 = (1 << OCIE0A);
}

static uint32_t millis(void) {
  // uint32_t read is not atomic on 8-bit AVR — guard with cli/sei.
  uint32_t m;
  uint8_t sreg = SREG;
  cli();
  m = millis_count;
  SREG = sreg;
  return m;
}

// =====================================================================
//          BUZZER via Timer1 CTC + manual ISR toggle on PD6
// =====================================================================
// PD6 is not a Timer1 OC pin, so we toggle it manually from ISR.
// Compare-match frequency is 2*tone_freq (toggle twice per period).
volatile uint8_t tone_active = 0;

ISR(TIMER1_COMPA_vect) {
  BUZZ_PORT ^= (1 << BUZZ_BIT);
}

static void tone_init(void) {
  TCCR1A = 0;
  TCCR1B = (1 << WGM12);
  TIMSK1 = 0;
}

static void tone_start(uint16_t freq_hz) {
  if (freq_hz == 0) return;
  // OCR1A = F_CPU / (2 * prescaler * freq) - 1, prescaler=8
  uint32_t ocr = (F_CPU / (2UL * 8UL * (uint32_t)freq_hz)) - 1;
  if (ocr > 0xFFFF) ocr = 0xFFFF;

  uint8_t sreg = SREG;
  cli();
  TCNT1 = 0;
  OCR1A = (uint16_t)ocr;
  TIMSK1 = (1 << OCIE1A);
  TCCR1B = (1 << WGM12) | (1 << CS11); // prescaler 8 -> start
  tone_active = 1;
  SREG = sreg;
}

static void tone_stop(void) {
  uint8_t sreg = SREG;
  cli();
  TIMSK1 = 0;
  TCCR1B = (1 << WGM12); // stop clock source
  CLR_BIT(BUZZ_PORT, BUZZ_BIT);
  tone_active = 0;
  SREG = sreg;
}

// =====================================================================
//          ADC
// =====================================================================
static void adc_init(void) {
  ADMUX  = (1 << REFS0); // AVcc reference, right-adjusted result
  ADCSRA = (1 << ADEN) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); // prescaler 128
}

static uint16_t adc_read(uint8_t channel) {
  ADMUX = (ADMUX & 0xF0) | (channel & 0x0F);
  ADCSRA |= (1 << ADSC);
  while (ADCSRA & (1 << ADSC)) { }
  return ADC;
}

// =====================================================================
//          EEPROM
// =====================================================================
static uint8_t eeprom_read_byte_raw(uint16_t addr) {
  while (EECR & (1 << EEPE)) { }
  EEAR = addr;
  EECR |= (1 << EERE);
  return EEDR;
}

static void eeprom_write_byte_raw(uint16_t addr, uint8_t data) {
  while (EECR & (1 << EEPE)) { }
  // EEMPE->EEPE sequence must execute within 4 cycles, no ISR between them.
  uint8_t sreg = SREG;
  cli();
  EEAR = addr;
  EEDR = data;
  EECR |= (1 << EEMPE);
  EECR |= (1 << EEPE);
  SREG = sreg;
}

// =====================================================================
//          PRNG (xorshift32)
// =====================================================================
static uint32_t prng_state = 1;

static void prng_seed(uint32_t s) {
  if (s == 0) s = 0xDEADBEEFUL;
  prng_state = s;
}

static uint32_t prng_next(void) {
  uint32_t x = prng_state;
  x ^= x << 13;
  x ^= x >> 17;
  x ^= x << 5;
  prng_state = x;
  return x;
}

static uint8_t prng_range(uint8_t n) {
  return (uint8_t)(prng_next() % n);
}

// =====================================================================
//          PINS
// =====================================================================
static void pins_init(void) {
  for (uint8_t i = 0; i < 4; i++) {
    SET_BIT(LED_DDR, led_bits[i]);
    CLR_BIT(LED_PORT, led_bits[i]);
  }
  for (uint8_t i = 0; i < 4; i++) {
    CLR_BIT(BTN_DDR, btn_bits[i]);
    SET_BIT(BTN_PORT, btn_bits[i]); // enable pull-up
  }
  SET_BIT(BUZZ_DDR, BUZZ_BIT);
  CLR_BIT(BUZZ_PORT, BUZZ_BIT);

  SET_BIT(SR_DATA_DDR, SR_DATA_BIT);   CLR_BIT(SR_DATA_PORT, SR_DATA_BIT);
  SET_BIT(SR_LATCH_DDR, SR_LATCH_BIT); CLR_BIT(SR_LATCH_PORT, SR_LATCH_BIT);
  SET_BIT(SR_CLK_DDR, SR_CLK_BIT);     CLR_BIT(SR_CLK_PORT, SR_CLK_BIT);

  for (uint8_t i = 0; i < 4; i++) {
    SET_BIT(DIG_DDR, dig_bits[i]);
    SET_BIT(DIG_PORT, dig_bits[i]); // OFF (common cathode)
  }
}

// =====================================================================
//          SHIFT REGISTER
// =====================================================================
static void shift_out_msb(uint8_t data) {
  for (int8_t i = 7; i >= 0; i--) {
    if (data & (1 << i)) SET_BIT(SR_DATA_PORT, SR_DATA_BIT);
    else                 CLR_BIT(SR_DATA_PORT, SR_DATA_BIT);
    SET_BIT(SR_CLK_PORT, SR_CLK_BIT);
    CLR_BIT(SR_CLK_PORT, SR_CLK_BIT);
  }
}

static void write_segments(uint8_t segs) {
  CLR_BIT(SR_LATCH_PORT, SR_LATCH_BIT);
  shift_out_msb(segs);
  SET_BIT(SR_LATCH_PORT, SR_LATCH_BIT);
}

static void all_digits_off(void) {
  for (uint8_t i = 0; i < 4; i++) {
    SET_BIT(DIG_PORT, dig_bits[i]);
  }
}

// =====================================================================
//          BUTTONS
// =====================================================================
static int8_t get_pressed_button(void) {
  for (uint8_t i = 0; i < 4; i++) {
    if (READ_BIT(BTN_PIN, btn_bits[i]) == 0) return (int8_t)i;
  }
  return -1;
}

// =====================================================================
//          GAME PARAMETERS
// =====================================================================
#define REACTION_TIME_EASY        500
#define REACTION_TIME_HARD        1000
#define TIME_DECREASE_PER_LEVEL   20
#define MIN_REACTION_TIME         300
#define POST_HIT_DELAY            200
#define SCORE_ROLL_DURATION_MS    180
#define SCORE_ROLL_BIG_MS         320
#define HITS_PER_LEVEL            7
#define LEVEL_UP_FLASH_COUNT      3
#define LEVEL_UP_FLASH_MS         100
#define LEVEL_UP_BANNER_MS        1000
#define GAME_OVER_LOCKOUT         1000
#define GAME_OVER_AUTO_IDLE_MS    5000UL
#define GAME_OVER_PHASE_MS        700UL

#define TONE_HIT_HZ        1200
#define TONE_HIT_MS        100
#define TONE_FAST_HZ       1600
#define TONE_FAST_MS       80
#define TONE_COUNTDOWN_HZ  800
#define TONE_COUNTDOWN_MS  120
#define TONE_GO_HZ         1500
#define TONE_GO_MS         250

#define IDLE_CYCLE_MS      1500UL

// Percentage thresholds over the reaction window (avoids float math).
#define FAST_THRESHOLD_NUM   33
#define NORMAL_THRESHOLD_NUM 66
#define POINTS_FAST          10
#define POINTS_NORMAL        5
#define POINTS_SLOW          1

// EEPROM magic byte detects a virgin chip (default 0xFF, not 0).
#define EE_ADDR_MAGIC  0
#define EE_ADDR_HI     1
#define EE_MAGIC_VAL   0x42

// =====================================================================
//          MELODIES
// =====================================================================
#define NOTE_C3  131
#define NOTE_E3  165
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_Cs4 277
#define NOTE_D4  294
#define NOTE_Eb4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_G4  392
#define NOTE_A4  440
#define NOTE_B4  494
#define NOTE_C5  523

typedef struct { uint16_t freq; uint16_t duration; } Note;

// Mario Death — authentic NES transcription (1985).
static const Note sadSong[] = {
  {NOTE_C4,  180},
  {NOTE_Cs4, 180},
  {NOTE_D4,  180},
  {0,        250},
  {NOTE_B3,  180},
  {NOTE_F4,  180},
  {0,        100},
  {NOTE_F4,  180},
  {NOTE_F4,  250},
  {NOTE_E4,  250},
  {NOTE_D4,  250},
  {NOTE_C4,  250},
  {NOTE_E3,  180},
  {0,        100},
  {NOTE_E3,  180},
  {NOTE_C3,  500},
};
#define SAD_SONG_LEN (sizeof(sadSong)/sizeof(sadSong[0]))

static const Note newHighSong[] = {
  {NOTE_C5, 120},
  {NOTE_E4, 120},
  {NOTE_G4, 120},
  {NOTE_C5, 250},
};
#define NEW_HIGH_SONG_LEN (sizeof(newHighSong)/sizeof(newHighSong[0]))

static const Note levelUpSong[] = {
  {NOTE_C5,  90},
  {NOTE_E4, 110},
  {NOTE_G4, 130},
  {NOTE_C5, 220},
};
#define LEVEL_UP_SONG_LEN (sizeof(levelUpSong)/sizeof(levelUpSong[0]))

// =====================================================================
//          SEGMENTS / CHARACTERS
// =====================================================================
// Mapping: bit N of byte -> 74HC595 output QN -> segment N
// (A=0, B=1, C=2, D=3, E=4, F=5, G=6, DP=7). Used with MSBFIRST.
#define SEG_A   (1 << 0)
#define SEG_B   (1 << 1)
#define SEG_C   (1 << 2)
#define SEG_D   (1 << 3)
#define SEG_E   (1 << 4)
#define SEG_F   (1 << 5)
#define SEG_G   (1 << 6)
#define SEG_DP  (1 << 7)

static const uint8_t numMap[10] = {
  SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F,
  SEG_B|SEG_C,
  SEG_A|SEG_B|SEG_G|SEG_E|SEG_D,
  SEG_A|SEG_B|SEG_C|SEG_D|SEG_G,
  SEG_F|SEG_G|SEG_B|SEG_C,
  SEG_A|SEG_F|SEG_G|SEG_C|SEG_D,
  SEG_A|SEG_F|SEG_G|SEG_E|SEG_C|SEG_D,
  SEG_A|SEG_B|SEG_C,
  SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F|SEG_G,
  SEG_A|SEG_B|SEG_C|SEG_D|SEG_F|SEG_G
};

// Letters missing from standard 7-seg are substituted with lowercase variant
// (arcade convention): m->n, v->lowercase v, r->lowercase r.
#define CHAR_H   (SEG_B|SEG_C|SEG_E|SEG_F|SEG_G)
#define CHAR_I   (SEG_B|SEG_C)
#define CHAR_G_  (SEG_A|SEG_C|SEG_D|SEG_E|SEG_F)
#define CHAR_O   (SEG_A|SEG_B|SEG_C|SEG_D|SEG_E|SEG_F)
#define CHAR_A_  (SEG_A|SEG_B|SEG_C|SEG_E|SEG_F|SEG_G)
#define CHAR_n   (SEG_C|SEG_E|SEG_G)
#define CHAR_E   (SEG_A|SEG_D|SEG_E|SEG_F|SEG_G)
#define CHAR_v   (SEG_C|SEG_D|SEG_E)
#define CHAR_r   (SEG_E|SEG_G)
#define CHAR_L   (SEG_D|SEG_E|SEG_F)
#define SEG_DASH SEG_G
#define SEG_BLANK 0x00

// =====================================================================
//          GAME STATE
// =====================================================================
typedef enum { ST_IDLE, ST_PLAYING, ST_GAME_OVER } GameState;

static GameState currentState = ST_IDLE;
static int16_t score = 0;
static int16_t hitCount = 0;
static int8_t  activeLedIndex = -1;
static uint32_t targetTime = 0;
static uint32_t ledOnTime = 0;
static uint32_t currentWindow = 0;
static uint32_t gameOverEnterTime = 0;
static uint8_t  newHighScoreAchieved = 0;
static int16_t  highScore = 0;

static void display_digits(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t dpMask);
static void display_score(int16_t num);
static void display_score_with_timer(int16_t num);
static void display_idle(void);
static void display_countdown(void);
static void display_game_over(void);
static void score_roll_animation(int16_t from, int16_t to, uint16_t duration_ms);
static void level_up_sequence(uint8_t new_level);
static void pick_next_led(void);
static void trigger_game_over(void);
static void custom_delay(uint32_t ms);
static void start_new_game(void);
static void play_melody(const Note *melody, uint8_t len);
static void load_high_score(void);
static void save_high_score(void);
static uint8_t compute_dp_mask(void);
static void tone_blocking(uint16_t freq, uint16_t ms);

// =====================================================================
//          HIGH SCORE
// =====================================================================
static void load_high_score(void) {
  uint8_t magic = eeprom_read_byte_raw(EE_ADDR_MAGIC);
  if (magic != EE_MAGIC_VAL) {
    highScore = 0;
    eeprom_write_byte_raw(EE_ADDR_MAGIC, EE_MAGIC_VAL);
    eeprom_write_byte_raw(EE_ADDR_HI,     0);
    eeprom_write_byte_raw(EE_ADDR_HI + 1, 0);
    return;
  }
  uint8_t hi = eeprom_read_byte_raw(EE_ADDR_HI);
  uint8_t lo = eeprom_read_byte_raw(EE_ADDR_HI + 1);
  int16_t v = (int16_t)((hi << 8) | lo);
  if (v < 0 || v > 9999) v = 0;
  highScore = v;
}

static void save_high_score(void) {
  int16_t s = highScore;
  if (s < 0) s = 0;
  if (s > 9999) s = 9999;
  eeprom_write_byte_raw(EE_ADDR_HI,     (uint8_t)((s >> 8) & 0xFF));
  eeprom_write_byte_raw(EE_ADDR_HI + 1, (uint8_t)(s & 0xFF));
}

// =====================================================================
//          DISPLAY MULTIPLEXING
// =====================================================================
static void display_digits(uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t dpMask) {
  uint8_t digits[4] = {d0, d1, d2, d3};
  for (uint8_t i = 0; i < 4; i++) {
    // Anti-ghosting: turn all digits off before loading new segment data.
    all_digits_off();
    uint8_t segs = digits[i];
    if (dpMask & (1 << i)) segs |= SEG_DP;
    write_segments(segs);
    CLR_BIT(DIG_PORT, dig_bits[i]);
    _delay_us(2000);
  }
  all_digits_off();
}

// Decimal points as timer bar: 4 dots when full window remains, fade off
// left-to-right as the window expires.
static uint8_t compute_dp_mask(void) {
  if (currentWindow == 0) return 0;
  uint32_t now = millis();
  if (now >= targetTime) return 0;
  uint32_t remaining = targetTime - now;
  if (remaining > (currentWindow * 3) / 4) return 0b1111;
  if (remaining > (currentWindow * 2) / 4) return 0b0111;
  if (remaining > (currentWindow * 1) / 4) return 0b0011;
  return 0b0001;
}

static void display_score_internal(int16_t num, uint8_t with_timer) {
  if (num < 0)    num = 0;
  if (num > 9999) num = 9999;

  uint8_t d[4];
  d[0] = (num / 1000) % 10;
  d[1] = (num / 100)  % 10;
  d[2] = (num / 10)   % 10;
  d[3] =  num         % 10;

  // Leading-zero blanking — no pow(), just integer threshold comparisons.
  uint8_t firstSig;
  if (num >= 1000)      firstSig = 0;
  else if (num >= 100)  firstSig = 1;
  else if (num >= 10)   firstSig = 2;
  else                  firstSig = 3;

  uint8_t segs[4];
  for (uint8_t i = 0; i < 4; i++) {
    segs[i] = (i < firstSig) ? SEG_BLANK : numMap[d[i]];
  }

  display_digits(segs[0], segs[1], segs[2], segs[3],
                 with_timer ? compute_dp_mask() : 0);
}

static void display_score(int16_t num)            { display_score_internal(num, 0); }
static void display_score_with_timer(int16_t num) { display_score_internal(num, 1); }

// IDLE: dashes when no high score yet; otherwise cycle through dashes, "HIGH"
// label, and the actual score (3 phases of IDLE_CYCLE_MS each).
static void display_idle(void) {
  if (highScore == 0) {
    display_digits(SEG_DASH, SEG_DASH, SEG_DASH, SEG_DASH, 0);
    return;
  }

  uint32_t phase = (millis() / IDLE_CYCLE_MS) % 3;
  switch (phase) {
    case 0: display_digits(SEG_DASH, SEG_DASH, SEG_DASH, SEG_DASH, 0); break;
    case 1: display_digits(CHAR_H, CHAR_I, CHAR_G_, CHAR_H, 0); break;
    case 2: display_score(highScore); break;
  }
}

// GAME_OVER cycle: skip the score phase if score is 0.
static void display_game_over(void) {
  uint8_t phases = (score > 0) ? 3 : 2;
  uint32_t phase = (millis() / GAME_OVER_PHASE_MS) % phases;
  switch (phase) {
    case 0: display_digits(CHAR_G_, CHAR_A_, CHAR_n, CHAR_E, 0); break;
    case 1: display_digits(CHAR_O, CHAR_v, CHAR_E, CHAR_r, 0); break;
    case 2: display_score(score); break;
  }
}

// Slot-machine effect: first 70% jumps randomly between from..to with noise,
// last 30% locks onto the final value.
static void score_roll_animation(int16_t from, int16_t to, uint16_t duration_ms) {
  if (from == to) return;
  uint32_t start = millis();
  while (1) {
    uint32_t elapsed = millis() - start;
    if (elapsed >= duration_ms) break;
    uint16_t progress = (uint16_t)((elapsed * 100UL) / duration_ms);
    int16_t shown;
    if (progress < 70) {
      uint8_t r = (uint8_t)prng_range(20);
      shown = from + (int16_t)(((int32_t)(to - from) * progress) / 100) + r - 10;
      if (shown < 0) shown = 0;
      if (shown > 9999) shown = 9999;
    } else {
      shown = to;
    }
    display_score_with_timer(shown);
  }
}

static void level_up_sequence(uint8_t new_level) {
  uint8_t d0 = CHAR_L;
  uint8_t d1 = CHAR_v;
  uint8_t d2, d3;
  if (new_level < 10) {
    d2 = SEG_BLANK;
    d3 = numMap[new_level];
  } else {
    d2 = numMap[(new_level / 10) % 10];
    d3 = numMap[new_level % 10];
  }

  for (uint8_t f = 0; f < LEVEL_UP_FLASH_COUNT; f++) {
    for (uint8_t i = 0; i < 4; i++) SET_BIT(LED_PORT, led_bits[i]);
    uint32_t start = millis();
    while ((millis() - start) < LEVEL_UP_FLASH_MS) {
      display_digits(d0, d1, d2, d3, 0);
    }
    for (uint8_t i = 0; i < 4; i++) CLR_BIT(LED_PORT, led_bits[i]);
    start = millis();
    while ((millis() - start) < LEVEL_UP_FLASH_MS) {
      display_digits(d0, d1, d2, d3, 0);
    }
  }

  uint32_t banner_start = millis();
  for (uint8_t i = 0; i < LEVEL_UP_SONG_LEN; i++) {
    if (levelUpSong[i].freq > 0) tone_start(levelUpSong[i].freq);
    else                          tone_stop();
    uint32_t note_start = millis();
    while ((millis() - note_start) < levelUpSong[i].duration) {
      display_digits(d0, d1, d2, d3, 0);
    }
  }
  tone_stop();

  while ((millis() - banner_start) < LEVEL_UP_BANNER_MS) {
    display_digits(d0, d1, d2, d3, 0);
  }
}

// =====================================================================
//          AUDIO
// =====================================================================
static void tone_blocking(uint16_t freq, uint16_t ms) {
  if (freq > 0) tone_start(freq);
  else          tone_stop();
  custom_delay((uint32_t)ms);
  tone_stop();
}

static void play_melody(const Note *melody, uint8_t len) {
  for (uint8_t i = 0; i < len; i++) {
    if (melody[i].freq > 0) tone_start(melody[i].freq);
    else                     tone_stop();
    custom_delay((uint32_t)melody[i].duration);
  }
  tone_stop();
}

// Non-blocking delay: keeps display refresh running during the wait.
static void custom_delay(uint32_t ms) {
  uint32_t start = millis();
  while ((millis() - start) < ms) {
    switch (currentState) {
      case ST_IDLE:      display_idle(); break;
      case ST_PLAYING:   display_score_with_timer(score); break;
      case ST_GAME_OVER: display_game_over(); break;
    }
  }
}

// =====================================================================
//          GAME SEQUENCES
// =====================================================================
static void display_countdown(void) {
  uint32_t start;

  start = millis();
  tone_start(TONE_COUNTDOWN_HZ);
  while ((millis() - start) < 120) display_digits(SEG_BLANK, SEG_BLANK, numMap[3], SEG_BLANK, 0);
  tone_stop();
  while ((millis() - start) < 700) display_digits(SEG_BLANK, SEG_BLANK, numMap[3], SEG_BLANK, 0);

  start = millis();
  tone_start(TONE_COUNTDOWN_HZ);
  while ((millis() - start) < 120) display_digits(SEG_BLANK, SEG_BLANK, numMap[2], SEG_BLANK, 0);
  tone_stop();
  while ((millis() - start) < 700) display_digits(SEG_BLANK, SEG_BLANK, numMap[2], SEG_BLANK, 0);

  start = millis();
  tone_start(TONE_COUNTDOWN_HZ);
  while ((millis() - start) < 120) display_digits(SEG_BLANK, SEG_BLANK, numMap[1], SEG_BLANK, 0);
  tone_stop();
  while ((millis() - start) < 700) display_digits(SEG_BLANK, SEG_BLANK, numMap[1], SEG_BLANK, 0);

  start = millis();
  tone_start(TONE_GO_HZ);
  while ((millis() - start) < TONE_GO_MS) display_digits(SEG_BLANK, CHAR_G_, CHAR_O, SEG_BLANK, 0);
  tone_stop();
  while ((millis() - start) < 600) display_digits(SEG_BLANK, CHAR_G_, CHAR_O, SEG_BLANK, 0);
}

static void start_new_game(void) {
  score = 0;
  hitCount = 0;
  newHighScoreAchieved = 0;
  for (uint8_t i = 0; i < 4; i++) CLR_BIT(LED_PORT, led_bits[i]);

  display_countdown();
  currentState = ST_PLAYING;
  pick_next_led();
}

static void pick_next_led(void) {
  // Avoid repeating the same LED twice in a row (only after the first hit).
  int8_t newIdx;
  do {
    newIdx = (int8_t)prng_range(4);
  } while (newIdx == activeLedIndex && score > 0);

  activeLedIndex = newIdx;
  SET_BIT(LED_PORT, led_bits[activeLedIndex]);

  // Difficulty steps once per level, not once per hit.
  // level = hitCount / HITS_PER_LEVEL, so the window is constant within a level
  // and drops by TIME_DECREASE_PER_LEVEL exactly when a new level begins.
  uint16_t potValue = adc_read(4);
  int32_t reactionTime = REACTION_TIME_EASY +
       ((int32_t)(REACTION_TIME_HARD - REACTION_TIME_EASY) * (int32_t)potValue) / 1023;
  int32_t level = (int32_t)hitCount / HITS_PER_LEVEL;
  reactionTime -= level * TIME_DECREASE_PER_LEVEL;
  if (reactionTime < MIN_REACTION_TIME) reactionTime = MIN_REACTION_TIME;

  ledOnTime     = millis();
  currentWindow = (uint32_t)reactionTime;
  targetTime    = ledOnTime + currentWindow;
}

static void trigger_game_over(void) {
  if (activeLedIndex != -1) {
    CLR_BIT(LED_PORT, led_bits[activeLedIndex]);
  }
  activeLedIndex = -1;
  currentState = ST_GAME_OVER;

  if (score > highScore) {
    highScore = score;
    save_high_score();
    newHighScoreAchieved = 1;
  }

  // AFK timer starts before the melody, so the 5s budget covers TOTAL game-over time.
  gameOverEnterTime = millis();

  if (newHighScoreAchieved) {
    play_melody(newHighSong, NEW_HIGH_SONG_LEN);
  } else {
    play_melody(sadSong, SAD_SONG_LEN);
  }
}

// =====================================================================
//          MAIN
// =====================================================================
int main(void) {
  pins_init();
  timer0_init();
  tone_init();
  adc_init();

  sei();

  write_segments(SEG_BLANK);

  // Seed PRNG from ADC noise on a floating pin (PC5).
  uint32_t seed = 0;
  for (uint8_t i = 0; i < 16; i++) {
    seed = (seed << 1) ^ adc_read(5);
  }
  prng_seed(seed);

  load_high_score();

  while (1) {
    switch (currentState) {
      case ST_IDLE:      display_idle(); break;
      case ST_PLAYING:   display_score_with_timer(score); break;
      case ST_GAME_OVER: display_game_over(); break;
    }

    switch (currentState) {

      case ST_IDLE: {
        if (get_pressed_button() != -1) {
          while (get_pressed_button() != -1) { display_idle(); }
          start_new_game();
        }
        break;
      }

      case ST_PLAYING: {
        int8_t btn = get_pressed_button();

        if (btn == activeLedIndex) {
          // Reaction is measured at PRESS time, not after release.
          uint32_t reaction = millis() - ledOnTime;
          while (get_pressed_button() != -1) { display_score_with_timer(score); }

          uint32_t fast_thr   = (currentWindow * FAST_THRESHOLD_NUM) / 100;
          uint32_t normal_thr = (currentWindow * NORMAL_THRESHOLD_NUM) / 100;

          int16_t pointsAwarded;
          uint8_t wasFast = 0;
          if (reaction <= fast_thr) {
            pointsAwarded = POINTS_FAST;
            wasFast = 1;
          } else if (reaction <= normal_thr) {
            pointsAwarded = POINTS_NORMAL;
          } else {
            pointsAwarded = POINTS_SLOW;
          }
          int16_t old_score = score;
          score += pointsAwarded;
          hitCount++;

          CLR_BIT(LED_PORT, led_bits[activeLedIndex]);
          if (wasFast) tone_blocking(TONE_FAST_HZ, TONE_FAST_MS);
          else         tone_blocking(TONE_HIT_HZ,  TONE_HIT_MS);

          // Score animation replaces the standard pause (does not add on top).
          if (pointsAwarded >= POINTS_FAST) {
            score_roll_animation(old_score, score, SCORE_ROLL_BIG_MS);
          } else if (pointsAwarded >= POINTS_NORMAL) {
            score_roll_animation(old_score, score, SCORE_ROLL_DURATION_MS);
          } else {
            custom_delay(POST_HIT_DELAY);
          }

          if (hitCount > 0 && (hitCount % HITS_PER_LEVEL) == 0) {
            uint8_t new_level = (uint8_t)(hitCount / HITS_PER_LEVEL) + 1;
            level_up_sequence(new_level);
          }

          pick_next_led();
        }
        else if (btn != -1 && btn != activeLedIndex) {
          trigger_game_over();
        }
        else if (millis() > targetTime) {
          trigger_game_over();
        }
        break;
      }

      case ST_GAME_OVER: {
        uint32_t sinceEnter = millis() - gameOverEnterTime;

        if (sinceEnter < (uint32_t)GAME_OVER_LOCKOUT) {
          break;
        }

        int8_t btn = get_pressed_button();
        if (btn != -1) {
          while (get_pressed_button() != -1) { display_game_over(); }
          start_new_game();
          break;
        }

        if (sinceEnter > GAME_OVER_AUTO_IDLE_MS) {
          currentState = ST_IDLE;
        }
        break;
      }
    }
  }

  return 0;
}
