// granular synth based on Peter Knight's
// https://code.google.com/archive/p/tinkerit/wikis/Auduino.wiki


#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/sleep.h>

// Smooth logarithmic mapping
//
constexpr uint16_t antilogTable[] = {
  64830,64132,63441,62757,62081,61413,60751,60097,59449,58809,58176,57549,56929,56316,55709,55109,
  54515,53928,53347,52773,52204,51642,51085,50535,49991,49452,48920,48393,47871,47356,46846,46341,
  45842,45348,44859,44376,43898,43425,42958,42495,42037,41584,41136,40693,40255,39821,39392,38968,
  38548,38133,37722,37316,36914,36516,36123,35734,35349,34968,34591,34219,33850,33486,33125,32768
};
constexpr uint16_t mapPhaseInc(uint16_t input) {
  return (antilogTable[input & 0x3f]) >> (input >> 6);
}

// Stepped chromatic mapping
//
//uint16_t midiTable[] = {
//  17,18,19,20,22,23,24,26,27,29,31,32,34,36,38,41,43,46,48,51,54,58,61,65,69,73,
//  77,82,86,92,97,103,109,115,122,129,137,145,154,163,173,183,194,206,218,231,
//  244,259,274,291,308,326,346,366,388,411,435,461,489,518,549,581,616,652,691,
//  732,776,822,871,923,978,1036,1097,1163,1232,1305,1383,1465,1552,1644,1742,
//  1845,1955,2071,2195,2325,2463,2610,2765,2930,3104,3288,3484,3691,3910,4143,
//  4389,4650,4927,5220,5530,5859,6207,6577,6968,7382,7821,8286,8779,9301,9854,
//  10440,11060,11718,12415,13153,13935,14764,15642,16572,17557,18601,19708,20879,
//  22121,23436,24830,26306
//};

constexpr uint16_t Notes[] = {
  145,163,183,206,231,
  259,291,326,366,411,461,518,581,652,
  732,822,923,1036,1163,1305,1465,1644,
  1845,2071,2325,2610,2930,3288,3691,4143,
  4650,5220,5859,6577,7382,
};
constexpr byte NumNotes = sizeof(Notes) / sizeof(Notes[0]);


#define PWM_VALUE     OCR2B
#define PWM_INTERRUPT TIMER2_OVF_vect

void initAudio() {
  // Set up PWM to 31.25kHz, phase accurate
  TCCR2A = _BV(COM2B1) | _BV(WGM20);
  TCCR2B = _BV(CS20);
}
void audioOn() {
  TIMSK2 = _BV(TOIE2);
}
void audioOff() {
  TIMSK2 = 0;
  PWM_VALUE = 0;
}



constexpr byte SoundPin = 3;
constexpr byte MotorPin = 8;
constexpr byte ButtonPin = 2;
constexpr byte LedPin = 13;

constexpr byte StartNote = 0;
constexpr byte EndNote = NumNotes;

uint8_t note = StartNote;
uint16_t syncPhaseAcc;
uint16_t syncPhaseInc;
uint16_t grainPhaseAcc;
uint16_t grainPhaseInc;
uint16_t grainAmp;
uint8_t grainDecay;
uint16_t grain2PhaseAcc;
uint16_t grain2PhaseInc;
uint16_t grain2Amp;
uint8_t grain2Decay;
uint8_t envelopeBlockPos = 0;

constexpr uint16_t grain1PhaseIncStart = mapPhaseInc(300) / 2;
constexpr uint16_t grain2PhaseIncStart = mapPhaseInc(340) / 2;
constexpr uint16_t grain1PhaseIncTarget = mapPhaseInc(150) / 2;
constexpr uint16_t grain2PhaseIncTarget = mapPhaseInc(115) / 2;


void initNote() {
  syncPhaseInc = Notes[note];
  grainPhaseInc = grain1PhaseIncStart;
  grain2PhaseInc = grain2PhaseIncStart;
  envelopeBlockPos = 0;
}


uint16_t delayMs;
void initRun() {
  note = StartNote;
  delayMs = 360;
  initNote();
}


void initPinInterrupt() {
  EICRA = _BV(ISC01);   // FALLING edge on INT0[pin2]
}

inline void enablePinInterrupt() {
  EIMSK |= 1;
}
inline void disablePinInterrupt() {
  EIMSK &= ~1;
}

ISR(INT0_vect) {
  // nada
}

uint32_t lastTimeMs;

void setup() {
  power_adc_disable();
  power_spi_disable();
  power_usart0_disable();

  initPinInterrupt();
  
  pinMode(MotorPin, OUTPUT);
  pinMode(LedPin, OUTPUT);
  pinMode(SoundPin, OUTPUT);
  pinMode(ButtonPin, INPUT_PULLUP);

  digitalWrite(MotorPin, 0);
  digitalWrite(LedPin, 1);
  
  syncPhaseInc = Notes[note];
  grainDecay = 3;
  grain2Decay = 16;

  randomSeed(((analogRead(1) & 0x7) << 3) | (analogRead(3) & 7));
  initRun();

  initAudio();

  lastTimeMs = millis();

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
}

bool playing = false;

void loop() {
  random(128);

  if (!playing && !digitalRead(ButtonPin)) {
    // start playing
    initRun();
    delay(1);
    audioOn();
    playing = true;
    digitalWrite(MotorPin, 1);
  }

  noInterrupts();
  if (!playing) {
    digitalWrite(LedPin, 0);
    enablePinInterrupt();
    sleep_enable();
    sleep_bod_disable();
    interrupts();
    sleep_cpu();
    sleep_disable();
    disablePinInterrupt();
    digitalWrite(LedPin, 1);
    lastTimeMs = millis();
  }
  interrupts();

  if (playing) {
    delay(delayMs);
    
    ++note;
    if (note >= EndNote) {
      audioOff();
      playing = false;
      digitalWrite(MotorPin, 0);
    }
    else if (note == 1) {
      delayMs -= 220;
    }
    syncPhaseInc = Notes[note];
    initNote();
  
    if (delayMs > 30)
      delayMs -= 7;
    
    delay(delayMs);
  }
}


inline uint16_t moveTowards(uint16_t from, uint16_t to) {
  uint16_t delta = to - from;
  uint16_t inc = delta >> 5;
  if (inc == 0)
    inc = 1;
  uint16_t next = from + inc;
  if (next > to)
    next = to;
  return next;
}


SIGNAL(PWM_INTERRUPT)
{
  syncPhaseAcc += syncPhaseInc;
  if (syncPhaseAcc < syncPhaseInc) {
    // Time to start the next grain
    grainPhaseAcc = 0;
    grainAmp = 0x7fff;
    grain2PhaseAcc = 0;
    grain2Amp = 0x7fff;    
  }

  // Tick our envelope if we need to
  if (!(envelopeBlockPos & 127)) {
    envelopeBlockPos = 0;
    grainPhaseInc = moveTowards(grainPhaseInc, grain1PhaseIncTarget);
    grain2PhaseInc = moveTowards(grain2PhaseInc, grain2PhaseIncTarget);
  }
  ++envelopeBlockPos;
  
  // Increment the phase of the grain oscillators
  grainPhaseAcc += grainPhaseInc;
  grain2PhaseAcc += grain2PhaseInc;

  // Convert phase into a triangle wave
  uint8_t osc1 = (grainPhaseAcc >> 7);
  if (grainPhaseAcc & 0x8000) osc1 = ~osc1;
  // Multiply by current grain amplitude to get sample
  uint16_t output = osc1 * (grainAmp >> 8);

  // Repeat for second grain
  uint8_t osc2 = (grain2PhaseAcc >> 7);
  if (grain2PhaseAcc & 0x8000) osc2 = ~osc2;
  output += osc2 * (grain2Amp >> 8);

  // Make the grain amplitudes decay by a factor every sample (exponential decay)
  grainAmp -= (grainAmp >> 8) * grainDecay;
  grain2Amp -= (grain2Amp >> 8) * grain2Decay;

  // Scale output to the available range, clipping if necessary
  output >>= 9;
  //if (output > 255) output = 255;

  // Output to PWM (this is faster than using analogWrite)  
  PWM_VALUE = output;
}
