/* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Synchronous Multiphase Buck Converter [Rev. 1.1]~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

   This code is under MIT License
   Copyright (c) 2021 Sayantan Sinha
*/

#define HS1 PB1                                        // D9
#define LS1 PB2                                        // D10
#define HS2 PB3                                        // D11
#define LS2 PD3                                        // D3
#define MAX_COUNT 255                                  // For 100 kHz switching frequency (in dual slope couning)
#define DT 2                                           // Dead time (DT / f_cpu = 0.5 us)
#define VREF 320                                       // Reference voltage for approx. 5V output
#define DUTY_MAX 160
#define DUTY_MIN 2
#define DUTY_FXD ((MAX_COUNT * 41) / 100)
#define H(X) (X / 10)                                  // A gain factor in the proportional control action

const byte pinFb = 0;                                  // Output voltage feedback at pin A0
const byte pinIsens = 1;
unsigned char mode = 1;

void setup()
{
  DDRB |= 1 << PB5;
  PORTB &= ~(1 << PB5);                              // Turn off the annoying LED @ pin 13

  TIMSK1 = 0;                                        // Disable Timer1 interrupts
  TIMSK2 = 0;                                        // Disable Timer2 interrupts

  ADMUX = 0b01000000;                                // Select A_ref = +5 V
  ADCSRA = 0b10010111;                               // ADC enable, prescaler = f_CPU / 128 [Ref: Atmega328P datasheet, pp. 317-320]

  DDRB |= (1 << HS1) | (1 << LS1) | (1 << HS2);      // Set D9 & D10 as output pins to get sPWM outputs
  DDRD |= (1 << LS2);                                // Set D3 as output
  PORTB &= ~((1 << LS1) | (1 << HS1) | (1 << HS2));  // All gate drive signals LOW
  PORTD &= ~(1 << LS2);                              //

  ICR1 = MAX_COUNT;                                  // Set the switching freq. f_sw = f_cpu / (2 * MAX_COUNT)
  TCCR1A = 0b00000000;                               // pin D9 (OC1A) & pin D10 (OC1B) disconnected to avoid any mistrigger
  TCCR1B = 0b00010001;                               // PWM, Phase & Frequemcy Correct (Mode 8); Clock Select = System clock (No Prescaling) [Ref. Data Sheet, p. 172]
  OCR1A = DUTY_FXD;
  OCR1B = OCR1A + DT;                                // Low side trigger pulse with a dead-time

  TCCR2A = 0b00000001;                               // Disconnect pin D11 (OC2A) and D3 (OC2B) to avoid any mistrigger
  TCCR2B = 0b00000001;                               // Timer2 PWM Phase Correct (Mode 1); no prescaler [Ref. Data Sheet, p. 205]
  OCR2A = OCR1A;
  OCR2B = OCR1B;                                     // Low side trigger pulse with a dead-time

  TCNT1 = MAX_COUNT;                                 // Advance the phase by 180 deg than Timer2
  TCNT2 = 3;

  TCCR1A |= 0b10110000;                              // pin D9 (OC1A): Set on Compare Match; pin D10 (OC1B): Clear on Compare Match
}

void loop()
{
  unsigned int vo = adcRead(pinFb);
  unsigned int i = adcRead(pinIsens);
  bool dmxSat = OCR1A == DUTY_MAX, dmnSat = OCR1A == DUTY_MIN;
  int e = VREF - vo;                                               // Error

  if (mode == 1) {
    if (i > 160) {                                                 // If load current > 2 A switch to mode 2 (turn on phase-2)
      mode = 2;
      TCCR2A |= 0b10110000;                                        // pin D11 (OC2A): Set on Compare Match; pin D3 (OC2B): Clear on Compare Match
    }
  }
  else if (mode == 2) {
    if (i < 140) {                                                 // If load current < 1.5 A switch to mode 1 (shut down phase-2)
      mode = 1;
      TCCR2A &= 0b00001111;                                        // pin D11 (OC2A): Disconnect; pin D3 (OC2B): Disconnect
    }
  }
  if (e > 0) {                                                    // Output is less than reference voltage
    if ((OCR1A + H(e)) < DUTY_MAX)
      OCR1A += H(e) > 5 ? 5 : H(e);
    else if (!dmxSat)
      OCR1A = DUTY_MAX;
    if (!dmxSat) {
      OCR1B = OCR1A + DT;
      OCR2A = OCR1A;
      OCR2B = OCR1B;
    }
  }
  else if (e < 0) {                                               // Output is higher than the reference voltage
    if (OCR1A > DUTY_MIN - H(e))
      OCR1A += H(e) < -5 ? -5 : H(e);
    else if (!dmnSat)
      OCR1A = DUTY_MIN;
    if (!dmnSat) {
      OCR1B = OCR1A + DT;
      OCR2A = OCR1A;
      OCR2B = OCR1B;
    }
  }
}

unsigned int adcRead(const byte ch)
{
  if (ch > 15)
    return 0;
  ADMUX &= 0xF0;
  ADMUX |= ch;
  ADCSRA |= 0x40;           // Start conversion
  while (ADCSRA & 0x40);    // Wait until ADC start conversion bit goes low
  unsigned int a = ADCL;
  a |= (ADCH << 8);
  return (a);
}
