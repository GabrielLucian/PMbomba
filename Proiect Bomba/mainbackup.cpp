#include <Arduino.h>
#include <avr/interrupt.h>
#include "Keypad.h"

const byte ROWS = 4; // four rows
const byte COLS = 4; // four columns
char keys[ROWS][COLS] = {
    {'1', '2', '3', 'A'},
    {'4', '5', '6', 'B'},
    {'7', '8', '9', 'C'},
    {'*', '0', '#', 'D'}};
byte colPins[ROWS] = {47, 46, 45, 44}; // connect to the row pinouts of the keypad
byte rowPins[COLS] = {53, 52, 51, 50}; // connect to the column pinouts of the keypad

Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);
volatile unsigned long timeElapsed = 0;
int dutyCycle = 128; // 25% duty cycle
int duration = 5000;

// setup an interrupt for when a certain key is pressed
ISR(INT0_vect)
{
  Serial.println("Timer Reset!");

  // reset timer
  timeElapsed = 0;
}

// Function to buzz the buzzer
void buzz(int dutyCycle, int frequency)
{
  // Use PWM to control the buzzer
  // Set PWM frequency to around 2kHz (you can adjust this frequency as needed)
  // Set the duty cycle using OCR2A (0 to 255)
  // Note: Timer2 is used for PWM on pin 3 and 11 on Arduino Mega
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); // Clear OC2B on compare match
  TCCR2B = _BV(WGM22) | _BV(CS20);                // Set prescaler to 1
  OCR2A = frequency;                              // Set the frequency (0 to 255)
  OCR2B = dutyCycle;                              // Adjust the duty cycle (0 to 255)

  Serial.println("Buzzing...");
  digitalWrite(3, HIGH);
  delay(5000);

  // Turn off PWM
  TCCR2A = 0;
  TCCR2B = 0;

  // Optionally, you can set the pin back to LOW to ensure the buzzer is off
  digitalWrite(3, LOW);
}

// another interrupt for when the timeLeft reaches 0
ISR(TIMER1_COMPA_vect)
{
  timeElapsed++;
  if (timeElapsed == 60)
  {
    Serial.println("Timer Done!");
  }


}

void setup()
{
  Serial.begin(9600);

  // set up the interrupt
  cli();              // disable interrupts
  EICRA = 0b00000010; // set INT0 to trigger on falling edge
  EIMSK = 0b00000001; // enable INT0

  TCCR1A = 0; // set entire TCCR1A register to 0
  TCCR1B = 0; // same for TCCR1B
  TCNT1 = 0;  // initialize counter value to 0
  // set compare match register to 60 seconds at 16MHz AVR clock, with a prescaler of 1024
  OCR1A = 15624;
  TCCR1B |= (1 << WGM12);              // turn on CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10); // set CS12 and CS10 bits for 1024 prescaler
  TIMSK1 |= (1 << OCIE1A);             // enable timer compare interrupt

  // Initialize Timer2 for PWM
  TCCR2A = _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); // Clear OC2B on compare match
  TCCR2B = _BV(WGM22) | _BV(CS20);                // Set prescaler to 1
  OCR2A = 128;                                    // Duty cycle (0 to 255), adjust as needed
  // set up the buzzer
  pinMode(3, OUTPUT);

  sei(); // enable interrupts

  Serial.println("Press a key");
}

void loop()
{

  // Get the key from the keypad
  if (keypad.getKeys())
  {
    for (int i = 0; i < LIST_MAX; i++) // Scan the whole key list.
    {
      if (keypad.key[i].stateChanged) // Only find keys that have changed state.
      {
        switch (keypad.key[i].kstate)
        { // Report active key state : IDLE, PRESSED, HOLD, or RELEASED
        case PRESSED:
          Serial.print("Pressed: ");
          Serial.println(keypad.key[i].kchar);

          if (keypad.key[i].kchar == 'A')
          {
            // trigger the interrupt
            PORTD |= 0b00000001;
            delay(100);
            PORTD &= 0b11111110;
          }
          break;
        case HOLD:
          Serial.print("Hold: ");
          Serial.println(keypad.key[i].kchar);
          break;
        case RELEASED:
          // Serial.print("Released: ");
          // Serial.println(keypad.key[i].kchar);
          break;
        case IDLE:
          // Serial.print("Idle: ");
          // Serial.println(keypad.key[i].kchar);
          break;
        }
      }
    }
  }

  // buzz(128, 500);

  if (timeElapsed > 0)
  {
    int minFrequency = 100; // Adjust as needed
    int maxFrequency = 255; // Adjust as needed
    int frequency = minFrequency + (timeElapsed * ((maxFrequency - minFrequency) / 60));
    if (frequency < 0)
    {
      frequency = 0;
    }
    buzz(dutyCycle, frequency);
  }
}
