#include <Arduino.h>
#include <avr/interrupt.h>
#include "Keypad.h"
#include "LiquidCrystal_I2C.h"
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <Wire.h>

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
volatile int beepInterval = 2950;
volatile bool buzzerState = false; // State variable to toggle the buzzer
volatile bool done = true;
volatile bool reset = false;
volatile bool lost = false;

// initialize LCD
LiquidCrystal_I2C lcd(0x27, 16, 2);

int code = 0;
int guess = 0;

int guessVector[4] = {-1, -1, -1, -1};
int codeVector[4] = {0, 0, 0, 0};
int correctVector[4] = {-1, -1, -1, -1};

// Timer 3 interrupt for resetting the game
ISR(TIMER3_COMPA_vect)
{
}

// another interrupt for when the timeLeft reaches 0
ISR(TIMER1_COMPA_vect)
{
  timeElapsed++;
  if (timeElapsed == 60)
  {
    lost = true;
  }
}

// Timer 2 interrupt for the buzzer
ISR(TIMER2_OVF_vect)
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();

  if (!done)
  {
    if (buzzerState == HIGH)
    {
      // Turn off buzzer after 1 second
      if (currentMillis - previousMillis >= 500)
      {
        previousMillis = currentMillis;
        digitalWrite(3, LOW);
        buzzerState = LOW;
      }
    }
    else
    {
      // Turn on buzzer after beepInterval
      if (currentMillis - previousMillis >= beepInterval)
      {
        previousMillis = currentMillis;
        digitalWrite(3, HIGH);
        buzzerState = HIGH;
        beepInterval -= 100;
      }
    }
  }
  else
  {
    // Turn off buzzer if the game is lost
    digitalWrite(3, LOW);
  }
}

void setup()
{

  // initialize the LCD
  lcd.begin();
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Press A to start!");

  Serial.begin(9600);

  // set up the interrupt
  cli(); // disable interrupts

  TCCR1A = 0;                          // set entire TCCR1A register to 0
  TCCR1B = 0;                          // same for TCCR1B
  TCNT1 = 0;                           // initialize counter value to 0
  OCR1A = 15624;                       // set compare match register to desired timer count
  TCCR1B |= (1 << WGM12);              // turn on CTC mode
  TCCR1B |= (1 << CS12) | (1 << CS10); // set CS12 and CS10 bits for 1024 prescaler
  TIMSK1 |= (1 << OCIE1A);             // enable timer compare interrupt

  pinMode(3, OUTPUT);

  // Configure Timer2 for PWM operation
  TCCR2A = _BV(COM2A1) | _BV(WGM21) | _BV(WGM20); // Set fast PWM mode with non-inverting output on OC2A (pin 11)
  TCCR2B = _BV(WGM22) | _BV(CS22);                // Set prescaler to 64 (Arduino Mega specific)
  OCR2A = 255;                                    // Set TOP value (resolution) for PWM

  // Set PWM frequency
  int prescaler = 64; // Prescaler value
  int pwmPeriod = 16000000 / (prescaler * 1000) - 1;
  // Calculate and set PWM frequency
  TCCR2B = (TCCR2B & 0b11111000) | (pwmPeriod & 0b00000111);

  // Set PWM duty cycle
  OCR2A = map(128, 0, 255, 0, pwmPeriod);

  // Enable Timer2 overflow interrupt
  TIMSK2 |= (1 << TOIE2);

  TCCR2B |= (1 << CS22); // Start Timer2 with prescaler 64

  // set up timer 3
  TCCR3A = 0;                          // set entire TCCR3A register to 0
  TCCR3B = 0;                          // same for TCCR3B
  TCNT3 = 0;                           // initialize counter value to 0
  OCR3A = 15624;                       // set compare match register to desired timer count
  TCCR3B |= (1 << WGM32);              // turn on CTC mode
  TCCR3B |= (1 << CS32) | (1 << CS30); // set CS32 and CS30 bits for 1024 prescaler
  TIMSK3 |= (1 << OCIE3A);             // enable timer compare interrupt

  sei(); // enable interrupts

  // // generate random 4 digit code
  randomSeed(analogRead(0));

  Serial.println("Press A to start!");
}

int getNthDigit(int num, int n)
{
  int len = 0;
  int temp = num;

  // Counting the number of digits
  while (temp > 0)
  {
    temp /= 10;
    len++;
  }

  // Adjusting position if it's greater than number of digits
  n = len - n;

  // Finding the digit at the specified position
  while (n > 1)
  {
    num /= 10;
    n--;
  }

  return num % 10;
}

void loop()
{
  if (reset)
  {
    Serial.println("Game Reset!");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Game Reset!");
    delay(1000);

    // reset timer
    timeElapsed = 0;
    beepInterval = 2950;
    done = false;

    // generate random 4 digit code
    code = random(1000, 10000);

    // reset the guess
    guess = 0;

    // reset the guess vector
    for (int i = 0; i < 4; i++)
    {
      guessVector[i] = -1;
      correctVector[i] = -1;
    }

    // print to the LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Guess: ");
    for (int i = 0; i < 4; i++)
    {
      lcd.print("_");
    }
    reset = false;
    lost = false;
  }

  if (lost)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("You Lost!");
    lcd.setCursor(0, 1);
    lcd.print("Code: ");
    lcd.print(code);
    delay(1000);
    lcd.clear();
    done = true;
  }

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
          if (keypad.key[i].kchar == 'A')
          {
            Serial.println("Game Started!");
            // trigger the reset interrupt
            reset = true;
          }
          else if (keypad.key[i].kchar == 'B')
          {
            // give up
            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("You Gave Up!");
            lcd.setCursor(0, 1);
            lcd.print("Code: ");
            lcd.print(code);
            delay(1000);
            done = true;
          }
          else if (keypad.key[i].kchar == 'C')
          {
          }
          else if (keypad.key[i].kchar == 'D')
          {
          }
          else
          {
            if (!done)
            {
              guess = guess * 10 + (keypad.key[i].kchar - '0');

              int guessLength = 0;
              int temp = guess;

              // Counting the number of digits
              while (temp > 0)
              {
                temp /= 10;
                guessLength++;
              }

              // update the guess vector with the last digit
              guessVector[guessLength - 1] = guess % 10;

              // check if current digit is correct
              if ((guess % 10) == getNthDigit(code, guessLength - 1))
              {
                correctVector[guessLength - 1] = guess % 10;
              }

              // print to the LCD
              lcd.clear();
              lcd.setCursor(0, 0);
              lcd.print("Guess: ");
              for (int i = 0; i < 4; i++)
              {
                if (correctVector[i] != -1)
                {
                  lcd.print(correctVector[i]);
                }
                else if (guessVector[i] != -1)
                {
                  lcd.print(guessVector[i]);
                }
                else
                {
                  lcd.print("_");
                }
              }

              if (guess == code)
              {
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("You Won!");
                done = true;
              }
              // if guess is 4 digits long, reset the guess
              else if (guess > 999)
              {
                guess = 0;

                // reset the guess vector
                for (int i = 0; i < 4; i++)
                {
                  guessVector[i] = -1;
                }

                delay(500);
                // reset display
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("Guess: ");
                for (int i = 0; i < 4; i++)
                {
                  if (correctVector[i] != -1)
                  {
                    lcd.print(correctVector[i]);
                  }
                  else
                  {
                    lcd.print("_");
                  }
                }
              }
            }
          }
          break;
        case HOLD:
          break;
        case RELEASED:
          break;
        case IDLE:
          break;
        }
      }
    }
  }
}