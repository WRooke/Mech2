/*
  Exercise 2: This exercise is for the stepper motor to cycle back and forth between clockwise and counter clockwise rotations. Do not use standard Functions
  Code written by:
          Member 1 SID xxxxxxxx
          Member 2 SID xxxxxxxx
          Member 3 SID xxxxxxxx
  Date: xx/xx/xxxx
*/
#define STEPS 4096
#define SEL_PB 640
#define UP_PB 100
#define DWN_PB 257
#define LFT_PB 410
#define RIT_PB 0
#include <avr/io.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
int Steps = 0;
bool Direction = true;
int stepsleft = STEPS;
int buttonVal = 1023;
bool motorActive = false;
int debounceCount = 0; 
int count = 0;
bool stepped = false;

void setup() 
{
  lcd.begin(16,2);
  ADCInit();
  Serial.begin(9600);
  // Set digital pins 13, 12, 11 and 3 to output
  DDRB |= (1<<DDB5)|(1<<DDB4)|(1<<DDB3);
  DDRD |= (1<<DDD3);
  
  // Set timer 2 to normal mode, set prescaler to 64, set initial count to 0
  TCCR2A = (1 << WGM21);
  TCCR2B = (1 << CS22);
  OCR2A = 250;
  TIMSK2 = (1 << OCIE2A);
  timer1Init();
}

void loop()
{
  // Serial.println(TCNT2);
  // Serial.println(debounceCount);
  if (debounceCount > 100)
  {
    buttonVal = ADCRead(0);
    debounceCount = 0;
  }
  
  if (buttonVal != 1023)
  {
    switch (buttonVal)
    {
      case SEL_PB:
        lcd.setCursor(0,0);
        lcd.clear();
        lcd.print("Sel");
        motorActive = !motorActive;
        break;
      case LFT_PB:
        lcd.setCursor(0,0);
        lcd.clear();
        lcd.print("Left");        
        Direction = true;
        break;
      case UP_PB:
        lcd.setCursor(0,0);
        lcd.clear();
        lcd.print("Up");
        break;
      case DWN_PB:
        lcd.setCursor(0,0);
        lcd.clear();
        lcd.print("Down");
        break;
      case RIT_PB:
        lcd.setCursor(0,0);
        lcd.clear();
        lcd.print("Right");
        Direction = false;
        break;
      default:
        break;
    }
    buttonVal = 1023;
  }
  // Serial.print("stepped:");
  // Serial.println(stepped);
  // Serial.print("motorActive:");
  // Serial.println(motorActive);
  if (!stepped && motorActive)
  {
    stepped = true;
    stepper();
  }
}

/* This function handles the digital pin toggles to drive the motor */
// Function operates by switching off all outputs in each case, then turning on the required ones
// It probably isn't the most efficient, but it makes sense for bidirectional operation
void stepper(void)
{
  // Serial.println("stepper reached");
  switch(Steps)
  {
    // Turn on D13
    case 0:
      PORTB = 0;
      PORTD = 0;
      PORTB |= (1<<PORTB5);
    break; 

    // Turn on D13 and D12
    case 1:
      PORTB = 0;
      PORTD = 0;
      PORTB |= (1<<PORTB5);
      PORTB |= (1<<PORTB4);
    break; 

    // Turn on D12
    case 2:
      PORTB = 0;
      PORTD = 0;
      PORTB |= (1<<PORTB4);
    break; 

    // Turn on D12 and D11
    case 3:
      PORTB = 0;
      PORTD = 0;
      PORTB |= (1<<PORTB4);
      PORTB |= (1<<PORTB3);
    break;
    
    // Turn on D11
    case 4:
      PORTB = 0;
      PORTD = 0;
      PORTB |= (1<<PORTB3);
    break;
    // Turn on D11 and D3
    case 5:
      PORTB = 0;
      PORTD = 0;
      PORTB |= (1<<PORTB3);
      PORTD |= (1<<PORTD3);
    break; 

    // Turn on D3
    case 6:
      PORTD = 0;
      PORTB = 0;
      PORTD |= (1<<PORTD3);
    break; 

    // Turn on D3 and D13
    case 7:
      PORTD = 0;
      PORTB = 0;
      PORTD |= (1<<PORTD3);
      PORTB |= (1<<PORTB5);
    break; 
  }
  SetDirection();
} 

/* This Function handles the resetting of direction, when reaching the end of the rotation */
void SetDirection()
{
  if(Direction==true)
  { 
    Steps++;
  }
  else
  { 
    Steps--; 
  }
  if(Steps>7){
    Steps=0;
  }
  if(Steps<0)
  {
    Steps=7; 
  }
}

void ADCInit()
{
  // Use interval voltage reference
  ADMUX |= (1 << REFS0);

  // Set 8-bit resolution
  // ADMUX |= (1 << ADLAR);

  // 128 prescale for 16Mhz (maybe change this, I dunno what the fuck it means)
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); 
  
  // Enable the ADC
  ADCSRA |= (1 << ADEN);

}

int ADCRead(int channel)
{
  if ((channel < 0) || (channel > 7))
  {
    return 0;
  }
  int ADCValue;
  ADMUX |= (0b00001111 & channel);

  ADCSRA |= (1 << ADSC);
  while (1)
  {
    if (~(ADCSRA & (1 << ADSC)))
    {
      ADCValue = ADC;
      break;
    }
  }
  return ADCValue;
}

void timer1Init()
{
  // Clear timer control registers, enusre correct values are being set
  TCCR1B = 0;
  TCCR1A = 0;

  // Set overflow clear value, will clear at 1s
  // f=f_io/(1024*(1+OCR1A))
  OCR1A = 15625;

  // Set timer to have 1024 prescaler and run in CTC mode
  TCCR1B = (1<<WGM12) | (1 << CS12) | (1<<CS10);

  // Ensure timer is not disabled
  PRR &= ~(1<<PRTIM1);

  // Enable compare interrupt
  TIMSK1 = (1 << OCIE1A);
}

ISR(TIMER1_COMPA_vect)
{

  // Serial.println("ISR entered");
  count++;
  TIFR1 = (1<<OCF1A);
  // lcd.print("ISR");
  lcd.setCursor(0,1);
  lcd.print(count);
  // Serial.println(count);
}

ISR(TIMER2_COMPA_vect)
{
  debounceCount++;
  TCNT2 = 0;
  stepped = false;
}