/*
  MX2 Assignment 2
  Written by W Rooke
  SN: 12051342
  Date 2/9/2020
*/

// Include necessary libraries
#include <LiquidCrystal.h>
#include <avr/io.h>
#include <math.h>


// Define LCD shield button values
// These are the ideal values read from the ADC when a button is pressed
const uint16_t STEPS =  4096;
const uint16_t SEL_PB =  640;
const uint16_t UP_PB =  100;
const uint16_t DWN_PB =  257;
const uint16_t LFT_PB =  410;
const uint16_t RIT_PB =  0;
const uint16_t NONE_PB =  1023;

// Define range for button value
// This is used as a +/- value for the ideals above because the readings are inconsistent
const uint16_t PB_BOUND =  20;


// Define menu modes
// Used to display and select menus
const uint8_t MD_START = 1;

const uint8_t MD_DBG_IR = 20;
const uint8_t MD_DBG_CM = 21;
const uint8_t MD_DBG_PM = 22;
const uint8_t MD_DBG_SET = 23;
const uint8_t MD_DBG_E = 24;

const uint8_t MD_IR = 3;

const uint8_t MD_CM_START = 40;
const uint8_t MD_CM_EXIT = 41;
const uint8_t MD_CM_RUNNING = 42;

const uint8_t MD_PM = 4;

const uint8_t MD_DRV_INIT = 50;
const uint8_t MD_DRV_MOV = 51;

const uint8_t MD_SET = 60;


// Define states for debug mode finite state machine
// Used to store current state of FSM
const uint8_t FSM_0Cor = 0;
const uint8_t FSM_1Cor = 1;
const uint8_t FSM_2Cor = 2;
const uint8_t FSM_3Cor = 3;
const uint8_t FSM_4Cor = 4;
const uint8_t FSM_5Cor = 5;


LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
int8_t Steps = 0;
bool clockwise = true;
uint16_t stepsLeft = STEPS;
uint16_t stepsSet = 100;
uint16_t buttonVal = 1023;
bool motorActive = false;
volatile uint16_t millisecs = 0;
volatile uint8_t seconds = 255;
volatile uint8_t minutes = 255;
volatile bool stepped = false;
uint8_t menuState = MD_START;
volatile bool blocked = false;
uint8_t FSMState = FSM_0Cor;
bool debugSel = false;
bool buttonRead = true;
uint16_t sensorVal = 0;
uint8_t motorSpeed = 2;
uint16_t motorDiv = 1;
uint8_t wheelSize = 20;
float numRevs = 0;
uint16_t debounceTime = 0;
uint16_t buttonElapsed = 0;
bool menuUpdate = true;
uint16_t menuElapsed = 0;
uint16_t menuTime = 0;
uint16_t prevButton = 1023;

uint16_t sensorReadings[31];

// const uint8_t IRLookup[] = {150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,150,148,146,145,143,141,140,138,137,135,134,133,131,130,129,127,126,125,124,122,121,120,119,118,117,116,115,114,113,112,111,110,109,108,107,106,105,105,104,103,102,101,100,100,99,98,97,97,96,95,95,94,93,93,92,91,91,90,90,89,88,88,87,87,86,85,85,84,84,83,83,82,82,81,81,80,80,79,79,78,78,77,77,77,76,76,75,75,74,74,74,73,73,72,72,72,71,71,71,70,70,69,69,69,68,68,68,67,67,67,66,66,66,65,65,65,64,64,64,64,63,63,63,62,62,62,62,61,61,61,60,60,60,60,59,59,59,59,58,58,58,58,57,57,57,57,56,56,56,56,56,55,55,55,55,54,54,54,54,54,53,53,53,53,53,52,52,52,52,52,51,51,51,51,51,50,50,50,50,50,50,49,49,49,49,49,49,48,48,48,48,48,48,47,47,47,47,47,47,46,46,46,46,46,46,46,45,45,45,45,45,45,45,44,44,44,44,44,44,44,43,43,43,43,43,43,43,43,42,42,42,42,42,42,42,42,41,41,41,41,41,41,41,41,40,40,40,40,40,40,40,40,40,39,39,39,39,39,39,39,39,39,38,38,38,38,38,38,38,38,38,38,37,37,37,37,37,37,37,37,37,37,36,36,36,36,36,36,36,36,36,36,36,35,35,35,35,35,35,35,35,35,35,35,35,34,34,34,34,34,34,34,34,34,34,34,34,34,33,33,33,33,33,33,33,33,33,33,33,33,33,32,32,32,32,32,32,32,32,32,32,32,32,32,32,31,31,31,31,31,31,31,31,31,31,31,31,31,31,31,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,30,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,29,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,28,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,27,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,26,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,25,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,24,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,23,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,22,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,21,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,20,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,19,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,18,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,17,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,16,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,15,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14,14};
uint8_t distance = 0;

void setup()
{
  for(uint8_t x = 0; x < (sizeof(sensorReadings)/sizeof(sensorReadings[0])); x++)
  {
    sensorReadings[x] = 0;
  }
  lcd.begin(16, 2);
  ADCInit();
  // Serial.begin(9600);
  // Set digital pins 13, 12, 11 and 3 to output
  DDRB |= (1 << DDB5) | (1 << DDB4) | (1 << DDB3);
  DDRD |= (1 << DDD3);

  // Set timer 2 to normal mode, set prescaler to 64, set initial count to 0
  TCCR2A = (1 << WGM21);
  TCCR2B = (1 << CS22);
  OCR2A = 250;
  TIMSK2 = (1 << OCIE2A);
  timer1Init();
  sei();
}

void loop()
{
  buttonElapsed = millisecs - debounceTime;
  
  buttonVal = buttonRound(ADCRead(0));
  if (buttonElapsed > 100)
  {
    debounceTime = millisecs;
    if ((buttonVal == prevButton) && (buttonVal != NONE_PB))
    {
      buttonRead = true;
    }
    else
    {
      prevButton = buttonVal;
    }   
  }
  
  
  menuElapsed = millisecs - menuTime;
  if (menuElapsed > 250)
  {
    menuTime = millisecs;
    menuUpdate = true;
  }


  switch (menuState)
  {
    // Start up mode
    case MD_START:
      // Serial.println(FSMState);
      lcd.setCursor(0, 0);
      lcd.print(minutes);
      lcd.print(":");
      lcd.print(seconds);
      printHelp("",0,0);
      lcd.setCursor(0, 1);
      printHelp("12051342", 0, 0);
      
      if (buttonRead)
      {
        switch (buttonVal)
        {
          case SEL_PB:
            lcd.clear();
            if (debugSel)
            {
              menuState = MD_DBG_IR;
              debugSel = false;
            }
            else
            {
              lcd.setCursor(0,0);
              lcd.print("Drive Mode");
              motorSpeed = 3;
              menuState = MD_DRV_INIT;
            }
            break;

          case NONE_PB:
            break;

          default:
            debugFSM();
            break;
        }
      }
      break;

    case MD_DBG_IR:
      lcd.setCursor(0, 0);
      printHelp("DEBUG Mode", 0, 0);
      lcd.setCursor(0, 1);
      printHelp("IR CM PM SET E", 0, 2);
      if (buttonRead)
      {
        switch (buttonVal)
        {
          case SEL_PB:
            lcd.clear();
            menuState = MD_IR;
            break;

          case LFT_PB:
            menuState = MD_DBG_E;
            break;

          case RIT_PB:
            menuState = MD_DBG_CM;
            break;

          default:
            break;
        }
      }
      break;

    case MD_DBG_CM:
      lcd.setCursor(0, 0);
      printHelp("DEBUG Mode", 0, 0);
      lcd.setCursor(0, 1);
      printHelp("IR CM PM SET E", 3, 2);
      if (buttonRead)
      {
        switch (buttonVal)
        {
          case SEL_PB:
            lcd.clear();
            motorSpeed = 2;
            menuState = MD_CM_START;
            break;

          case LFT_PB:
            menuState = MD_DBG_IR;
            break;

          case RIT_PB:
            menuState = MD_DBG_PM;
            break;

          default:
            break;
        }
      }
      break;

    case MD_DBG_PM:
      lcd.setCursor(0, 0);
      printHelp("DEBUG Mode", 0, 0);
      lcd.setCursor(0, 1);
      printHelp("IR CM PM SET E", 6, 2);
      if (buttonRead)
      {
        switch (buttonVal)
        {
          case SEL_PB:
            lcd.clear();
            menuState = MD_PM;
            lcd.setCursor(0,0);
            motorSpeed = 3;
            stepsSet = 100;
            lcd.print("PM Mode");
            break;

          case LFT_PB:
            menuState = MD_DBG_CM;
            break;

          case RIT_PB:
            menuState = MD_DBG_SET;
            break;

          default:
            break;
        }
      }
      break;

    case MD_DBG_SET:
      lcd.setCursor(0, 0);
      printHelp("DEBUG Mode", 0, 0);
      lcd.setCursor(0, 1);
      printHelp("IR CM PM SET E", 9, 3);
      if (buttonRead)
      {
        switch (buttonVal)
        {
          case SEL_PB:
            lcd.clear();
            menuState = MD_SET;
            break;

          case LFT_PB:
            menuState = MD_DBG_PM;
            break;

          case RIT_PB:
            menuState = MD_DBG_E;
            break;

          default:
            break;
        }
      }
      break;

    case MD_DBG_E:
      lcd.setCursor(0, 0);
      printHelp("DEBUG Mode", 0, 0);
      lcd.setCursor(0, 1);
      printHelp("IR CM PM SET E", 13, 1);
      if (buttonRead)
      {
        switch (buttonVal)
        {
          case SEL_PB:
            lcd.clear();
            menuState = MD_START;
            break;

          case LFT_PB:
            menuState = MD_DBG_SET;
            break;

          case RIT_PB:
            menuState = MD_DBG_IR;
            break;

          default:
            break;
        }
      }
      break;


    
    case MD_IR:
      arrayIncrement(sensorReadings,sizeof(sensorReadings),ADCRead(2));
      lcd.setCursor(0,0);
      printHelp("IR Mode", 0, 0);
      lcd.setCursor(0,1);
      // distance = IRLookup[arrayAverage(sensorReadings, sizeof(sensorReadings))];
      distance = IRFunc(arrayAverage(sensorReadings, sizeof(sensorReadings)));
      lcd.print(distance);
      printHelp("cm",0,0);
      if (buttonRead)
      {
        if (buttonVal == SEL_PB)
        {
          lcd.clear();
          menuState = MD_DBG_IR;
        }
      }
      break;
    
    case MD_CM_START:
      lcd.setCursor(0,0);
      printHelp("CM Mode", 0, 0);
      lcd.setCursor(0,1);
      printHelp("Start Exit", 0, 5);
      if (buttonRead)
      {
        switch (buttonVal)
        {
          case SEL_PB:
            lcd.clear();
            lcd.setCursor(0,0);
            lcd.print("CM Mode");
            lcd.setCursor(0,1);
            lcd.print("  CW");
            menuState = MD_CM_RUNNING;
            motorActive = true;
            break;

          case LFT_PB:
            menuState = MD_CM_EXIT;
            break;

          case RIT_PB:
            menuState = MD_CM_EXIT;
            break;

          default:
            break;
        }
      }
      break;
    
    case MD_CM_EXIT:
      lcd.setCursor(0,0);
      printHelp("CM Mode", 0, 0);
      lcd.setCursor(0,1);
      printHelp("Start Exit", 6, 4);
      if (buttonRead)
      {
        switch (buttonVal)
        {
          case SEL_PB:
            lcd.clear();
            menuState = MD_DBG_CM;
            break;

          case LFT_PB:
            menuState = MD_CM_START;
            break;

          case RIT_PB:
            menuState = MD_CM_START;
            break;

          default:
            break;
        }
      }
      break;
    
    case MD_CM_RUNNING:
      lcd.setCursor(0,1);
      lcd.print(motorSpeed);
      
      if (buttonRead)
      {
        switch (buttonVal)
        {
          case SEL_PB:
            lcd.clear();
            menuState = MD_CM_START;
            motorSpeed = 2;
            clockwise = true;
            motorActive = false;
            break;

          case LFT_PB:
            clockwise = true;
            lcd.print(" CW ");
            break;

          case RIT_PB:
            clockwise = false;
            lcd.print(" CCW ");
            break;

          case UP_PB:
            if (motorSpeed < 3)
            {
              motorSpeed++;
            }
            else
            {
              motorSpeed = 3;
            }
            break;
            
          case DWN_PB:
            if (motorSpeed > 0)
            {
              motorSpeed--;
            }
            else
            {
              motorSpeed = 0;
            }
            break;

          default:
            break;
        }
      }
      break;

    case MD_PM:
      lcd.setCursor(0, 1);
      lcd.print(stepsSet);
      lcd.print(" ");
      
      if (!motorActive)
      {
        printHelp("",0,0);
      }
      else
      {
        lcd.print(" ");
        lcd.print(stepsLeft);
        lcd.print(" ");
      }
      
      if (buttonRead && !motorActive)
      {
        switch (buttonVal)
        {
        case UP_PB:
          if (stepsSet < 65500)
          {
            stepsSet += 100;
          }
          break;
        
        case DWN_PB:
          if (stepsSet != 0)
          {
            stepsSet -= 100;
          }
          break;

        case LFT_PB:
          stepsSet = 100;
          break;

        case RIT_PB:
          motorActive = true;
          stepsLeft = stepsSet;
          break;

        case SEL_PB:
          stepsSet = 100;
          motorActive = false;
          lcd.clear();
          menuState = MD_DBG_PM;
          break;

        default:
          break;
        }
      }
      else if (buttonRead)
      {
        if (buttonVal == SEL_PB)
        {
          stepsSet = 100;
          motorActive = false;
          lcd.clear();
          menuState = MD_DBG_PM;
        }
      }
      break;

    case MD_SET:
      motorActive = false;
      lcd.setCursor(0,0);
      printHelp("SETTINGS Mode", 0, 0);
      lcd.setCursor(0, 1);
      lcd.print("Wheel: ");
      lcd.print(wheelSize);
      printHelp("cm", 0, 0);
      if (buttonRead)
      {
        switch (buttonVal)
        {
        case UP_PB:
          if (wheelSize < 250)
          {
            wheelSize += 10;
          }
          break;
        
        case DWN_PB:
          if (wheelSize > 10)
          {
            wheelSize -= 10;
          }
          break;

        case SEL_PB:
          menuState = MD_DBG_SET;
          break;

        default:
          break;
        }
      }
      break;

    case MD_DRV_INIT:
      if (!motorActive)
      {
        lcd.setCursor(0, 1);
        arrayIncrement(sensorReadings,sizeof(sensorReadings),ADCRead(2));
        distance = IRFunc(arrayAverage(sensorReadings, sizeof(sensorReadings)));
        numRevs = round((((float)distance/(float)wheelSize) * 10)) / 10.0;
        stepsLeft = numRevs * (float)STEPS;
        lcd.print(distance);
        lcd.print("cm ");
        lcd.setCursor(6,1);
        lcd.print(numRevs, 1);
      }

      if (menuUpdate)
      {
        lcd.setCursor(10,1);
        lcd.print(stepsLeft);
        lcd.print("    ");
      }
      if (buttonRead && !motorActive)
      {
        switch (buttonVal)
        {
          case UP_PB:
            motorActive = true;
            clockwise = true;
            break;

          case DWN_PB:
            motorActive = true;
            clockwise = false;
            break;

          case SEL_PB:
            motorActive = false;
            menuState = MD_START;
            debugSel = false;
            FSMState = FSM_0Cor;
            break;
          
          default:
            break;
          }
      }
      else if (buttonRead)
      {
        if (buttonVal == SEL_PB)
        {
            motorActive = false;
            menuState = MD_START;
            debugSel = false;
            FSMState = FSM_0Cor;
        }
      }
      break;

  }

  buttonRead = false;
  menuUpdate = false;

  if ((!stepped && motorActive) && (stepsLeft > 0))
  {
    stepped = true;
    stepper();
    if (menuState != MD_CM_RUNNING)
    {
      stepsLeft--;
    }
  }
  else if (stepsLeft == 0)
  {
    motorActive = false;
  }

}

/* This function handles the digital pin toggles to drive the motor */
// Function operates by switching off all outputs in each case, then turning on the required ones
// It probably isn't the most efficient, but it makes sense for bidirectional operation
void stepper(void)
{
  switch (Steps)
  {
    // Turn on D13
    case 0:
      PORTB = 0;
      PORTD = 0;
      PORTB |= (1 << PORTB5);
      break;

    // Turn on D13 and D12
    case 1:
      PORTB = 0;
      PORTD = 0;
      PORTB |= (1 << PORTB5);
      PORTB |= (1 << PORTB4);
      break;

    // Turn on D12
    case 2:
      PORTB = 0;
      PORTD = 0;
      PORTB |= (1 << PORTB4);
      break;

    // Turn on D12 and D11
    case 3:
      PORTB = 0;
      PORTD = 0;
      PORTB |= (1 << PORTB4);
      PORTB |= (1 << PORTB3);
      break;

    // Turn on D11
    case 4:
      PORTB = 0;
      PORTD = 0;
      PORTB |= (1 << PORTB3);
      break;
    // Turn on D11 and D3
    case 5:
      PORTB = 0;
      PORTD = 0;
      PORTB |= (1 << PORTB3);
      PORTD |= (1 << PORTD3);
      break;

    // Turn on D3
    case 6:
      PORTD = 0;
      PORTB = 0;
      PORTD |= (1 << PORTD3);
      break;

    // Turn on D3 and D13
    case 7:
      PORTD = 0;
      PORTB = 0;
      PORTD |= (1 << PORTD3);
      PORTB |= (1 << PORTB5);
      break;
  }
  SetDirection();
}

/* This Function handles the resetting of direction, when reaching the end of the rotation */
void SetDirection()
{
  if (clockwise == true)
  {
    Steps++;
  }
  else
  {
    Steps--;
  }
  if (Steps > 7)
  {
    Steps = 0;
  }
  if (Steps < 0)
  {
    Steps = 7;
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

uint16_t ADCRead(uint8_t channel)
{
  if ((channel < 0) || (channel > 7))
  {
    return 0;
  }
  ADMUX = (1 << REFS0);
  ADMUX |= (0b00001111 & channel);

  ADCSRA |= (1 << ADSC);
  while ((ADCSRA & (1 << ADSC)))
    ;
  return ADC;
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
  TCCR1B = (1 << WGM12) | (1 << CS12) | (1 << CS10);

  // Ensure timer is not disabled
  PRR &= ~(1 << PRTIM1);

  // Enable compare interrupt
  TIMSK1 = (1 << OCIE1A);
}

void printHelp(char inString[], uint8_t index, uint8_t numToBlock)
{
  size_t arraySize = strlen(inString);
  char outString[16];
  for (uint8_t x = 0; x < 16; x++)
  {
    if (x < arraySize)
    {
      outString[x] = inString[x];
    }
    else
    {
      outString[x] = ' ';
    }
  }
  if (blocked)
  {
    for (uint8_t x = index; x < (index + numToBlock); x++)
    {
      outString[x] = 0xFF;
    }
    lcd.print(outString);
  }
  else
  {
    lcd.print(outString);
  }
}

int buttonRound(int checkValue)
{
  if ((checkValue >= (LFT_PB - PB_BOUND)) && (checkValue <= (LFT_PB + PB_BOUND)))
  {
    return LFT_PB;
  }
  if ((checkValue >= (RIT_PB - PB_BOUND)) && (checkValue <= (RIT_PB + PB_BOUND)))
  {
    return RIT_PB;
  }
  if ((checkValue >= (UP_PB - PB_BOUND)) && (checkValue <= (UP_PB + PB_BOUND)))
  {
    return UP_PB;
  }
  if ((checkValue >= (DWN_PB - PB_BOUND)) && (checkValue <= (DWN_PB + PB_BOUND)))
  {
    return DWN_PB;
  }
  if ((checkValue >= (SEL_PB - PB_BOUND)) && (checkValue <= (SEL_PB + PB_BOUND)))
  {
    return SEL_PB;
  }
  if ((checkValue >= (NONE_PB - PB_BOUND)) && (checkValue <= (NONE_PB + PB_BOUND)))
  {
    return NONE_PB;
  }
}

void debugFSM(void)
{
  if (buttonVal != NONE_PB)
  {
    switch (FSMState)
    {
      case FSM_0Cor:
        if (buttonVal == LFT_PB)
        {
          FSMState = FSM_1Cor;
        }
        else
        {
          FSMState = FSM_0Cor;
        }
        break;

      case FSM_1Cor:
        if (buttonVal == LFT_PB)
        {
          FSMState = FSM_2Cor;
        }
        else
        {
          FSMState = FSM_0Cor;
        }
        break;

      case FSM_2Cor:
        if (buttonVal == LFT_PB)
        {
          FSMState = FSM_2Cor;
        }
        else if (buttonVal == UP_PB)
        {
          FSMState = FSM_3Cor;
        }
        else
        {
          FSMState = FSM_0Cor;
        }
        break;

      case FSM_3Cor:
        if (buttonVal == RIT_PB)
        {
          FSMState = FSM_4Cor;
          debugSel = true;
        }
        else
        {
          FSMState = FSM_0Cor;
        }
        break;

      case FSM_4Cor:
        FSMState = FSM_0Cor;
        debugSel = false;
    }
  }
}

void arrayIncrement(uint16_t inArray[], size_t arraySize, uint16_t newValue)
{
  for (size_t x = (arraySize / sizeof(inArray[0])); x > 0; x--)
  {
    inArray[x] = inArray[x-1];
  }
  inArray[0] = newValue;
}

uint16_t arrayAverage(uint16_t inArray[], size_t arraySize)
{
  uint32_t sum = 0;
  for (size_t x = 0; x <= (arraySize / sizeof(inArray[0])); x++)
  {    
    sum += inArray[x];
  }
  return (sum / (arraySize / sizeof(inArray[0])));
}

uint8_t IRFunc (uint16_t inVal)
{
  uint8_t outVal = round(pow((float)inVal/18109,1.0/-1.09));
  if (outVal >= 150)
  {
    return 150;
  }
  else
  {
    return outVal;
  }
}

ISR(TIMER1_COMPA_vect)
{
  if (seconds >= 59)
  {
    seconds = 0;
    minutes++;
  }
  else
  {
    seconds++;
  }
  TIFR1 = (1 << OCF1A);
  blocked = !blocked;
}

ISR(TIMER2_COMPA_vect)
{
  millisecs++;
  motorDiv++;
  if (motorDiv > ((3 - motorSpeed) * 2))
  {
    motorDiv = 0;
    stepped = false;
  }

}