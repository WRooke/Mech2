/*
  MX2 Assignment 3
  Written by W Rooke
  SN: 12051342
  Date 6/10/2020
*/

// Include necessary libraries
#include <LiquidCrystal.h>
#include <avr/io.h>
#include <float.h>
#include <math.h>
#include <avr/pgmspace.h>

typedef struct pose{
  uint8_t x;
  uint8_t y;
  uint16_t heading;
} Pose;

typedef struct wavenode{
  uint8_t x;
  uint8_t y;
  uint16_t value;
} WaveNode;



// Define LCD shield button values
// These are the ideal values read from the ADC when a button is pressed
const uint16_t STEPS = 4096;
const uint16_t SEL_PB = 640;
const uint16_t UP_PB = 100;
const uint16_t DWN_PB = 257;
const uint16_t LFT_PB = 410;
const uint16_t RIT_PB = 0;
const uint16_t NONE_PB = 1023;

const uint8_t GRIDSIZE = 20;

// Define range for button value
// This is used as a +/- value for the ideals above because the readings are inconsistent
const uint16_t PB_BOUND = 20;

// Define menu modes
// Used to display and select menus
const uint8_t MD_START_CON = 10;
const uint8_t MD_START_SWP = 11;
const uint8_t MD_START_WF = 12;
const uint8_t MD_START_NAV = 13;

const uint8_t MD_CON = 20;

const uint8_t MD_SWP = 30;

const uint8_t MD_WF = 40;

const uint8_t MD_NAV = 50;
const uint8_t MD_NAV_FIN = 51;

const uint8_t OCCUPIED = 0;
const uint8_t EXPLORED = 1;
const uint8_t UNEXPLORED = 2;

const uint16_t NTH = 0;
const uint16_t NTHEST = 45;
const uint16_t EST = 90;
const uint16_t STHEST = 135;
const uint16_t STH = 180;
const uint16_t STHWST = 225;
const uint16_t WST = 270;
const uint16_t NTHWST = 315;

uint8_t menuState = MD_START_CON;

// Initialise LCD
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

// Initialise values for button checking and debouncing
uint16_t buttonVal = 1023;
uint16_t prevButton = 1023;
uint16_t debounceTime = 0;
uint16_t buttonElapsed = 0;
bool buttonRead = true;

// Initialise time variables
// Seconds and minutes overflow instantly to zero, only initialised to 255 to patch a bug
volatile uint16_t millisecs = 0;
volatile uint8_t seconds = 255;
volatile uint8_t minutes = 255;

// Menu helper variable
volatile bool blocked = false;

// String variable used for sending commands
String commandString = "";

const bool occupancyGrid[GRIDSIZE][GRIDSIZE] PROGMEM = {
  {true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true},
  {true, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, true},
  {true, true, true, false, false, false, false, false, false, false, false, false, false, false, false, false, true, false, false, true},
  {true, false, false, false, false, false, false, false, false, false, false, true, false, false, false, true, true, false, false, true},
  {true, false, false, false, false, false, false, false, true, true, true, true, false, false, false, false, true, false, false, true},
  {true, false, false, false, false, false, false, false, false, false, false, true, false, false, false, false, false, false, false, true},
  {true, true, true, true, true, true, false, false, false, false, false, true, false, false, true, true, true, true, true, true},
  {true, false, false, false, false, true, false, false, false, false, false, true, false, false, false, false, false, false, false, true},
  {true, false, false, false, false, true, false, false, true, false, false, true, false, false, false, false, false, false, false, true},
  {true, false, false, false, false, true, false, false, true, false, false, true, false, false, false, false, false, false, false, true},
  {true, false, false, false, false, false, false, false, true, false, false, true, true, true, true, true, false, false, false, true},
  {true, false, false, false, false, false, false, false, true, false, false, false, true, false, false, true, false, false, false, true},
  {true, false, false, false, false, false, false, false, true, false, false, false, true, false, false, true, false, false, false, true},
  {true, false, false, false, false, true, true, true, true, false, false, false, true, false, false, true, true, false, false, true},
  {true, false, false, false, false, true, false, false, false, false, false, false, true, false, false, false, false, false, false, true},
  {true, false, false, false, false, false, false, false, false, false, false, false, true, false, false, false, false, false, false, true},
  {true, false, false, false, false, false, false, false, false, false, false, false, true, false, false, false, false, false, false, true},
  {true, false, false, false, false, false, false, false, true, false, false, false, true, false, false, false, false, false, false, true},
  {true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true, true},
  {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false}
};

Pose currentPose;


uint8_t exploreGrid[GRIDSIZE][GRIDSIZE];

void setup()
{
  currentPose.x = 8;
  currentPose.y = 16;
  currentPose.heading = EST;

  for (uint8_t x = 0; x < GRIDSIZE; x++)
  {
    for (uint8_t y = 0; y < GRIDSIZE; y++)
    {
      if(occupancyGrid[x][y])
      {
        exploreGrid[x][y] = OCCUPIED;
      }
      else
      {
        exploreGrid[x][y] = UNEXPLORED;
      }      
    }
  }
  

  // Start LCD
  lcd.begin(16, 2);

  // Init ADC
  ADCInit();

  // Initialise timer 1
  timer1Init();

  // Set timer 2 to CTC mode, set prescaler to 64, set overflow value to 250, enable overflow interrupt
  // Equates to approx 1ms period
  TCCR2A = (1 << WGM21);
  TCCR2B = (1 << CS22);
  OCR2A = 250;
  TIMSK2 = (1 << OCIE2A);

  // Start serial
  Serial.begin(9600);

  // Send start command to sim
  PrintMessage("CMD_START");
  // Enable interrupts
  sei();

}

void loop()
{
  // Read and round button value
  buttonVal = buttonRound(ADCRead(0));

  // Check how much time has elapsed since last button read
  // If over 100ms, check if same as previous value
  // If same, set buttonRead flag, if not, save previous value and do nothing
  buttonElapsed = millisecs - debounceTime;

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

  // Case switch statement which deals with the various menu states
  switch (menuState)
  {
    // Main menu with control mode flashing
    case MD_START_CON:
      // Print SN and flash con mode
      lcd.setCursor(0, 0);
      lcd.print("12051342");
      printHelp("", 0, 0);
      lcd.setCursor(0, 1);
      printHelp("Main Menu Con", 10, 3);

      // If a button has been read, handle it
      if (buttonRead)
      {
        switch (buttonVal)
        {
          // Select goes to con mode
          case SEL_PB:
            lcd.clear();
            menuState = MD_CON;
            break;

          // Down cycles menu
          case DWN_PB:
            menuState = MD_START_SWP;
            break;

          default:
            break;
        }
      }
      break;

    // Main menu with sweep mode flashing
    case MD_START_SWP:
      // Pring SN and flash sweep mode
      lcd.setCursor(0, 0);
      lcd.print("12051342");
      printHelp("", 0, 0);
      lcd.setCursor(0, 1);
      printHelp("Main Menu Sweep", 10, 5);

      // If a button has been read, handle it
      if (buttonRead)
      {
        switch (buttonVal)
        {
          // Select, go to Sweep mode
          case SEL_PB:
            lcd.clear();
            menuState = MD_SWP;
            break;

          // Down, cycle through menu
          case DWN_PB:
            menuState = MD_START_WF;
            break;

          // Anything else, do nothing
          default:
            break;
        }
      }
      break;

    // Main menu with WF flashing
    case MD_START_WF:
      lcd.setCursor(0, 0);
      lcd.print("12051342");
      printHelp("", 0, 0);
      lcd.setCursor(0, 1);
      printHelp("Main Menu WF", 10, 2);

      // Handle button press
      if (buttonRead)
      {
        switch (buttonVal)
        {
          case SEL_PB:
            // Select, start wall follow procedure
            // Update menu, then sweep and adjust for 2m gap, then begin following wall
            break;

          // Down, cycle menu
          case DWN_PB:
            menuState = MD_START_NAV;
            break;

          default:
            break;
        }
      }
      break;

    // Main menu with Nav flashing
    case MD_START_NAV:
      lcd.setCursor(0, 0);
      lcd.print("12051342");
      printHelp("", 0, 0);
      lcd.setCursor(0, 1);
      printHelp("Main Menu Nav", 10, 3);
      // Handle button press
      if (buttonRead)
      {
        switch (buttonVal)
        {
          // Select, start navigating to goal
          case SEL_PB:
            menuState = MD_NAV;
            lcd.clear();
            break;

          // Down, navigate menu
          case DWN_PB:
            menuState = MD_START_CON;
            break;

          default:
            break;
        }
      }
      break;

    // Control mode
    case MD_CON:

      lcd.setCursor(0, 0);
      lcd.print("12051342");
      lcd.setCursor(0, 1);
      lcd.print("Control");
      // Handle button press
      if (buttonRead)
      {
        switch (buttonVal)
        {
          // Select, go to main menu
          case SEL_PB:
            lcd.clear();
            menuState = MD_START_CON;
            break;

          // Left and right, rotate bot
          case LFT_PB:
            PrintMessage("CMD_ACT_ROT_0_5");
            break;

          case RIT_PB:
            PrintMessage("CMD_ACT_ROT_1_5");
            break;

          // Up and down, move backward and forward
          case UP_PB:
            PrintMessage("CMD_ACT_LAT_1_0.5");
            break;

          case DWN_PB:
            PrintMessage("CMD_ACT_LAT_0_0.5");
            break;

          default:
            break;
        }
      }
      break;

    // Sweep mode
    case MD_SWP:

      lcd.setCursor(0, 0);
      lcd.print("12051342");
      lcd.setCursor(0, 1);
      lcd.print("Sweep");
      // Handle button press
      if (buttonRead)
      {
        switch (buttonVal)
        {
          // Select returns to start up mode
          case SEL_PB:
            lcd.clear();
            menuState = MD_START_SWP;
            break;

          // Up, sweep
          case UP_PB:
            break;

          default:
            break;
        }
      }
      break;

    // WF Mode
    case MD_WF:

      lcd.setCursor(0, 0);
      lcd.print("12051342");
      lcd.setCursor(0, 1);
      lcd.print("Wall follow");

      // Handle button press
      if (buttonRead)
      {
        switch (buttonVal)
        {
          // Select returns to main menu
          case SEL_PB:
            lcd.clear();
            menuState = MD_START_WF;
            break;
          
          // Up, stop following wall
          case UP_PB:
            break;
        }
      }
      break;

    // Nav Mode
    case MD_NAV:
      lcd.setCursor(0, 0);
      lcd.print("12051342");
      lcd.setCursor(0, 1);
      lcd.print("Navigation");

      // Handle button press
      if (buttonRead)
      {
        switch (buttonVal)
        {
          // Select, stop navigating and return to main menu
          case SEL_PB:
            lcd.clear();
            menuState = MD_START_NAV;
            break;
        }
      }
      break;

    // Navigation finished mode
    case MD_NAV_FIN:
      lcd.setCursor(0, 0);
      lcd.print("Finished");
      lcd.setCursor(0, 1);
      lcd.print("Navigation");
      // Handle button press
      if (buttonRead)
      {
        switch (buttonVal)
        {
          // Select returns to main menu
          case SEL_PB:
            lcd.clear();
            menuState = MD_START_NAV;
            break;
        }
      }
      break;
  }

  // Buttons have been handled and menu has been updated, set to false to ensure they don't get read again until necessary
  buttonRead = false;
}


void PlanPath(Pose goal, Pose current)
{
  uint16_t planGrid[GRIDSIZE][GRIDSIZE];
  for (uint8_t x = 0; x < GRIDSIZE; x++)
  {
    for (uint8_t y = 0; y < GRIDSIZE; y++)
    {
      if(!occupancyGrid[x][y])
      {
        
      }
    }
  }
}

// Function for finding bearing and distance of goal from 2 distance readings
// Used trilateration to find the goal
void FindGoal(float distanceA, float distanceB)
{
  // Ensure distanceA is always greater than distanceB
  // Helped with some weird angle NaN errors
  if (distanceB > distanceA)
  {
    float temp = distanceB;
    distanceB = distanceA;
    distanceA = temp;
  }

  // Find X and Y coordinates
  // Formulae from https://en.wikipedia.org/wiki/True-range_multilateration#Two_Cartesian_dimensions,_two_measured_slant_ranges_(Trilateration)
  // This method gives two "points of interest" (POIs) at (x,y) and (x,-y) so both must be checked
  float x = (pow(distanceA, 2) - pow(distanceB, 2) + pow(0.5, 2)) / (2 * 0.5);
  float y = sqrt(pow(distanceA, 2) - (pow(x, 2)));
  
  // If y is NaN, abort function to avoid crashes
  if (y != y)
  {
    return;
  }

  // Find bearing and distance of goal using pythagorus
  float GoalAngle = RadsToDegrees(atan(y/x));
  float goalRange = sqrt(pow(x, 2) + pow(y, 2));

  // Rotate to first POI and move to it
  commandString = String("CMD_ACT_ROT_0_" + String(GoalAngle));
  PrintMessage(commandString);
  commandString = String("CMD_ACT_LAT_1_" + String(goalRange));
  PrintMessage(commandString);

  // Ping goal distance, if it isn't 0 and within 0.5m, consider it found and stop navigating
  PrintMessage("CMD_SEN_PING");
  float findGoalDist = SerialRead();
  if ((findGoalDist <= 0.5) && (findGoalDist > 0))
  {
    menuState = MD_NAV_FIN;
  }

  // If the goal was not at the first POI, check the second by moving back to the initial point,
  // rotating 2x the initial angle in the opposite direction, and moving towards the second POI
  else
  {
    commandString = String("CMD_ACT_LAT_0_" + String(goalRange));
    PrintMessage(commandString);
    commandString = String("CMD_ACT_ROT_1_" + String(2 * GoalAngle));
    PrintMessage(commandString);
    commandString = String("CMD_ACT_LAT_1_" + String(goalRange));
    PrintMessage(commandString);

    // Ping goal distance, if it isn't 0 and within 0.5m, consider it found and stop navigating
    PrintMessage("CMD_SEN_PING");
    findGoalDist = SerialRead();
    if ((findGoalDist <= 0.5) && (findGoalDist > 0))
    {
      menuState = MD_NAV_FIN;
    }
  }
}

// Reads from the serial port and puts the read value into float form
float SerialRead()
{
  // Wait until there is data available
  while (Serial.available() == 0);

  // Read string until terminator carriage return and newline are found
  String inString = Serial.readStringUntil('\r\n');

  // If the string starts with an N, its a NAN and should be considered the largest number
  if (inString[0] == 'N')
  {
    return FLT_MAX;
  }

  // If not NaN, return the float value
  else
  {
    return inString.toFloat();
  }
}



// Converts radians to degrees
double RadsToDegrees(double radAngle)
{
  return radAngle * (180.0 / M_PI);
}

// Initialised timer 1
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

// Takes a string, pads it to 16 characters and blocks characters from index to index+numToBlock
// Used to make menus blink and ensure no stray characters are left printed to the LCD
void printHelp(char inString[], uint8_t index, uint8_t numToBlock)
{
  // Find size of string
  size_t arraySize = strlen(inString);

  // Create new 16 char long string
  char outString[16];

  // Copy inString to outString and pad with spaces
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

  // If the menu is to have block chars instead of regular characters, block out the desired chars and print to LCD
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

// Rounds the button values
// They can be inconsistent so this just makes life easier
int buttonRound(int checkValue)
{
  // Check if the value is within the given range for a given button value
  // If it is, return the ideal value
  if ((checkValue >= (LFT_PB - PB_BOUND)) && (checkValue <= (LFT_PB + PB_BOUND)))
  {
    return LFT_PB;
  }
  if (checkValue <= (RIT_PB + PB_BOUND))
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

// Initialise ADC
void ADCInit()
{
  // Use interval voltage reference
  ADMUX |= (1 << REFS0);

  // Set 8-bit resolution
  // ADMUX |= (1 << ADLAR);

  // 128 prescale for 16Mhz
  ADCSRA |= (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  // Enable the ADC
  ADCSRA |= (1 << ADEN);
}

// Reads from the ADC
uint16_t ADCRead(uint8_t channel)
{
  // If the channel is out of range, return zero
  if ((channel < 0) || (channel > 7))
  {
    return 0;
  }

  // Set ADCMux to zero and select VCC as reference
  ADMUX = (1 << REFS0);

  // Mask and select ADC channel to read from
  ADMUX |= (0b00001111 & channel);

  // Start ADC read
  ADCSRA |= (1 << ADSC);

  // Do nothing while reading
  while ((ADCSRA & (1 << ADSC)))
    ;

  // Return read ADC value
  return ADC;
}


// Outputs serial command followed by terminators for MATLAB reading
void PrintMessage(String message)
{
  Serial.print(message);
  Serial.write(13); //carriage return character (ASCII 13, or '\r')
  Serial.write(10); //newline character (ASCII 10, or '\n')
}

// Timer 1 ISR, runs every 1s
ISR(TIMER1_COMPA_vect)
{
  // Increment seconds and minutes
  if (seconds >= 59)
  {
    seconds = 0;
    minutes++;
  }
  else
  {
    seconds++;
  }
  // Clear interrupt flag, not strictly necessary because it gets cleared when the ISR runs
  TIFR1 = (1 << OCF1A);

  // Flip value which blocks out menu selection
  blocked = !blocked;
}

// Timer 2 ISR, runs every 1ms
ISR(TIMER2_COMPA_vect)
{
  // Increment millisecond count
  millisecs++;
}