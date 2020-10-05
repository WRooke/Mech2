/*
  MX2 Assignment 2
  Written by W Rooke
  SN: 12051342
  Date 2/9/2020
*/

// Include necessary libraries
#include <LiquidCrystal.h>
#include <avr/io.h>
#include <float.h>
#include <math.h>

// Define LCD shield button values
// These are the ideal values read from the ADC when a button is pressed
const uint16_t STEPS = 4096;
const uint16_t SEL_PB = 640;
const uint16_t UP_PB = 100;
const uint16_t DWN_PB = 257;
const uint16_t LFT_PB = 410;
const uint16_t RIT_PB = 0;
const uint16_t NONE_PB = 1023;

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

// Variables used for distance calculations and sensor readings
const uint8_t NUMREADINGS = 72;
float sensorReadings[NUMREADINGS];
uint16_t sensorVal = 0;
uint8_t wheelSize = 20;
float numRevs = 0;
float distance = 0.0;

// Menu helper variables
bool menuUpdate = true;
uint16_t menuElapsed = 0;
uint16_t menuTime = 0;
volatile bool blocked = false;

uint16_t sensorRotation = 0;
String commandString = "";
float sensorRead = 0;

bool wallFollow = false;
bool nav = false;

float prevWallDist = 2.0;
float currWallDist = 0.0;
float wallAngle = 90.0;
float wfDistance = 0.5;

uint8_t farCorrections = 0;
uint8_t nearCorrections = 0;

void setup()
{
  // Set array of sensor readings to zero on startup
  for (uint8_t x = 0; x < NUMREADINGS; x++)
  {
    sensorReadings[x] = FLT_MAX;
  }

  // Start LCD
  lcd.begin(16, 2);

  ADCInit();
  // Initialise timer 1
  timer1Init();

  // Set timer 2 to CTC mode, set prescaler to 64, set overflow value to 250, enable overflow interrupt
  // Equates to approx 1ms period
  TCCR2A = (1 << WGM21);
  TCCR2B = (1 << CS22);
  OCR2A = 250;
  TIMSK2 = (1 << OCIE2A);

  Serial.begin(9600);
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
    // Start up mode
    case MD_START_CON:
      // Print minutes, seconds since startup and SN
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

          case SEL_PB:
            lcd.clear();
            menuState = MD_CON;
            break;

          case DWN_PB:
            menuState = MD_START_SWP;
            break;

          // REMOVE ME BEFORE SUBMISSION
          case UP_PB:
            PrintMessage("CMD_CLOSE");
            break;

          default:
            break;
        }
      }
      break;

    // Debug mode with IR mode blinking
    case MD_START_SWP:
      // Pring debug mode and menu string
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

          case DWN_PB:
            menuState = MD_START_WF;
            break;

            // REMOVE ME BEFORE SUBMISSION
          case UP_PB:
            PrintMessage("CMD_CLOSE");
            break;

          // Anything else, do nothing
          default:
            break;
        }
      }
      break;

    // Debug mode with CM flashing
    case MD_START_WF:
      lcd.setCursor(0, 0);
      lcd.print("12051342");
      printHelp("", 0, 0);
      lcd.setCursor(0, 1);
      printHelp("Main Menu WF", 11, 2);

      // Handle button press
      if (buttonRead)
      {
        switch (buttonVal)
        {
          case SEL_PB:
            menuState = MD_WF;
            distance = sensorReadings[Sweep(true)];
            if (distance < 2.0)
            {
              commandString = String("CMD_ACT_LAT_0_" + String(2.0 - distance));
            }
            else
            {
              commandString = String("CMD_ACT_LAT_1_" + String(distance - 2.0));
            }
            PrintMessage(commandString);
            PrintMessage("CMD_ACT_ROT_1_90");
            lcd.clear();
            wallFollow = true;
            break;

          case DWN_PB:
            menuState = MD_START_NAV;
            break;

            // REMOVE ME BEFORE SUBMISSION
          case UP_PB:
            PrintMessage("CMD_CLOSE");
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
      printHelp("Main Menu Nav", 11, 3);
      // Handle button press
      if (buttonRead)
      {
        switch (buttonVal)
        {
          case SEL_PB:
            menuState = MD_NAV;
            lcd.clear();
            nav = true;
            break;

          // Left and right navigate menu
          case DWN_PB:
            menuState = MD_START_CON;
            break;

            // REMOVE ME BEFORE SUBMISSION
          case UP_PB:
            PrintMessage("CMD_CLOSE");
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

          // Rotate bot
          case LFT_PB:
            PrintMessage("CMD_ACT_ROT_0_5");
            break;

          case RIT_PB:
            PrintMessage("CMD_ACT_ROT_1_5");
            break;

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

          // Start rotating
          case UP_PB:
            Sweep(true);
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
        // Select, exit to debug mode
        switch (buttonVal)
        {
          // Select returns to start up mode
          case SEL_PB:
            lcd.clear();
            menuState = MD_START_WF;
            break;

          case UP_PB:
            wallFollow = false;
            break;
        }
      }
      break;

    // Nav Mode
    case MD_NAV:
      // Print CM mode and Start Exit menu
      lcd.setCursor(0, 0);
      lcd.print("12051342");
      lcd.setCursor(0, 1);
      lcd.print("Navigation");

      // Handle button press
      if (buttonRead)
      {
        switch (buttonVal)
        {
          case SEL_PB:
            lcd.clear();
            menuState = MD_START_NAV;
            nav = false;
            break;
        }
      }
      break;

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
          case SEL_PB:
            lcd.clear();
            menuState = MD_START_NAV;
            nav = false;
            break;
        }
      }
      break;
  }

  // Buttons have been handled and menu has been updated, set to false to ensure they don't get read again until necessary
  buttonRead = false;
  menuUpdate = false;

  if (wallFollow)
  {
    FollowWall();
  }

  if (nav)
  {
    NavToGoal();
  }
}

void NavToGoal()
{
  PrintMessage("CMD_SEN_PING");
  float goalDistA = SerialRead();
  if (goalDistA != 0)
  {
    if (goalDistA <= 0.5)
    {
      nav = false;
      menuState = MD_NAV_FIN;
    }
    else
    {
      Serial.print("Distance to goal:");
      Serial.println(goalDistA);
      float goalDistB;
      PrintMessage("CMD_ACT_LAT_0_0.5");
      PrintMessage("CMD_SEN_PING");
      goalDistB = SerialRead();
      if (goalDistB != 0)
      {
        FindGoal(goalDistA, goalDistB);
      }
      else
      {
        PrintMessage("CMD_ACT_LAT_1_0.5");
        PrintMessage("CMD_SEN_ROT_90");
        PrintMessage("CMD_SEN_IR");
        if (SerialRead() > 0.5)
        {
          PrintMessage("CMD_ACT_ROT_0_90");
          PrintMessage("CMD_ACT_LAT_1_0.5");
        }
      }
    }
  }
  else
  {
    Serial.println("Goal not found");
    distance = sensorReadings[Sweep(false)];
    if (distance < 5)
    {
      commandString = String("CMD_ACT_LAT_1_" + String(distance - 0.5));
      PrintMessage(commandString);
    }
    else
    {
      PrintMessage("CMD_ACT_LAT_1_4");
    }
  }
}

void FindGoal(float distanceA, float distanceB)
{
  if (distanceB > distanceA)
  {
    float temp = distanceB;
    distanceB = distanceA;
    distanceA = temp;
  }
  Serial.print("dista:");
  Serial.println(distanceA);
  Serial.print("distb:");
  Serial.println(distanceB);
  float x = (pow(distanceA, 2) - pow(distanceB, 2) + pow(0.5, 2)) / (2 * 0.5);
  float yPos = sqrt(pow(distanceA, 2) - (pow(x, 2)));
  if (yPos != yPos)
  {
    return;
  }
  Serial.print("x:");
  Serial.println(x);
  Serial.print("y:");
  Serial.println(yPos);
  float GoalAngle = RadsToDegrees(atan(yPos/x));
  float goalRange = sqrt(pow(x, 2) + pow(yPos, 2));

  commandString = String("CMD_ACT_ROT_0_" + String(GoalAngle));
  PrintMessage(commandString);
  commandString = String("CMD_ACT_LAT_1_" + String(goalRange));
  PrintMessage(commandString);
  PrintMessage("CMD_SEN_PING");
  float findGoalDist = SerialRead();
  if ((findGoalDist <= 0.5) && (findGoalDist > 0))
  {
    nav = false;
    menuState = MD_NAV_FIN;
  }
  else
  {
    commandString = String("CMD_ACT_LAT_0_" + String(goalRange));
    PrintMessage(commandString);
    commandString = String("CMD_ACT_ROT_1_" + String(2 * GoalAngle));
    PrintMessage(commandString);
    commandString = String("CMD_ACT_LAT_1_" + String(goalRange));
    PrintMessage(commandString);
  }
}

float SerialRead()
{
  while (Serial.available() == 0)
    ;
  String inString = Serial.readStringUntil('\r\n');
  if (inString[0] == 'N')
  {
    return FLT_MAX;
  }
  else
  {
    return inString.toFloat();
  }
}

uint8_t Sweep(bool min)
{
  // uint8_t index = NUMREADINGS;
  uint8_t rotIndex = 0;
  for (int16_t sensorRotation = 355; sensorRotation >= 0; sensorRotation -= 5)
  {
    commandString = String("CMD_SEN_ROT_" + String(sensorRotation));
    PrintMessage(commandString);
    PrintMessage("CMD_SEN_IR");
    sensorReadings[(sensorRotation * 2) / 10] = SerialRead();
  }
  if (min)
  {
    rotIndex = arrayMin(sensorReadings);
  }
  else
  {
    rotIndex = arrayMax(sensorReadings);
  }
  PrintMessage("CMD_SEN_ROT_0");
  commandString = String("CMD_ACT_ROT_0_" + String((rotIndex * 10) / 2));
  PrintMessage(commandString);
  return rotIndex;
}

void FollowWall()
{
  // Check distance from parallel wall
  PrintMessage("CMD_SEN_ROT_90");
  PrintMessage("CMD_SEN_IR");
  currWallDist = SerialRead();
  Serial.print("current wall distance:");
  Serial.println(currWallDist);
  if ((currWallDist - 2.0) > 0.15)
  {

    PrintMessage("CMD_ACT_ROT_0_90");
    commandString = String("CMD_ACT_LAT_1_" + String(currWallDist - 2.0));
    PrintMessage(commandString);
    PrintMessage("CMD_ACT_ROT_1_90");
    farCorrections++;
  }
  else if ((currWallDist - 2.0) < -0.15)
  {
    PrintMessage("CMD_ACT_ROT_0_90");
    commandString = String("CMD_ACT_LAT_0_" + String(2.0 - currWallDist));
    PrintMessage(commandString);
    PrintMessage("CMD_ACT_ROT_1_90");
    nearCorrections++;
  }
  if (farCorrections > 1)
  {
    PrintMessage("CMD_ACT_ROT_0_5");
    farCorrections = 0;
  }
  if (nearCorrections > 1)
  {
    PrintMessage("CMD_ACT_ROT_1_5");
    nearCorrections = 0;
  }

  // Check distance from upcoming wall
  PrintMessage("CMD_SEN_ROT_0");
  PrintMessage("CMD_SEN_IR");
  wfDistance = SerialRead();
  if (wfDistance == FLT_MAX)
  {
    wfDistance = 3.0;
    commandString = String("CMD_ACT_LAT_1_" + String(wfDistance));
    PrintMessage(commandString);
  }
  else
  {
    commandString = String("CMD_ACT_LAT_1_" + String(wfDistance - 2.0));
    PrintMessage(commandString);
    PrintMessage("CMD_ACT_ROT_1_90");
    // prevWallDist = 2.0;
    // currWallDist = 2.0;
  }

  // If wall not found, move forward x amount

  // If wall found, move to wall and rotate 90deg
}

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

// Finds index of minimum value in an array
uint8_t arrayMin(float inArray[])
{
  float min = FLT_MAX;
  uint8_t index = 0;
  for (uint8_t x = 0; x < NUMREADINGS; x++)
  {
    if (inArray[x] < min)
    {
      index = x;
      min = inArray[x];
    }
  }
  return index;
}

uint8_t arrayMax(float inArray[])
{
  float max = 0;
  uint8_t index = 0;
  for (uint8_t x = NUMREADINGS; x > 0; x--)
  {
    if (inArray[x] > max)
    {
      index = x;
      max = inArray[x];
    }
  }
  return index;
}

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

ISR(TIMER2_COMPA_vect)
{
  // Increment millisecond count
  millisecs++;
}