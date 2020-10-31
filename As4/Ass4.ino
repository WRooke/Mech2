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


typedef struct pathnode{
  uint8_t x;
  uint8_t y;
  struct pathnode* next;
}PathNode;

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
const uint8_t MD_START = 10;
const uint8_t MD_START_SWP = 11;
const uint8_t MD_START_WF = 12;
const uint8_t MD_START_NAV = 13;

const uint8_t MD_CON = 20;

const uint8_t MD_SWP = 30;

const uint8_t MD_GO = 40;

const uint8_t MD_NAV = 50;
const uint8_t MD_NAV_FIN = 51;

const uint8_t OCCUPIED = 255;
const uint8_t PLACEHOLDER = 200;

const uint16_t NTH = 0;
const uint16_t EST = 90;
const uint16_t STH = 180;
const uint16_t WST = 270;


uint8_t menuState = MD_START;

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

uint8_t occupancyGrid[GRIDSIZE][GRIDSIZE] = {
  {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
  {255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255},
  {255,0,0,0,0,0,0,0,255,0,0,0,255,0,0,0,0,0,0,255},
  {255,0,0,0,0,0,0,0,0,0,0,0,255,0,0,0,0,0,0,255},
  {255,0,0,0,0,0,0,0,0,0,0,0,255,0,0,0,0,0,0,255},
  {255,0,0,0,0,255,0,0,0,0,0,0,255,0,0,0,0,0,0,255},
  {255,0,0,0,0,255,255,255,255,0,0,0,255,0,0,255,255,0,0,255},
  {255,0,0,0,0,0,0,0,255,0,0,0,255,0,0,255,0,0,0,255},
  {255,0,0,0,0,0,0,0,255,0,0,0,255,0,0,255,0,0,0,255},
  {255,0,0,0,0,0,0,0,255,0,0,255,255,255,255,255,0,0,0,255},
  {255,0,0,0,0,255,0,0,255,0,0,255,0,0,0,0,0,0,0,255},
  {255,0,0,0,0,255,0,0,255,0,0,255,0,0,0,0,0,0,0,255},
  {255,0,0,0,0,255,0,0,0,0,0,255,0,0,0,0,0,0,0,255},
  {255,255,255,255,255,255,0,0,0,0,0,255,0,0,255,255,255,255,255,255},
  {255,0,0,0,0,0,0,0,0,0,0,255,0,0,0,0,0,0,0,255},
  {255,0,0,0,0,0,0,0,255,255,255,255,0,0,0,0,255,0,0,255},
  {255,0,0,0,0,0,0,0,0,0,0,255,0,0,0,255,255,0,0,255},
  {255,255,255,0,0,0,0,0,0,0,0,0,0,0,0,0,255,0,0,255},
  {255,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,255},
  {255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255}
};

bool exploredGrid[GRIDSIZE][GRIDSIZE];

Pose currentPose;


void setup()
{
  currentPose.x = 2;
  currentPose.y = 4;
  currentPose.heading = EST;

  for(uint8_t x = 0; x < GRIDSIZE; x++)
  {
    for(uint8_t y = 0; y < GRIDSIZE; y++)
    {
      if((occupancyGrid[y][x] != OCCUPIED))
      {
        occupancyGrid[y][x] = PLACEHOLDER;
        exploredGrid[y][x] = false;
      }
      else
      {
        exploredGrid[y][x] = true;
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
  // PrintMessage("CMD_ACT_LAT_1_0.5");
  // PrintMessage("CMD_ACT_ROT_0_90");
  // PrintMessage("CMD_ACT_LAT_1_0.5");
  // PrintMessage("CMD_ACT_ROT_1_90");
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
    case MD_START:
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
            // lcd.clear();
            // menuState = MD_GO;
            computePath(currentPose.x,currentPose.y, 15, 4);
            break;
          case UP_PB:
            PrintMessage("CMD_CLOSE");
          default:
            break;
        }
      }
      break;

    // Main menu with sweep mode flashing
    case MD_GO:
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
  }
    

  // Buttons have been handled and menu has been updated, set to false to ensure they don't get read again until necessary
  buttonRead = false;
}

void localise()
{
  const uint16_t priorHeading = currentPose.heading;
  // Ping distance to compass points
  // Compare sensor distance to "expected" pose and move accordingly
  float compassDistance[4];

  for(uint8_t count = 0; count < 4; count++)
  {
    commandString = String("CMD_SEN_ROT_" + String(count * 90));
    PrintMessage(commandString);
    PrintMessage("CMD_SEN_IR");
    compassDistance[count] = SerialRead();
  }

  // Ideal distances
  int8_t northWall = currentPose.y + 1;
  int8_t southWall = currentPose.y - 1;
  int8_t eastWall = currentPose.x + 1;
  int8_t westWall = currentPose.x - 1;

  Serial.print("current pose x:");
  Serial.println(currentPose.x);
  Serial.print("current pose y");
  Serial.println(currentPose.y);

  float northDist = FLT_MAX;
  float southDist = FLT_MAX;
  float eastDist = FLT_MAX;
  float westDist = FLT_MAX;

  // Get wall locations
  while((occupancyGrid[northWall][currentPose.x] != OCCUPIED) && (northWall < 20))
  {
    northWall++;
  }
  while((occupancyGrid[southWall][currentPose.x] != OCCUPIED) && (southWall > 0))
  {
    southWall--;
  }
  while((occupancyGrid[currentPose.y][eastWall] != OCCUPIED) && (eastWall < 20))
  {
    eastWall++;
  }
  while((occupancyGrid[currentPose.y][westWall] != OCCUPIED) && (westWall > 0))
  {
    westWall--;
  }

  Serial.print("grid locatiopn of north wall:");
  Serial.println(northWall);
  Serial.print("grid locatiopn of south wall:");
  Serial.println(southWall);
  Serial.print("grid locatiopn of east wall:");
  Serial.println(eastWall);
  Serial.print("grid locatiopn of west wall:");
  Serial.println(westWall);

  northWall = abs(currentPose.y - northWall);
  southWall = abs(currentPose.y - southWall - 1);
  eastWall = abs(currentPose.x - eastWall);
  westWall = abs(currentPose.x - westWall - 1);


  Serial.print("ideal metres to north wall:");
  Serial.println(northWall);
  Serial.print("ideal metres to south wall:");
  Serial.println(southWall);
  Serial.print("ideal metres to east wall:");
  Serial.println(eastWall);
  Serial.print("ideal metres to west wall:");
  Serial.println(westWall);

  // Gotta compare ideals with reals but not fuck up the heading. Maybe get current heading and add multiples of array index on top. If over 360, subtract 360

  uint16_t currentHeading = currentPose.heading;

  if (currentHeading == NTH)
  {
    northDist = compassDistance[0];
    westDist = compassDistance[1];
    southDist = compassDistance[2];
    eastDist = compassDistance[3];
  }

  if (currentHeading == WST)
  {
    westDist = compassDistance[0];
    southDist = compassDistance[1];
    eastDist = compassDistance[2];
    northDist = compassDistance[3];
  }

  if (currentHeading == STH)
  {
    southDist = compassDistance[0];
    eastDist = compassDistance[1];
    northDist = compassDistance[2];
    westDist = compassDistance[3];
  }

  if (currentHeading == EST)
  {
    eastDist = compassDistance[0];
    northDist = compassDistance[1];
    westDist = compassDistance[2];
    southDist = compassDistance[3];
  }


  Serial.print("measured metres to north wall:");
  Serial.println(northDist);
  Serial.print("measured metres to south wall:");
  Serial.println(southDist);
  Serial.print("measured metres to east wall:");
  Serial.println(eastDist);
  Serial.print("measured metres to west wall:");
  Serial.println(westDist);

  if (northDist != FLT_MAX)
  {
    northDist = northDist - (float)northWall;
    if (northDist > 0.2)
    {
      Serial.print("difference between pose and north wall");
      Serial.println(northDist);
      cumulativeRotate(NTH);
      Serial.println("moving");
      commandString = String("CMD_ACT_LAT_1_" + String(northDist));
      PrintMessage(commandString);
      cumulativeRotate(priorHeading);
    }
    else if (northDist < -0.2)
    {
      Serial.print("difference between pose and north wall");
      Serial.println(northDist);
      cumulativeRotate(NTH);
      Serial.println("moving");
      commandString = String("CMD_ACT_LAT_0_" + String(fabs(northDist)));
      PrintMessage(commandString);
      cumulativeRotate(priorHeading);
    }
  }
  else if (southDist != FLT_MAX)
  {
    southDist = southDist - (float)southWall;
    if (southDist > 0.2)
    {
      Serial.print("difference between pose and south wall");
      Serial.println(southDist);
      cumulativeRotate(STH);
      Serial.println("moving");
      commandString = String("CMD_ACT_LAT_0_" + String(southDist));
      PrintMessage(commandString);
      cumulativeRotate(priorHeading);
    }
    else if (southDist < -0.2)
    {
      Serial.print("difference between pose and south wall");
      Serial.println(southDist);
      cumulativeRotate(STH);
      Serial.println("moving");
      commandString = String("CMD_ACT_LAT_1_" + String(fabs(southDist)));
      PrintMessage(commandString);
      cumulativeRotate(priorHeading);
    }
  }
  if (eastDist != FLT_MAX)
  {
    eastDist = eastDist - (float)eastWall;
    if (eastDist > 0.2)
    {
      Serial.print("difference between pose and east wall");
      Serial.println(eastDist);
      cumulativeRotate(EST);
      Serial.println("moving");
      commandString = String("CMD_ACT_LAT_0_" + String(eastDist));
      PrintMessage(commandString);
      cumulativeRotate(priorHeading);
    }
    else if (eastDist < -0.2)
    {
      Serial.print("difference between pose and east wall");
      Serial.println(eastDist);
      cumulativeRotate(EST);
      Serial.println("moving");
      commandString = String("CMD_ACT_LAT_1_" + String(fabs(eastDist)));
      PrintMessage(commandString);
      cumulativeRotate(priorHeading);
    }
  }
  else if (westDist != FLT_MAX)
  {
    westDist = westDist - (float)westWall;
    if (westDist > 0.2)
    {
      Serial.print("difference between pose and west wall");
      Serial.println(westDist);
      cumulativeRotate(WST);
      Serial.println("moving");
      commandString = String("CMD_ACT_LAT_0_" + String(westDist));
      PrintMessage(commandString);
      cumulativeRotate(priorHeading);
    }
    else if (westDist < -0.2)
    {
      Serial.print("difference between pose and west wall");
      Serial.println(westDist);
      cumulativeRotate(WST);
      Serial.println("moving");
      commandString = String("CMD_ACT_LAT_1_" + String(fabs(westDist)));
      PrintMessage(commandString);
      cumulativeRotate(priorHeading);
    }
  }
}

void cumulativeRotate(uint16_t desiredHeading)
{
  int16_t angle = desiredHeading - currentPose.heading;
  uint8_t angleStep = 1;
  if(angle == 0)
  {
    return;
  }

  Serial.print("CumulativeRotate input angle:");
  Serial.println(angle);
  bool clockwise = true;
  if (angle < 0)
  {
    Serial.print("angle is negative, new angle:");
    angle = abs(angle);
    Serial.println(angle);
    Serial.print("rotating counterclockwise");
    // for (int16_t count = 0; count < angle; count += angleStep)
    for (float count = 0; count < angle; count += angleStep)
    {
      commandString = String("CMD_ACT_ROT_0_" + String(angleStep));
      PrintMessage(commandString);
    }
  }
  else
  {
    Serial.print("rotating clockwise");
    // for (int16_t count = 0; count < angle; count += angleStep)
    for (float count = 0; count < angle; count += angleStep)
    {
      commandString = String("CMD_ACT_ROT_1_" + String(angleStep));
      PrintMessage(commandString);
    }
  }
  currentPose.heading = desiredHeading;  
}

// Takes the value OF THE ORIGINAL CELL, NOT THE NEIGHBOUR
void propagateWavefront(uint8_t x, uint8_t y, uint16_t value)
{
  if (occupancyGrid[y+1][x] == PLACEHOLDER)
  {
    occupancyGrid[y+1][x] = value + 1;
  }
  if (occupancyGrid[y-1][x] == PLACEHOLDER)
  {
    occupancyGrid[y-1][x] = value + 1;
  }
  if (occupancyGrid[y][x+1] == PLACEHOLDER)
  {
    occupancyGrid[y][x+1] = value + 1;
  }
  if (occupancyGrid[y][x-1] == PLACEHOLDER)
  {
    occupancyGrid[y][x-1] = value + 1;
  }
}

void computePath(uint8_t startX, uint8_t startY, uint8_t goalX, uint8_t goalY)
{
  uint16_t wavefrontValue = 0;
  occupancyGrid[goalY][goalX] = wavefrontValue;
  while (1)
  {
    for(uint8_t x = 0; x < GRIDSIZE; x++)
    {
      for(uint8_t y = 0; y < GRIDSIZE; y++)
      {
        if(occupancyGrid[y][x] == wavefrontValue)
        {
          propagateWavefront(x, y, wavefrontValue);
        }
      }
    }
    wavefrontValue++;
    if (occupancyGrid[startY][startX] != PLACEHOLDER)
    {
      // Determine path
      executePath(startX, startY);
      // Reset occupancy grid
      break;
    }
    
  }
}

void executePath(uint8_t startX, uint8_t startY)
{
  uint16_t wavefrontValue = occupancyGrid[startY][startX];

  while(wavefrontValue != 0)
  {
    uint16_t heading = checkNeighbours(currentPose.x,currentPose.y,wavefrontValue);
    moveToNextCell(heading);    
    wavefrontValue--;
  }
}

uint16_t checkNeighbours(uint8_t x, uint8_t y, uint16_t wavefrontValue)
{
  if (occupancyGrid[y+1][x] == wavefrontValue - 1)
  {
    return NTH;
  }
  if (occupancyGrid[y-1][x] == wavefrontValue - 1)
  {
    return STH;
  }
  if (occupancyGrid[y][x+1] == wavefrontValue - 1)
  {
    return EST;
  }
  if (occupancyGrid[y][x-1] == wavefrontValue - 1)
  {
    return WST;
  }
  return 0;
}



void moveToNextCell(uint16_t desiredHeading)
{
  Serial.print("DesiredHeading:");
  Serial.println(desiredHeading);
  Serial.print("CurrentHeading:");
  Serial.println(currentPose.heading);
  int16_t commandHeading = desiredHeading - currentPose.heading;
  Serial.print("CommandHeading:");
  Serial.println(commandHeading);
  cumulativeRotate(desiredHeading);
  // commandString = String("CMD_ACT_ROT_1_" + String(commandHeading));
  // PrintMessage(commandString);
  PrintMessage("CMD_ACT_LAT_1_1");
  if(currentPose.heading == NTH)
  {
    currentPose.y++;
  }
  if(currentPose.heading == STH)
  {
    currentPose.y--;
  }
  if(currentPose.heading == EST)
  {
    currentPose.x++;
  }
  if(currentPose.heading == WST)
  {
    currentPose.x--;
  }
  localise();
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