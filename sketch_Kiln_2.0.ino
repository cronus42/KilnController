/* Kiln Controller v2.0
 *   
 */

// Libraries to include
#include <LiquidCrystal_I2C.h>  // LCD display library
#include <Adafruit_MAX31855.h>
#include <PID_v1.h>         // PID temp control library
#include <SPI.h>            // Serial Peripheral Interface library
#include <avr/pgmspace.h>   // PROGMEM library

// Setup user variables (CHANGE THESE TO MATCH YOUR SETUP)
const int lcdRefresh = 2500;           // Refresh rate to update screen when running (ms)
const int maxTemp = 1600;              // Maximum temperature (degrees).  If reached, will shut down.
const int numZones = 1;                // Number of heating element + thermostat zones (max of 3)
const int pidCycle = 2500;             // Time for a complete PID on/off cycle for the heating elements (ms)
double pidInput[numZones];             // Input array for PID loop (actual temp reading from thermocouple).  Don't change.
double pidOutput[numZones];            // Output array for PID loop (relay for heater).  Don't change.
double pidSetPoint[numZones];          // Setpoint array for PID loop (temp you are trying to reach).  Don't change.
PID pidCont[numZones] = {PID(&pidInput[0], &pidOutput[0], &pidSetPoint[0], 800, 47.37, 4.93, DIRECT)};  // PID controller array for each zone.  Set arguments 4/5/6 to the Kp, Ki, Kd values after tuning.
const long saveCycle = 15000;          // How often to save current temp / setpoint (ms) 
const int tempOffset[numZones] = {0};  // Array to add a temp offset for each zone (degrees).  Use if you have a cold zone in your kiln or if your thermocouple reading is off.  This gets added to the setpoint.
const int tempRange = 2;               // This is how close the temp reading needs to be to the set point to shift to the hold phase (degrees).  Set to zero or a positive integer.
const char tempScale = 'F';            // Temperature scale.  F = Fahrenheit.  C = Celsius

// Setup pin connections (CHANGE THESE TO MATCH YOUR SETUP)
                                                // Pins 0 + 1 are used for serial port
const int upPin = 2;                            // Pin # connected to up arrow button
const int downPin = 3;                          // Pin # connected to down arrow button
const int selectPin = 4;                        // Pin # connected to select / start button
Adafruit_MAX31855 thermo[numZones] = {Adafruit_MAX31855(5, 6 ,7)};  // Pins connected to the thermocouple card.  This is an array for each thermocouple (zone).
const int heaterPin[numZones] = {9};            // Pins connected to relays for heating elements.  This is an array for each output pin (zone).
                                                // Pins 10 thru 13 are for SD card.  These are automatically setup.
// LiquidCrystal lcd(19, 18, 17, 16, 15, 14);      // LCD display (connected to analog inputs / reverse order so I don't have to twist ribbon cable)
LiquidCrystal_I2C lcd(0x27, 20, 4); //I2C LCD

//Set up firing schedules and reference table
const char schedule_0[] PROGMEM = "Bisque Fire;Cone 04;by Plainsman;180,212,60;270,1760,0;180,1940,5";
const char schedule_1[] PROGMEM = "Glaze Fire;Cone 6;by Plainsman;100,220,60;300,1733,0;108,2195,15;150,2095,30";
const char* const schedule_table[] PROGMEM = {schedule_0, schedule_1};
 
// Setup other variables (DON'T CHANGE THESE)
double calcSetPoint;        // Calculated set point (degrees)
unsigned long holdStart;    // Exact time the hold phase of the segment started (ms).  Based on millis().
int i = 0;                  // Simple loop counter
int lastSeg = 0;            // Last segment number in firing schedule
int lastTemp;               // Last setpoint temperature (degrees)
unsigned long lcdStart;     // Exact time you refreshed the lcd screen (ms).  Based on millis().
int optionNum = 1;          // Option selected from screen #3
unsigned long pidStart;     // Exact time you started the new PID cycle (ms).  Based on millis().
double rampHours;           // Time it has spent in ramp (hours)
unsigned long rampStart;    // Exact time the ramp phase of the segment started (ms).  Based on millis().
unsigned long saveStart;    // Exact time you saved the temps (ms).  Based on millis().
char schedDesc1[21];        // Schedule description #1 (first line of text file)
int schedNum = 0;           // Current firing schedule number.  This ties to the file name (ex: 1.txt, 2.txt).
boolean schedOK = false;    // Is the schedule you loaded OK?
unsigned long schedStart;   // Exact time you started running the schedule (ms).  Based on millis().
int screenNum = 1;          // Screen number displayed during firing (1 = temps / 2 = schedule info / 3 = tools / 4 = done
int segHold[20];            // Hold time for each segment (min).  This starts after it reaches target temp.
int segNum = 0;             // Current segment number running in firing schedule.  0 means a schedule hasn't been selected yet.
boolean segPhase = 0;       // Current segment phase.  0 = ramp.  1 = hold.
int segRamp[20];            // Rate of temp change for each segment (deg/hr).
int segTemp[20];            // Target temp for each segment (degrees).

//******************************************************************************************************************************
//  SETUP: INITIAL SETUP (RUNS ONCE DURING START)
//******************************************************************************************************************************
void setup() {

  // Setup all pin modes on board.  Remove INPUT_PULLUP if you have resistors in your wiring.
  pinMode(upPin, INPUT_PULLUP);
  pinMode(downPin, INPUT_PULLUP);
  pinMode(selectPin, INPUT_PULLUP); 
  for (i = 0; i < numZones; i++) {
    pinMode(heaterPin[i], OUTPUT);
  }
  for (i = 14; i <= 19; i++) {  // Change analog inputs to digital outputs for LCD screen
    pinMode(i, OUTPUT);
  }
  
  // Setup lcd display (20 columns x 4 rows)
  lcd.begin();

  // Intro screens (i'm bored / delete if you want)
  lcd.print(F("  Kiln Controller"));
  lcd.setCursor(2, 1);
  lcd.print(F("---------------"));
  delay(2000);
/*
  lcd.clear();
  lcd.print(F("<----- On / Off"));
  lcd.setCursor(0, 3);
  lcd.print(F("<----- Heater On LED"));
  delay(2000);

  lcd.clear();
  lcd.print(F("               ^   ^"));
  lcd.setCursor(15, 1);
  lcd.print(F("|   |"));
  lcd.setCursor(0, 2);
  lcd.print(F("USB Connection--   |"));
  lcd.setCursor(0, 3);
  lcd.print(F("microSD Card--------"));
  delay(2000);

  lcd.clear();
  lcd.print(F("Up ---------------->"));
  lcd.setCursor(0, 1);
  lcd.print(F("Down -------------->"));
  lcd.setCursor(0, 2);
  lcd.print(F("Select / Start ---->"));
  delay(2000);

  lcd.clear();
  lcd.print(F("   LCD Brightness"));
  lcd.setCursor(10, 1);
  lcd.print(F("|"));
  lcd.setCursor(10, 2);
  lcd.print(F("|"));
  lcd.setCursor(10, 3);
  lcd.print(F("|"));
  delay(2000);
*/
  // Open firing shedule # 1
  openSched();

}

//******************************************************************************************************************************
//  LOOP: MAIN LOOP (CONTINUOUS)
//******************************************************************************************************************************
void loop() {

  //******************************
  // Shutdown if too hot
  for (i = 0; i < numZones; i++) {
    if (pidInput[i] >= maxTemp) {
      lcd.clear();
      lcd.print(F("       ERROR:"));
      lcd.setCursor(2, 2);
      lcd.print(F("Max temp reached"));
      lcd.setCursor(0, 3);
      lcd.print(F("System was shut down"));    
      shutDown();
    }
  }

  //******************************
  // Select a firing schedule
  if (segNum == 0) {
    
    // Up arrow button
    if (digitalRead(upPin) == LOW && schedNum > 0) {
      schedNum = schedNum - 1;
      openSched();
      btnBounce(upPin);
    }
    
    // Down arrow button
    if (digitalRead(downPin) == LOW) {
      schedNum = schedNum + 1;
      openSched();
      btnBounce(downPin);          
    }
    
    // Select / Start button
    if (digitalRead(selectPin) == LOW && schedOK == true) {
      setupPIDs();
      segNum = 1;
      lcdStart = millis();
      pidStart = millis();
      rampStart = millis();
      schedStart = millis();
      updateLCD();
      btnBounce(selectPin);      
    }
  }

  //******************************
  // Running the firing schedule
  if (segNum >= 1) {

    // Up arrow button
    if (digitalRead(upPin) == LOW) {
      if (screenNum == 2 || (screenNum == 3 && optionNum == 1)) {
        screenNum = screenNum - 1;
      }
      else if (screenNum == 3 && optionNum >= 2) {
        optionNum = optionNum - 1;
      }
      updateLCD();
      btnBounce(upPin); 
    }
    
    // Down arrow button
    if (digitalRead(downPin) == LOW) {
      if (screenNum <= 2) {
        screenNum = screenNum + 1;
      }
      else if (screenNum == 3 && optionNum <= 2) {
        optionNum = optionNum + 1;
      }
      updateLCD();
      btnBounce(downPin); 
    }

    // Select / Start button
    if (digitalRead(selectPin) == LOW && screenNum == 3) {
      if (optionNum == 1) {  // Add 5 min
        segHold[segNum - 1] = segHold[segNum - 1] + 5;
        optionNum = 1;
        screenNum = 2;
      }

      if (optionNum == 2) {  // Add 5 deg
        segTemp[segNum - 1] = segTemp[segNum - 1] + 5;
        optionNum = 1;
        screenNum = 1;
      }

      if (optionNum == 3) {  // Goto next segment
        segNum = segNum + 1;
        optionNum = 1;
        screenNum = 2;
      }

      updateLCD();
      btnBounce(selectPin);           
    }
 
    // Update PID's / turn on heaters / update segment info
    if (screenNum < 4) {
      if (millis() - pidStart >= pidCycle) {
        pidStart = millis();
        updatePIDs();
      }
      htrControl();
      updateSeg();
    }

    // Refresh the LCD
    if (millis() - lcdStart >= lcdRefresh) {
      updateLCD();
      lcdStart = millis();
    }
/*
    // Save the temps to a file on SD card
    if (millis() - saveStart >= saveCycle && pidInput[0] > 0 && screenNum < 4) {
      saveFile = SD.open("temps.txt", FILE_WRITE);
        saveFile.print((millis() - schedStart) / 60000.0); // Save in minutes
        for (i = 0; i < numZones; i++) {
          saveFile.print(",");
          saveFile.print(pidInput[i]);
        }
        saveFile.print(",");
        saveFile.println(pidSetPoint[0]);
      saveFile.close();

      saveStart = millis();
    }  
*/    
  }

}

//******************************************************************************************************************************
//  BTNBOUNCE: HOLD UNTIL BUTTON IS RELEASED.  DELAY FOR ANY BOUNCE
//******************************************************************************************************************************
void btnBounce(int btnPin) {

  while (digitalRead(btnPin) == LOW);
  delay(40);
  
}

//******************************************************************************************************************************
//  HTRCONTROL: TURN HEATERS ON OR OFF
//******************************************************************************************************************************
void htrControl() {

  // Loop thru all zones
  for (i = 0; i < numZones; i++) {
    if (pidOutput[i] >= millis() - pidStart) {
      digitalWrite(heaterPin[i], HIGH);
    }
    else {
      digitalWrite(heaterPin[i], LOW);      
    }
  }
  
}

//******************************************************************************************************************************
//  INTLENGTH: GET THE LENGTH OF A INTEGER
//******************************************************************************************************************************
int intLength(int myInt) {

  myInt = abs(myInt);

  if (myInt >= 10000) {
    return 5;
  }
  else if (myInt >= 1000) {
    return 4;
  }
  else if (myInt >= 100) {
    return 3;
  }
  else if (myInt >= 10) {
    return 2;
  }
  else {
    return 1;
  }
  
}


//******************************************************************************************************************************
//  split stings without pain
//******************************************************************************************************************************
String getValue(String data, char separator, int index)
{
    int found = 0;
    int strIndex[] = { 0, -1 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}


//******************************************************************************************************************************
//  OPENSCHED: OPEN AND LOAD A FIRING SCHEDULE FILE / DISPLAY ON SCREEN
//******************************************************************************************************************************
void openSched() {

  // Setup all variables
  int col = 1;          // Column number (of text file).  First column is one.
  int row = 1;          // Row number (of text file).  First row is one.
  char tempLine[21];    // Temporary character array holder
  int tempLoc = 0;      // Current location of next character to place in tempLine array
  char schedDesc2[21];  // Schedule description #2 (second line of text file)
  char schedDesc3[21];  // Schedule description #3 (third line of text file)
  char mySchedule[128];
  String line;

  // copy the schedule from PROGMEM to mySchedule string
  strcpy_P(mySchedule, (char*)pgm_read_word(&(schedule_table[schedNum])));

  // Clear the arrays
  memset(schedDesc1, 0, sizeof(schedDesc1));
  memset(segRamp, 0, sizeof(segRamp));
  memset(segTemp, 0, sizeof(segTemp)); 
  memset(segHold, 0, sizeof(segHold));  


  if (mySchedule == false) {
    lcd.clear();
    lcd.print(F("SELECT SCHEDULE: "));
    lcd.print(schedNum);
    lcd.setCursor(0, 2);
    lcd.print(F("Can't find/open schedule"));
    schedOK = false;
    return;
  }

  // Break down the schedule string by semicolons
  // Store the first three lines in the description
  // Break the rest up by commas and store the triplets in the seg* vars
  for (int i = 0; i <= 8; i++) { // 8 is arbitrary
    line = getValue(mySchedule,';',i);
    if (strcmp(line.c_str(), "")) { // ignore empty sets
      if (i == 0) {
        memcpy(schedDesc1, line.c_str(), 21);
      } else if (i == 1) {
        memcpy(schedDesc2, line.c_str(), 21);
      } else if (i == 2) {
        memcpy(schedDesc3, line.c_str(), 21);
      }
      if (i > 2) {
        segRamp[i-3] = atoi(getValue(line,',',0).c_str());
        segTemp[i-3] = atoi(getValue(line,',',1).c_str());
        segHold[i-3] = atoi(getValue(line,',',2).c_str());
      }
      row = i + 1;
      }
  }
  

  // Set some variables
  lastSeg = row - 3;
  schedOK = true;

  // Fix Ramp values so it will show the correct sign (+/-).  This will help to determine when to start hold.
  for (i = 0; i < lastSeg; i++) {
    segRamp[i] = abs(segRamp[i]);
    if (i >= 1) {
      if (segTemp[i] < segTemp[i - 1]) {
        segRamp[i] = -segRamp[i];
      }
    }
  } 
  
  sprintf(tempLine, "%d.txt", schedNum);

  // Display on the screen
  lcd.clear();
  lcd.print(F("SELECT SCHEDULE: "));
  lcd.print(schedNum);
  lcd.setCursor(0, 1);
  lcd.print(schedDesc1);
  lcd.setCursor(0, 2);
  lcd.print(schedDesc2);    
  lcd.setCursor(0, 3);
  lcd.print(schedDesc3);

  // Cut down schedule description so it shows better on other screen when running
  schedDesc1[14 - intLength(schedNum)] = '\0';

}

//******************************************************************************************************************************
//  READTEMPS: Read the temperatures
//******************************************************************************************************************************
void readTemps() {

  // Loop thru all zones
  for (i = 0; i < numZones; i++) {
    if (tempScale == 'C') {
      pidInput[i] = thermo[i].readCelsius();
    }
    if (tempScale == 'F') {
      pidInput[i] = thermo[i].readFarenheit();
    }    
  }

}

//******************************************************************************************************************************
//  SETUPPIDS: INITIALIZE THE PID LOOPS
//******************************************************************************************************************************
void setupPIDs() {

  for (i = 0; i < numZones; i++) {
    pidCont[i].SetSampleTime(pidCycle);
    pidCont[i].SetOutputLimits(0, pidCycle);
    pidCont[i].SetMode(AUTOMATIC);
  }

}

//******************************************************************************************************************************
//  SHUTDOWN: SHUT DOWN SYSTEM
//******************************************************************************************************************************
void shutDown() {

  // Turn off all zones (heating element relays)
  for (i = 0; i < numZones; i++) {
    digitalWrite(heaterPin[i], LOW);
  }
  
  // Disable interrupts / Infinite loop
  cli();
  while (1);
  
}

//******************************************************************************************************************************
//  UPDATELCD: UPDATE THE LCD SCREEN WHEN RUNNING
//******************************************************************************************************************************
void updateLCD() {

  // Clear screen and set cursor to top left
  lcd.clear();

  // Temperatures
  if (screenNum == 1) {
    lcd.print(F("TEMPS "));
    lcd.print((char)223);
    lcd.print(tempScale);
    lcd.print(F(" Rdg / SetPt"));    
     
    for (i = 0; i < numZones; i++) {
      lcd.setCursor(1, i + 1);
      lcd.print(F("Zone "));
      lcd.print(i + 1);
      lcd.setCursor(12 - intLength((int)pidInput[i]), i + 1);
      lcd.print((int)pidInput[i]);
      lcd.print(F(" / "));
      lcd.print((int)pidSetPoint[i]);      
    }
  }

  // Schedule / segment info
  if (screenNum == 2) {
    lcd.print(F("SCH "));
    lcd.print(schedNum);
    lcd.print(F(": "));
    lcd.print(schedDesc1);

    lcd.setCursor(0, 1);
    lcd.print(F("SEG: "));
    lcd.print(segNum);
    lcd.print(F(" / "));
    lcd.print(lastSeg);
 
    if (segPhase == 0) {
      lcd.setCursor(2, 2);
      lcd.print(F("Ramp to "));
      lcd.print(segTemp[segNum - 1]);
      lcd.print(F(" "));
      lcd.print((char)223);
      lcd.print(tempScale);

      lcd.setCursor(2, 3);
      lcd.print(F("at "));
      lcd.print(segRamp[segNum - 1]);
      lcd.print(F(" "));
      lcd.print((char)223);
      lcd.print(F("/hr"));
    }
    else {
      lcd.setCursor(2, 2);
      lcd.print(F("Hold at "));
      lcd.print(segTemp[segNum - 1]);
      lcd.print(F(" "));
      lcd.print((char)223);
      lcd.print(tempScale);
      
      lcd.setCursor(2, 3);
      lcd.print(F("for "));
      lcd.print((millis() - holdStart) / 60000);
      lcd.print(F(" / "));
      lcd.print(segHold[segNum - 1]);
      lcd.print(F(" min"));
    }
  }

  // Tools
  if (screenNum == 3) {
    lcd.print(F("       TOOLS:"));
    lcd.setCursor(2, 1);
    lcd.print(F("Add 5 min"));
    lcd.setCursor(2, 2);
    lcd.print(F("Increase 5 deg"));
    lcd.setCursor(2, 3);
    lcd.print(F("Skip to next seg"));
    lcd.setCursor(0, optionNum);
    lcd.print(F(">"));
    lcd.setCursor(19, optionNum);
    lcd.print(F("<"));
  }
  
  // Schedule completed
  if (screenNum == 4) {
    readTemps();
    
    lcd.print(F(" SCHEDULE COMPLETE"));
    lcd.setCursor(2, 1);
    lcd.print(F("Wait until cool"));
    lcd.setCursor(2, 2);
    lcd.print(F("before you open"));
    lcd.setCursor(2, 3);
    lcd.print(F("Zone 1: "));
    lcd.print((int)pidInput[0]);
    lcd.print(F(" "));
    lcd.print((char)223);
    lcd.print(tempScale);
  }

}

//******************************************************************************************************************************
//  UPDATEPIDS: UPDATE THE PID LOOPS
//******************************************************************************************************************************
void updatePIDs() {

  // Get the last target temperature
  if (segNum == 1) {  // Set to room temperature for first segment
    if (tempScale == 'C') {
      lastTemp = 24;
    }
    if (tempScale == 'F') {
      lastTemp = 75;
    }
  }
  else {
    lastTemp = segTemp[segNum - 2];
  }

  // Calculate the new setpoint value.  Don't set above / below target temp  
  if (segPhase == 0) {
    rampHours = (millis() - rampStart) / 3600000.0;
    calcSetPoint = lastTemp + (segRamp[segNum - 1] * rampHours);  // Ramp
    if (segRamp[segNum - 1] >= 0 && calcSetPoint >= segTemp[segNum - 1]) {
      calcSetPoint = segTemp[segNum - 1];
    }
    if (segRamp[segNum - 1] < 0 && calcSetPoint <= segTemp[segNum - 1]) {
      calcSetPoint = segTemp[segNum - 1];
    }
  }
  else {
    calcSetPoint = segTemp[segNum - 1];  // Hold
  }

  // Read the temperatures
  readTemps();
  
  // Loop thru all PID controllers
  for (i = 0; i < numZones; i++) {

    // Set the target temp.  Add any offset.
    pidSetPoint[i] = calcSetPoint + tempOffset[i];

    // Update the PID based on new variables
    pidCont[i].Compute();

  }

}

//******************************************************************************************************************************
//  UPDATESEG: UPDATE THE PHASE AND SEGMENT
//******************************************************************************************************************************
void updateSeg() {

  // Start the hold phase
  if ((segPhase == 0 && segRamp[segNum - 1] < 0 && pidInput[0] <= (segTemp[segNum - 1] + tempRange)) || 
      (segPhase == 0 && segRamp[segNum - 1] >= 0 && pidInput[0] >= (segTemp[segNum - 1] - tempRange))) {
    segPhase = 1;
    holdStart = millis();
  }

  // Go to the next segment
  if (segPhase == 1 && millis() - holdStart >= segHold[segNum - 1] * 60000) {
    segNum = segNum + 1;
    segPhase = 0;
    rampStart = millis();
  }

  // Check if complete / turn off all zones
  if (segNum - 1 > lastSeg) {
    for (i = 0; i < numZones; i++) {
      digitalWrite(heaterPin[i], LOW);
    }
    screenNum = 4;
  }

}
