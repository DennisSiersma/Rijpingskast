/*
    A System Controller for climatronics
    Created by Dennis Siersma
    Inspired in part on some great work by Chiu-Yuan Fang's System Controller for climatronics,
    Chris Hennes' Curing Chamber and many examples online by others.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    If the RTC battery is changed, you may have to reset the time using the SetTime example sketch in the DS1307RTC folder.


*/

/****************************************************************************************
 * Include Libraries
 ****************************************************************************************/

#include <SPI.h>                    // For ethernet and sd card
#include <Ethernet2.h>
#include <EthernetUdp2.h>           // For UDP NTS sync
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>
#include <dht.h>
#include <DS1307RTC.h>
#include <Time.h>
#include <EEPROMex.h>
//#include "BMP280.h"
//#include <elapsedMillis.h>  
#include <SimpleTimer.h> // Possible alternative for elapsedMillis?
//#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space
#include <BlynkSimpleEthernet2.h>

/****************************************************************************************
 * Definitions and pin
 ****************************************************************************************/

#define P0 1013.25
#define dht_intPin 22
#define dht_extPin 23
//#define bmp280_temperaturePin 42
#define relay_fridgePin 37
#define relay_humidifierPin 36
#define relay_fanPin 35
#define relay_heaterPin 34
#define fridge_Led 24
#define Vfridge_Led V18
#define humidifier_Led 25
#define Vhumidifier_Led V19
#define heater_Led 26
#define Vheater_Led V20
#define fan_Led 27
#define Vfan_Led V21

/****************************************************************************************
 * Initialize sensors
 ****************************************************************************************/

dht DHT_int; //// Initialize DHT sensor for normal 16mhz Arduino
dht DHT_ext;
//BMP280 bmp; // Initialize BMP280 sensor

/****************************************************************************************
 * Set parameters
 ****************************************************************************************/

float temp_target;
float hum_target;
int temp_hysteresis;
int hum_hysteresis;
int keypadDebounce;         // Keypad debounce. 
int timeZone;               // timezone
unsigned long TestTime1;    // Timeinterval for a certain task to run
unsigned long TestTime2;
unsigned long TestTime3;
unsigned long TestTime4;
unsigned long interruptTimeout; // Disable interrupt for timeout period while siphon dumps. 600000mS = 10 mins
unsigned long relay_fanDuration; // Duration that fan is on after triggered
unsigned long relay_fanTimeout; // Duration that the fan is inactive after a trigger. 60000mS = 1 mins


/****************************************************************************************
 * Declare variables
 ****************************************************************************************/

char key; // Keypad input character
uint8_t relay_fridgePinState = 0;
uint8_t relay_humidifierPinState = 0;
uint8_t relay_fanPinState = 0;
uint8_t relay_heaterPinState = 0;
uint8_t fridge_LedState = 0;
uint8_t humidifier_LedState = 0;
uint8_t fan_LedState = 0;
uint8_t heater_LedState = 0;
int keyInput = 0; // Keypad char string converted to int
float keyInputFloat = 00.00; // Keypad char string converted to float
float keyInputFloatTemp = 00.00; // Keypad char string converted to float
int keyInputInt = 00;
int keyInputIntTemp = 00;
unsigned long keyInputLong = 0; // Keypad char string converted to long
int first_run = 0;
int debugFlag = 0;
int value = 0; // For reading pin states
float hum_int;  //Stores internal humidity value
float temp_int; //Stores internal temperature value
float hum_ext;
float temp_ext;
//float baro_ext;
int currentDay = 0;
//unsigned long rtcEpoch;
unsigned long wait = 0; // used as alternative for delay function. Blynk doesn't like delays'
uint32_t _lastCheckTime;
unsigned long _numWraparounds; // How many times has the clock wrapped
unsigned long _runtimeInSeconds; // 4 bytes, so a max of 4,294,967,295 (something like 136 years)
uint32_t MAX_SWITCH_RATE = 180; // Seconds (3 minutes)

/****************************************************************************************
 * Controlling states
 ****************************************************************************************/

enum State {
  NOT_CONTROLLING,
  CONTROLLING_UP,
  CONTROLLING_DOWN
};

struct ControlledValue {
  float current;
  float target;
  float tolerance;
  int pinToIncrease;
  int pinToDecrease;
  uint32_t lastStateChangeTime; // In **seconds**, not milliseconds!
  State state;
};

ControlledValue temperature = {0, temp_target, temp_hysteresis, relay_heaterPin, relay_fridgePin, 0, NOT_CONTROLLING };
ControlledValue humidity = {0, hum_target, hum_hysteresis, relay_humidifierPin, relay_fanPin, 0, NOT_CONTROLLING };


/****************************************************************************************
 * Timing functions
 ****************************************************************************************/

//elapsedMillis sinceTest1;
//elapsedMillis sinceTest2;
//elapsedMillis sinceTest3;
//elapsedMillis sinceTest4;
SimpleTimer timer;



/****************************************************************************************
 * Setup LCD screens (physical 20x4 and virtual 16x2)
 ****************************************************************************************/

LiquidCrystal_I2C lcd(0x27, 20, 4); // set the LCD address to 0x27 for a 20 chars and 4 line display
WidgetLCD Vlcd(V7);

/***********************************************************************************************
 * keymap defines the key pressed according to the row and columns just as appears on the keypad
 ***********************************************************************************************/

const byte numRows = 4; //number of rows on the keypad
const byte numCols = 4; //number of columns on the keypad

char keymap[numRows][numCols] =
{
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

//Code that shows the the keypad connections to the arduino terminals
byte rowPins[numRows] = {38, 39, 41, 40}; //Rows 0 to 3
byte colPins[numCols] = {42, 43, 44, 45}; //Columns 0 to 3


//initializes an instance of the Keypad class
Keypad myKeypad = Keypad(makeKeymap(keymap), rowPins, colPins, numRows, numCols);

/****************************************************************************************
 * Blynk setup
 ****************************************************************************************/

char auth[] = "43489365ffa145be9bace5a3a67d25d2";
WidgetTerminal terminal(V10);
//IPAddress server_ip (10, 0, 0, 10); // If you have a private blynk server

/****************************************************************************************
 * Ethernet Setup
 ****************************************************************************************/

// ***** Set this MAC address for a unique value on your network.***********
byte mac[] = { 0x90, 0xA2, 0xDA, 0x10, 0x50, 0xA8 };
IPAddress ip ( 10,   52,   7,  18);
IPAddress dns_ip     (  10,   52,   7,  100);
IPAddress gateway_ip ( 10,   52,   7,  100);
IPAddress subnet_mask(255, 255, 255,   0);


/****************************************************************************************
 * Setup routine
 ****************************************************************************************/

 void setup()
{
  
 //Read parameters from EEPROM
  temp_hysteresis = EEPROM.readInt(0);
  hum_hysteresis = EEPROM.readInt(2);
  keypadDebounce = EEPROM.readInt(4);    
  timeZone = EEPROM.readInt(6);
  TestTime1 = EEPROM.readLong(8);       
  TestTime2 = EEPROM.readLong(12);       
  TestTime3 = EEPROM.readLong(16); 
  TestTime4 = EEPROM.readLong(20);    
  temp_target = EEPROM.readLong(24);
  hum_target = EEPROM.readLong(28);
  interruptTimeout = EEPROM.readLong(32);     
  relay_fanDuration = EEPROM.readLong(36);    
  relay_fanTimeout = EEPROM.readLong(40);      
  first_run = EEPROM.readInt(44); 
  debugFlag = EEPROM.readInt(46);
  
  // Timekeeping
  byte timerClock  = timer.setInterval(TestTime1, digitalClockDisplay);
  byte timerLeds  = timer.setInterval(TestTime1, setLeds);
  byte timerDHT  = timer.setInterval(TestTime2, Dhtvalue);
  byte timerControlTemp  = timer.setInterval(TestTime2, usTemp);
  byte timerControlHum  = timer.setInterval(TestTime2, usHum);
  //byte timerCheckPres  = timer.setInterval(TestTime3, Pressure);
  byte timerDebug  = timer.setInterval(TestTime3, Debug);
  byte timerBlynkUpdate  = timer.setInterval(TestTime3, BlynkUpdate);
  
  _numWraparounds = 0;
  _runtimeInSeconds = 0;
  _lastCheckTime = 0;
  
  lcd.init();                       // initialize the lcd
  Serial.begin(9600);               // Start serial for debugging

  //Ethernet.begin(mac, ip, dnServer, gateway); 
  Serial.println("Setting up Blynk...");
  Blynk.begin(auth, "blynk-cloud.com", 8442, ip, dns_ip, gateway_ip, subnet_mask, mac); 
  while (Blynk.connect() == false) 
    {
        // Wait until connected
    }
  terminal.println(F("Blynk v" BLYNK_VERSION ": Device started")); 
  
  //Set pinModes
  //pinMode(relay_fridgePin, OUTPUT);
  //pinMode(relay_humidifierPin, OUTPUT);
  //pinMode(relay_fanPin, OUTPUT);
  //pinMode(relay_heaterPin, OUTPUT);
  
  pinMode(temperature.pinToIncrease, OUTPUT);
  pinMode(temperature.pinToDecrease, OUTPUT);
  pinMode(humidity.pinToIncrease, OUTPUT);
  pinMode(humidity.pinToDecrease, OUTPUT);
  digitalWrite (temperature.pinToIncrease, LOW);
  digitalWrite (temperature.pinToDecrease, LOW);
  digitalWrite (humidity.pinToIncrease, LOW);
  digitalWrite (humidity.pinToDecrease, LOW);
  pinMode(fan_Led, OUTPUT);
  pinMode(heater_Led, OUTPUT);
  pinMode(humidifier_Led, OUTPUT);
  pinMode(fridge_Led, OUTPUT);
  digitalWrite(fan_Led, LOW);
  Blynk.virtualWrite(Vfan_Led, LOW);
  digitalWrite(heater_Led, LOW);
  Blynk.virtualWrite(Vheater_Led, LOW);
  digitalWrite(humidifier_Led, LOW);
  Blynk.virtualWrite(Vhumidifier_Led, LOW);
  digitalWrite(fridge_Led, LOW);
  Blynk.virtualWrite(Vfridge_Led, LOW);
  
  lcd.backlight();
  //lcd.setCursor(0,1);
  //lcd.print("Tekst voor regel 2");
  lcd.setCursor(0, 2);
  lcd.print("");
  lcd.setCursor(3, 3);
  lcd.print("Dennis Siersma");

//  if (!bmp.begin())
//  {
//    Serial.println("BMP init failed!");
//    terminal.println("BMP init failed!");
//    while (1);
//  }
//  else Serial.println("BMP init success!");
//       terminal.println("BMP init success!"); 
//
//  bmp.setOversampling(4);
//
  setSyncProvider(RTC.get);   // the function to get the time from the RTC

  if (timeStatus() != timeSet)
  {
    Serial.println("Unable to sync with the RTC");
    terminal.println("Unable to sync with the RTC");
    //terminal.flush;
  }
  else
  {
    Serial.println("RTC has set the system time");
    terminal.println("RTC has set the system time");
    //terminal.flush;
  }

  lcd.setCursor(0, 3);
  lcd.print("                    ");
  
  checkEeprom();
  homeScreen();
}

/****************************************************************************************
 * Main loop
 ****************************************************************************************/
void loop()
{
    uint32_t currentTime = millis();
    if (currentTime < _lastCheckTime) 
    {
        // Happens once every 50 days or so
            _numWraparounds++;
    } 
        
    _lastCheckTime = currentTime;
    _runtimeInSeconds = _numWraparounds * (UINT32_MAX/1000) + (currentTime/1000);
    const uint32_t pollRate = 15000; // Defaults to every fifteen seconds in the normal case
    
    
  //checkEeprom();
 
  
  //if (sinceTest1 >= TestTime1)
  //{
  //  sinceTest1 = sinceTest1 - TestTime1;
  //  digitalClockDisplay();
  //  setLeds();
  //  
  //}
  //if (sinceTest2 >= TestTime2)
  //{
  //  sinceTest2 = sinceTest2 - TestTime2;
  //  Dhtvalue();
  //  updateState (temperature);
  //  updateState (humidity);
  //}
  //if (sinceTest3 >= TestTime3)
  //{
  //  sinceTest3 = sinceTest3 - TestTime3;
  //  Pressure();
  //  Debug();
  //  BlynkUpdate();
  //  //terminal.println("Debug = ");
  //  //terminal.print (debugFlag);
  //
  //}
  //if (sinceTest4 >= TestTime4)
  //{
  //  sinceTest4 = sinceTest4 - TestTime4;
  //} 


  // Keypad Input to enter menu
  key = myKeypad.getKey();
  if (key)
  {
    splashScreen();
  }
  // Blink call
  Blynk.run();
  timer.run();
}

/****************************************************************************************
 * Menu and screen system
 ****************************************************************************************/
void homeScreen () {
  lcd.clear();
  digitalClockDisplay();
  Dhtvalue();
  //Pressure();
}

void splashScreen() {
  lcd.clear();
  lcd.setCursor(7, 0);
  lcd.print(F("Siersma"));
  lcd.setCursor(5, 1);
  lcd.print(F("Klimaatkast"));
  lcd.setCursor(7, 2);
  lcd.print(F("Systeem"));
  lcd.setCursor(6, 3);
  lcd.print(F("Controller"));
  delay(3000);
  mainMenu();
}

void mainMenu() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Druk A voor Targets"));
  lcd.setCursor(0, 1);
  lcd.print(F("Druk B voor Extra"));
  lcd.setCursor(0, 2);
  lcd.print(F("Druk C voor Settings"));
  lcd.setCursor(0, 3);
  lcd.print(F("Druk D voor Exit"));
  key = myKeypad.waitForKey();
  if (key == 'A') {
    targetScreen();
  }
  else if (key == 'B') {
    extraScreen();
  }
  else if (key == 'C') {
    settingsScreen();
  }
  else {
    lcd.clear();
    homeScreen();
    loop();
  }
}

void targetScreen() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("1:Doel temperatuur"));
  lcd.setCursor(0, 1);
  lcd.print(F("2:Doel LV"));
  //lcd.setCursor(0, 2);
  //lcd.print(F("3:"));
  lcd.setCursor(0, 3);
  lcd.print(F("D:Hoofdmenu"));
  key = myKeypad.waitForKey();
  switch (key) {

    case 'D':
      mainMenu();
      break;

    case '1':
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Doeltemperatuur"));
      lcd.setCursor(0, 1);
      lcd.print(F("Huidig = "));
      lcd.print(temp_target);
      lcd.setCursor(0, 2);
      lcd.print(F("Toets waarde = "));
      lcd.setCursor(0, 3);
      lcd.print(F("A:Set D:Cancel"));
      lcd.setCursor(15, 2);
      lcd.cursor();
      lcd.blink();
      do {
        key = myKeypad.waitForKey();
        lcd.noBlink();
        lcd.noCursor();
        if (key >= '0' && key <= '9') {
          keyInputFloat = ((keyInputFloat * 10) + (key - '0'));
          keyInputFloatTemp = keyInputFloat / 100;
          lcd.setCursor(15, 2);
          lcd.print(keyInputFloatTemp);
        }
      } while (key >= '0' && key <= '9');
      if (key == 'A') {
        temp_target = keyInputFloatTemp;
        temperature.target = temp_target;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Huidig doel = "));
        lcd.print(temp_target);
        lcd.setCursor(0, 1);
        lcd.print(F("Data vastgelegd"));
        keyInputFloat = 0;
        delay(3000);
        targetScreen();
      } else {
        keyInputFloat = 0;
        targetScreen();
      }
      break;

    case '2':
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("LV doelpercentage"));
      lcd.setCursor(0, 1);
      lcd.print(F("Huidig = "));
      lcd.print(hum_target);
      lcd.setCursor(0, 2);
      lcd.print(F("Toets Waarde = "));
      lcd.setCursor(0, 3);
      lcd.print(F("A:Set D:Cancel"));
      lcd.setCursor(13, 2);
      lcd.cursor();
      lcd.blink();
      do {
        key = myKeypad.waitForKey();
        lcd.noBlink();
        lcd.noCursor();
        if (key >= '0' && key <= '9') {
          keyInputFloat = ((keyInputFloat * 10) + (key - '0'));
          keyInputFloatTemp = keyInputFloat / 100;
          lcd.setCursor(13, 2);
          lcd.print(keyInputFloatTemp);
        }
      } while (key >= '0' && key <= '9');
      if (key == 'A') {
        hum_target = keyInputFloatTemp;
        humidity.target = hum_target;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("LV doel = "));
        lcd.print(hum_target);
        lcd.setCursor(0, 1);
        lcd.print(F("Data vastgelegd"));
        keyInputFloat = 0;
        delay(3000);
        targetScreen();
      } else {
        keyInputFloat = 0;
        targetScreen();
      }
      break;

    case '3':
      
      break;
  }
}

void extraScreen()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("A: Toggle KK"));
  lcd.setCursor(0, 1);
  lcd.print(F("B: Toggle Hum"));
  lcd.setCursor(0, 2);
  lcd.print(F("C: Toggle Fan"));
  lcd.setCursor(0, 3);
  lcd.print(F("D: Toggle Heater"));

  key = myKeypad.waitForKey();
  if (key == 'A')
  {
    value = digitalRead (relay_fridgePin);
    if (value == HIGH)
    {
      digitalWrite (relay_fridgePin, LOW);
      digitalWrite(fridge_Led, LOW);
      Blynk.virtualWrite(Vfridge_Led, 0);
      
    }
    else
    {
      digitalWrite (relay_fridgePin, HIGH);
      digitalWrite(fridge_Led, HIGH);
      Blynk.virtualWrite(Vfridge_Led, 1000);
    }
    extraScreen();
  }
  else if (key == 'B')
  {
    value = digitalRead (relay_humidifierPin);
    if (value == HIGH)
    {
      digitalWrite (relay_humidifierPin, LOW);
    }
    else
    {
      digitalWrite (relay_humidifierPin, HIGH);
    }
    extraScreen();
  }
  else if (key == 'C')
  {
    value = digitalRead (relay_fanPin);
    if (value == HIGH)
    {
      digitalWrite (relay_fanPin, LOW);
    }
    else
    {
      digitalWrite (relay_fanPin, HIGH);
    }
    extraScreen();
  }
  else if (key == 'D')
  {
    value = digitalRead (relay_heaterPin);
    if (value == HIGH)
    {
      digitalWrite (relay_heaterPin, LOW);
    }
    else
    {
      digitalWrite (relay_heaterPin, HIGH);
    }
    extraScreen();
  }
  else {
    lcd.clear();
    homeScreen();
    loop();
  }
}

void settingsScreen() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("1:T hysteresis"));
  lcd.setCursor(0, 1);
  lcd.print(F("2:LV hysteresis"));
  lcd.setCursor(0, 2);
  lcd.print(F("3:Debug"));
  lcd.setCursor(0, 3);
  lcd.print(F("D:Hoofdmenu"));
  key = myKeypad.waitForKey();
  switch (key) {

    case 'D':
      mainMenu();
      break;

    case '1':
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Temphysteresis"));
      lcd.setCursor(0, 1);
      lcd.print(F("Huidig = "));
      lcd.print(temp_hysteresis);
      lcd.setCursor(0, 2);
      lcd.print(F("Toets waarde = "));
      lcd.setCursor(0, 3);
      lcd.print(F("A:Set D:Cancel"));
      lcd.setCursor(15, 2);
      lcd.cursor();
      lcd.blink();
      do {
        key = myKeypad.waitForKey();
        lcd.noBlink();
        lcd.noCursor();
        if (key >= '0' && key <= '9') {
          keyInputInt = ((keyInputInt * 10) + (key - '0'));
          keyInputIntTemp = keyInputInt / 100;
          lcd.setCursor(15, 2);
          lcd.print(keyInputIntTemp);
        }
      } while (key >= '0' && key <= '9');
      if (key == 'A') {
        temp_hysteresis = keyInputIntTemp;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("Huidige Th = "));
        lcd.print(temp_hysteresis);
        lcd.setCursor(0, 1);
        lcd.print(F("Data vastgelegd"));
        keyInputInt = 0;
        delay(3000);
        settingsScreen();
      } else {
        keyInputInt = 0;
        settingsScreen();
      }
      break;

    case '2':
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("LV hysteresis"));
      lcd.setCursor(0, 1);
      lcd.print(F("Huidig Hh = "));
      lcd.print(hum_hysteresis);
      lcd.setCursor(0, 2);
      lcd.print(F("Toets Waarde = "));
      lcd.setCursor(0, 3);
      lcd.print(F("A:Set D:Cancel"));
      lcd.setCursor(13, 2);
      lcd.cursor();
      lcd.blink();
      do {
        key = myKeypad.waitForKey();
        lcd.noBlink();
        lcd.noCursor();
        if (key >= '0' && key <= '9') {
          keyInputFloat = ((keyInputFloat * 10) + (key - '0'));
          keyInputFloatTemp = keyInputFloat / 100;
          lcd.setCursor(13, 2);
          lcd.print(keyInputFloatTemp);
        }
      } while (key >= '0' && key <= '9');
      if (key == 'A') {
        hum_target = keyInputFloatTemp;
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("LV hysteresis = "));
        lcd.print(hum_hysteresis);
        lcd.setCursor(0, 1);
        lcd.print(F("Data vastgelegd"));
        keyInputFloat = 0;
        delay(3000);
        settingsScreen();
      } else {
        keyInputFloat = 0;
        settingsScreen();
      }
      break;

    case '3':
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(F("Toggle debug"));
      lcd.setCursor(0, 1);
      lcd.print(F("Nu: "));
      lcd.print(debugFlag);
      //lcd.setCursor(0, 2);
      //lcd.print(F(""));
      lcd.setCursor(0, 3);
      lcd.print(F("A:Toggle D:Cancel"));
      
      do 
      {
        key = myKeypad.waitForKey();
        lcd.noBlink();
        lcd.noCursor();
        
      } while (key >= '0' && key <= '9');
      if (key == 'A') {
        if (debugFlag == 0)
        {
            debugFlag = 1;
        }
        else {
            debugFlag = 0;
        }
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(F("debug = "));
        lcd.print(debugFlag);
        lcd.setCursor(0, 1);
        lcd.print(F("Data vastgelegd"));
        delay(3000);
        
            lcd.clear();
            settingsScreen();
            
      } else {
        settingsScreen();
      }
      break;
  }
}

/****************************************************************************************
 * Debug routine (when debugflag is set)
 ****************************************************************************************/
void Debug() 
{
  if (debugFlag >= 1)
  {  
      // debug
      Serial.println("");
      Serial.print(F("Ram:"));
      Serial.print(freeRam());
      Serial.print(F(","));
      Serial.print(F("Millis:"));
      Serial.print(millis());
      Serial.println(F(","));
      Serial.println(("Parameters:"));
      Serial.print("temp_hysteresis: ");
      Serial.println(temp_hysteresis);
      Serial.print("hum_hysteresis: "); 
      Serial.println(hum_hysteresis); 
      Serial.print("keypadDebounce: "); 
      Serial.println(keypadDebounce); 
      Serial.print("timeZone: ");
      Serial.println(timeZone);
      Serial.print("TestTime1: "); 
      Serial.println(TestTime1); 
      Serial.print("TestTime2: ");
      Serial.println(TestTime2); 
      Serial.print("TestTime3: "); 
      Serial.println(TestTime3); 
      Serial.print("TestTime4: "); 
      Serial.println(TestTime4);
      Serial.print("temp_target: "); 
      Serial.println(temp_target); 
      Serial.print("hum_target: "); 
      Serial.println(hum_target); 
      Serial.print("interruptTimeout: "); 
      Serial.println(interruptTimeout);
      Serial.print("relay_fanDuration: "); 
      Serial.println(relay_fanDuration); 
      Serial.print("relay_fanTimeout: ");
      Serial.println(relay_fanTimeout);
      Serial.print("fPS "); 
      Serial.println(relay_fridgePinState);
      Serial.print("hPS "); 
      Serial.println(relay_humidifierPinState);
      Serial.print("fanPS "); 
      Serial.println(relay_fanPinState);
      Serial.print("htPS "); 
      Serial.println(relay_heaterPinState);
      Serial.print("fLS "); 
      Serial.println(fridge_LedState);
      Serial.print("hLS "); 
      Serial.println(humidifier_LedState);
      Serial.print("fanLS "); 
      Serial.println(fan_LedState);
      Serial.print("htLS "); 
      Serial.println(heater_LedState);
      Serial.print("first_run: "); 
      Serial.println(first_run);
      
      Serial.println(debugFlag);
      terminal.println("");
      terminal.print(F("Ram:"));
      terminal.print(freeRam());
      terminal.print(F(","));
      terminal.print(F("Millis:"));
      terminal.print(millis());
      terminal.println(F(","));
      terminal.println(("Parameters:"));
      terminal.print("temp_hysteresis: ");
      terminal.println(temp_hysteresis);
      terminal.print("hum_hysteresis: "); 
      terminal.println(hum_hysteresis); 
      terminal.print("keypadDebounce: "); 
      terminal.println(keypadDebounce); 
      terminal.print("timeZone: ");
      terminal.println(timeZone);
      terminal.print("TestTime1: "); 
      terminal.println(TestTime1); 
      terminal.print("TestTime2: ");
      terminal.println(TestTime2); 
      terminal.print("TestTime3: "); 
      terminal.println(TestTime3); 
      terminal.print("TestTime4: "); 
      terminal.println(TestTime4);
      terminal.print("temp_target: "); 
      terminal.println(temp_target); 
      terminal.print("hum_target: "); 
      terminal.println(hum_target); 
      terminal.print("interruptTimeout: "); 
      terminal.println(interruptTimeout);
      terminal.print("relay_fanDuration: "); 
      terminal.println(relay_fanDuration); 
      terminal.print("relay_fanTimeout: ");
      terminal.println(relay_fanTimeout);
      terminal.print("first_run: "); 
      terminal.println(first_run);
      terminal.print("temperature.state "); 
      terminal.println(temperature.state);
      terminal.print("fPS "); 
      terminal.println(relay_fridgePinState);
      terminal.print("hPS "); 
      terminal.println(relay_humidifierPinState);
      terminal.print("fanPS "); 
      terminal.println(relay_fanPinState);
      terminal.print("htPS "); 
      terminal.println(relay_heaterPinState);
      terminal.print("fLS "); 
      terminal.println(fridge_LedState);
      terminal.print("hLS "); 
      terminal.println(humidifier_LedState);
      terminal.print("fanLS "); 
      terminal.println(fan_LedState);
      terminal.print("htLS "); 
      terminal.println(heater_LedState);
      terminal.print("debugFlag: ");
      terminal.println(debugFlag);
      terminal.println("");
      terminal.flush();
      }
      
}

/****************************************************************************************
 * DHT sensor routine
 ****************************************************************************************/
void Dhtvalue() {
  int readDHTi = DHT_int.read22(dht_intPin); // read temps and humidity from inside DHT-22

  switch (readDHTi) {
    case DHTLIB_OK:
      temp_int = DHT_int.temperature;
      hum_int = DHT_int.humidity;
      temperature.current = temp_int;
      humidity.current = hum_int;
      lcd.setCursor(0, 2);
      lcd.print("Tint:      C Lv:   %");
      lcd.setCursor(6, 2);
      lcd.print(temp_int, 1); lcd.print((char)223);
      lcd.setCursor(16, 2);
      lcd.print(hum_int, 0);
      break;
    case DHTLIB_ERROR_CHECKSUM:
      lcd.setCursor(0, 2);
      lcd.print("Error Checksum      ");
      break;
    case DHTLIB_ERROR_TIMEOUT:
      lcd.setCursor(0, 2);
      lcd.print("Wacht op antwoord...");
      break;
    default:
      break;
  }
  
  int readDHTe = DHT_ext.read11(dht_extPin); // read temps and humidity from inside DHT-22

  switch (readDHTe) {
    case DHTLIB_OK:
      temp_ext = DHT_ext.temperature; 
      hum_ext = DHT_ext.humidity; 
      lcd.setCursor(0, 3);
      lcd.print("Text:      C Lv:   %");
      lcd.setCursor(6, 3);
      lcd.print(temp_ext, 1); lcd.print((char)223);
      lcd.setCursor(16, 3);
      lcd.print(hum_ext, 0);
      break;
    case DHTLIB_ERROR_CHECKSUM:
      lcd.setCursor(0, 3);
      lcd.print("Error Checksum      ");
      break;
    case DHTLIB_ERROR_TIMEOUT:
      lcd.setCursor(0, 3);
      lcd.print("Wacht op antwoord...");
      break;
    default:
      break;
  }
}

/****************************************************************************************
 * Clock on lcd
 ****************************************************************************************/
void digitalClockDisplay()
{
  // digital clock display of the time
  String cTime = String(hour()) + ":" + minute() + ":" + second();
  lcd.setCursor(0, 0);                                            // Set LCD cursor position (column, row)
  lcd.print(hour());
  Vlcd.print(0,0, cTime);
  
  printDigits(minute());
  
  printDigits(second());
  lcd.print(" ");
  lcd.setCursor(9, 0);
  lcd.print("| ");
  lcd.setCursor(11, 0);                                           // Set LCD cursor position (column,row)
  lcd.print(day());
  lcd.print(".");
  lcd.print(month());
  lcd.print(".");
  lcd.print(year());
}

void printDigits(int digits) // Makes readout nice and tidy with leading zero and semicolon
{
  lcd.print(":");
  if (digits < 10)
    lcd.print('0');
    lcd.print(digits);
}

/****************************************************************************************
 * BMP280 barometer and external temp
 ****************************************************************************************/
//void Pressure()
//{
//  double T, P;
//  char result = bmp.startMeasurment();
//
//  Serial.print(result);
//
//  if (result != 0) {
//    delay(result);
//    result = bmp.getTemperatureAndPressure(T, P);
//    lcd.setCursor(0, 3);
//    if (result != 0)
//    {
//      double A = bmp.altitude(P, P0);
//      lcd.setCursor(0, 3);
//      //lcd.print("Ti: ");lcd.print(T,1); //lcd.print(" C  ");
//      lcd.print("Ldruk: "); lcd.print(P, 2); lcd.print(" hPa");
//      //lcd.print("A = \t");lcd.print(A,2); lcd.print(" m");
//    }
//    else {
//      lcd.setCursor(0, 3);
//      lcd.println("Error.");
//    }
//  }
//  else {
//    lcd.setCursor(0, 3);
//    lcd.println("Error.");
//  }
//  baro_ext = P;
//  temp_ext = T;
//}

/****************************************************************************************
 * Available ram routine
 ****************************************************************************************/
int freeRam() 
{
  extern int __heap_start, *__brkval;
  int v;
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

/****************************************************************************************
 * Store and update variables in EEPROM
 ****************************************************************************************/
void checkEeprom()
{
  if (first_run != 1234) 
  {
    Serial.println("Treating as first run, setting EEPROM...");
    terminal.println("Treating as first run, setting EEPROM...");
    temp_hysteresis = 2;
    hum_hysteresis = 5;
    keypadDebounce = 10;    // Keypad debounce. Default is 10mS
    timeZone = 1;
    TestTime1 = 1000;       // Timeinterval a certain task should run - 1 sec
    TestTime2 = 5000;       // 5 sec
    TestTime3 = 60000;      // 10 sec
    TestTime4 = 3600000;    // one hour
    temp_target = 18;
    hum_target = 70;
    interruptTimeout = 600000;     
    relay_fanDuration = 100000;    // Duration that fan is on after triggered
    relay_fanTimeout = 3600000;     // 
    first_run = 1234;                   // Flag that reset has happened
    debugFlag = 0;
    EEPROM.updateInt(0,temp_hysteresis);
    EEPROM.updateInt(2,hum_hysteresis);
    EEPROM.updateInt(4,keypadDebounce);
    EEPROM.updateInt(6,timeZone);
    EEPROM.updateLong(8,TestTime1);
    EEPROM.updateLong(12,TestTime2);
    EEPROM.updateLong(16,TestTime3);
    EEPROM.updateLong(20,TestTime4);
    EEPROM.updateLong(24,temp_target);
    EEPROM.updateLong(28,hum_target);
    EEPROM.updateLong(32,interruptTimeout);
    EEPROM.updateLong(36,relay_fanDuration);
    EEPROM.updateLong(40,relay_fanTimeout);
    EEPROM.updateInt(44,first_run);
    EEPROM.updateInt(46,debugFlag);
  }
} 

void updateEeprom()
{
    Serial.println("Updating settings in EEPROM...");
    terminal.println("Updating settings in EEPROM...");
    EEPROM.updateInt(0,temp_hysteresis);
    EEPROM.updateInt(2,hum_hysteresis);
    EEPROM.updateInt(4,keypadDebounce);
    EEPROM.updateInt(6,timeZone);
    EEPROM.updateLong(8,TestTime1);
    EEPROM.updateLong(12,TestTime2);
    EEPROM.updateLong(16,TestTime3);
    EEPROM.updateLong(20,TestTime4);
    EEPROM.updateLong(24,temp_target);
    EEPROM.updateLong(28,hum_target);
    EEPROM.updateLong(32,interruptTimeout);
    EEPROM.updateLong(36,relay_fanDuration);
    EEPROM.updateLong(40,relay_fanTimeout);
    EEPROM.updateInt(44,first_run);
    EEPROM.updateInt(46,debugFlag);
}

/****************************************************************************************
 * Update blynk widgets and virtual pins
 ****************************************************************************************/
void BlynkUpdate()
{
  //int tempHumid = (int)hum_int;
  //int tempTi  = (int)temp_int;
  //int tempTe  = (int)temp_int;
  
  //String humid  = (String)tempHumid;
  //String temp_i   = (String)tempTi;
  //String temp_e   = (String)tempTe;
  
  //humid += "%";
  //temp_i += "°C";
  //temp_e += "°C";
  
  Blynk.virtualWrite(V1, temp_int);
  Blynk.virtualWrite(V2, hum_int);
  Blynk.virtualWrite(V3, hum_ext);
  Blynk.virtualWrite(V4, temp_ext);
  //Blynk.virtualWrite(V5, temp_e);
}

/****************************************************************************************
 * Blink terminal
 ****************************************************************************************/
BLYNK_WRITE(V10) //Virtual pin for terminal is set as v10 in app
{

  // Menu interaction
  if (String("debug") == param.asStr()) 
  {
    if (debugFlag == 0)
    {
        terminal.println("Debug is on") ;
        debugFlag = 1;
        updateEeprom();
    }
    else {
        terminal.println("Debug is off") ;
        debugFlag = 0;
        updateEeprom();
    }
  }  
  else if (String("clear") == param.asStr())  
  {
    first_run = 0 ;  
  } 
  else if (String("vars") == param.asStr())  
  {
    Debug();
    //terminal.println(now);  
  } 
  else if (String("help") == param.asStr())  
  {
    terminal.println("Current keywords:") ;
    terminal.println("debug, clear, help, vars, ledon, ledoff") ;
    terminal.println("fridgeon, humon, heaton, fanon and -off") ;
    terminal.println("update") ;
    terminal.flush();  
  } 
  else if (String("ledon") == param.asStr())  
  {
    terminal.println("Turning led's on '") ;
    terminal.flush(); 
    
    digitalWrite(fridge_Led, HIGH);
    digitalWrite(humidifier_Led, HIGH);
    digitalWrite(fan_Led, HIGH);
    digitalWrite(heater_Led, HIGH);    
  } 
  else if (String("ledoff") == param.asStr())  
  {
    terminal.println("Turning led's off '") ;
    terminal.flush(); 
    
    digitalWrite(fridge_Led, LOW);
    digitalWrite(humidifier_Led, LOW);
    digitalWrite(fan_Led, LOW);
    digitalWrite(heater_Led, LOW);    
  } 
  else if (String("fridgeon") == param.asStr())  
  {
    terminal.println("Turning fridge on ") ;
    terminal.flush(); 
    
    digitalWrite(relay_fridgePin, HIGH);
    
  }
  else if (String("humon") == param.asStr())  
  {
    terminal.println("Turning humidifier on ") ;
    terminal.flush(); 
    
    digitalWrite(relay_humidifierPin, HIGH);
    
  }
  else if (String("humoff") == param.asStr())  
  {
    terminal.println("Turning humidifier 0ff ") ;
    terminal.flush(); 
    
    digitalWrite(relay_humidifierPin, LOW);
    
  }
  else if (String("heaton") == param.asStr())  
  {
    terminal.println("Turning heater on ") ;
    terminal.flush(); 
    
    digitalWrite(relay_heaterPin, HIGH);
  }
  else if (String("heatoff") == param.asStr())  
  {
    terminal.println("Turning heater off ") ;
    terminal.flush(); 
    
    digitalWrite(relay_heaterPin, LOW);
  }
  else if (String("fanon") == param.asStr())  
  {
    terminal.println("Turning fan on ") ;
    terminal.flush(); 
    
    digitalWrite(relay_fanPin, HIGH);
  }
  else if (String("fanoff") == param.asStr())  
  {
    terminal.println("Turning fan off ") ;
    terminal.flush(); 
    
    digitalWrite(relay_fanPin, LOW);
  }
  else if (String("fridgeoff") == param.asStr())  
  {
    terminal.println("Turning fridge off ") ;
    terminal.flush(); 
    
    digitalWrite(relay_fridgePin, LOW);
  }
    else if (String("update") == param.asStr())  
  {
    terminal.println("Updating EEPROM ") ;
    terminal.flush(); 
    updateEeprom();
  }
  else 
  {
    // Send it back
    terminal.print("I didn't get that... ");
    //terminal.write(param.getBuffer(), param.getLength());
    terminal.println();
  }

  // Ensure everything is sent
  terminal.flush();
}

/****************************************************************************************
 * Control logic
 ****************************************************************************************/
void usTemp(){updateState (temperature);} // SimpleTimer only accepts void routines

void usHum() {updateState (humidity);}    // So these are wrappers
 
void updateState (ControlledValue &variable)
{
  switch (variable.state) {
    case NOT_CONTROLLING:
      if (_runtimeInSeconds > variable.lastStateChangeTime+MAX_SWITCH_RATE) {
        if (variable.current > variable.target + variable.tolerance) {
          variable.lastStateChangeTime = _runtimeInSeconds;
          variable.state = CONTROLLING_DOWN;
          digitalWrite (variable.pinToDecrease, HIGH);
          terminal.println("Controlling down") ;
          lcd.setCursor(0, 1);
          lcd.print("Controlling down");
          Vlcd.print(0,1,"Controlling down");
        } else if (variable.current < variable.target - variable.tolerance) {
          variable.lastStateChangeTime = _runtimeInSeconds;
          variable.state = CONTROLLING_UP;
          digitalWrite (variable.pinToIncrease, HIGH);
          terminal.println("Controlling up  ") ;
          lcd.setCursor(0, 1);
          lcd.print("Controlling up  ");
          Vlcd.print(0,1,"Controlling up  ");
        }
      }
      break;
    case CONTROLLING_UP:
      if (variable.current > variable.target) {
        variable.lastStateChangeTime = _runtimeInSeconds;
        variable.state = NOT_CONTROLLING;
        digitalWrite (variable.pinToIncrease, LOW);
        terminal.println("Not controlling ") ;
        lcd.setCursor(0, 1);
        lcd.print("Not controlling ");
        Vlcd.print(0,1,"Not controlling ");
      }
      break;
    case CONTROLLING_DOWN:
      if (variable.current < variable.target) {
        variable.lastStateChangeTime = _runtimeInSeconds;
        variable.state = NOT_CONTROLLING;
        digitalWrite (variable.pinToDecrease, LOW);
        terminal.println("Not controlling ") ;
        lcd.setCursor(0, 1);
        lcd.print("Not controlling ");
        Vlcd.print(0,1,"Not controlling ");
      }
      break;
  }
}

/****************************************************************************************
 * Timekeeping
 ****************************************************************************************/
void getCurrentRuntime (unsigned long &days, unsigned long &hours, unsigned long &minutes, unsigned long &seconds)
{
  const unsigned long secondsPerMinute = 60;
  const unsigned long secondsPerHour = 3600;
  const unsigned long secondsPerDay = 86400;
  
  unsigned long computationSeconds = _runtimeInSeconds;
  days = computationSeconds / secondsPerDay;
  computationSeconds = computationSeconds % secondsPerDay;
  hours = computationSeconds / secondsPerHour;
  computationSeconds = computationSeconds % secondsPerHour;
  minutes = computationSeconds / secondsPerMinute;
  computationSeconds = computationSeconds % secondsPerMinute;
  seconds = computationSeconds;
}

/****************************************************************************************
 * Show relevant status led's on case and blynk app.
 ****************************************************************************************/
void setLeds() 
{
    relay_fridgePinState = digitalRead(relay_fridgePin);
    relay_humidifierPinState = digitalRead(relay_humidifierPin);
    relay_fanPinState = digitalRead(relay_fanPin);
    relay_heaterPinState = digitalRead(relay_heaterPin);
    fridge_LedState = digitalRead(fridge_Led);
    humidifier_LedState = digitalRead(humidifier_Led);
    fan_LedState = digitalRead(fan_Led);
    heater_LedState = digitalRead(heater_Led);

    if (relay_fridgePinState == 1 && fridge_LedState == 0)
    {
            digitalWrite(fridge_Led, HIGH);
            Blynk.virtualWrite(Vfridge_Led, 1000);
    }

    if (relay_fridgePinState == 0 && fridge_LedState == 1)
    {
            digitalWrite(fridge_Led, LOW);
            Blynk.virtualWrite(Vfridge_Led, 0);
    }
    if (relay_humidifierPinState == 1 && humidifier_LedState == 0)
    {
        digitalWrite(humidifier_Led, HIGH);
        Blynk.virtualWrite(Vhumidifier_Led, 1000);
    }
    if (relay_humidifierPinState == 0 && humidifier_LedState == 1)
    {
        digitalWrite(humidifier_Led, LOW);
        Blynk.virtualWrite(Vhumidifier_Led, 0);
    }
    if (relay_fanPinState == 1 && fan_LedState == 0)
    {
        digitalWrite(fan_Led, HIGH);
        Blynk.virtualWrite(Vfan_Led, 1000);
    }
    if (relay_fanPinState == 0 && fan_LedState == 1)
    {
        digitalWrite(fan_Led, LOW);
        Blynk.virtualWrite(Vfan_Led, 0);
    }
    if (relay_heaterPinState == 1 && heater_LedState == 0)
    {
        digitalWrite(heater_Led, HIGH);
        Blynk.virtualWrite(Vheater_Led, 1000);
    }
    if (relay_heaterPinState == 0 && heater_LedState == 1)
    {
        digitalWrite(heater_Led, HIGH);
        Blynk.virtualWrite(Vheater_Led, 0);
    }
    
}