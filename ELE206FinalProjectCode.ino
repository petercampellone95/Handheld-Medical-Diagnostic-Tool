/******************************************************************
   Program Name: ELE 206 Final Project
   Created by: Peter M Campellone
   Date Created: 3/31/2016
   Program Description: This program is used to control a 3D
   printer handheld medical diagnostic tool. Tee functions of
   this device include IR temperature measurement and pulse rate
   measurement. These values are outputted to a 16x2 LCD connected
   using the I2C protocol made possible by a serial to parallel board
   specifically made for HD44780 controlled LCDs.
   Numerous parts of this code are taken from open-source
   libarires, etc. Credit for these portions is attributed below:

    Pulse Sensor Code: World Famous Electronics LLC. (https://github.com/WorldFamousElectronics/PulseSensor_Amped_Arduino)
    MLX90614 Code/Library: Adafruit Inc. (https://github.com/adafruit/Adafruit-MLX90614-Library)
    Serial I2C LCD Code: F. Malpartida (https://bitbucket.org/fmalpartida/new-liquidcrystal/wiki/Home)

    Pin Assignment:

    Pulse Sensor: A1 (Data)
    LCD: A4 (SDA), A5 (SCL)
    MLX90614: A4 (SDA), A5 (SCL)
    Buttons: 4 (Up), 5 (Down)
    Battery Meter: A3


 ******************************************************************/

//Libraries

#include <LiquidCrystal_I2C.h> //Open Source Library for controlling an LCD using I2C, provided by F. Malpartida
#include <Adafruit_MLX90614.h> //Adafruit's Open Source Library for the MLX90614 Temp. Sensor
#include <Wire.h> //Built-in Arduino library needed for using the I2C communication protocol

#define BACKLIGHT_PIN 3

LiquidCrystal_I2C  lcd(0x3F,2,1,0,4,5,6,7);
Adafruit_MLX90614 mlx = Adafruit_MLX90614(); //Setting up the MLX90614 library, The MLX90614 uses I2C which uses pins A4 and A5 (SDA, SCL)

//Menu System and Switch Variables
const int upButtonPin = 9; //The up button will be connected to digital pin 4
const int downButtonPin = 10; //The down button will be connected to digital pin 5

int lastButtonPushed = 0; //Stores the pin number of the last button pushed (either pin 4 or 5 in this case)

int upButtonLastState = LOW; //Stores the last state of the upButton (used to recognize button presses)
int downButtonLastState = LOW; //Stores the last state of the upButton (used to recognize button presses)

long upButtonDebounceTime = 0; //Long variable to store the debounce time of the up button
long downButtonDebounceTime = 0; //Long variable to store the debounce time of the down button

long debounceDelay = 250; //Long variable storing a typical delay to account for mechanical switch debounce, 350ms in this case works well

//Other variables
int mode = 0; //Modes, 0 = All off, 1 = Temp Sensor, 2 = Heart Rate Sensor, 3 = Battery Meter
const int batteryPin = A3; //Analog pin to read battery voltage

double tempF = 0; //Double variable storing the Fahrenheit temperature as outputted by the MLX90614
double voltage = 0; //Voltage of battery calculated from analog read on A3
int percentage = 0; //Percentage of charge left on battery

const int pulsePowerPin = 12;

long startTime = 0;
long endTime = 0;
long totalTime = 0;


//Pulse Sensor Pins
const int pulsePin = A1;                 // Pulse Sensor purple wire connected to analog pin 0

// Volatile Variables, used in the interrupt service routine!
volatile int BPM;                   // int that holds raw Analog in 0. updated every 2mS
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // int that holds the time interval between beats! Must be seeded!
volatile boolean Pulse = false;     // "True" when User's live heartbeat is detected. "False" when not a "live beat".
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.

static boolean serialVisual = false;   // Set to 'false' by Default.  Re-set to 'true' to see Arduino Serial Monitor ASCII Visual Pulse
// Only set to true for testing, disabled otherwise


void setup() {

  Serial.begin(115200);             //Turn on serial monitor at a baud rate of 115200 bps
  pinMode(batteryPin, INPUT);
  pinMode(upButtonPin, INPUT);      //pin that will be connected to the up button
  pinMode(downButtonPin, INPUT);    //pin that will be connected to the down button
  pinMode(pulsePowerPin, OUTPUT);

  lcd.setBacklightPin(BACKLIGHT_PIN,POSITIVE);
  lcd.setBacklight(HIGH);

  interruptSetup();// sets up to read Pulse Sensor signal every 2mS
  // may want to move down to case statement, see if location causes time delays
  lcd.begin(16, 2); //Intializing a 16 row by 2 column LCD
  mlx.begin();      //Intializing the MLX90614 temperature sensor

  lcd.setCursor(0,0);
  lcd.println("Handheld Medical");
  lcd.setCursor(0,1);
  lcd.println("Diagnostic Tool ");
  delay(2000);
 

}


void loop() {

  readButtons(); //Calls readButtons()to check if a button has been pressed
  moveMenu();   //Calls moveMenu() to move the LCD menu if a button has been pressed
  modeOutput(); //Tells the tool to perform a specific function based on switch input

}

void readButtons() {
  //Function used to check if the up or down buttons were pressed, incorporates debounce calcualtions to improve stability

  int reading;                //Integer variable to store the digital reading from the two switches
  int upButtonState = LOW;    //Integer variable to store the current state of the up button
  int downButtonState = LOW;  //Integer variable to store the current state of the down button

  //Checking if the up button was pressed
  reading = digitalRead(upButtonPin);

  if (reading != upButtonLastState) //If the reading is different the previous state the button has changed
  {
    upButtonDebounceTime = millis(); //Start a timer to calculate debounce
  }

  if ((millis() - upButtonDebounceTime) > debounceDelay) //If the was pressed longer than the debounce delay then it was an actual button press
  {
    upButtonState = reading; //Set the up button state to the reading on pin 4
    upButtonDebounceTime = millis(); //Set the debounce time for the up button equal to millis(), effectively the debounce delay of the up button
  }

  upButtonLastState = reading; //Set the last state of the up button equal to variable reading

  //Checking to see if the down button was pressed
  reading = digitalRead(downButtonPin);

  if (reading != downButtonLastState) //If the reading is different the previous state the button has changed
  {
    downButtonDebounceTime = millis(); //Start a timer to calculate debounce
  }

  if ((millis() - downButtonDebounceTime) > debounceDelay) //If the was pressed longer than the debounce delay then it was an actual button press
  {
    downButtonState = reading; //Set the up button state to the reading on pin 5
    downButtonDebounceTime = millis(); //Set the debounce time for the down button equal to millis(), effectively the debounce delay of the down button
  }

  downButtonLastState = reading; //Set the last state of the down button equal to variable reading

  //Reporting which button was pushed

  if (upButtonState == HIGH) {
    lastButtonPushed = upButtonPin;
    Serial.println("Up Button Pushed");

  }
  else if (downButtonState == HIGH) {
    lastButtonPushed = downButtonPin;
    Serial.println("Down Button Pushed");
  }

  else {
    lastButtonPushed = 0; //No button was pushed
  }
}

boolean moveMenu() {
  //Boolean function used to change the mode based on whatever the last button pushed was

  if (lastButtonPushed == upButtonPin) //If the last button pushed was the up button then increment mode and return true
  {
    delay(100);
    mode++; //Increment mode
    return true; //True means a button was pushed regardless of which one it was
  }

  if (lastButtonPushed == downButtonPin) //If the last button pushed was the down button then decrement mode and return true
  {
    delay(100);
    mode--; //Decrement mode
    return true;//True means a button was pushed regardless of which one it was
  }

  lastButtonPushed = 0; //Neither button was pushed
  return false; //Return false meaning that no button was pushed and that the mode shouldn't change
}


void modeOutput() {
  //Function used to perform various functions depending on the current value of mode
  //Current functions are: Mode 0: All off
  //                       Mode 1: Temperature Measurement
  //                       Mode 2: Pulse Rate Measurement
  //                       Mode 3: Battery Meter

  lcd.setCursor(0,1);
  lcd.println("                ");

  if (mode < 0) //If mode is less than zero than it is out of bounds and will be set to 2
  {
    mode = 3;
  }

  if (mode > 3) //If mode is greater than two then it is out of bounds and will be set to 0
  {
    mode = 0;
  }

  switch (mode) //Switch-Case statement used to switch between different modes
  {
    case 0: //If Mode = 0, All Off

      startTime = millis();
      digitalWrite(pulsePowerPin,LOW);
      lcd.setCursor(0,0);
      lcd.println("    All Off     "); //Prints message to LCD that all modes are off
      endTime += millis() - startTime; 
      Serial.println(endTime);
      if(endTime >= 15000)
      {
        lcd.setBacklight(LOW);
        break;
      }
      break;

    case 1: //If Mode = 1, Use the Temp. Sensor
    
      endTime = 0;
      lcd.setBacklight(HIGH);
      digitalWrite(pulsePowerPin,LOW);
      delay(20);
      tempF = readTemp(); //Read the temperature in Fahrenheit from the temp. sensor
      lcd.setCursor(0, 0);
      lcd.print("Temp:");
      lcd.print(tempF+2);
      lcd.write(0xDF);
    lcd.println("F    ");
      delay(200); //100 ms delay so that the value on the LCD is readable

      readButtons(); //Check for button presses after each iteration
      moveMenu(); //Move menu if there was a button pressed

      break;

    case 2: //If Mode = 2, Use the Pulse Sensor
      
      endTime = 0;
      lcd.setBacklight(HIGH);
      digitalWrite(pulsePowerPin,HIGH);
      delay(20);
      getPulse(); //Calls the getPulse() function which gets the pulse rate from the pulse sensor
      lcd.setCursor(0, 0);
      lcd.print("H. Rate: ");
      lcd.print(BPM);
      lcd.println("BPM   ");
      delay(200);

      readButtons(); //Check for button presses after each iteration
      moveMenu(); //Move menu if there was a button pressed

      break;


    case 3:
    
      endTime = 0;
      lcd.setBacklight(HIGH);
      digitalWrite(pulsePowerPin,LOW);
      batteryMeter();
      lcd.setCursor(0, 0);
      lcd.print("Bat. Charge:");
      lcd.print(percentage);
      lcd.println("%  ");

      delay(400);
      
 
      if (percentage <= 20)
      {
        lcd.setCursor(1, 1);
        lcd.println(" LOW BATTERY!  ");
      }

      else{
        lcd.setCursor(0,1);
        lcd.println("                ");
      }

      readButtons();
      moveMenu();

      break;

  }
}

double readTemp() {
  //Function used to read the temperature reported by the MLX90614

  return (mlx.readObjectTempF());

  //Serial.print("*F\tObject = "); Serial.print(mlx.readObjectTempF()); Serial.println("*F");
  //delay(500);
}

void getPulse() {
  //Function used to get the pulse rate from the pulse sensor
  //Slightly modified version of the pulse sensor code provided by World Famous Electronics LLC.

  serialOutput() ;

  if (QS == true) {    // A Heartbeat Was Found
    // BPM and IBI have been Determined
    // Quantified Self "QS" true when arduino finds a heartbeat
   
    // Set 'fadeRate' Variable to 255 to fade LED with pulse
    serialOutputWhenBeatHappens();   // A Beat Happened, Output that to serial.
    QS = false;                      // reset the Quantified Self flag for next time
  }
  delay(20);                             //  take a break
}

void batteryMeter() {
  //Function used to calculate the amount of charge left on the battery. For single cell LiPo batteries max is 4.20V and min is 3.70V.

  int reading = analogRead(batteryPin);
  Serial.println(reading);
  voltage = (5.0 / 1023) * reading;
  Serial.println(voltage);
  percentage = ((voltage - 3.70) / (4.30 - 3.70)) * 100;

}
 
//END OF PROGRAM




