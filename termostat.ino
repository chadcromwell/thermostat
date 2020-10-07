#include <LiquidCrystal.h>

//Constants (Pins, etc)
const int temperaturePin = 0;
const int RED_PIN = 9;
const int GREEN_PIN = 10;
const int BLUE_PIN = 11;
const int BUTTON_PIN = 2;

//Demo constants
float demodegreesC = 19; //Demo temperature used to simulate PID control

//PID constants
const double P = demodegreesC*1.980354879594423; // Proportional control - Dampened significantly as to not overshoot correction. We don't want to turn on the heater when we're trying to cool or the opposite as this is not efficient. We want to get as close to 0 as possible without going over
const double I = demodegreesC*0.000002574461343; // Integral - Brings error to within approximately .0001
const double D = demodegreesC*99.017743979721166; // Derivative - Handles any overshoot, attempt to limit overshoot to one occurance
const float bias = 0; //If needed
const float pidDelay = 100; //Iteration length in ms, how many times we want to sense the temperature and make an adjustment to the heater or A/C unit
const float acceptableError = .05; //The amount of acceptable error (+/-0.05 degrees)

//PID variables
float error = 0; //To hold current error amount
float oldError = 0; //To hold old error amount
float integral = 0; //To hold integral amount
float derivative = 0; //To hold derivative amount
float correction = 0; //To hold correction amount
bool err; //To determine if there is error and PID control is needed to correct it

//Variables
float voltage, degreesC, degreesF; //To hold voltage coming in from temperature sensor
int ticker = 0; //Keep track of how many loops occur/time passed
int timePassed; //Keep track of time passed in seconds
int delayTime = 1000; //How long each loop iteration lasts in miliseconds, must be above 0 in order for keepTime() to work
int displayDelay = 5; //How long the LCD delays in between switching screens - IN SECONDS
float desiredTemp = -1; //Initialize desired temperature
bool changeTemp = false; //Determine if the temperature needs to be changed
bool initialize = true; //Used for demonstration purposes
LiquidCrystal lcd(8,7,6,5,4,3); //Set LCD pins

//SETUP*****************************************************************************************************

//setup() - Set up the pins
void setup() {
  Serial.begin(9600); //Set baud rate to 9600
  pinMode(RED_PIN, OUTPUT); //RED_PIN (9) used for OUTPUT
  pinMode(GREEN_PIN, OUTPUT); //GREEN_PIN (10) used for OUTPUT
  pinMode(BLUE_PIN, OUTPUT); //BLUE_PIN (11) used for OUTPUT
  pinMode(BUTTON_PIN, INPUT_PULLUP); //BUTTON_PIN (2) used for INPUT. PULLUP used for FALLING (quicker response to button instead of LOW)
  attachInterrupt(digitalPinToInterrupt(2), set, FALLING); //Attach Interrupt to the button, when button is pushed it will execute set() function
  lcd.begin(16, 2); //Declare LCD Rows/Colums (16 char, 2 rows)
  lcd.clear(); //Clears the data on the LCD so that old data isn't displayed from previous sketches
}

//FUNCTIONS*****************************************************************************************************

///pidControl() - Acts as a relay switch. If the user's desired temperature is lower than the current room temperature, it turns the A/C ON and furnace OFF using PID control to maximize efficiency.
//If the user's desired temperature is higher than the current room temperature, it turns the A/C OFF and turns the furnace ON at varying speeds until the perfect temperature is reached and it keeps it there.
//Replace analogWrite used for LED pins to the appropriate code for your A/C and Furnace.

void pidControl() {
  if (desiredTemp != -1) {
  //If the desired temperature is not reached
  if (desiredTemp != demodegreesC) {
    err = true; //Set err to true to state that error needs to be corrected
  }
  //If error needs to be corrected
  if (err == true) {
    //Assign current values
    error = desiredTemp - demodegreesC; //Error between desired temperature and current room temperature
    integral = integral+(error*pidDelay); //Integral amount, error*iteration time
    derivative = (error-oldError)/pidDelay; //Derivative, current error-old amount of error/iteration time
    correction = P*error+I*integral+D*derivative+bias; //Correction amount, P*error+I*integral+D*derivative+bias(if neeeded)
    oldError = error; //Assign current error as old error for next iteration
      
    //If the amount of error is less than 0, it needs to cool
    if (error < -acceptableError) {

      //Need to make correction amount positive if it is negative so it applies a positive amount to the A/C or heater
        if (correction < 0) {
        correction = correction*-1; //Multiply itself by -1 to make it positive
      }
    
       //If correction amount is higher than 255, limit it to 255 (max amount of power)
       if (correction > 255) {
        correction = 255;
       }
        //Turn on A/C, turn off heater
        analogWrite(RED_PIN, 0);
        analogWrite(GREEN_PIN, 0);
        analogWrite(BLUE_PIN, correction);

        //Display amounts to serial window for PID demo
        Serial.print("COOL: ");
        Serial.print(correction, 4);
        Serial.print("  Error: ");
        Serial.print(error, 4);
        Serial.print("  Current temperature: ");
        Serial.print(demodegreesC);
        Serial.print("\n");
        //If we're cooling, it means it is hot outside so simulate rising ambient temperature and speed of cooling
        if (desiredTemp < demodegreesC) {
          demodegreesC = demodegreesC-(correction*.005)+.005; //Simalute heating of room temperature. Current temp * speed of cooling + ambient temperature rising over time
        }
        //If we're heating, it means it is cold outside so simulate falling ambient temperature and speed of heating
        if (desiredTemp > demodegreesC) {
          demodegreesC = demodegreesC+(correction*.005)-.005; //Simalute heating of room temperature. Current temp * speed of cooling + ambient temperature rising over time
        }
      }

      //If the amount of error is greater than 0, it needs to heat
      if (error > acceptableError) {
        if (correction > 255) { //If correction amount is higher than 255, set it to 255 (max amount)
          correction = 255;
        }

        //Turn on heater, turn off A/C
        analogWrite(RED_PIN, correction);
        analogWrite(GREEN_PIN, 0);
        analogWrite(BLUE_PIN, 0);

        //Display amounts to serial window for PID demo
        Serial.print("HEAT: ");
        Serial.print(correction, 4);
        Serial.print("  Error: ");
        Serial.print(error, 4);
        Serial.print("  Current temperature: ");
        Serial.print(demodegreesC);
        Serial.print("\n");
        //If we're cooling, it means it is hot outside so simulate rising ambient temperature and speed of cooling
        if (desiredTemp < demodegreesC) {
          demodegreesC = demodegreesC-(correction*.005)+.005; //Simalute heating of room temperature. Current temp * speed of cooling + ambient temperature rising over time
        }
        //If we're heating, it means it is cold outside so simulate falling ambient temperature and speed of heating
        if (desiredTemp > demodegreesC) {
          demodegreesC = demodegreesC+(correction*.005)-.005; //Simalute heating of room temperature. Current temp * speed of cooling + ambient temperature rising over time
        }
      }

      //If the desired temperature is reached, turn off A/C and furnace.
      if (error <= acceptableError && error >= -acceptableError) {
        analogWrite(RED_PIN, 0); //Turn OFF Furnace (R LED)
        analogWrite(GREEN_PIN, 255); //Turn on G LED for demonstration purposes to show that A/C and Furnace are OFF
        analogWrite(BLUE_PIN, 0); //Turn OFF A/C (B LED)
        Serial.print("Comfort achieved");
        Serial.print("\n");
       err = false;
      }

      //Delay iteration
      delay(pidDelay);
    }
  }
}

//getVoltage(int pin)
//Accepts a pin and returns the voltage converted to a 255 bit int
float getVoltage(int pin) {
  
  return (analogRead(pin) * 0.004882814);
  
}

//set()
//Interrupt Service Routine (ISR). If button is pressed, it interrupts loop and asks for user input
void set() {
  changeTemp = true;
}

//timeKeep()
//Keeps time based on how many loops have occured and the delay per loop
//ticks*delay = miliseconds that have passed. ticks*delay/1000 = seconds that have passed
//updates timePassed to be in seconds
void timeKeep() {
  ticker++;
  timePassed = ticker*delayTime/1000;
  if (timePassed == displayDelay) {
    timePassed = 0;
  }
}

//updateDisplay()
//Prints text and information to the LCD screen
//If it's cooling, heating, or if desired temperature is reached, etc.
void updateDisplay() {
  
  //While initialize is true - For demo purposes
  while (initialize == true) {
    lcd.clear(); //Clear LCD buffer due to old data possibly being there
    
    //Ask user to enter a desired temperature
    lcd.setCursor(0, 0); //Row one
    lcd.print("Please enter");
    
    lcd.setCursor(0, 1); //Row two
    lcd.print("desired temp: ");
    initialize = false; //No need to initialize anymore - For demo purposes
  }

  //If the desired temperature has not been reached within acceptable error and desired temperature has been entered by user
  if (desiredTemp > degreesC+acceptableError && desiredTemp != -1 || desiredTemp < degreesC-acceptableError && desiredTemp != -1) {

    //If the room is warmer than desired temperature, tell the user the current temp and what it is cooling to
   if (desiredTemp < degreesC) {
    lcd.clear();
    //Display current temperature on top line
    lcd.setCursor(0, 0); //Row one
    lcd.print("Current temp ");
    lcd.print(round(degreesC));
    
    lcd.setCursor(0, 1); //Row two
    lcd.print("Cooling to ");
    lcd.print(desiredTemp, 0);
   }

   //If the room is cooler than desired temperature, tell the user the current temp and what it is heating to
   if (desiredTemp > degreesC) {
    lcd.clear();
    //Display current temperature on top line
    lcd.setCursor(0, 0); //Row one
    lcd.print("Current temp ");
    lcd.print(round(degreesC));
    
    lcd.setCursor(0, 1); //Row two
    lcd.print("Heating to ");
    lcd.print(desiredTemp, 0);   
   }
 }

  //If the desired temperature has been reached, tell the user the current temperature and that their comfort has been achieved
  if (desiredTemp == round(degreesC)){
    lcd.setCursor(0,0);//Row one
    lcd.print("Current temp ");
    lcd.print(round(degreesC));
  
    lcd.setCursor(0,1); //Row two
    lcd.print("Comfort achieved");
 }
}

//flushIt()
//Flushes the serial Input buffer so that old input isn't passed to the current interaction, call this before getting any serial input
void flushIt() {
  while(Serial.available()) { //While there is data in the serial Input buffer
    char flushIt = Serial.read(); //Assign data to char and do nothing with it
  }
}

void serialIO() {
  //If changeTemp is true (set true when button is pressed due to interrupt handling)
  while(changeTemp == true) {
    flushIt();; //Clear buffer
    Serial.print("\nEnter a temperature: "); //Ask the user to enter a temperature
    while(!Serial.available()); //Wait for user input
    desiredTemp = Serial.parseInt(); //Assign user input to desiredTemp
    Serial.println(desiredTemp); //Print to display user's desired temperature
    changeTemp = false; //Set changeTemp to false to break out of this if statement
  }
}

void tempSensor() {
  //Assign the voltage from the temperature sensor to voltage
  voltage = getVoltage(temperaturePin);

  //Convert voltage to Celcius
  degreesC = (voltage - 0.5) * 100.0;

  //Convert Celcius to Fahrenheit
  degreesF = degreesC * (9.0/5.0) + 32.0;
}

//MAIN LOOP*****************************************************************************************************

void loop() {
  serialIO(); //Call serialIO to handle serial IO (for user input of desired temperature. Would have been handled on the LCD screen, however there is no room left for buttons on the bread board)
  tempSensor(); //Call tempSensor to handle temperature sensor data, provides temp in C and F
  updateDisplay(); //Call updateDisplay() to control the LCD and user input
  pidControl(); //Call pidControl() to turn ON or OFF A/C or Furnace if needed due to current temperature reading
  delay(delayTime); //Delay loop, must be above 0 in order for keepTime() to work
  timeKeep(); //Call timeKeep() to keep track of displayDelay
}
