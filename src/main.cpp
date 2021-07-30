#include <Arduino.h>
#include <PID_v1.h>
#include <LiquidCrystal.h> // includes the LiquidCrystal Library 

// PWM singal settings
const byte motor_pin = 6; // the PWM pin the motor is attached to
int analogue_in = 0;
double motor_speed_pwm = 0; // The PWM value that will be inputted to control the speed (0-255 is the list of pwm)
int sensorPin = A0;
float effective_v = 0;
int update_interval = 10; // in ms
int display_update_interval = 20;
unsigned long timeold_display_update = 0;

// RPM
const byte encoder_pin = 2; // The pin the encoder is connected
double rpm = 0;             // rpm reading, needs to be double for PID function
unsigned int rpm_workings;
unsigned long pulses = 0; // number of pulses
unsigned long timeold = 0;
unsigned int pulsesperturn = 25; // The number of pulses per revolution depends on your index disc!!
int max_rpm = 3500;

// PID
double motor_speed_setpoint;
double Kp = 2.51; // values gotten from pid_manual tuning seemed to work fairly well 
double Ki = 1.39;// 
double Kd = 0.94;// 
int min = 0, max = 255;
//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID(&rpm, &motor_speed_pwm, &motor_speed_setpoint, Kp, Ki, Kd, DIRECT); //Specify the links and initial tuning parameters

// LCD 
int Contrast=150;
LiquidCrystal lcd(1, 2, 4, 5, 6, 7); // Creates an LC object. Parameters: (rs, enable, d4, d5, d6, d7) 
int brightness_control = 9;


void counter()
{
  //Update count
  pulses++;

}

// the setup routine runs once when you press reset:
void setup()
{
  // declare pin to be an output:
  pinMode(encoder_pin, INPUT);
  pinMode(motor_pin, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(encoder_pin), counter, FALLING); //Triggers on FALLING (change from HIGH to LOW)

  rpm_workings = (60 * 1000) / pulsesperturn;

  // PID Controls:
  myPID.SetOutputLimits(min, max);
  myPID.SetMode(AUTOMATIC);

  // LCD 
  analogWrite(brightness_control, Contrast);
  lcd.begin(16,2); // Initializes the interface to the LCD screen, and specifies the dimensions (width and height) of the display } 


  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop()
{

  if (millis() - timeold >= update_interval)
  { //Uptade every one second, this will be equal to reading frecuency (Hz).
    //---- calculate rpm -----
    //------------------------

    detachInterrupt(digitalPinToInterrupt(encoder_pin)); //Don’t process interrupts during calculations
    rpm = (rpm_workings * pulses) / (millis() - timeold);
    //Serial.print("RPM = ");
    //Serial.println(rpm);
    pulses = 0;

    //----- write new speed from analog -----
    //---------------------------

    analogue_in = analogRead(sensorPin);
    motor_speed_setpoint = round(float(analogue_in) / 1023 * max_rpm); //*255

    //----- PID settings ----------------------
    //------------------------------------

    myPID.Compute(); //this computes what motor_speed_pwm will be set to based on PID values

    if (millis() - timeold_display_update >= display_update_interval) //update less frequency 
    {
      //Serial.print("Setpoint = ");
      //Serial.print(motor_speed_setpoint);
      //Serial.println("RPM");
      //Serial.print("Real RPM = ");
      //Serial.println(rpm);

      lcd.clear(); // Clears the LCD screen

      lcd.print("RPM Setpoint = "); // Prints "Arduino" on the LCD 
      lcd.print(motor_speed_setpoint);
      lcd.setCursor(2,1); // Sets the location at which subsequent text written to the LCD will be displayed 
      lcd.print("Real RPM = "); 
      lcd.print(rpm); 
      lcd.noBlink(); // Turns off the blinking LCD cursor 
      lcd.noCursor(); // Hides the LCD cursor 
      

      //Serial.print(" ");
      //Serial.println("");

      // Print results of PID in various forms
      //Serial.print("Motor Speed PWM = ");
      //Serial.println(motor_speed_pwm);
      //Serial.println("");
      //Serial.print(" ");
      //effective_v = float(motor_speed_pwm) / 255;
      //effective_v = effective_v * 5;
      //Serial.print("Voltage Setpoint = ");
      //Serial.print(effective_v, 2);
      //Serial.println("V");
      timeold_display_update = millis();
    }
    analogWrite(motor_pin, motor_speed_pwm); // set the motor_speed_pwm

    attachInterrupt(digitalPinToInterrupt(encoder_pin), counter, FALLING); //Restart the interrupt processing do it after all the serial prints as they are slow
    timeold = millis();
  }
}
