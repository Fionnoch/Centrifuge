/* #include <Arduino.h>
#include <PID_v1.h>

// PWM singal settings
const byte motor_pin = 6; // the PWM pin the motor is attached to
int analogue_in = 0;
double motor_speed_pwm = 0; // The PWM value that will be inputted to control the speed (0-255 is the list of pwm)
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
unsigned int pulsesperturn = 50; // The number of pulses per revolution depends on your index disc!!

// PID

const byte Kp_potentiometer_pin = A0;
const byte Ki_potentiometer_pin = A1;
const byte Kd_potentiometer_pin = A2;

double motor_speed_setpoint = 2000;
double Kp = 0;      //2;
double Ki = 0;   // 1.07;
double Kd = 0;    // 0.5;
double Kp_temp = 0; //2.5
double Ki_temp = 0;
double Kd_temp = 0;

double Max_Kp = 20; //2.5
double Max_Ki = 20;
double Max_Kd = 20;

//PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
PID myPID(&rpm, &motor_speed_pwm, &motor_speed_setpoint, Kp, Ki, Kd, DIRECT); //Specify the links and initial tuning parameters

void counter()
{
    //Update count
    pulses++;
}

// the setup routine runs once when you press reset:
void setup()
{
    // declare pin to be an output:
    //pinMode(encoder_pin, INPUT);

    pinMode(Kp_potentiometer_pin, INPUT);
    pinMode(Ki_potentiometer_pin, INPUT);
    pinMode(Kd_potentiometer_pin, INPUT);

    pinMode(motor_pin, OUTPUT);

    attachInterrupt(digitalPinToInterrupt(encoder_pin), counter, FALLING); //Triggers on FALLING (change from HIGH to LOW)

    rpm_workings = (60 * 1000) / pulsesperturn;

    // PID Controls:
    myPID.SetMode(AUTOMATIC);

    Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop()
{

    if (millis() - timeold >= update_interval)
    {
        //---- calculate rpm -----
        //------------------------

        detachInterrupt(digitalPinToInterrupt(encoder_pin)); //Don’t process interrupts during calculations
        rpm = (rpm_workings * pulses) / (millis() - timeold);
        pulses = 0;

        //----- write new speed from analog -----
        //---------------------------

        //analogue_in = analogRead(sensorPin);
        //motor_speed_setpoint = round(float(analogue_in) / 1023 * max_rpm); //*255

        //----- PID settings ----------------------
        //-----------------------------------------

        //Serial.print("RPM_Setpoint ,");

        myPID.Compute(); //this computes what motor_speed_pwm will be set to based on PID values

        if (millis() - timeold_display_update >= display_update_interval) //update less frequency
        {
            timeold_display_update = millis();

            Kp_temp = Max_Kp * analogRead(Kp_potentiometer_pin) / 1023;
            Ki_temp = Max_Ki * analogRead(Ki_potentiometer_pin) / 1023;
            Kd_temp = Max_Kd * analogRead(Kd_potentiometer_pin) / 1023;

            if (Kp_temp != Kp || Ki_temp != Ki || Kd_temp != Kd)
            {
                Kp = Kp_temp;
                Ki = Ki_temp;
                Kd = Kd_temp;
                myPID.SetTunings(Kp, Ki, Kd);
 
            }
        }
        analogWrite(motor_pin, motor_speed_pwm); // set the motor_speed_pwm

        attachInterrupt(digitalPinToInterrupt(encoder_pin), counter, FALLING); //Restart the interrupt processing do it after all the serial prints as they are slow
        timeold = millis();
    }

    Serial.print(float(round(motor_speed_setpoint *100)/100, 2);
    Serial.print(",");
    Serial.print(rpm, 2);
    Serial.print(",");
    Serial.print(Kp, 2);
    Serial.print(",");
    Serial.print(Ki, 2);
    Serial.print(",");
    Serial.print(Kd, 2);
    Serial.print(",");

    Serial.println("");
}
 */