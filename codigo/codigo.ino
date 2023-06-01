//LCD config
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3f,16,2);  //sometimes the adress is not 0x3f. Change to 0x27 if it dosn't work.

//Thermistor needed libraries
#include <thermistor.h>           //Download it here: https://electronoobs.com/eng_arduino_thermistor.php
thermistor therm1(A0,0);          //Connect thermistor on A0, 0 represents TEMP_SENSOR_0 ( configuration.h for more)


//I/O
int PWM_pin = 5;                  //Pin for PWM signal to the MOSFET driver (the BJT npn with pullup)
int speed_pot = A1;
int but1 = 7;
int EN = 2;
int STEP = 3;
int DIR = 4;
int LED = 13;

//Variables
float set_temperature = 200;            //Default temperature setpoint. Leave it 0 and control it with rotary encoder
float temperature_read = 0.0;
float PID_error = 0;
float previous_error = 0;
float elapsedTime, Time, timePrev;
float PID_value = 0;
int button_pressed = 0;
int menu_activated=0;
float last_set_temperature = 0;
int max_PWM = 255;

//Stepper Variables
int max_speed = 1000;
int main_speed = 0;
bool but1_state = true;
bool activate_stepper = false;
int rotating_speed = 0;



#include <AccelStepper.h>
// Define a stepper and the pins it will use
AccelStepper stepper1(1, STEP, DIR); // (Type of driver: with 2 pins, STEP, DIR)

//PID constants
//////////////////////////////////////////////////////////
int kp = 90;   int ki = 30;   int kd = 80;
//////////////////////////////////////////////////////////

int PID_p = 0;    int PID_i = 0;    int PID_d = 0;
float last_kp = 0;
float last_ki = 0;
float last_kd = 0;

int PID_values_fixed =0;



void setup() {  
  pinMode(EN, OUTPUT);
  digitalWrite(EN, HIGH);     //Stepper driver is disbled
  stepper1.setMaxSpeed(max_speed);  
  pinMode(but1, INPUT_PULLUP);
  pinMode(speed_pot, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  
  pinMode(PWM_pin,OUTPUT);
  TCCR0B = TCCR0B & B11111000 | B00000010;    // D6 adn D6 PWM frequency of 7812.50 Hz
  Time = millis();

  TCCR1A = 0;             //Reset entire TCCR1A register
  TCCR1B = 0;             //Reset entire TCCR1B register
  TCCR1A |= B00000010;    //   /8
  TCNT1 = 0;              //Reset Timer 1 value to 0
  
  lcd.init();
  lcd.backlight();
}

void loop() {
  if(!digitalRead(but1) && but1_state){
    but1_state = false;
    activate_stepper = !activate_stepper;
    delay(10);
  }
  else if(digitalRead(but1) && !but1_state){
    but1_state = true;
  }
  
  if(activate_stepper){
    digitalWrite(LED, HIGH);
    digitalWrite(EN, LOW);    //We activate stepper driver
    rotating_speed = map(analogRead(speed_pot),0,1024,main_speed,max_speed);
    stepper1.setSpeed(rotating_speed);
    //stepper1.runSpeed();
  }
  else
  {
    digitalWrite(EN, HIGH);    //We deactivate stepper driver
    digitalWrite(LED, LOW);
    stepper1.setSpeed(0);
    //stepper1.runSpeed();
  }
  
  // First we read the real value of temperature
  temperature_read = therm1.analog2temp(); // read temperature
  
  //Next we calculate the error between the setpoint and the real value
  PID_error = set_temperature - temperature_read + 6;
  //Calculate the P value
  PID_p = 0.01*kp * PID_error;
  //Calculate the I value in a range on +-6
  PID_i = 0.01*PID_i + (ki * PID_error);
  
  
  //For derivative we need real time to calculate speed change rate
  timePrev = Time;                            // the previous time is stored before the actual time read
  Time = millis();                            // actual time read
  elapsedTime = (Time - timePrev) / 1000; 
  //Now we can calculate the D calue
  PID_d = 0.01*kd*((PID_error - previous_error)/elapsedTime);
  //Final total PID value is the sum of P + I + D
  PID_value = PID_p + PID_i + PID_d;

  
  //We define PWM range between 0 and 255
  if(PID_value < 0){
    PID_value = 0;
  }
  if(PID_value > max_PWM){
    PID_value = max_PWM;
  }
  
  //Now we can write the PWM signal to the mosfet on digital pin D5
  analogWrite(PWM_pin,PID_value);
  previous_error = PID_error;     //Remember to store the previous error for next loop.
  
  delay(250); //Refresh rate + delay of LCD print
  lcd.clear();
  
  lcd.setCursor(0,0);
  lcd.print("T: ");  
  lcd.print(temperature_read,1);
  lcd.print(" S: ");  
  lcd.print(rotating_speed);
  
  lcd.setCursor(0,1);
  lcd.print("PID: ");
  lcd.print(PID_value);
 
}//Void loop end


ISR(TIMER1_COMPA_vect){
  TCNT1  = 0;                  //First, set the timer back to 0 so it resets for next interrupt
  stepper1.runSpeed();
}
