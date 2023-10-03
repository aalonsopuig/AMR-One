
/*
 * Program AMR-board.
 * Used for the low level control of the AMR
 * by Alejandro Alonso Puig. April 2022
 * 
 * 
 * Copyright 2022 Alejandro Alonso Puig. All Rights Reserved.
 * 
 *  Permission is hereby granted, free of charge, to any person obtaining a copy of this
 *  software and associated documentation files (the "Software"), to deal in the Software
 *  without restriction, including without limitation the rights to use, copy, modify,
 *  merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 *  permit persons to whom the Software is furnished to do so.
 * 
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 *  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 *  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 *  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 *  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 *  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 * Versions
 * 0.01: Only LCD
 * 0.02: added keypad
 * 0.03: added PID control over motor and Encoder measurement
 * 0.04: Changed PID library with this one: FastPID: https://github.com/mike-matera/FastPID 
 * 0.05: Two motors with PID
 * 0.06: Buzzer added
 * 0.07: Safety stop implemented
 * 0.08: Pinhook implemented
 * 0.09: Lidar management implemented
 * 0.10: LCD screens + Adaptation to AMR motors (PWM from 0 to 250 duty, being 127:stop, 0:max reverse and 255:max forward. Freq betweern 1khz and 3khz
 * 0.11: Improve PID management (timings and stable readings for encoders) using timer1 interrupts
 * 0.12: Improvement on PID to include global setpoint and deviation control
 * 0.13: Incorporated signals from lidar, mag sensor, safety PLC, rearm button 
 * 0.14: Incorporated basic magnetic guidance with PID (defective version in PIDs)
 * 0.15: Deep review on PID to control limits, strange behaviors,...
 * 0.16: New deep review of PID after Drive configuration without automatic switch off at stop
 * 0.17: Added LCD screen with aditional data (Magnetic guidance, RFID tag code, data from Lidar signals and safety, time on measured) 
 * 0.18: Deviation control in a smoother way
 * 0.19: Some bugs corrected in deviation control
 * 0.20: RFID interrogator integration
 * 0.21: ROSSERIAL integration. Jira issue AMR-95 (not finished)
 * 0.22: 2 keys keypad + control of LCD screens + some cleaning on code
 * 0.23: Implementation of bip-bip sounds when Lidar detects objects in its protective field (2Hz bip) or warning field (1Hz bip). Buzzer_warnings() function
 * 0.24: Delay of 20 seconds until buzzer warning related to Lidar, as Lidar needs 20 second to boot up completely
 * 0.25: Added buzzer warning (beep every 4 seconds) if safety stop button (local or remote) is active
 * 0.26: Review Magnetic Guidance
 * 0.27: RFID tags actions implemented
 * 0.28: Mag band by default, added control of pinhook via serial
 * 0.29: ROSSERIAL integration. Jira issue AMR-247. Deactivated serial interface with user (see Jira issue AMR-248)
 */

#define VERSION 0.29
 
// PINS MAP

// PWM/Digital (02..13)
#define PIN_ENCODER1A        3 //Input.   Pins where connected encoder. First should have interrupt (Options: 2, 3, 18, 19, 20, 21 in Mega)
#define PIN_ENCODER1B        4 //Input.   Pins where connected encoder. 
#define PIN_ENCODER2A        2 //Input.   Pins where connected encoder. First should have interrupt (Options: 2, 3, 18, 19, 20, 21 in Mega)
#define PIN_ENCODER2B        5 //Input.   Pins where connected encoder.
#define PIN_MOTOR1_PWM       6 //Output.  Motor 1 PWM output. Default freq. is 490 Hz, except pin 4 and 13 whose default frequency is 980Hz.
#define PIN_MOTOR2_PWM       7 //Output.  Motor 2 PWM output. Default freq. is 490 Hz, except pin 4 and 13 whose default frequency is 980Hz.
#define PIN_BUZZER           8 //Output.  Buzzer
#define PIN_KBD_Inc          9 //Output.  Keyboard. Increment button
#define PIN_KBD_Dec         10 //Output.  Keyboard. Decrement button
#define PIN_KBD_CMN         11 //Output.  Keyboard. common

//Digital (22..53)
#define PIN_SAFETYSTOP      22 //Input.   Safety Stop input (From Safety PLC Flexysoft) 1=Safety stop, 0=No Safety stop
#define PIN_BUTTON          23 //Input.   Multipurpose button (Rearm button)
#define PIN_MOTORS_ACTIVE   24 //Output.  Motor Drives active (Indicates if Motor Drives should be active (1) or not (0). To Safety PLC)
#define PIN_LIDAR_WARNING   25 //Input.   Lidar Warning Field compromised (From Lidar)
#define PIN_LIDAR_PROTECT   26 //Input.   Lidar Protective Field compromised (From Lidar)
#define PIN_LIDAR_DIRTY     27 //Input.   Lidar Dirty lens warning (From Lidar)
#define PIN_LIDAR_RESET     28 //Output.  Lidar Reset
#define PIN_LIDAR_CASEBIT0  29 //Output.  Lidar Case bit0 (To Safety PLC. It activates in Lidat via EFI)
#define PIN_LIDAR_CASEBIT1  30 //Output.  Lidar Case bit1 (To Safety PLC. It activates in Lidat via EFI)
#define PIN_PINHOOK_DIR0    31 //Output.  Pinhook direction bit0
#define PIN_PINHOOK_DIR1    32 //Output.  Pinhook direction bit1
#define PIN_MAG_SEL1        33 //Output.  Magnetic band sensor. Selector signal 1
#define PIN_MAG_SEL2        34 //Output.  Magnetic band sensor. Selector signal 2
#define PIN_MAG_GATE        35 //Input.   Magnetic band sensor. Negative logic. LOW:Band detected, HIGH:Not detected
#define PIN_BUTTON_LED      36 //Output.  Multipurpose button led

//Analog (A0..A15)
#define PIN_MAG_DEV         A0 //Input.  Magnetic band sensor. Deviation. Analog

 
//ROS integration
#include <ros.h>              //rosserial library https://github.com/frankjoshua/rosserial_arduino_lib
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <std_msgs/UInt16.h>
ros::NodeHandle  nh;
std_msgs::Float32 velocity_lin_msg;
std_msgs::Float32 velocity_ang_msg;
std_msgs::UInt16 safety_status_msg;
std_msgs::String rfid_tag_msg;
ros::Publisher pub_velocity_lin("velocity_lin", &velocity_lin_msg);
ros::Publisher pub_velocity_ang("velocity_ang", &velocity_ang_msg);
ros::Publisher pub_safety_status("safety_status", &safety_status_msg);
ros::Publisher pub_rfid_tag("rfid_tag", &rfid_tag_msg);
char rfidcode[5] = "    ";

  
#include <Key.h>
#include <Keypad.h>           //Keypad library https://playground.arduino.cc/Code/Keypad/

#include <Wire.h>             //I2C Library
#include <LCD03.h>            //LCD screen https://github.com/andatche/arduino-lcd03

#include <Encoder.h>          //Encoder library https://www.pjrc.com/teensy/td_libs_Encoder.html
#include <FastPID.h>          // PID library https://github.com/mike-matera/FastPID
#include <uptime_formatter.h> // Uptime Library. https://github.com/YiannisBourkelis/Uptime-Library
#include <uptime.h>

#define PID_Interrupt_ms  70  //Time in ms between two interrupts to evaluate PID
#define MOTORS_PWM_STOP  127  //PWM at which motors stops
//#define MOTORS_MinSetPoint  0   //Minimum setpoint for motors (below this value could become unstable or stop in low PWM values)
//#define MOTORS_MaxSetPoint  800  //Maximum setpoint for motors (above it could be to fast for the application) 
//#define MOTORS_MaxDeviation 83.6875 //Maximum deviation allowd (equivalent to 1rad/s)
#define MOTORS_MaxDeviation 13 //Maximum deviation allowed (equivalent to 10%) for Magnetic guidance
#define MOTORS_MaxAngular_Velocity 4.00 //Maximum angular velocity in rad/s
//#define MOTORS_MaxAngular_Velocity 3.14 //Maximum angular velocity in rad/s
#define MOTORS_MaxLinear_Velocity 1.0 //Maximum linear velocity in m/sg
#define MAG_SETPOINT  640 //Setpoint for magnetic sensor

//RFID Tags
#define RFID_ACTION_PINHOOK_UP  1
#define RFID_ACTION_PINHOOK_DWN 2
#define RFID_ACTION_FAST_SPEED  3
#define RFID_ACTION_SLOW_SPEED  4
#define RFID_ACTION_ROTATE180   5
struct tag_and_actions
{
  String  tag;
  byte    action;
  String  description;
};
tag_and_actions RFID_tag [] = { {"1D05", RFID_ACTION_PINHOOK_UP   ,"Pinhook UP" },
                                {"E140", RFID_ACTION_PINHOOK_DWN  ,"Pinhook DW"},
                                {"416C", RFID_ACTION_FAST_SPEED   ,"Fast Speed" },
                                {"1D04", RFID_ACTION_SLOW_SPEED   ,"Slow Speed" },
                                {"48A8", RFID_ACTION_ROTATE180    ,"Rotate 180" }};
tag_and_actions RFID_read; //RFID tag read

#include "pitches.h"          //pitch values for typical notes (buzzer) https://www.arduino.cc/en/Tutorial/BuiltInExamples/toneMelody
int melody[] = {NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4}; // notes in the melody
int noteDurations[] = {4, 8, 8, 4, 4, 4, 4, 4};  // note durations: 4 = quarter note, 8 = eighth note, etc.

//Timer variables for ROS topic publishing
long ROSpub_t2 = 0;
long ROSpub_t1 = 0;
long ROSpub_t_interval = 1000; //Milliseconds between publishes

//Timer variables for Buzzer warnings
long Buzzer_t2 = 0;
long Buzzer_t1 = 0;
long Buzzer_t_interval = 0; 
bool Buzzer_warnings_allowed = false; //Allow or not the buzzer warning

//Timer variables for Magnetic guidance use
long Mag_t1 = 0;
long Mag_t2 = 0;
long Mag_t_interval = PID_Interrupt_ms; //Milliseconds between two PID loops

//Timer variables for RFID use
long RFID_t1 = 0;
long RFID_t2 = 0;
long RFID_t_interval = 500; //Milliseconds between two RFID reads

//Timer variables for external commands check use
long ExtCmd_t1 = 0;
long ExtCmd_t2 = 0;
long ExtCmd_t_interval = 500; //Milliseconds between external commands check

//Timer variables for LCD refresh use
long LCD_t1 = 0;
long LCD_t2 = 0;
long LCD_t_interval = 500; //Milliseconds between LCD refresh
byte LCD_Scr_Num = 0; //LCD Screen to show
byte LCD_Max_Scr_Num = 4;

//For Keyboard
char customKey; //Key from keypad (instant)
char KbdKey; //Key from keyboard

byte LidarCase = 0; // (0:Short case, 1:Long case, 2:Park mode, 3:No detection) 
bool MagGuidance_Active = false; //Indicates if magnetic guidance control is active or not
byte magnetic_mode = 0; //Magnetic band mode 0:Straight, 1:Left, 2:Right

bool RFID_Active = true; //Indicates if RFID control is active or not
byte Lidar_ready = false; //Indicates when Lidar is ready after booting up

long timer1_counter; //For Timer interrupt

bool PinHook_UP = false; // If Pinhook is UP or not

//BEGIN Keypad 
  const byte ROWS = 2; 
  const byte COLS = 1;  
  char hexaKeys[ROWS][COLS] = {'<', '>'};  
  byte rowPins[ROWS] = {PIN_KBD_Dec, PIN_KBD_Inc}; //Pins where keypad is connected
  byte colPins[COLS] = {PIN_KBD_CMN};    //Pins where keypad is connected
  Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS); 
//END Keypad


LCD03 lcd;                                  // Create new LCD03 instance

//BEGIN PID_Control for drive motors
  bool  Motors_active_status = false;       //Indicates if motors are active or not. Changing this value changes the value sent to safety PLC
  bool  set_stop = false;                
  int16_t Motor1_feedback = 0;                //Speed varible measured by encoders (Motor 1)
  int16_t Motor2_feedback = 0;                //Speed varible measured by encoders (Motor 2)
  int16_t Motors_setpoint = 0;                //SetPoint for the Pid Controllers of motors measured in cycles (signal measured by encoders) per every 20ms (time PID interruptions)
  int16_t Motor1_setpoint = 0;                //SetPoint for the Pid Controllers of motor 1 measured in cycles (signal measured by encoders) per every 20ms (time PID interruptions)
  int16_t Motor2_setpoint = 0;                //SetPoint for the Pid Controllers of motor 2 measured in cycles (signal measured by encoders) per every 20ms (time PID interruptions)

  uint8_t PID_output1 = 0;                  //Output from PID algorithm (accumulated error)
  uint8_t PID_output2 = 0;                  //Output from PID algorithm (accumulated error)
  uint8_t Motor1_PWM = MOTORS_PWM_STOP;     //PWM value for motor drive 1
  uint8_t Motor2_PWM = MOTORS_PWM_STOP;     //PWM value for motor drive 2
  float Kp1=0.02, Ki1=0.03, Kd1=0.00, Hz1=1000/PID_Interrupt_ms;    //Specify initial tuning parameters Motor 1. Kp,Ki, Kd [0.00390625 to 255]
  float Kp2=0.02, Ki2=0.02, Kd2=0.00, Hz2=1000/PID_Interrupt_ms;    //Specify initial tuning parameters Motor 2. Kp,Ki, Kd  [0.00390625 to 255]
  float Kpm=0.05, Kim=0.00, Kdm=0.20, Hzm=1000/PID_Interrupt_ms;    //Specify initial tuning parameters Mag guidance (angular velocity). Kp,Ki, Kd  [0.00390625 to 255]
  int output_bits = 8;
  bool output_signed = false;
  FastPID Motor1_PID(Kp1, Ki1, Kd1, Hz1, output_bits, output_signed); //PID for Motor 1 (right)
  FastPID Motor2_PID(Kp2, Ki2, Kd2, Hz2, output_bits, output_signed); //PID for Motor 2 (left)
  FastPID Mag_PID(Kpm, Kim, Kdm, Hzm, output_bits, output_signed);    //PID for magnetic guidance
//END PID_Control

//For velocities
float Global_Linear_Velocity = 0; 
float Global_Angular_Velocity = 0;
float Global_Linear_Velocity_pre = 0;  //previous value
float Global_Angular_Velocity_pre = 0;
int16_t Angular_Velocity_Int = 0; //Temporary variable used in Magnetic guidance


//BEGIN Encoder
  // Change these two numbers to the pins connected to your encoder.
  //   Best Performance: both pins have interrupt capability
  //   Good Performance: only the first pin has interrupt capability
  //   Low Performance:  neither pin has interrupt capability
  Encoder Encoder1(PIN_ENCODER1A, PIN_ENCODER1B);   //Pins where connected encoder. First should have interrupt (Options: 2, 3, 18, 19, 20, 21 in Mega)
  Encoder Encoder2(PIN_ENCODER2A, PIN_ENCODER2B);   //Pins where connected encoder. First should have interrupt (Options: 2, 3, 18, 19, 20, 21 in Mega)
  //   avoid using pins with LEDs attached
//END Encoder


//FUNCTIONS

#define STATUS  2
#define UP      1
#define DOWN    0
#define STOP    3
#define ON      1
#define OFF     0

byte PinHook (byte command) {
  //Allows moving up or down the pinhook and asking for status (up or down)
  switch (command) {
      case STATUS:
        return (PinHook_UP); //1 up, 0 down
        break;
      case UP:
        digitalWrite(PIN_PINHOOK_DIR0, HIGH);
        digitalWrite(PIN_PINHOOK_DIR1, LOW);
        PinHook_UP = true;
        break;
      case DOWN:
        digitalWrite(PIN_PINHOOK_DIR0, LOW);
        digitalWrite(PIN_PINHOOK_DIR1, HIGH);
        PinHook_UP = false;
        break;
      default:
        // if nothing else matches is stop
        digitalWrite(PIN_PINHOOK_DIR0, LOW);
        digitalWrite(PIN_PINHOOK_DIR1, LOW);
      break;
    }
}


byte GetLidarStatus () {
  //Returns status of Lidar in a byte, being b0 the warning field status, b1 the protective field and b2 the dirty signal
  byte LidarStatus=0;
  if (not digitalRead(PIN_LIDAR_WARNING)) bitSet(LidarStatus,0); 
  if (not digitalRead(PIN_LIDAR_PROTECT)) bitSet(LidarStatus,1); 
  if (not digitalRead(PIN_LIDAR_DIRTY)) bitSet(LidarStatus,2); 
  return(LidarStatus); 
}

void ResetLidar (){
  //Sends a pulse for resetting Lidar
  digitalWrite(PIN_LIDAR_RESET, HIGH);
  delay(10);
  digitalWrite(PIN_LIDAR_RESET, LOW);
}

void SetMotorsVelocity(float Linear_Velocity, float Angular_Velocity) {
  //Set Motors_setpoint and Motors_deviation based on the inputs: Speed in meters/second (-MOTORS_MaxLinear_Velocity to MOTORS_MaxLinear_Velocity) 
  //and Rotation in radians/second (-MOTORS_MaxAngular_Velocity to MOTORS_MaxAngular_Velocity)

  int16_t Motors_deviation = 0;             //Direction of AMR. negative turns to the left, positive turns to the right. Same units as Motors_setpoint

  //Control limits
  if (Linear_Velocity  < -MOTORS_MaxLinear_Velocity) Linear_Velocity  = -MOTORS_MaxLinear_Velocity;
  if (Linear_Velocity  >  MOTORS_MaxLinear_Velocity) Linear_Velocity  =  MOTORS_MaxLinear_Velocity;
  if (Angular_Velocity < -MOTORS_MaxAngular_Velocity) Angular_Velocity = -MOTORS_MaxAngular_Velocity;
  if (Angular_Velocity >  MOTORS_MaxAngular_Velocity) Angular_Velocity =  MOTORS_MaxAngular_Velocity;

 // if ((Linear_Velocity > -0.15) and (Linear_Velocity < 0.15)) Linear_Velocity = 0.0; //range between -0.099 and 0.099 is considered as 0

  Motors_setpoint = Linear_Velocity * PID_Interrupt_ms * 12.875;
  /*  How is this formula defined:
   *  
   *  Initial constants:
   *  -encoder CPR: 200CPR (cycles per revolution in motor axel)
   *  -gear ratio of motors: 103/4
   *  -CPR_on_wheel (Resulting cycles per revolution in wheel axel): encoder CPR * gear ratio = 5150 cycles per revolution of wheel
   *  -Diameter of wheel: 130mm = 13cm = 0.13m
   *  -Perimeter_of_wheels_in_meters: Pi * Diameter = 0.4m
   *  -1 minute = 60000ms
   *  
   *  Motors_setpoint is measured in cycles of encoder per measuring time (PID_Interrupt_ms)
   *  
   *  
   *  Speed_meters_per_minute = Linear_Velocity(m/s) * 60 seconds
   *  revolutions_per_minute = Speed_meters_per_minute / Perimeter_of_wheels_in_meters = Linear_Velocity * 60 / 0.4 = Linear_Velocity * 150
   *  revolutions_per_ms = revolutions_per_minute / 60000 = (Linear_Velocity * 150) / 60000 = Linear_Velocity * 0,0025
   *  
   *  Motors_setpoint = revolutions_per_ms * PID_Interrupt_ms * CPR_on_wheel = (Linear_Velocity * 0,0025) * PID_Interrupt_ms * 5150 = Linear_Velocity * PID_Interrupt_ms * 12,875
   */

  Motors_deviation = Angular_Velocity * PID_Interrupt_ms * 0.836875; 
  /*  
   *  linear speed = angular speed x radius of the rotation
   *  v = ωr
   *  v = linear speed (m/s)
   *  ω = angular speed (radians/s)
   *  r = radius of the rotation (m)
   *  
   *  Then:
   *  Distance between wheels (L): 325mm = 32,5cm = 0.325m
   *  Radius_of_rotation (L/2) = 0.065m, because the centre of rotation (T) is in the middle of the robot.
   *  Linear Speed on the wheel (v) = Angular_Velocity (ωp) * Radius_of_rotation (L/2)
   *  
   *  This Linear Speed should be applied in positive to one wheel and in negative to the other (differential drive) 
   *  not altering the linear velocity of the robot (Linear_Velocity) defined in the previous page. 
   *  To apply it properly we need to add and subtract to Motors_Setpoint but in the same units, 
   *  so we need to apply to the Linear Speed on the wheel the same conversion we did previously 
   *  for Linear Velocity getting Motors_deviation:
   *  
   *  Motors_deviation = Linear Speed on the wheel (m/s) * PID_Interrupt_ms * 12,875 =
   *  (Angular_Velocity(r/s) * Radius_of_rotation) * PID_Interrupt_ms * 12,875
   *  (Angular_Velocity(r/s) * 0.065m) * PID_Interrupt_ms * 12,875 =
   *  Angular_Velocity * PID_Interrupt_ms * 0,836875
   *  
   *  Then we add and subtract to Motors_setpoint (VT) of each wheel getting Motor1_Setpoint (VR) and Motor2_Setpoint (VL)
   */

  Motor1_setpoint = Motors_setpoint - Motors_deviation;
  Motor2_setpoint = Motors_setpoint + Motors_deviation;

}

  
void SetLidarCase(byte cases) {
  //Set Lidar Cases (0:Short case, 1:Long case, 2:Park mode, 3:No detection) 
  digitalWrite(PIN_LIDAR_CASEBIT0, bitRead(cases, 0));
  digitalWrite(PIN_LIDAR_CASEBIT1, bitRead(cases, 1));
}





void LCDShow (byte screen_num) {
  //Show specific LCD screen
  switch (screen_num) {
    
      case 0: //Version, time on
        lcd.clear();                          // Clear the LCD
        lcd.backlight();                          // Turn on the backlight  
        // Line1:
        lcd.print("AMR Board v");              
        lcd.print(VERSION);              
        lcd.print("\rUP Time: ");          //DTIME DD HH:MM:SS.ss  00 04:31:17.01    
        uptime::calculateUptime();
  
        lcd.print(uptime::getDays());
        lcd.print("d ");                
        lcd.print(uptime::getHours());
        lcd.print(":");               
        lcd.print(uptime::getMinutes());
        lcd.print(":");              
        lcd.print(uptime::getSeconds());
        lcd.print("\r");
        break;
        
      case 1: //Motors related values: PID gains
        lcd.clear();                          // Clear the LCD
        lcd.backlight();                          // Turn on the backlight  
        // Line0:
        lcd.print("MOTOR1. PID Gains \r");
        // Line1:
        lcd.print("P:");              
        lcd.print(Kp1);              
        lcd.print(" I:");              
        lcd.print(Ki1);              
        lcd.print(" D:");              
        lcd.print(Kd1);              
        // Line2:
        lcd.print("MOTOR2. PID Gains \r");
        // Line3:
        lcd.print("P:");              
        lcd.print(Kp2);              
        lcd.print(" I:");              
        lcd.print(Ki2);              
        lcd.print(" D:");              
        lcd.print(Kd2);              
                    
        break;
        
      case 2: //Motors related values: Set Point, Encoders feedback, Pwm generated
        lcd.clear();                          // Clear the LCD
        lcd.backlight();                          // Turn on the backlight  
        // Line0:
        lcd.print("MOTOR1. ");
        if (Motors_active_status) lcd.print("ON\r");
        else lcd.print("OFF\r");
       // Line1:
        lcd.print("S:");              
        lcd.print(int(Motor1_setpoint));              
        lcd.print(" E:");              
        lcd.print(int(Motor1_feedback));  
        lcd.print(" P:");              
        lcd.print(int(Motor1_PWM));  
        lcd.print("\r");
        // Line2:
        lcd.print("MOTOR2. ");
        if (Motors_active_status) lcd.print("ON\r");
        else lcd.print("OFF\r");
        // Line3:
        lcd.print("S:");              
        lcd.print(int(Motor2_setpoint));              
        lcd.print(" E:");              
        lcd.print(int(Motor2_feedback));  
        lcd.print(" P:");              
        lcd.print(int(Motor2_PWM));  
        break;

      case 3: //Safety related values: Safety status, Laser Case, Warning, Protective, Laser dirty, rearm button
        lcd.clear();                          // Clear the LCD
        lcd.backlight();                          // Turn on the backlight  
        // Line0:
        lcd.print("SAFETY ");
        if (digitalRead(PIN_SAFETYSTOP)) lcd.print("STOP\r");
        else lcd.print("OK\r");
        // Line1:
        lcd.print("Laser Case:");              
        lcd.print(LidarCase);              
        lcd.print("\r");
        // Line2:
        lcd.print("Warn:");              
        lcd.print(not digitalRead(PIN_LIDAR_WARNING));  
        lcd.print(" Prot:");              
        lcd.print(not digitalRead(PIN_LIDAR_PROTECT));  
        lcd.print(" Dirt:");              
        lcd.print(not digitalRead(PIN_LIDAR_DIRTY));  
        // Line3:
        lcd.print("Push Button:");              
        lcd.print(digitalRead(PIN_BUTTON));  
        lcd.print("\r");
        break;

      case 4: //Magnetic guidance related values: Band detection, Mag.sensor mode, Mag. value, RFID tag and action
        lcd.clear();                          // Clear the LCD
        lcd.backlight();                          // Turn on the backlight  
        // Line0:
        lcd.print("MAG. GUIDANCE ");
        if (MagGuidance_Active) lcd.print("ON\r");
        else lcd.print("OFF\r");
        // Line1:
        lcd.print("Gate:");              
        if (not digitalRead(PIN_MAG_GATE)) lcd.print("OK");   
        else lcd.print("NO"); 
        lcd.print("  Mode:");              
        lcd.print(magnetic_mode);                    
        lcd.print("\r");
        // Line2:
        lcd.print("Value:");              
        lcd.print(analogRead(PIN_MAG_DEV));  
        lcd.print("\r");
        // Line3:
        lcd.print("RFID:");              
        lcd.print(RFID_read.tag);
        lcd.print(" ");
        lcd.print(RFID_read.description);
        break;
        
      default:
        lcd.clear();                          // Clear the LCD
        lcd.backlight();                          // Turn on the backlight  
      break;
  }
}


void HMI_update(){ 
  //Check keys pressed and update LCD screen (Human-Machine interface) every LCD_t_interval msec

 //LCD display timer
  LCD_t2 = millis();
  if (LCD_t2 - LCD_t1 > LCD_t_interval) {
    LCD_t1 = LCD_t2;
    //Move from one LCD screen to other based on keypad push
    if ((KbdKey == '<') or (KbdKey == '>'))                    tone(PIN_BUZZER, 800, 10);             // beep to confirm key pushed
    if ((KbdKey == '<') and (LCD_Scr_Num > 0))                 LCD_Scr_Num = LCD_Scr_Num-1;
    if ((KbdKey == '>') and (LCD_Scr_Num < LCD_Max_Scr_Num))   LCD_Scr_Num = LCD_Scr_Num+1;
    KbdKey = '.';
    LCDShow (LCD_Scr_Num);
  }
}


void SetMagMode(byte mag_mode) {
  //Set Magnetic band mode (0:﻿Straight, 1:Left, 2:Right) 
  digitalWrite(PIN_MAG_SEL1, bitRead(mag_mode, 0));
  digitalWrite(PIN_MAG_SEL2, bitRead(mag_mode, 1));
}

void MagGuidance() {
  //Magnetic guidance process
  
  int   Mag_feedback;
  
  //Only executes the function if timer is over
  Mag_t2 = millis();
  if (Mag_t2 - Mag_t1 > Mag_t_interval) {
    Mag_t1 = Mag_t2;
    SetMagMode(magnetic_mode);  //Configure Mag reader 
    
    //Check that mag band is available. If not, stop 
    if(digitalRead(PIN_MAG_GATE)) { //No magnetic band detected so we stop
      set_stop = true;  //Internal request to stop
      Motors_active_status = false; //Force stop
    }
    else {  //Magnetic band detected. Release internal request to stop
      set_stop = false;
    }
  
    //PID_Control
    if (Motors_active_status) {       
      Mag_feedback = analogRead(PIN_MAG_DEV);
      Angular_Velocity_Int = Angular_Velocity_Int - Mag_PID.step(MAG_SETPOINT, Mag_feedback);
      //Limits control
      if (Angular_Velocity_Int < -MOTORS_MaxAngular_Velocity*100) Angular_Velocity_Int = -MOTORS_MaxAngular_Velocity*100;
      if (Angular_Velocity_Int >  MOTORS_MaxAngular_Velocity*100) Angular_Velocity_Int =  MOTORS_MaxAngular_Velocity*100;
      Global_Angular_Velocity = Angular_Velocity_Int/100; //PID use only integers, but we need float values. It was set a range of -MOTORS_MaxAngular_Velocity to MOTORS_MaxAngular_Velocity but multiplied by 100
//Serial.print(MAG_SETPOINT); 
//Serial.print(","); 
//Serial.print(Mag_feedback); 
//Serial.print(","); 
//Serial.println(Angular_Velocity_Int); 
    }
  }
  else Angular_Velocity_Int = 0;  //Temporary variableused in the pid is reinitialized
}


void RFID_Actions() {
  //RFID check and actions
  
  //Only executes the function if timer is over
  RFID_t2 = millis();
  if (RFID_t2 - RFID_t1 > RFID_t_interval) {
    RFID_t1 = RFID_t2;

    //Check if RFID interrogator detects a tag
    int bytes_to_read = Serial1.available();  //Bytes available in the buffer
    RFID_read.tag="";                            //We reset the RFID value read
    RFID_read.action=0;                          //We reset the RFID value read
    RFID_read.description="";                    //We reset the RFID value read
    byte RFID_action_associated = 0;          //We reset any action associated
    if (bytes_to_read > 0) {                  //Read the incoming data
      char incoming_data [bytes_to_read]; 
      Serial1.readBytes(incoming_data, bytes_to_read);
  
      for (byte index = 0; index< (sizeof(RFID_tag)/sizeof(RFID_tag[0])); index++) {
        if (String(incoming_data).indexOf(RFID_tag[index].tag) >= 0) {
          RFID_read.tag           = RFID_tag[index].tag;
          RFID_read.action        = RFID_tag[index].action;
          RFID_read.description   = RFID_tag[index].description;
        }
      }

      //Execute actions depending on the tag read
      switch (RFID_read.action) {
        case RFID_ACTION_PINHOOK_UP:
        PinHook(UP);
        break; 
  
        case RFID_ACTION_PINHOOK_DWN:
        PinHook(DOWN);
        break; 
  
        case RFID_ACTION_FAST_SPEED:
        Global_Linear_Velocity = 0.5;
        break; 
        
        case RFID_ACTION_SLOW_SPEED:
        Global_Linear_Velocity = 0.2;
        break; 
  
        case RFID_ACTION_ROTATE180:
        RFID_Active = false;                                                  //Stop RFID to prevent reading the same command during the process
        MagGuidance_Active = false;                                           //Stop Magnetic Guidance
        Global_Linear_Velocity_pre = Global_Linear_Velocity;                  //Store the linear velocity to set it back again later
        Global_Linear_Velocity = 0.0;                                         //Stop vehicle
        SetMotorsVelocity(Global_Linear_Velocity, Global_Angular_Velocity);
        Global_Angular_Velocity = MOTORS_MaxAngular_Velocity;                 //Vehicle rotates counter clockwise m
        SetMotorsVelocity(Global_Linear_Velocity, Global_Angular_Velocity);
        while (not digitalRead(PIN_MAG_GATE));                                //magnetic band is still detected, so we loop until the AMR rotates enough to loose the mag band
        delay(1000);
        while (digitalRead(PIN_MAG_GATE));                                    //magnetic band is still notdetected, so we loop until the AMR rotates enough to find the mag band again
        delay(500);
        while (analogRead(PIN_MAG_DEV) < (MAG_SETPOINT-MAG_SETPOINT/10) or analogRead(PIN_MAG_DEV) > (MAG_SETPOINT+MAG_SETPOINT/10)); //continue rotating until the AMR is centered over the band (-10% + 10% from centre)
        Global_Linear_Velocity = 0.0;                                         //Stop vehicle
        Global_Angular_Velocity = 0.0;
        SetMotorsVelocity(Global_Linear_Velocity, Global_Angular_Velocity);
        while (Serial1.available()) Serial1.read();                           //Clear RFID buffer
        delay(20);
        RFID_Active = true;                                                   //reactivate RFID
        MagGuidance_Active = true;                                            //Reactivate Magnetic Guidance
        Global_Linear_Velocity = Global_Linear_Velocity_pre;
        SetMotorsVelocity(Global_Linear_Velocity, Global_Angular_Velocity);   //Set again previous speed
        break; 
      }
    }  
  }
}



ISR(TIMER1_OVF_vect)        // interrupt service routine 
//Under this interrupt it is done the reading of encoders and PID for both traction motors
{
  TCNT1 = timer1_counter;   // preload timer
  //Encoders
  Motor1_feedback = Encoder1.read();
  Motor2_feedback = Encoder2.read();
  Encoder1.write(0);                //Reset encoder reading
  Encoder2.write(0);                //Reset encoder reading


  if (Motors_active_status) {   
 
    //PID_Control Motor1 -------------------------------------------------------------------------
    PID_output1 = PID_output1 + Motor1_PID.step(Motor1_setpoint, Motor1_feedback);
    Motor1_PWM = MOTORS_PWM_STOP + PID_output1;
    
    //PID_Control Motor2 -------------------------------------------------------------------------
    PID_output2 = PID_output2 + Motor2_PID.step(Motor2_setpoint, Motor2_feedback);
    Motor2_PWM = MOTORS_PWM_STOP + PID_output2;


    //Output Limits filtering Motor1 -------------------------------------------------------------------------
    if ((Motor1_setpoint >= 0) and (Motor1_PWM < MOTORS_PWM_STOP)) {
      Motor1_PWM = MOTORS_PWM_STOP; //Limit the value of PWM not to go out of forward range
      PID_output1 = 0; //Prevents accumulated values
    }
    if ((Motor1_setpoint <= 0) and (Motor1_PWM > MOTORS_PWM_STOP)) {
      Motor1_PWM = MOTORS_PWM_STOP; //Limit the value of PWM not to go out of reverse range
      PID_output1 = 0; //Prevents accumulated values
    }
    
    //Output Limits filtering Motor2 -------------------------------------------------------------------------
    if ((Motor2_setpoint >= 0) and (Motor2_PWM < MOTORS_PWM_STOP)) {
      Motor2_PWM = MOTORS_PWM_STOP; //Limit the value of PWM not to go out of forward range
      PID_output2 = 0; //Prevents accumulated values
    }
    if ((Motor2_setpoint <= 0) and (Motor2_PWM > MOTORS_PWM_STOP)) {
      Motor2_PWM = MOTORS_PWM_STOP; //Limit the value of PWM not to go out of reverse range
      PID_output2 = 0; //Prevents accumulated values
    }

  }

  analogWrite(PIN_MOTOR1_PWM, Motor1_PWM);
  analogWrite(PIN_MOTOR2_PWM, Motor2_PWM);

}


void TestMotors() {
  //Test motors for debugging
  int delaytime = 200;
  float minval_lin = 0;
  float maxval_lin = 0.0;
  float increment_lin = 0.01;
  float minval_ang = 0;
  float maxval_ang = MOTORS_MaxAngular_Velocity;
  float increment_ang = 0.1;
  
  for (float speedtest = minval_lin; speedtest<=maxval_lin; speedtest=speedtest+increment_lin) {
    for (float angtest = minval_ang; angtest<=maxval_ang; angtest=angtest+increment_ang) {
      SetMotorsVelocity(speedtest, angtest);
      delay (delaytime);    
    }
  }
  for (float speedtest = maxval_lin-increment_lin; speedtest>=(minval_lin+increment_lin); speedtest=speedtest-increment_lin) {
    SetMotorsVelocity(speedtest, 0);
    delay (delaytime);    
  }
}





        


void ExtCmds_Actions() {
  //External commands from serial

  //Only executes the function if timer is over
  ExtCmd_t2 = millis();
  if (ExtCmd_t2 - ExtCmd_t1 > ExtCmd_t_interval) {
    ExtCmd_t1 = ExtCmd_t2;

    if (Serial.available() > 0) {
      // read the incoming byte:
      byte incomingCmd = Serial.read();
      
      switch (incomingCmd) {
    
        case 's': //Stop by direct action the safety PLC
        set_stop = true;
        Global_Linear_Velocity = 0;
        Global_Angular_Velocity = 0;
        break;
  
        case '0': //Stop but by speed control
        Global_Linear_Velocity = 0;
        Global_Angular_Velocity = 0;
        set_stop = false;
        break;
  
        case 'g': //Go
        set_stop = false;
        break;
  
  
        case 'q': //Forward speed increase by 0.1m/s
        Global_Linear_Velocity = Global_Linear_Velocity + 0.1;
        set_stop = false;
        break;
  
        case 'a': //Backwards speed decrease by -0.1m/s
        Global_Linear_Velocity = Global_Linear_Velocity - 0.1;
        set_stop = false;
        break;
  
        case 'o': //Decrease angular speed by 0.1r/s (tend to turn to the left)
        Global_Angular_Velocity = Global_Angular_Velocity - 0.1;
        set_stop = false;
        break;
  
        case 'p': //Increase angular speed by 0.1r/s (tend to turn to the right)
        Global_Angular_Velocity = Global_Angular_Velocity + 0.1;
        set_stop = false;
        break;
  
        case '2': //Set fwd speed to 0.2m/s
        Global_Linear_Velocity = 0.2;
        set_stop = false;
        break;
  
        case '3': //Set fwd speed to 0.3m/s
        Global_Linear_Velocity = 0.3;
        set_stop = false;
        break;
  
        case '4': //Set fwd speed to 0.4m/s
        Global_Linear_Velocity = 0.4;
        set_stop = false;
        break;
  
        case '5': //Set fwd speed to 0.5m/s
        Global_Linear_Velocity = 0.5;
        set_stop = false;
        break;
  
        case 'm': //Magnetic guidance activation
        MagGuidance_Active = true;
        Global_Linear_Velocity = 0.2;
        break;
        
        case 'n': //Magnetic guidance deactivation
        MagGuidance_Active = false;
        Global_Linear_Velocity = 0.0;
        break;

        case 'h': //Pin down
        PinHook(DOWN);  //Pinhook down 
        PinHook_UP = false;
        break;

        case 'y': //Pin up
        PinHook(UP);  //Pinhook down 
        PinHook_UP = true;
        break;

        case 't': //Test
        SetMotorsVelocity(0.0, 0);
        set_stop = false;
        TestMotors();
        Global_Linear_Velocity = 0.0;
        break;
  
        default:
          // if nothing else matches, do the default
          // default is optional
        break;
      }
    }
  }
}


void Buzzer_warnings(){ 
  //Check if there are warning conditions and inform users via buzzer
  //Implementation of bip-bip sounds when 
  //-Lidar Protective field compromised (2Hz bip) or 
  //-Warning field compromised (1Hz bip) or
  //-Safety Stop button (local or remote) active. 1 bip every 4sec

  if (Lidar_ready) {
    if      (not digitalRead(PIN_LIDAR_PROTECT))  Buzzer_t_interval = 500;    // Protective field compromised. 2Hz timer
    else if (not digitalRead(PIN_LIDAR_WARNING))  Buzzer_t_interval = 1000;   // Warning field compromised. 1Hz timer
    else if (digitalRead(PIN_SAFETYSTOP))         Buzzer_t_interval = 4000;   // Safety Stop button (local or remote) active. 1 bit every 4 seconds
    else                                          Buzzer_t_interval = 0;
  }
  
  if (Buzzer_t_interval > 0) {
    //Buzzer warnings timer
    Buzzer_t2 = millis();
    if (Buzzer_t2 - Buzzer_t1 > Buzzer_t_interval) {
      Buzzer_t1 = Buzzer_t2;
      tone(PIN_BUZZER, 800, 100);             // beep 
    }
  }
}


// #####################################################
//                        SETUP
// #####################################################


void setup() {
//  TCCR4B = TCCR4B & B11111000 | B00000010;  // for PWM frequency of 3921.16 Hz for D6, D7 & D8. Source: https://www.etechnophiles.com/how-to-change-pwm-frequency-of-arduino-mega/
  pinMode(PIN_SAFETYSTOP, INPUT_PULLUP);
  pinMode(PIN_MOTORS_ACTIVE, OUTPUT);
  digitalWrite(PIN_MOTORS_ACTIVE, false);    //Motor Drives stopped until we have a PWM signal available

  pinMode(PIN_PINHOOK_DIR0, OUTPUT);
  pinMode(PIN_PINHOOK_DIR1, OUTPUT);
  
  pinMode(PIN_LIDAR_RESET, OUTPUT);
  pinMode(PIN_LIDAR_WARNING, INPUT_PULLUP);
  pinMode(PIN_LIDAR_PROTECT, INPUT_PULLUP);
  pinMode(PIN_LIDAR_DIRTY, INPUT_PULLUP);
  pinMode(PIN_LIDAR_CASEBIT0, OUTPUT);
  pinMode(PIN_LIDAR_CASEBIT1, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_BUTTON_LED, OUTPUT);
  pinMode(PIN_MAG_SEL1, OUTPUT);
  pinMode(PIN_MAG_SEL2, OUTPUT);
  pinMode(PIN_MAG_GATE, INPUT_PULLUP);
  
  
  
  Serial.begin(9600);                     //for debugging feedback (USB connection to PC for programming and debugging)
  Serial1.begin(57600);             //For RFID interrogator


  //ROS integration
  nh.initNode();  //<------This part has been commented because somehow the serial to the debugging PC start working at 57600 instead of at 9.600
  nh.advertise(pub_velocity_lin);
  nh.advertise(pub_velocity_ang);
  nh.advertise(pub_safety_status);
  nh.advertise(pub_rfid_tag);

//  PinHook(UP);  //Pinhook down if it was up after a reboot
//  delay (4000);
  PinHook(DOWN);  //Pinhook down if it was up after a reboot
  PinHook_UP = false;

  
  lcd.begin(20, 4);                         // Initialise a 20x4 LCD 
  LCDShow (0);

  //BEGIN Initial audio tones
    // iterate over the notes of the melody:
    for (int thisNote = 0; thisNote < 8; thisNote++) {
  
      // to calculate the note duration, take one second divided by the note type.
      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
      int noteDuration = 1000 / noteDurations[thisNote];
      tone(PIN_BUZZER, melody[thisNote], noteDuration);
  
      // to distinguish the notes, set a minimum time between them.
      // the note's duration + 30% seems to work well:
      int pauseBetweenNotes = noteDuration * 1.30;
      delay(pauseBetweenNotes);
      // stop the tone playing:
   //   noTone(PIN_BUZZER);
    }
  //END Initial audio tones


  tone(PIN_BUZZER, 800, 100);             // beep to mention everything is ok to start

  //PID and motors
  Motor1_PID.setOutputRange(-MOTORS_PWM_STOP, MOTORS_PWM_STOP); //Limits of the PID (Later we will add MOTORS_PWM_STOP to the output)
  Motor2_PID.setOutputRange(-MOTORS_PWM_STOP, MOTORS_PWM_STOP); //Limits of the PID (Later we will add MOTORS_PWM_STOP to the output)
  Mag_PID.setOutputRange(-MOTORS_MaxAngular_Velocity*100, MOTORS_MaxAngular_Velocity*100); //Limits of the PID output (we multiply by 100 because this PID is Int16, so we would loose decimals

  //Interrupt configuration for PID control
  // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;

  // Set timer1_counter to the correct value for our interrupt interval
  timer1_counter = 65536 - 62.5*PID_Interrupt_ms; //65536 - (16000000/256) * (PID_Interrupt_ms/1000)   (PID_Interrupt_ms ms)

  
  TCNT1 = timer1_counter;   // preload timer
  TCCR1B |= (1 << CS12);    // 256 prescaler 
  TIMSK1 |= (1 << TOIE1);   // enable timer overflow interrupt
  interrupts();             // enable all interrupts
  //END Interrupts
  
  //LIDAR
    LidarCase = 0; // (0:Short case, 1:Long case, 2:Park mode, 3:No detection) 
    SetLidarCase(LidarCase);  

SetMotorsVelocity(0.0, 0);

MagGuidance_Active = true; //Activate by default magnetic guidance
Global_Linear_Velocity = 0.2;

//Serial.println("LLC started");
}

// #####################################################
//                        LOOP
// #####################################################

 
void loop() { 

  //ROS integration
  ROSpub_t2 = millis();
  if (ROSpub_t2 - ROSpub_t1 > ROSpub_t_interval) {
    ROSpub_t1 = ROSpub_t2;
    
    //Velocity messages
    velocity_lin_msg.data = Global_Linear_Velocity;
    velocity_ang_msg.data = Global_Angular_Velocity;
    pub_velocity_lin.publish( &velocity_lin_msg );
    pub_velocity_ang.publish( &velocity_ang_msg );

    //Safety message
    safety_status_msg.data = 0;
    if (bitRead(LidarCase, 0))              bitSet(safety_status_msg.data,0); //Lidar case
    if (bitRead(LidarCase, 1))              bitSet(safety_status_msg.data,1); //Lidar case
    if (not digitalRead(PIN_LIDAR_WARNING)) bitSet(safety_status_msg.data,2); //Lidar warning
    if (not digitalRead(PIN_LIDAR_PROTECT)) bitSet(safety_status_msg.data,3); //Lidar protective
    if (not digitalRead(PIN_LIDAR_DIRTY))   bitSet(safety_status_msg.data,4); //Lidar dirty
    if (digitalRead(PIN_SAFETYSTOP))        bitSet(safety_status_msg.data,5); //Safety stop (from Safety PLC)
    if (digitalRead(PIN_BUTTON))            bitSet(safety_status_msg.data,6); //Button pressed
    pub_safety_status.publish( &safety_status_msg );
    
    //RFID message
    strcpy(rfidcode, RFID_read.tag.c_str()); // copying the contents of the string to char array
    rfid_tag_msg.data = rfidcode; //Last RFID value read
    pub_rfid_tag.publish( &rfid_tag_msg );
    
    nh.spinOnce();
  }
  

  //Check if any key from membrane keypad is pressed
  customKey = customKeypad.getKey();  
  if (customKey) KbdKey = customKey;  

  //After booting, the Lidar Scanner needs 20 seconds to be ready
  if (not Lidar_ready) {
    uptime::calculateUptime();
    if (uptime::getSeconds() > 20) Lidar_ready = true;
  }
  
  if (Buzzer_warnings_allowed) Buzzer_warnings(); //Check if there are warning conditions and inform users via buzzer

  HMI_update(); //Check keys pressed and update LCD screen (Human-Machine interface) every LCD_t_interval msec
  
  if (MagGuidance_Active) MagGuidance();   //Mag Guidance

  if (RFID_Active) RFID_Actions(); //Check if RFID interrogator detects any RFID tag

  //ExtCmds_Actions(); //Checks if external commands comming via serial <<<<----- commented as it interfere with Rosserial, as flushes the same serial buffer used to keep communication between HLC and LLC

  //General Purpose button at the back of the AMR
  if (digitalRead(PIN_BUTTON)==true) digitalWrite(PIN_BUTTON_LED, HIGH); //If button is pressed, switch on its led
  else                               digitalWrite(PIN_BUTTON_LED, LOW);  //Otherwise, switch it off

  //Motors STOP conditions
  if (digitalRead(PIN_SAFETYSTOP) or set_stop) {                 //Safety Stop active or set_stop activated  (Drives are off. We need to stop PIDs to prevent overshooting)
    Motors_active_status = false; //Condition to stop motors
    Motor1_PID.clear();
    Motor2_PID.clear();
    Motor1_PWM = MOTORS_PWM_STOP;
    Motor2_PWM = MOTORS_PWM_STOP;
    PID_output1 = 0; //Prevents accumulated values
    PID_output2 = 0; //Prevents accumulated values
    delay(100);
    digitalWrite(PIN_MOTORS_ACTIVE, false);   //Stop motors signal 
  } 
  else if ( Motors_active_status == false) { //Comes from a stop condition but no longer should be stopped
    digitalWrite(PIN_MOTORS_ACTIVE, true);   //Activate motors signal  
    Motors_active_status = true; //Condition to keep motors on    
  }

  //Motors speed update
  if((Global_Linear_Velocity != Global_Linear_Velocity_pre) or (Global_Angular_Velocity != Global_Angular_Velocity_pre)) {
    SetMotorsVelocity(Global_Linear_Velocity, Global_Angular_Velocity);
    Global_Linear_Velocity_pre = Global_Linear_Velocity;
    Global_Angular_Velocity_pre = Global_Angular_Velocity; 
  }


}
