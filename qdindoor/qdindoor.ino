/* ********************************************************************** */
/*                    (ArduIMU Quadcopter DRONE)                          */
/*          INDOOR TEST CODE TO WORK WITH IR DISTANCE SENSORS             */
/*                                                                        */
/* Code based on ArduIMU DCM code from Diydrones.com                      */
/* Date : 10-05-2010                                                      */
/* Author : Jose Julio                                                    */
/* Version : 1.29 (mini indoor version)                                   */
/* Hardware : ArduIMU+ v2 flat                                            */
/* This version supports the Magnetometer   [Optional]                    */
/*   with the magnetometer we eliminate the yaw drift                     */
/* Added Sonar support por altitude control [Optional]                    */
/* Prepared for external Arduino pro mini with IR distance sensors        */
/*   to do obstacle avoidance routine (indoor flight)                     */
/* Added Altitude Hold using a sonar module                               */
/* Added new Automic Flight mode [beta]          	                  */
/*   This mode performs this tasks :					  */
/*      - Automatic takeoff						  */
/*      - Ascend (up to a predefined height)                              */
/*      - Obstacle avoiding (during a predefined time)                    */
/*      - Automatic descend						  */
/*      - Landing                    					  */
/* ********************************************************************** */

#include <inttypes.h>
#include <math.h>
#include <Wire.h>   // For magnetometer readings

/* ***************************************************************************** */
/*  CONFIGURATION PART                                                           */
/* ***************************************************************************** */

#define RADIO_TEST_MODE 0   // 0:Normal  1:Radio Test mode (to test radio channels)

#define MAGNETOMETER 1  // 0 : No magnetometer    1: Magnetometer

//Adjust this parameter for your lattitude (for GPS use)
#define GEOG_CORRECTION_FACTOR 0.87 // cos(lattitude) 

// QuadCopter Attitude control PID GAINS
#define KP_QUAD_ROLL 2.0  //1.4
#define KD_QUAD_ROLL 0.4  //0.33
#define KI_QUAD_ROLL 0.6   //0.32
#define KP_QUAD_PITCH 2.0 //1.75 //1.8 // 2.2   //1.75
#define KD_QUAD_PITCH 0.4 //0.4 //0.42 // 0.54  //0.45
#define KI_QUAD_PITCH 0.6 //0.42 // 0.45  //0.5
#define KP_QUAD_YAW 3.6 //3.4// 4.6  //3.2 //2.6
#define KD_QUAD_YAW 0.85 //0.8// 0.7  //0.8 //0.4
#define KI_QUAD_YAW 0.15 // 0.2  //0.15

#define KD_QUAD_COMMAND_PART 0.0  //13 // for special KD implementation (in two parts). Higher values makes the quadcopter more responsive to user inputs

// Range Finder (RF) Position control PID GAINS
#define KP_RF_ROLL 0.09
#define KD_RF_ROLL 0.06
#define KI_RF_ROLL 0.025
#define KP_RF_PITCH 0.09
#define KD_RF_PITCH 0.06
#define KI_RF_PITCH 0.025

#define RF_MAX_ANGLE 5  // Maximun command roll and pitch angle from position control

// Altitude control PID GAINS
#define KP_ALTITUDE 0.9
#define KD_ALTITUDE 0.5
#define KI_ALTITUDE 0.25

// The IMU should be correctly adjusted : Gyro Gains and also initial IMU offsets:
// We have to take this values with the IMU flat (0� roll, 0�pitch)
#define acc_offset_x 508 
#define acc_offset_y 504
#define acc_offset_z 501         // We need to rotate the IMU exactly 90� to take this value  
#define gyro_offset_roll 370  
#define gyro_offset_pitch 373
#define gyro_offset_yaw 380

// We need to now the number of channels to read from radio
// If this value is not well adjusted you will have bad radio readings... (you can check the radio at the begining of the setup process)
#define MAX_CHANNELS    7       // Number of radio channels to read

#define MIN_THROTTLE 1037       // Throttle pulse width at minimun...
#define MAX_THROTTLE 1850
#define HOVER_THROTTLE 1610     // tested throttle value at hover point

#define CHANN_CENTER 1500

#define SPEKTRUM 1  // 1 for Spektrum radio (This is because the radio channels order)

/* *************************************************** */
/*           END OF CONFIGURATION PART                 */
/* *************************************************** */

// ADC : Voltage reference 3.3v / 10bits(1024 steps) => 3.22mV/ADC step
// ADXL335 Sensitivity(from datasheet) => 330mV/g, 3.22mV/ADC step => 330/3.22 = 102.48
// Tested value : 101
#define GRAVITY 101 //this equivalent to 1G in the raw data coming from the accelerometer 
#define Accel_Scale(x) x*(GRAVITY/9.81)//Scaling the raw data of the accel to actual acceleration in meters for seconds square

#define ToRad(x) (x*0.01745329252)  // *pi/180
#define ToDeg(x) (x*57.2957795131)  // *180/pi

// LPR530 & LY530 Sensitivity (from datasheet) => 3.33mV/º/s, 3.22mV/ADC step => 1.03
// Tested values : 0.96,0.96,0.94
#define Gyro_Gain_X 0.92 //X axis Gyro gain
#define Gyro_Gain_Y 0.92 //Y axis Gyro gain
#define Gyro_Gain_Z 0.94 //Z axis Gyro gain
#define Gyro_Scaled_X(x) x*ToRad(Gyro_Gain_X) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) x*ToRad(Gyro_Gain_Y) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) x*ToRad(Gyro_Gain_Z) //Return the scaled ADC raw data of the gyro in radians for second

#define Kp_ROLLPITCH 0.008  //0.0125 //0.010 // Pitch&Roll Proportional Gain
#define Ki_ROLLPITCH 0.000010 // Pitch&Roll Integrator Gain
#define Kp_YAW 1.0 // Yaw Porportional Gain  
#define Ki_YAW 0.00005 // Yaw Integrator Gain

/*Min Speed Filter for Yaw drift Correction*/
#define SPEEDFILT 3 // >1 use min speed filter for yaw drift cancellation, 0=do not use

/*For debugging purposes*/
#define OUTPUTMODE 1  //If value = 1 will print the corrected data, 0 will print uncorrected data of the gyros (with drift), 2 Accel only data

uint8_t sensors[6] = {6,7,3,0,1,2};  // For Hardware v2 flat

//Sensor: GYROX, GYROY, GYROZ, ACCELX, ACCELY, ACCELZ
int SENSOR_SIGN[]={1,-1,-1,1,-1,1,-1,-1,-1}; //{1,-1,-1,-1,1,-1}

float AN[6]; //array that store the 6 ADC filtered data
float AN_OFFSET[6]; //Array that stores the Offset of the gyros

float G_Dt=0.02;    // Integration time for the gyros (DCM algorithm)

float Accel_Vector[3]= {0,0,0}; //Store the acceleration in a vector
float Accel_Vector_unfiltered[3]= {0,0,0}; //Store the acceleration in a vector
float Accel_magnitude;
float Accel_weight;
float Gyro_Vector[3]= {0,0,0};//Store the gyros rutn rate in a vector
float Omega_Vector[3]= {0,0,0}; //Corrected Gyro_Vector data
float Omega_P[3]= {0,0,0};//Omega Proportional correction
float Omega_I[3]= {0,0,0};//Omega Integrator
float Omega[3]= {0,0,0};

float errorRollPitch[3]= {0,0,0};
float errorYaw[3]= {0,0,0};
float errorCourse=0;
float COGX=0; //Course overground X axis
float COGY=1;

float roll=0;
float pitch=0;
float yaw=0;

//Magnetometer variables
int magnetom_x;
int magnetom_y;
int magnetom_z;
float MAG_Heading;
float mag_heading_x;
float mag_heading_y;


unsigned int counter=0;
long loop_counter=0;

float DCM_Matrix[3][3]= {
  {1,0,0}
  ,{0,1,0}
  ,{0,0,1}
}; 
float Update_Matrix[3][3]={{0,1,2},{3,4,5},{6,7,8}};

float Temporary_Matrix[3][3]={
  {0,0,0}
  ,{0,0,0}
  ,{0,0,0}
};

// PPM Rx signal read (ICP) constants and variables
#define Servo1Pin  9            // Servo pins...
#define Servo2Pin 10
#define icpPin	    8           // Input Capture Pin (Rx signal reading)
//#define MAX_CHANNELS    4       // Number of radio channels to read
#define SYNC_GAP_LEN    8000    // we assume a space at least 4000us is sync (note clock counts in 0.5 us ticks)
#define MIN_IN_PULSE_WIDTH (750)  //a valid pulse must be at least 750us (note clock counts in 0.5 us ticks)
#define MAX_IN_PULSE_WIDTH (2250) //a valid pulse must be less than  2250us
static volatile unsigned int Pulses[ MAX_CHANNELS + 1];  // Pulse width array
static volatile uint8_t  Rx_ch = 0;
int Neutro[MAX_CHANNELS+1];    // Valores para los neutros en caso de fallos...
byte radio_status=0;           // radio_status = 1 => OK, 0 => No Radio signal
static volatile unsigned int ICR1_old = 0;
static volatile unsigned int Timer1_last_value;  // to store the last timer1 value before a reset
int ch1;    // Channel values
int ch2;
int ch3;
int ch4;
int ch_aux;
int ch_aux2;

// PPM Rx signal read END
// Servo Timer2 variables (Servo Timer2)
#define SERVO_MAX_PULSE_WIDTH 2000
#define SERVO_MIN_PULSE_WIDTH 900
#define SERVO_TIMER2_NUMSERVOS 4         // Put here the number of servos. In this case 4 ESC´s
typedef struct {
  uint8_t pin;
  int value;
  uint8_t counter;
}  servo_t;
uint8_t num_servos=SERVO_TIMER2_NUMSERVOS;
servo_t Servos[SERVO_TIMER2_NUMSERVOS];
static volatile uint8_t Servo_Channel;
static volatile uint8_t ISRCount=0;
static volatile unsigned int Servo_Timer2_timer1_start;
static volatile unsigned int Servo_Timer2_timer1_stop;
static volatile unsigned int Servo_Timer2_pulse_length;
// Servo Timer2 variables END 

// Servo variables (OC1 and OC2) for standard servos [disabled in this version]
unsigned int Servo1;
unsigned int Servo2;

// Navigation control variables
//float alt_error;
//float course_error;
//float course_error_old;

// ADC variables
volatile uint8_t MuxSel=0;
volatile uint8_t analog_reference = DEFAULT;
volatile uint16_t analog_buffer[8];
volatile uint8_t analog_count[8];
int an_count;

long timer=0; //general porpuse timer 
long timer_old;

// Sonar variables
static volatile unsigned long sonar_start_ms;
static volatile unsigned char sonar_start_t0;
static volatile unsigned long sonar_pulse_start_ms;
static volatile unsigned char sonar_pulse_start_t0;
static volatile unsigned long sonar_pulse_end_ms;
static volatile unsigned char sonar_pulse_end_t0;
static volatile byte sonar_status=0;
static volatile byte sonar_new_data=0;
int sonar_value=0;

// Attitude control variables
float command_rx_roll=0;        // comandos recibidos rx
float command_rx_roll_old;
float command_rx_roll_diff;
float command_rx_pitch=0;
float command_rx_pitch_old;
float command_rx_pitch_diff;
float command_rx_yaw=0;
float command_rx_yaw_diff;
int control_roll;           // resultados del control
int control_pitch;
int control_yaw;
float K_aux;

// Attitude control
float roll_I=0;
float roll_D;
float err_roll;
float pitch_I=0;
float pitch_D;
float err_pitch;
float yaw_I=0;
float yaw_D;
float err_yaw;

//Position control
long target_longitude;
long target_lattitude;
byte target_position;
float RF_err_roll;
float RF_err_roll_old;
float RF_roll_D;
float RF_roll_I=0;
float RF_err_pitch;
float RF_err_pitch_old;
float RF_pitch_D;
float RF_pitch_I=0;
float command_RF_roll;
float command_RF_pitch;

//Altitude control
int Initial_Throttle;
int target_sonar_altitude;
int err_altitude;
int err_altitude_old;
float command_altitude;
float altitude_I;
float altitude_D;

// AutoPilot Mode
// AP_mode : 1=> Position hold  2=>Stabilization assist mode (normal mode)  3=> Automatic flight
byte AP_mode = 2;             

long t0;
int num_iter;
float aux_debug;

// Variables for automatic flight
byte automatic_mode;
long automatic_mode_time;
int automatic_mode_initial_altitude;
int automatic_mode_altitude;
int automatic_ascend;
int automatic_descend;

// Variables for serial port (to read the external Range Finder)
#define SERIAL_BUFFERSIZE 80
char buffer[SERIAL_BUFFERSIZE];
int bufferidx;

char *parseptr;
byte RF_new_data;
int RF_Sensor1;
int RF_Sensor2;
int RF_Sensor3;
int RF_Sensor4;


// We read the IR distance sensors values from serial port (external Arduino pro mini)
void Serial_mode_read()
{
  char c;
  int numc;
  int i;
 
  numc = Serial.available();
  //Serial.print(numc);
  if (numc > 0)
    for (i=0;i<numc;i++)
      {
      c = Serial.read();
      if (c == '$'){                      // Line Start
        bufferidx = 0;
        buffer[bufferidx++] = c;
        continue;
        }
      if (c == '\r'){                     // Line End
        buffer[bufferidx++] = 0;
	parse_serial();
        }
      else {
        if (bufferidx < (SERIAL_BUFFERSIZE-1)){
          buffer[bufferidx++] = c;
          }
	else
	  bufferidx=0;   // Buffer overflow : restart
        }
    }   
}

// Parse the serial port strings
void parse_serial()
{
  parseptr = strchr(buffer, '$')+1;
  RF_Sensor1 = parsedecimal(parseptr,3);
  parseptr = strchr(parseptr, ',')+1;
  RF_Sensor2 = parsedecimal(parseptr,3);
  parseptr = strchr(parseptr, ',')+1;
  RF_Sensor3 = parsedecimal(parseptr,3);
  parseptr = strchr(parseptr, ',')+1;
  RF_Sensor4 = parsedecimal(parseptr,3);
  RF_new_data=1;   // New Range finder data available
  
  Serial.println();
  Serial.print("RFP:");
  Serial.print(RF_Sensor1);
  Serial.print(",");
  Serial.print(RF_Sensor2);
  Serial.print(",");
  Serial.print(RF_Sensor3);
  Serial.print(",");
  Serial.print(RF_Sensor4);
  Serial.println();  
}

// Decimal number parser
long parsedecimal(char *str,byte num_car) {
  long d = 0;
  byte i;
  
  i = num_car;
  while ((str[0] != 0)&&(i>0)) {
     if (str[0]=='-')
       d = -1*d;
     else if ((str[0] > '9') || (str[0] < '0'))
       return d;
     d *= 10;
     d += str[0] - '0';
     str++;
     i--;
     }
  return d;
}

/* ************************************************************ */
/* Obstacle avoidance routine */
void Position_control(int RF_Sensor1, int RF_Sensor2,int RF_Sensor3, int RF_Sensor4)
{
  int RF_pair1;
  int RF_pair2;
  int RF_err_roll;
  int RF_err_pitch;
  long temp;
  
  
  // We apply a non linear equation to ensure that near distances have more weight...
  /*
  RF_Sensor1 = constrain(RF_Sensor1,20,150);
  RF_Sensor2 = constrain(RF_Sensor2,20,150);
  RF_Sensor3 = constrain(RF_Sensor3,20,150);
  RF_Sensor4 = constrain(RF_Sensor4,20,150);
  temp = 150-RF_Sensor1;
  RF_Sensor1 = (temp*temp)/300 + temp;
  temp = 150-RF_Sensor2;
  RF_Sensor2 = (temp*temp)/300 + temp;
  temp = 150-RF_Sensor3;
  RF_Sensor3 = (temp*temp)/300 + temp;
  temp = 150-RF_Sensor4;
  RF_Sensor4 = (temp*temp)/300 + temp;
  
  RF_pair1 = RF_Sensor1 - RF_Sensor3;  // Front left sensor - Back right sensor
  RF_pair2 = RF_Sensor2 - RF_Sensor4;  // Front right sensor - Back left sensor  
  */
  
  RF_pair1 = RF_Sensor3 - RF_Sensor1;  // Back right sensor - Front left sensor
  RF_pair2 = RF_Sensor4 - RF_Sensor2;  // Back left sensor - Front right sensor
  
  // ROLL
  RF_err_roll_old = RF_err_roll;
  RF_err_roll = RF_pair1 - RF_pair2;
  
  RF_roll_D = (RF_err_roll-RF_err_roll_old)/0.06;   // RF_IR frequency is 1/60ms
  
  RF_roll_I += RF_err_roll*0.06;    // RF_IR frequency is 1/60ms
  RF_roll_I = constrain(RF_roll_I,-25,25);
  
  command_RF_roll = KP_RF_ROLL*RF_err_roll + K_aux*RF_roll_D + KI_RF_ROLL*RF_roll_I;
  command_RF_roll = constrain(command_RF_roll,-RF_MAX_ANGLE,RF_MAX_ANGLE); // Limit max command
  
  // PITCH
  RF_err_pitch_old = RF_err_pitch;
  RF_err_pitch = RF_pair1 + RF_pair2;
  
  RF_pitch_D = (RF_err_pitch-RF_err_pitch_old)/0.06;
  
  RF_pitch_I += RF_err_pitch*0.06;
  RF_pitch_I = constrain(RF_pitch_I,-25,25);
  
  command_RF_pitch = KP_RF_PITCH*RF_err_pitch + K_aux*RF_pitch_D + KI_RF_PITCH*RF_pitch_I;
  command_RF_pitch = constrain(command_RF_pitch,-RF_MAX_ANGLE,RF_MAX_ANGLE); // Limit max command
}

/* ************************************************************ */
/* Automatic flight pattern... */
void Automatic_flight(int target_sonar_altitude, long flight_time)
{
  //Serial.print((int)automatic_mode);
  //Serial.print(":");
  //Serial.print(sonar_value);
  //Serial.print(":");
  //Serial.print((int)command_altitude);

  if (automatic_mode==0)   // Auto Take off
    {
    if (sonar_value<37)
      {
      command_altitude+=2;   // Increment throttle until we reach a minimun safe altitude
      }
    else
      {
      automatic_mode=1;
      automatic_mode_time = loop_counter;
      automatic_mode_initial_altitude = sonar_value;
      automatic_ascend = 0;
      err_altitude = 0;
      altitude_I = 0;
      Initial_Throttle = HOVER_THROTTLE;    //command_altitude-45;
      }
    }
  else if (automatic_mode==1)  // Altitude hold
    {
    digitalWrite(7,HIGH);
    if (automatic_mode_altitude < target_sonar_altitude)
      {
      automatic_ascend++;
      automatic_mode_altitude = automatic_mode_initial_altitude + (automatic_ascend>>2);
      }
    Altitude_control(automatic_mode_altitude);
    if (loop_counter > (automatic_mode_time + flight_time))
      {
      automatic_mode = 2;  // Start descending
      automatic_descend = 0;
      }
    }
  else if (automatic_mode==2)    // Controled descend
    {
    digitalWrite(7,LOW);
    automatic_descend++;
    Altitude_control(target_sonar_altitude-(automatic_descend>>2));  
    if (sonar_value < 45)
      automatic_mode = 3;
    }
  else if (automatic_mode==3)    // Final Landing
    {
    digitalWrite(7,HIGH);
    command_altitude-=2;
    }
  command_altitude = constrain(command_altitude,MIN_THROTTLE,MAX_THROTTLE);
}


/* ************************************************************ */
/* Altitude control based on sonar sensor */
void Altitude_control(int target_sonar_altitude)
{
  //Serial.print(":");
  //Serial.print(target_sonar_altitude);
  //Serial.print(":");
  //Serial.print(err_altitude);
  err_altitude_old = err_altitude;
  err_altitude = target_sonar_altitude - sonar_value;
  err_altitude = constrain(err_altitude,-60,60);  
  altitude_D = (float)(err_altitude-err_altitude_old)/G_Dt;
  altitude_I += (float)err_altitude*G_Dt;
  altitude_I = constrain(altitude_I,-150,150);
  command_altitude = Initial_Throttle + KP_ALTITUDE*err_altitude + KD_ALTITUDE*altitude_D + KI_ALTITUDE*altitude_I;
}


/* ************************************************************ */
// ROLL, PITCH and YAW PID controls...
void Attitude_control()
{
  // ROLL CONTROL    
  if ((AP_mode==3)||(AP_mode==1))
    {
    command_rx_roll += command_RF_roll;     // Add position control term
    command_rx_roll_diff = 0;
    }
  
  err_roll = command_rx_roll - ToDeg(roll); 
    
  err_roll = constrain(err_roll,-25,25);  // to limit max roll command...
  
  roll_I += err_roll*G_Dt;
  roll_I = constrain(roll_I,-20,20);
  // D term implementation => two parts: gyro part and command part
  // To have a better (faster) response we can use the Gyro reading directly for the Derivative term...
  // Omega[] is the raw gyro reading plus Omega_I, so it´s bias corrected
  // We also add a part that takes into account the command from user (stick) to make the system more responsive to user inputs
  roll_D = command_rx_roll_diff*KD_QUAD_COMMAND_PART - ToDeg(Omega[0]);  // Take into account Angular velocity of the stick (command)
  
  // PID control
  control_roll = KP_QUAD_ROLL*err_roll + KD_QUAD_ROLL*roll_D + KI_QUAD_ROLL*roll_I; 
  
  // PITCH CONTROL
  if ((AP_mode==3)||(AP_mode==1))
    {
    command_rx_pitch += command_RF_pitch;     // Add position control term
    command_rx_pitch_diff = 0;
    }
    
  err_pitch = command_rx_pitch - ToDeg(pitch);
   
  err_pitch = constrain(err_pitch,-25,25);  // to limit max pitch command...
  
  pitch_I += err_pitch*G_Dt;
  pitch_I = constrain(pitch_I,-20,20);
  // D term
  pitch_D = command_rx_pitch_diff*KD_QUAD_COMMAND_PART - ToDeg(Omega[1]);
 
  // PID control
  control_pitch = KP_QUAD_PITCH*err_pitch + KD_QUAD_PITCH*pitch_D + KI_QUAD_PITCH*pitch_I; 
  
  // YAW CONTROL
  
  err_yaw = command_rx_yaw - ToDeg(yaw);
  if (err_yaw > 180)    // Normalize to -180,180
    err_yaw -= 360;
  else if(err_yaw < -180)
    err_yaw += 360;
  
  err_yaw = constrain(err_yaw,-60,60);  // to limit max yaw command...
  
  yaw_I += err_yaw*G_Dt;
  yaw_I = constrain(yaw_I,-50,50);
  yaw_D = command_rx_yaw_diff*KD_QUAD_COMMAND_PART - ToDeg(Omega[2]);
 
  // PID control
  control_yaw = KP_QUAD_YAW*err_yaw + KD_QUAD_YAW*yaw_D + KI_QUAD_YAW*yaw_I;
}

int channel_filter(int ch, int ch_old)
{
  if (ch_old==0)
    return(ch);
  else
    return((ch+ch_old)>>1);     // small filtering (average filter)
}

/* ****** SETUP ********************************************************************* */
void setup(){
  
  int i;
  int j;
  int aux;
 
  Serial.begin(38400);
  
  //pinMode(2,OUTPUT);   //Serial Mux //Not connected for the external Arduino pro mini
  //digitalWrite(2,LOW); //Serial Mux
  
  pinMode(4,INPUT);    // Sonar input
  pinMode(8,INPUT);    // Rx Radio Input
  pinMode(5,OUTPUT);   // Red LED
  pinMode(6,OUTPUT);   // BLue LED
  pinMode(7,OUTPUT);   // Yellow LED
  pinMode(9,OUTPUT);   // Servo1
  pinMode(10,OUTPUT);  // Servo2  Right Motor
  pinMode(11,OUTPUT);  // Servo3  Left Motor
  pinMode(12,OUTPUT);  // Servo4  Front Motor
  pinMode(13,OUTPUT);  // Servo5  Back Motor
  
  Sonar_Init();
 
  ch1=MIN_THROTTLE;
  ch2=MIN_THROTTLE;
  ch3=MIN_THROTTLE;
  ch4=MIN_THROTTLE;
  
  // Assign pins to servos
  num_servos = 4;
  Servos[0].pin = 10;       // Left motor
  Servos[1].pin = 11;       // Right motor
  Servos[2].pin = 12;       // Front motor
  Servos[3].pin = 13;       // Back motor
  Servo_Timer2_set(0,MIN_THROTTLE);   // First assign values to servos
  Servo_Timer2_set(1,MIN_THROTTLE);
  Servo_Timer2_set(2,MIN_THROTTLE);
  Servo_Timer2_set(3,MIN_THROTTLE);
  Servo_Timer2_ini();              // Servo Interrupt initialization
  
  delay(100);
  command_rx_yaw = 0;
  Servo1 = 1500;
  Servo2 = 1500;
  Serial.println();
  Serial.println("ArduIMU Quadcopter 1.29 mini");
  RxServoInput_ini();
  delay(3000);
   
 // Take neutral radio values...
 for (j=1;j<=6;j++)
   Neutro[j] = RxGetChannelPulseWidth(j);

 for (i=0; i<80; i++)
   {
   for (j=1;j<=6;j++)
     Neutro[j] = (Neutro[j]*0.8 + RxGetChannelPulseWidth(j)*0.2);
   delay(25);
   }
 
 Serial.print("Rx values: ");
 for (i=1; i<=6; i++)
   {
   Serial.print(",");
   Serial.println(Neutro[i]);
   }
 
 // Roll, Pitch and Throttle have fixed neutral values (the user can trim with the radio)
 #if SPEKTRUM==1
   Neutro[3] = CHANN_CENTER;
   Neutro[2] = CHANN_CENTER;
   Neutro[1] = MIN_THROTTLE;
 #else
   Neutro[1] = CHANN_CENTER;
   Neutro[2] = CHANN_CENTER;
   Neutro[3] = MIN_THROTTLE;
 #endif
 
 Analog_Reference(EXTERNAL);
 Analog_Init();
 
 #if (MAGNETOMETER==1)
 // Magnetometer initialization
 I2C_Init();
 delay(100);
 Compass_Init();
 #endif
 
 Read_adc_raw();
 delay(20);
 
 // Offset values for accels and gyros...
 AN_OFFSET[3] = acc_offset_x;
 AN_OFFSET[4] = acc_offset_y;
 AN_OFFSET[5] = acc_offset_z;
 AN_OFFSET[0] = gyro_offset_roll;
 AN_OFFSET[1] = gyro_offset_pitch;
 AN_OFFSET[2] = gyro_offset_yaw;
 
 // Take the gyro offset values
 for(i=0;i<300;i++)
    {
    Read_adc_raw();
    for(int y=0; y<=2; y++)   // Read initial ADC values for offset.
      AN_OFFSET[y]=AN_OFFSET[y]*0.8 + AN[y]*0.2;
    delay(20);
    }
 
 Serial.print("AN:");
 for (i=0; i<6; i++)
   {
   Serial.print(",");
   Serial.println(AN_OFFSET[i]);
   }
 
 delay(500);
 
 // Wait until throttle stick is at bottom
 #if (SPEKTRUM)
 while (RxGetChannelPulseWidth(1)>(MIN_THROTTLE+50)){
 #else
 while (RxGetChannelPulseWidth(3)>(MIN_THROTTLE+50)){
 #endif
   Serial.println("Move throttle stick to bottom to start !!!");
 }

 #if (RADIO_TEST_MODE)    // RADIO TEST MODE TO TEST RADIO CHANNELS
 while(1)
   {
   if (radio_status == 1){
      radio_status=2;   // Radio frame read
      #if (SPEKTRUM)
        ch1 = channel_filter(RxGetChannelPulseWidth(2),ch1);   // Aileron
        ch2 = channel_filter(RxGetChannelPulseWidth(3),ch2);   // Elevator
        ch3 = channel_filter(RxGetChannelPulseWidth(1),ch3);   // Throttle
        ch4 = channel_filter(RxGetChannelPulseWidth(4),ch4);   // Ruder
        ch_aux = channel_filter(RxGetChannelPulseWidth(6),ch_aux);    // Aux
        ch_aux2 = channel_filter(RxGetChannelPulseWidth(5),ch_aux2);  // Aux2
      #else
        ch1 = channel_filter(RxGetChannelPulseWidth(1),ch1);   // Aileron
        ch2 = channel_filter(RxGetChannelPulseWidth(2),ch2);   // Elevator
        ch3 = channel_filter(RxGetChannelPulseWidth(3),ch3);   // Throttle
        ch4 = channel_filter(RxGetChannelPulseWidth(4),ch4);   // Ruder
        ch_aux = channel_filter(RxGetChannelPulseWidth(6),ch_aux);  
        ch_aux2 = channel_filter(RxGetChannelPulseWidth(5),ch_aux2);  
      #endif      
   Serial.print("AIL:");
   Serial.print(ch1);
   Serial.print("ELE:");
   Serial.print(ch2);
   Serial.print("THR:");
   Serial.print(ch3);
   Serial.print("YAW:");
   Serial.print(ch2);
   Serial.print("AUX:");
   Serial.print(ch_aux);
   Serial.print("AUX2:");
   Serial.print(ch_aux2);
   Serial.println();
   delay(200);
   }
 #endif

 Read_adc_raw();   // Start ADC readings...
 timer = millis();
 delay(20);
 
 digitalWrite(7,HIGH);
}

/* *** MAIN LOOP *** */
void loop(){
  
  int aux;
  float aux_float;
  
  if((millis()-timer)>=14)   // 14ms => 70 Hz loop rate 
  {
    counter++;
    loop_counter++;
    timer_old = timer;
    timer=millis();
    G_Dt = (timer-timer_old)/1000.0;      // Real time of loop run 
    num_iter++;
	
    // IMU DCM Algorithm
    Read_adc_raw();
    #if (MAGNETOMETER==1)
    if (counter > 8)  // Read compass data at about 10Hz... (8 loop runs)
      {
      counter=0;
      Read_Compass();    // Read I2C magnetometer
      Compass_Heading(); // Calculate magnetic heading  
      }
    #endif
    Matrix_update(); 
    Normalize();
    Drift_correction();
    Euler_angles();
    // *****************
    
    // Telemetry data...
    
    //Serial.print(K_aux);    
    //Serial.print(",");
    Serial.print("$");    // Line Begin char
    if ((AP_mode==1)||(AP_mode==3))
      Serial.print("P");           // Position Hold mode    
    
    aux = ToDeg(roll);
    Serial.print(aux);
    Serial.print(",");
    aux = ToDeg(pitch);
    Serial.print(aux);
    Serial.print(",");
    aux = ToDeg(yaw);
    Serial.print(aux);

    //Serial.print(",");    
    //Serial.print(K_aux*100);
    
    //Serial.print((int)command_rx_roll);
    //Serial.print(",");
    //Serial.print((int)command_rx_pitch);
        
    if (sonar_new_data)
      {
      sonar_new_data = 0;        
      sonar_value = Filter(Get_Sonar_Pulse(),sonar_value,5);  // Filtering
      Serial.print(",");
      Serial.print(sonar_value);
      }
      
    if (radio_status == 1){
      radio_status=2;   // Radio frame read
      #if SPEKTRUM==1
        ch1 = channel_filter(RxGetChannelPulseWidth(2),ch1);   // Aileron
        ch2 = channel_filter(RxGetChannelPulseWidth(3),ch2);   // Elevator
        ch3 = channel_filter(RxGetChannelPulseWidth(1),ch3);   // Throttle
        ch4 = channel_filter(RxGetChannelPulseWidth(4),ch4);   // Ruder
        ch_aux = channel_filter(RxGetChannelPulseWidth(6),ch_aux);    // Aux
        ch_aux2 = channel_filter(RxGetChannelPulseWidth(5),ch_aux2);  // Aux2
      #else
        ch1 = channel_filter(RxGetChannelPulseWidth(1),ch1);   // Aileron
        ch2 = channel_filter(RxGetChannelPulseWidth(2),ch2);   // Elevator
        ch3 = channel_filter(RxGetChannelPulseWidth(3),ch3);   // Throttle
        ch4 = channel_filter(RxGetChannelPulseWidth(4),ch4);   // Ruder
        ch_aux = channel_filter(RxGetChannelPulseWidth(6),ch_aux);  
        ch_aux2 = channel_filter(RxGetChannelPulseWidth(5),ch_aux2);  
      #endif      
      
      // Commands from radio Rx... 
      // Stick position defines the desired angle in roll, pitch and yaw
      command_rx_roll_old = command_rx_roll;
      command_rx_roll = (ch1-CHANN_CENTER)/13.0;
      command_rx_roll_diff = command_rx_roll-command_rx_roll_old;
      command_rx_pitch_old = command_rx_pitch;
      command_rx_pitch = (ch2-CHANN_CENTER)/13.0;
      command_rx_pitch_diff = command_rx_pitch-command_rx_pitch_old;
      aux_float = (ch4-Neutro[4])/200.0;
      command_rx_yaw += aux_float;
      command_rx_yaw_diff = aux_float;
      if (command_rx_yaw > 180)         // Normalize yaw to -180,180 degrees
        command_rx_yaw -= 360.0;
      else if (command_rx_yaw < -180)
        command_rx_yaw += 360.0;
      
      // I use K_aux (channel 6) to adjust gains linked to a knob in the radio... [not used now]
      //K_aux = K_aux*0.8 + ((ch_aux-1500)/100.0 + 0.6)*0.2;
      K_aux = K_aux*0.8 + ((ch_aux-1500)/1000.0 + 0.02)*0.2;
      if (K_aux < 0)
        K_aux = 0;
   
      // We read the Mode from Channel AUX2 (Channel 5)
      if (ch_aux2 < 1200)
        {
        if (AP_mode==2)
          {
          if (ch3<(MIN_THROTTLE+30))   // If throttle is down => automatic flight
            {
            AP_mode = 3;           // Automatic pattern fligth
            digitalWrite(5,HIGH);  // Red LED On
            digitalWrite(7,LOW);   // Yellow led Off
            //Serial.print("A");
            }
          else
            {
            AP_mode = 1;           // Position hold mode (RF Position control)
            digitalWrite(5,HIGH);  // Red LED On
            digitalWrite(7,HIGH);
            //Serial.print("P");
            }
          }
        }
      else
        {
          AP_mode = 2;         // Normal mode (Stabilization assist mode)
          digitalWrite(5,LOW); // Red LED off
          digitalWrite(7,HIGH);
        }
      }
    else if (radio_status==0)
      {  // Radio_status = 0 Lost radio signal => Descend...
      ch3--;   // Descend  (Reduce throttle)
      if (ch3<MIN_THROTTLE)
        ch3 = MIN_THROTTLE;
      command_rx_roll = 0;     // Stabilize to roll=0, pitch=0, yaw not important here
      command_rx_pitch = 0;
      Attitude_control();
      // Quadcopter mix
      Servo_Timer2_set(0,ch3 - control_roll - control_yaw);    // Right motor
      Servo_Timer2_set(1,ch3 + control_roll - control_yaw);    // Left motor
      Servo_Timer2_set(2,ch3 + control_pitch + control_yaw);   // Front motor
      Servo_Timer2_set(3,ch3 - control_pitch + control_yaw);   // Back motor
      //Serial.println("NO RADIO!");
      }
  
    if ((AP_mode==1)||(AP_mode==3))  // Position Control
      {
      //Serial.println("Aqui");
      if (target_position==0)   // If this is the first time we switch to Position control, actual position is our target position
        {
        target_position = 1;
        
        if (sonar_value<300)   // Maximun 3 meters for altitude control...
          target_sonar_altitude = sonar_value;
        else
          target_sonar_altitude = 300;
        Initial_Throttle = ch3;
        // Reset I terms
        altitude_I = 0;
        RF_roll_I = 0;
        RF_pitch_I = 0;
        if (AP_mode==3)   // Automatic mode
          {
          command_altitude = MIN_THROTTLE;
          target_sonar_altitude = 140;
          automatic_mode = 0;
          }
        }
      if (RF_new_data)
        {
        RF_new_data = 0;
        if (target_position)
          {
          Position_control(RF_Sensor1,RF_Sensor2,RF_Sensor3,RF_Sensor4);  // Call Range Finder position control routine
          //Serial.print(";CRF");
          //Serial.print((int)command_RF_roll);
          //Serial.print(",");
          //Serial.print((int)command_RF_pitch);
          }
        else
          {
          command_RF_roll=0;
          command_RF_pitch=0;
          }
        }
        
      if (AP_mode==1)
        {
        Altitude_control(target_sonar_altitude);
        ch3 = command_altitude;
        }
      else if (AP_mode==3)
        {
        Automatic_flight(target_sonar_altitude,1000);
        ch3 = command_altitude;  
        }
      }
    else
      target_position=0;
    
    // Attitude control (Roll, Pitch, yaw)
    Attitude_control();
      
    // Quadcopter mix
    if (ch3 > (MIN_THROTTLE+30))  // Minimun throttle to start control
      {
      Servo_Timer2_set(0,ch3 - control_roll - control_yaw);    // Right motor
      Servo_Timer2_set(1,ch3 + control_roll - control_yaw);    // Left motor
      Servo_Timer2_set(2,ch3 + control_pitch + control_yaw);   // Front motor
      Servo_Timer2_set(3,ch3 - control_pitch + control_yaw);   // Back motor
      }
    else
      {
      roll_I = 0;  // reset I terms...
      pitch_I = 0;
      yaw_I = 0;
      Servo_Timer2_set(0,MIN_THROTTLE);  // Motors stoped
      Servo_Timer2_set(1,MIN_THROTTLE);
      Servo_Timer2_set(2,MIN_THROTTLE);
      Servo_Timer2_set(3,MIN_THROTTLE);
      // Initialize yaw command to actual yaw
      command_rx_yaw = ToDeg(yaw);
      command_rx_yaw_diff = 0;
      }
      
    Serial.println();    // Line END
    Serial_mode_read();  // Parse serial port info
    }
}

