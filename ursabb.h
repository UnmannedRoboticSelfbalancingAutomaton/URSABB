#ifndef ursabbh
#define ursabbh
#include <PID_v1.h>
#include <Wire.h>  // scl=22 sda=21
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WiFiAP.h>
#include <EEPROM.h>

#define ROBOT_ID 255  // unique robot ID, sent to DS, and used to name wifi network
#define MODEL_NO 255  // unique configuration of robot which can be used to identify additional features
#define WiFiLossDisableIntervalMillis 500  // if no data packet has been recieved for this number of milliseconds, the robot stops moving
#define DACUnitsPerVolt 200.0  // Use to calibrate voltage read through voltage divider. Divide analogRead value by this constant to get voltage. Analog read is from 0 to 4095 corresponding to 0 to 3.3 volts.
float MAX_ACCEL = 180;  // limits maximum change in speed value per loop, a limit of the motor
float COMPLEMENTARY_FILTER_CONSTANT = .9997;  // higher = more gyro based, lower=more accelerometer based, for pitch calculation
int MAX_SPEED = 1500;  // max speed (in steps/sec) that the motors can run at
float MAX_TIP = 14;  // angle the robot shouldn't go too much past, the output limit for the speed PID loop
float TIPPED_TIP = 30; // if the robot is sitting at a pitch over this angle, the robot is tipped over and can't get back up with just driving.
float DRIVE_SPEED_SCALER = .85;  // what proportion of MAX_SPEED the robot's target driving speed can be-some extra speed must be kept in reserve to remain balanced
float TURN_SPEED_SCALER = .05;  // what proportion of MAX_SPEED can be given differently to each wheel in order to turn-controls maximum turn rate
float PITCH_OFFSET_CHANGE = .999994;  // larger = pitchOffset changes slower
float pitchOffset = -0.000;  // subtracted from the output in readMPU6050 so that zero pitch can correspond to balenced. Because the MPU6050 may not be mounted in the robot perfectly or because the robot's weight might not be perfectly centered, zero may not otherwise respond to perfectly balanced.

// The following lines define STEP pins and DIR pins. STEP pins are used to
// trigger a step (when rides from LOW to HIGH) whereas DIR pins are used to
// change the direction at which the motor is driven.
#define LEFT_STEP_PIN GPIO_NUM_32
#define LEFT_DIR_PIN GPIO_NUM_33
#define RIGHT_STEP_PIN GPIO_NUM_25
#define RIGHT_DIR_PIN GPIO_NUM_26
#define ENS_PIN GPIO_NUM_23  // pin wired to both motor driver chips' ENable pins, to turn on and off motors
#define LED_BUILTIN GPIO_NUM_2
#define VOLTAGE_PIN GPIO_NUM_36  // ADC1 CH0

// for checking on startup if still before calibrating
#define movementThreshold 20
#define movementMeasurements 20

#define maxWifiRecvBufSize 50  // max number of bytes to receive
#define maxWifiSendBufSize 50  // max number of bytes to send

byte voltage = 0;  // 0v=0 13v=255

// since multiple tasks are running at once, we don't want two tasks to try and use one array at the same time.
SemaphoreHandle_t mutexReceive;  // used to check whether receiving tasks can safely change shared variables

byte controlMode = 0;
#define M_DISABLED 0
#define M_PARK 1
#define M_SEGWAY 2
#define M_SELF_RIGHT_TO_SEGWAY 3
#define M_OUTRIGGER 4
#define M_MAGHEAD_SEGWAY 5
byte lastControlMode = 0;  // to know if controlMode has changed
boolean DSControlMode = false;  // what control mode does the driver want?
boolean tipped = false;

double targetPitch = 0.000;  // what angle the balancing control loop should aim for the robot to be at, the output of the speed control loop
double motorSpeed = 0.000;  // how much movement in the forwards/backwards direction the motors should move-only one set of control loops is used for balancing, not one for each motor
volatile int leftMotorWriteSpeed = 0;  // after acceleration
volatile int rightMotorWriteSpeed = 0;
double motorAccel = 0;  // how many stepper ticks per second per loop cycle the motors should be made to accelerate at, used as output of angle balancing loop
double speedVal = 0;  // how many stepper ticks per second the robot should try to drive at-the input to the speed control loop.
int turnSpeedVal = 0;  // (positive=turn right, negative=turn left)

double kP_angle, kI_angle, kD_angle = 0.0000;  // PID gains for the Angle control loop
double kP_speed, kI_speed, kD_speed = 0.0000;  // PID gains for the Speed control loop

int16_t accelerationX, accelerationY, accelerationZ, rotationX, rotationY, rotationZ = 0;
int32_t rotationOffsetX, rotationOffsetY, rotationOffsetZ = 0;  // "offset" values used to zero the MPU6050 gyro on startup
uint32_t lastCalcedMPU6050 = 0;  // micros() value of last orientation read. used to integrate gyro data to get rotation
double rotationDPS_X, rotationDPS_Y, rotationDPS_Z = 0.000;  // rotation in Degrees Per Second around the X,Y, and Z axes, with x left right, y forwards and backwards and z up and down
double pitch = 0.000;  // output (in degrees) from the MPU6050 reading code. negative=forwards, positive=back Pitch matters for self balancing.

hw_timer_t *leftStepTimer = NULL;
hw_timer_t *rightStepTimer = NULL;

byte numBytesToSend = 0;
// Define the SSID and password for the robot's access point
char robotSSID[12];  // defined in the setup method
const char *robotPass = "sert2521";
volatile byte recvdData[maxWifiRecvBufSize] = {0};  // array to hold data recieved from DS.
volatile boolean receivedNewData = false;  // set true when data gotten, set false when parsed
volatile byte dataToSend[maxWifiSendBufSize] = {0};  // array to hold data to send to DS.
char packetBuffer[maxWifiRecvBufSize];
byte numAuxRecv = 0;  // how many bytes of control data for extra things
byte auxRecvArray[12] = {0};  // size of numAuxRecv
byte numSendAux = 0;  // how many bytes of sensor data to send
byte auxSendArray[12] = {0};  // size of numAuxSend
volatile uint32_t lastMessageTimeMillis = 0;
byte saverecallState = 0;  // 0=don't send don't save  1=send  2=save

WiFiUDP Udp;

PID PIDA(&pitch, &motorAccel, &targetPitch, kP_angle, kI_angle, kD_angle, DIRECT);  // setup the Angle PID loop  PID(&Input, &Output, &Setpoint, Kp, Ki, Kd, Direction)
PID PIDS(&motorSpeed, &targetPitch, &speedVal, kP_speed, kI_angle, kD_angle, DIRECT);  // setup the Speed PID loop

#endif
