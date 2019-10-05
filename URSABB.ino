#include "ursabb.h"
void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);
  pinMode(ENS_PIN, OUTPUT);
  digitalWrite(ENS_PIN, HIGH);
  pinMode(VOLTAGE_PIN, INPUT);
  pinMode(LEFT_STEP_PIN, OUTPUT);
  pinMode(RIGHT_STEP_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  digitalWrite(LEFT_STEP_PIN, LOW);
  digitalWrite(RIGHT_STEP_PIN, LOW);
  digitalWrite(LEFT_DIR_PIN, LOW);
  digitalWrite(RIGHT_DIR_PIN, LOW);

  Serial.begin(115200);  // for debugging. Set the serial monitor to the same value or you will see nothing or gibberish.

  mutexReceive = xSemaphoreCreateMutex();
  sprintf(robotSSID, "SERT_URSA_%02d", ROBOT_ID);  // create unique network SSID
  EEPROM.begin(64);  // size in bytes

  PIDA.SetMode(MANUAL);  // PID loop off
  PIDS.SetMode(MANUAL);
  PIDA.SetSampleTime(10);  // tell the PID loop how often to run (in milliseconds) We have to call PID.Compute() at least this often
  PIDS.SetSampleTime(10);
  PIDA.SetOutputLimits(-MAX_ACCEL, MAX_ACCEL);
  PIDS.SetOutputLimits(-MAX_TIP, MAX_TIP);

  recallSettings();
  setupWifi();
  setupMPU6050();  // this function starts the connection to the MPU6050 gyro/accelerometer board using the I2C Wire library, and tells the MPU6050 some settings to use
  zeroMPU6050();  // this function averages some gyro readings so later the readings can be calibrated to zero. This function blocks until the robot is held stil, so the robot needs to be set flat on the ground on startup

  setupStepperTimers();

  leftServo.attach(LEFT_SERVO_PIN);
  rightServo.attach(RIGHT_SERVO_PIN);

  digitalWrite(LED_BUILTIN, LOW);

}

void loop() {  // on core 1. the balancing control loop will be here, with the goal of keeping this loop as fast as possible

  readMPU6050();

  voltage = map(analogRead(VOLTAGE_PIN) * 1000.00 / DACUnitsPerVolt, 0, 13000.0, 0, 255);

  if (receivedNewData) {
    if (xSemaphoreTake(mutexReceive, 1) == pdTRUE) {
      parseDataReceived();
      numBytesToSend = createDataToSend();
      receivedNewData = false;
      xSemaphoreGive(mutexReceive);
    }
  }

  if (abs(pitch) > TIPPED_TIP) {
    tipped = true;
  } else {
    tipped = false;
  }

  if (millis() - lastMessageTimeMillis > WiFiLossDisableIntervalMillis) {
    controlMode = 0;
  }

  if (controlMode == M_DISABLED) { // disabled
    digitalWrite(LED_BUILTIN, HIGH);
    motorsStop();
    targetPitch = 0;
    motorAccel = 0;
    motorSpeed = 0;
    PIDA.SetMode(MANUAL);
    PIDS.SetMode(MANUAL);
    leftServo.detach();
    rightServo.detach();
    leftMotorWriteSpeed = 0;
    rightMotorWriteSpeed = 0;
    digitalWrite(ENS_PIN, HIGH);  // disables stepper motors
  } else {
    digitalWrite(LED_BUILTIN, (millis() % 500 < 150));
    switch (controlMode) {
      case M_SEGWAY:
        if (lastControlMode != M_SEGWAY) { // restart segway code
          digitalWrite(ENS_PIN, LOW);  // enables stepper motors
          PIDA.SetMode(AUTOMATIC);  // turn on the PID
          PIDS.SetMode(AUTOMATIC);  // turn on the PID
        }
        if (tipped) {
          controlMode = M_SELF_RIGHT_TO_SEGWAY;
        }
        PIDA.SetTunings(kP_angle, kI_angle, kD_angle);
        PIDS.SetTunings(kP_speed, kI_speed, kD_speed);
        PIDA.Compute();
        if (PIDS.Compute()) {
          motorSpeed += constrain(motorAccel, -MAX_ACCEL, MAX_ACCEL);
          motorSpeed = constrain(motorSpeed, -MAX_SPEED, MAX_SPEED);
          leftMotorWriteSpeed = motorSpeed + turnSpeedVal;
          rightMotorWriteSpeed = motorSpeed - turnSpeedVal;
        }
        break;
      case M_PARK:
        motorsStop();
        targetPitch = 0;
        motorAccel = 0;
        motorSpeed = 0;
        break;
      case M_MAGHEAD_SEGWAY:
        break;
      case M_SELF_RIGHT_TO_SEGWAY:

        break;
      case M_OUTRIGGER:

        break;
    }
    if ((lastControlMode == M_SEGWAY || lastControlMode == M_MAGHEAD_SEGWAY) && (controlMode != M_SEGWAY && controlMode != M_MAGHEAD_SEGWAY)) { // if switched away from a self balancing mode
      PIDA.SetMode(MANUAL);
      PIDS.SetMode(MANUAL);
    }
    if (lastControlMode == M_DISABLED && controlMode != M_DISABLED) {
      leftServo.attach(LEFT_SERVO_PIN);
      rightServo.attach(RIGHT_SERVO_PIN);
    }
  }
  lastControlMode = controlMode;
}
byte createDataToSend() {
  byte counter = 0;

  addByteToBuffer(controlMode, counter);
  addBoolToBuffer(tipped, counter);
  addByteToBuffer(ROBOT_ID, counter);
  addByteToBuffer(MODEL_NO, counter);
  addFloatToBuffer(pitch, counter);
  addByteToBuffer(voltage, counter);
  addIntToBuffer(leftMotorWriteSpeed, counter);
  addIntToBuffer(rightMotorWriteSpeed, counter);
  addFloatToBuffer(targetPitch, counter);
  addFloatToBuffer(pitchOffset, counter);
  addByteToBuffer(numSendAux, counter);  // how many bytes of extra data

  for (int i = 0; i < numSendAux; i++) {
    addByteToBuffer(auxSendArray[i], counter);  // extra data
  }

  if (saverecallState == 1) {
    recallSettings();
    addBoolToBuffer(true, counter);
    addFloatToBuffer(kP_angle, counter);
    addFloatToBuffer(kI_angle, counter);
    addFloatToBuffer(kD_angle, counter);
    addFloatToBuffer(kP_speed, counter);
    addFloatToBuffer(kI_speed, counter);
    addFloatToBuffer(kD_speed, counter);
  } else {
    addBoolToBuffer(false, counter);
  }

  return counter;
}

void parseDataReceived() {  // put parse functions here
  byte counter = 0;
  controlMode = readByteFromBuffer(counter);
  speedVal = map(readByteFromBuffer(counter), 0, 200, -MAX_SPEED * DRIVE_SPEED_SCALER, MAX_SPEED * DRIVE_SPEED_SCALER);
  turnSpeedVal = map(readByteFromBuffer(counter), 0, 200, -MAX_SPEED * TURN_SPEED_SCALER, MAX_SPEED * TURN_SPEED_SCALER);
  numAuxRecv = readByteFromBuffer(counter);  // how many bytes of control data for extra things

  for (int i = 0; i < numAuxRecv; i++) {
    auxRecvArray[i] = readByteFromBuffer(counter);
  }

  if (readBoolFromBuffer(counter)) {
    kP_angle = readFloatFromBuffer(counter);
    kI_angle = readFloatFromBuffer(counter);
    kD_angle = readFloatFromBuffer(counter);
    kP_speed = readFloatFromBuffer(counter);
    kI_speed = readFloatFromBuffer(counter);
    kD_speed = readFloatFromBuffer(counter);
  }

  saverecallState = readByteFromBuffer(counter);
  if (saverecallState == 2) {
    saveSettings();
  }
}
