void runMotors(int leftMotorWriteSpeed, int rightMotorWriteSpeed) {
  leftMotorWriteSpeed = constrain(leftMotorWriteSpeed, -MAX_SPEED, MAX_SPEED); // combine turnSpeedVal and the motor speed required for forwards/backwards movement so the robot can move and turn
  rightMotorWriteSpeed = constrain(rightMotorWriteSpeed, -MAX_SPEED, MAX_SPEED); // positive turn=turn to the right -> right wheel needs to slow down -> subtract turnSpeedVal for right motor

  if (leftMotorWriteSpeed >= 0) {
    digitalWrite(LEFT_DIR_PIN, HIGH);
  } else {
    digitalWrite(LEFT_DIR_PIN, LOW);
  }
  if (rightMotorWriteSpeed >= 0) {
    digitalWrite(RIGHT_DIR_PIN, HIGH);
  } else {
    digitalWrite(RIGHT_DIR_PIN, LOW);
  }

  if (abs(leftMotorWriteSpeed) >= 1) {
    timerAlarmWrite(leftStepTimer, 1000000 / abs(leftMotorWriteSpeed), true);  // 1Mhz / # =  rate
  } else {
    timerAlarmWrite(leftStepTimer, 1e17, true);  // don't step
  }
  if (abs(rightMotorWriteSpeed) >= 1) {
    timerAlarmWrite(rightStepTimer, 1000000 / abs(rightMotorWriteSpeed), true);  // 1Mhz / # =  rate
  } else {
    timerAlarmWrite(rightStepTimer, 1e17, true);  // don't step
  }
  timerAlarmEnable(leftStepTimer);
  timerAlarmEnable(rightStepTimer);
}
void motorsStop() {
  PIDA.SetMode(MANUAL);
  PIDS.SetMode(MANUAL);
  timerAlarmWrite(leftStepTimer, 1e17, true);  // 1Mhz / # =  rate
  timerAlarmWrite(rightStepTimer, 1e17, true);  // 1Mhz / # =  rate
  timerAlarmEnable(leftStepTimer);
  timerAlarmEnable(rightStepTimer);
  leftMotorWriteSpeed = 0;
  rightMotorWriteSpeed = 0;
  targetPitch = 0;
  motorAccel = 0;
  motorSpeed = 0;
}
