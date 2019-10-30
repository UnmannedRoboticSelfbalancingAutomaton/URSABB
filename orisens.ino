#include "ursabb.h"


// start I2C communication and send commands to set up the MPU
// A command is set by starting a transmission, writing a byte (written here in hexadecimal) to signal what register should be changed,
// and then sending a new register value
void setupMPU9250() {

}

void readMPU9250() {
  getValuesMPU9250();
  rotationDPS_X = (rotationX - rotationOffsetX) * 1000.00 / 32766;  // zero gyro with offset values recorded on startup and convert to degrees per second
  rotationDPS_Y = (rotationY - rotationOffsetY) * 1000.00 / 32766;
  rotationDPS_Z = (rotationZ - rotationOffsetZ) * 1000.00 / 32766;

  if (micros() < lastCalcedMPU) {  // try to handle micros' long overflow in a harmless way
    lastCalcedMPU = micros() - 10000;
  }
  // complementary filter combines gyro and accelerometer tilt data in a way that takes advantage of short term accuracy of the gyro and long term accuracy of the accelerometer
  pitch = COMPLEMENTARY_FILTER_CONSTANT * ((pitch) + rotationDPS_X * (micros() - lastCalcedMPU) / 1000000.000)  // add rotation rate as measured by the gyro to current pitch - valid in short term
          + (1 - COMPLEMENTARY_FILTER_CONSTANT) * (degrees(atan2(accelerationY, accelerationZ)) - pitchOffset);   // in the long term drift towards the angle of gravity measured by the accelerometer

  if (controlMode == 2) { // only adjust pitchOffset when the robot is enabled
    pitchOffset = (pitch + pitchOffset) * (1 - PITCH_OFFSET_CHANGE) + pitchOffset * (PITCH_OFFSET_CHANGE);  // slowly move pitchOffset towards the current pitch value, the overall average pitch value should be close to the balance point
  }
  lastCalcedMPU = micros();  // record time of last calculation so we know next time how much time has passed (how much time to integrate rotation rate for)
}

void zeroMPU9250() {  // find how much offset each gyro axis has to zero out drift. should be run on startup (when robot is still)
  do {
    getValuesMPU9250();
    int16_t lastrotationX = rotationX;
    int16_t lastrotationY = rotationY;
    int16_t lastrotationZ = rotationZ;
    rotationOffsetX = 0;
    rotationOffsetY = 0;
    rotationOffsetZ = 0;

    for (int i = 0; i < movementMeasurements; i++) {
      getValuesMPU9250();
      rotationOffsetX += abs(rotationX - lastrotationX);
      rotationOffsetY += abs(rotationY - lastrotationY);
      rotationOffsetZ += abs(rotationZ - lastrotationZ);
      lastrotationX = rotationX;
      lastrotationY = rotationY;
      lastrotationZ = rotationZ;
      digitalWrite(LED_BUILTIN, i % 2);
      delay(15);
    }
  } while (abs(rotationOffsetX) > movementThreshold * movementMeasurements || abs(rotationOffsetY) > movementThreshold * movementMeasurements || abs(rotationOffsetZ) > movementThreshold * movementMeasurements);

  rotationOffsetX = 0;
  rotationOffsetY = 0;
  rotationOffsetZ = 0;
  digitalWrite(LED_BUILTIN, LOW);
  for (int i = 0; i < 50; i++) {  // run the following code 50 times so we can get many measurements to average into an offset value
    getValuesMPU9250();
    rotationOffsetX += rotationX;  // add all the reads together
    rotationOffsetY += rotationY;
    rotationOffsetZ += rotationZ;
    delay(10 + i / 5);  // add some time between reads, changing the delay each time a bit to be less likely to be thrown by a periodic oscillation
  }

  rotationOffsetX /= 50;  // devide by the number of reads that were taken to get an average value
  rotationOffsetY /= 50;
  rotationOffsetZ /= 50;
}
void getValuesMPU9250() {
  //  Wire.beginTransmission(0x68);
  //  Wire.write(0x3B);  // location of first byte of data
  //  Wire.endTransmission(false);
  //  Wire.requestFrom(0x68, 14, true);  // ask for accel and gyro data bytes
  //  accelerationX = Wire.read() << 8 | Wire.read();  // read two bytes and put them together into a sixteen bit integer value
  //  accelerationY = Wire.read() << 8 | Wire.read();
  //  accelerationZ = Wire.read() << 8 | Wire.read();
  //  Wire.read(); Wire.read();  // throw away temperature, it's annoying they put it in the middle here
  //  rotationX = Wire.read() << 8 | Wire.read();
  //  rotationY = Wire.read() << 8 | Wire.read();
  //  rotationZ = Wire.read() << 8 | Wire.read();
}
