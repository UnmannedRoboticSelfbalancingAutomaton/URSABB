#include "ursabb.h"

void bluetoothTaskFunction(void * pvParameters) {
  while (true) {  // infinite loop
    // bluetooth recieve code:

    //
    //  while (SerialBT.available()) {
    //    SerialBT.write(SerialBT.read());
    //  }
    //
    //    int packetSize = Udp.parsePacket();
    //    if (packetSize) {
    //      if (xSemaphoreTake(mutexReceive, 1) == pdTRUE) {
    //        receivedNewData = true;
    //        lastMessageTimeMillis = millis();
    //
    //        Udp.read(packetBuffer, maxWifiRecvBufSize);
    //        for (int i = 0; i < maxWifiRecvBufSize; i++) {
    //          recvdData[i] = (byte)((int)(256 + packetBuffer[i]) % 256);
    //        }
    //
    //        Udp.beginPacket();
    //        for (byte i = 0; i < numBytesToSend; i++) {  // send response, maybe change to go less frequently
    //          Udp.write(dataToSend[i]);
    //        }
    //
    //        Udp.endPacket();
    //
    //        xSemaphoreGive(mutexReceive);
    //      }
    //    }
    vTaskDelay(25 / portTICK_PERIOD_MS);
  }
}

void setupBluetooth() {
  SerialBT.begin(robotSSID);
  SerialBT.setPin(robotPin);
  xTaskCreatePinnedToCore(  // create task to run WiFi recieving
    bluetoothTaskFunction,   /* Function to implement the task */
    "bluetoothTask",  /* Name of the task */
    48000,       /* Stack size in words */
    NULL,        /* Task input parameter */
    0,           /* Priority of the task */
    NULL,        /* Task handle. */
    0            /* Core on which task should run */
  );
}

boolean readBoolFromBuffer(byte &pos) {  // return boolean at pos position in recvdData
  byte msg = recvdData[pos];
  pos++;  // increment the counter for the next value
  return (msg == 1);
}

byte readByteFromBuffer(byte &pos) {  // return byte at pos position in recvdData
  byte msg = recvdData[pos];
  pos++;  // increment the counter for the next value
  return msg;
}

int readIntFromBuffer(byte &pos) {  // return int from four bytes starting at pos position in recvdData
  union {  // this lets us translate between two variable types (equal size, but one's four bytes in an array, and one's a four byte int)  Reference for unions: https:// www.mcgurrin.info/robots/127/
    byte b[4];
    int v;
  } d;  // d is the union, d.b acceses the byte array, d.v acceses the int

  for (int i = 0; i < 4; i++) {
    d.b[i] = recvdData[pos];
    pos++;
  }

  return d.v;  // return the int form of union d
}

float readFloatFromBuffer(byte &pos) {  // return float from 4 bytes starting at pos position in recvdData
  union {  // this lets us translate between two variable types (equal size, but one's 4 bytes in an array, and one's a 4 byte float) Reference for unions: https:// www.mcgurrin.info/robots/127/
    byte b[4];
    float v;
  } d;  // d is the union, d.b acceses the byte array, d.v acceses the float

  for (int i = 0; i < 4; i++) {
    d.b[i] = recvdData[pos];
    pos++;
  }

  return d.v;
}

void addBoolToBuffer(boolean msg, byte &pos) {  // add a boolean to the tosendData array
  dataToSend[pos] = msg;
  pos++;
}

void addByteToBuffer(byte msg, byte &pos) {  // add a byte to the tosendData array
  dataToSend[pos] = msg;
  pos++;
}

void addIntToBuffer(int msg, byte &pos) {  // add an int to the tosendData array (four bytes)
  union {
    byte b[4];
    int v;
  } d;  // d is the union, d.b acceses the byte array, d.v acceses the int

  d.v = msg;  // put the value into the union as an int

  for (int i = 0; i < 4; i++) {
    dataToSend[pos] = d.b[i];
    pos++;
  }
}

void addFloatToBuffer(float msg, byte &pos) {  // add a float to the tosendData array (four bytes)
  union {  // this lets us translate between two variables (equal size, but one's 4 bytes in an array, and one's a 4 byte float Reference for unions: https:// www.mcgurrin.info/robots/127/
    byte b[4];
    float v;
  } d;  // d is the union, d.b acceses the byte array, d.v acceses the float

  d.v = msg;

  for (int i = 0; i < 4; i++) {
    dataToSend[pos] = d.b[i];
    pos++;
  }
}
