#include "ursabb.h"

void bluetoothTaskFunction(void * pvParameters) {
  while (true) {  // infinite loop
    // bluetooth recieve code:

    while (SerialBT.available() > 0) {
      Serial.println(SerialBT.peek());
      if (charToByte(SerialBT.peek()) == 255) { //end of line
        SerialBT.read();
        for (int i = 0; i < recvByteCounter; i++) {
          Serial.print(recvdData[i]);
          Serial.print(",");
        }
        Serial.println("data recieved");
        recvByteCounter = 0;
        endline();//and make response
      } else {
        byte hd = charToByte(SerialBT.read());
        if (charToByte(SerialBT.peek()) == 255 && hd <= 127) { //suprise end of line
          /* this code shouldn't ever run. The first end of line check should catch the end of a message
              and there should never be half of a split byte and then an endline caught by this if.
              Still, if communication gets out of sync, it's better to drop one packet and reset than never
              catch the endline again.
              TODO: add errorchecking to communication code.
          */
          SerialBT.read();
          recvByteCounter = 0;
        } else {
          if (xSemaphoreTake(mutexReceive, 1) == pdTRUE) {
            if (hd == 128) {
              recvdData[recvByteCounter] = 0;
              recvByteCounter++;
            } else if (hd == 129) {
              recvdData[recvByteCounter] = 1;
              recvByteCounter++;
            } else {
              recvdData[recvByteCounter] = (charToByte(SerialBT.read()) << 4) | hd;  // combine bytes, second byte more significant
              recvByteCounter++;
            }
            xSemaphoreGive(mutexReceive);
          }
        }
      }
    }
    vTaskDelay(2 / portTICK_PERIOD_MS);
  }
}
void endline() {
  if (xSemaphoreTake(mutexReceive, 1) == pdTRUE) {
    receivedNewData = true;
    lastMessageTimeMillis = millis();
    byte toSend[numBytesToSend + 1];
    for (byte i = 0; i < numBytesToSend; i++) {
      toSend[i] = dataToSend[i];
      Serial.print(toSend[i]);
      Serial.print(",");
    }
    Serial.println();
    xSemaphoreGive(mutexReceive);
    toSend[numBytesToSend] = 255;
    SerialBT.write(toSend, numBytesToSend + 1);
    Serial.println("data sent");

  }
}
byte charToByte(char i) {
  return (byte)((int)(256 + i) % 256);
}
void setupBluetooth() {
  SerialBT.begin(robotSSID);
  xTaskCreatePinnedToCore(  // create task to run WiFi recieving
    bluetoothTaskFunction,   /* Function to implement the task */
    "bluetoothTask",  /* Name of the task */
    16000,       /* Stack size in words */
    NULL,        /* Task input parameter */
    0,           /* Priority of the task */
    NULL,        /* Task handle. */
    0            /* Core on which task should run */
  );
}

boolean readBoolFromBuffer(byte &pos) { // return boolean at pos position in recvdData
  byte msg = recvdData[pos];
  pos++;  // increment the counter for the next value
  return (msg == 1);
}

byte readByteFromBuffer(byte &pos) { // return byte at pos position in recvdData
  byte msg = recvdData[pos];
  pos++;  // increment the counter for the next value
  return msg;
}

int readIntFromBuffer(byte &pos) { // return int from four bytes starting at pos position in recvdData
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

float readFloatFromBuffer(byte &pos) { // return float from 4 bytes starting at pos position in recvdData
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

void addBoolToBuffer(boolean msg, byte &pos) { // add a boolean to the tosendData array
  if (msg == true) {
    dataToSend[pos] = 129;
  } else {
    dataToSend[pos] = 128;
  }
  pos++;
}

void addByteToBuffer(byte msg, byte &pos) { // add a byte to the tosendData array
  dataToSend[pos] = ((msg) | B00001111);  // add less significant half
  pos++;
  dataToSend[pos] = (msg >> 4); // add second more significant half
  pos++;
}

void addIntToBuffer(int msg, byte &pos) { // add an int to the tosendData array (four bytes)
  union {
    byte b[4];
    int v;
  } d;  // d is the union, d.b acceses the byte array, d.v acceses the int

  d.v = msg;  // put the value into the union as an int

  for (int i = 0; i < 4; i++) {
    dataToSend[pos] = ((d.b[i]) | B00001111);  // add less significant half
    pos++;
    dataToSend[pos] = (d.b[i] >> 4); // add second more significant half
    pos++;
  }
}

void addFloatToBuffer(float msg, byte &pos) { // add a float to the tosendData array (four bytes)
  union {  // this lets us translate between two variables (equal size, but one's 4 bytes in an array, and one's a 4 byte float Reference for unions: https:// www.mcgurrin.info/robots/127/
    byte b[4];
    float v;
  } d;  // d is the union, d.b acceses the byte array, d.v acceses the float

  d.v = msg;

  for (int i = 0; i < 4; i++) {
    dataToSend[pos] = ((d.b[i]) | B00001111);  // add less significant half
    pos++;
    dataToSend[pos] = (d.b[i] >> 4); // add second more significant half
    pos++;
  }
}
