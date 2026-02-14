#include <Wire.h>
#include <ArduinoQueue.h>
#include "Geiger.h"

constexpr uint8_t I2C_ADDR = 0x2C;

//#define DEBUG_MODE    // Comment this line out for production code, uncomment it for testing

#define LED_PIN 22
#define GEIGER_1_INTERRUPT_PIN 21
#define GEIGER_2_INTERRUPT_PIN 20
#define GEIGER_QUEUE_SIZE 100

#define GEIGER_1_COUNT_REGISTER 1
#define GEIGER_1_DATA_REGISTER  2

#define GEIGER_2_COUNT_REGISTER 3
#define GEIGER_2_DATA_REGISTER  4

volatile uint8_t registerNumber = 0;

Geiger geiger_1(GEIGER_1_INTERRUPT_PIN, GEIGER_QUEUE_SIZE);
Geiger geiger_2(GEIGER_2_INTERRUPT_PIN, GEIGER_QUEUE_SIZE);

void handleInterrupt() {
  unsigned long m;
  m = millis();

  if (!(sio_hw->gpio_in & (1 << GEIGER_1_INTERRUPT_PIN)))   // If the pin is low, we have an interrupt on that pin
  {
    if (geiger_1.enqueueItem(m) == false)
    {
      Serial.println("Guiger 1 Queue is full.");
    }
  }

  if (!(sio_hw->gpio_in & (1 << GEIGER_2_INTERRUPT_PIN)))   // If the pin is low, we have an interrupt on that pin
  {
    if (geiger_2.enqueueItem(m) == false)
    {
      Serial.println("Guiger 2 Queue is full.");
    }
  }
}

void onI2CRequest() {
  switch (registerNumber)
  {
    case GEIGER_1_COUNT_REGISTER:
      {
        unsigned short s = geiger_1.getQueueCount();
        byte byteArray[sizeof(unsigned short)];
        memcpy(byteArray, &s, sizeof(unsigned short));
        Wire.write(byteArray, 2);
      }
      break;

    case GEIGER_1_DATA_REGISTER:
      {
        unsigned long l = geiger_1.dequeueItem();
        byte byteArray[sizeof(unsigned long)];
        memcpy(byteArray, &l, sizeof(unsigned long));
        Wire.write(byteArray, 4);
      }
      break;

    case GEIGER_2_COUNT_REGISTER:
      {
        unsigned short s = geiger_2.getQueueCount();
        byte byteArray[sizeof(unsigned short)];
        memcpy(byteArray, &s, sizeof(unsigned short));
        Wire.write(byteArray, 2);
      }
      break;

    case GEIGER_2_DATA_REGISTER:
      {
        unsigned long l = geiger_2.dequeueItem();
        byte byteArray[sizeof(unsigned long)];
        memcpy(byteArray, &l, sizeof(unsigned long));
        Wire.write(byteArray, 4);
      }
      break;
  }
}

void onI2CReceive(int numBytes) {
  while (Wire.available())
    registerNumber = Wire.read();
}

void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_ADDR);
  Wire.onRequest(onI2CRequest);
  Wire.onReceive(onI2CReceive);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  pinMode(GEIGER_1_INTERRUPT_PIN, INPUT_PULLUP);
  pinMode(GEIGER_2_INTERRUPT_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(GEIGER_1_INTERRUPT_PIN), handleInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(GEIGER_2_INTERRUPT_PIN), handleInterrupt, FALLING);

  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
}

void loop() {
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  delay(500);                      // wait for a second
  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  delay(500);                      // wait for a second

  if ((geiger_1.getQueueCount() > 0) || (geiger_2.dequeueItem() > 0))
  {
    digitalWrite(LED_PIN, HIGH);
  }
  else
  {
    digitalWrite(LED_PIN, LOW);
  }

#ifdef DEBUG_MODE
  int n1 = geiger_1.getQueueCount();
  if (n1 != 0)
  {
    Serial.println("Geiger 1 Queue Data: ");

    for (int i=0; i<n1; i++)
    {
      unsigned long l = geiger_1.dequeueItem();
      Serial.println(l);
      byte byteArray[sizeof(unsigned long)];
      memcpy(byteArray, &l, sizeof(unsigned long));
      for (size_t j = 0; j < sizeof(unsigned long); j++)
      {
        Serial.println("byte[" + String(j) + "]: 0x" + String(byteArray[j], HEX));
      }
    }
  }
  else
  {
    Serial.println("Geiger 1 Queue is empty.");
  }

  int n2 = geiger_2.getQueueCount();
  if (n2 != 0)
  {
    Serial.println("Geiger 2 Queue Data: ");

    for (int i=0; i<n2; i++)
    {
      unsigned long l = geiger_2.dequeueItem();
      Serial.println(l);
      byte byteArray[sizeof(unsigned long)];
      memcpy(byteArray, &l, sizeof(unsigned long));
      for (size_t j = 0; j < sizeof(unsigned long); j++)
      {
        Serial.println("byte[" + String(j) + "]: 0x" + String(byteArray[j], HEX));
      }
    }
  }
  else
  {
    Serial.println("Geiger 2 Queue is empty.");
  }
#endif
}
