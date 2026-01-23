#include <Wire.h>

constexpr uint8_t I2C_ADDR = 0x2C;
constexpr int POT_PIN = 1;

const byte LED_PIN = 22;
const byte INTERRUPT_PIN = 21;

volatile uint8_t potByte = 0;

volatile uint8_t registerNumber = 0;

volatile byte state = LOW;
volatile int hitCount = 0;

void handleInterrupt() {
  state = !state;
  hitCount++;
  digitalWrite(LED_PIN, state);
}

uint8_t readPotByte() {
  return 0x31;
}

void onI2CRequest() {
  //uint8_t value = potByte;
  //Wire.write(&value, 1);
  String sensorData = "";
  if (registerNumber == 1)
  {
    //sensorData = "S1&T1:70&T2:90&T3:200&T4:300";
    sensorData = String(hitCount);
  }
  else if (registerNumber ==2)
  {
    //sensorData = "S1&C1:170&C2:290&C3:200&C4:350";
    sensorData = String(hitCount);
  }
  else
  {
    //sensorData = "S1&M1:7&M2:9&M3:20&M4:8";
    sensorData = String(hitCount);
  }
  //int bufferLength = sensorData.length() + 1;
  int bufferLength = sensorData.length();

  for (int i = 0; i < 32 - bufferLength; i++)
  {
    sensorData.concat(" ");
  }
  char charArray[32];
  sensorData.toCharArray(charArray, 32);
  Wire.write(charArray, 32);

  hitCount = 0;
}

void onI2CReceive(int numBytes) {
  while (Wire.available())
    //(void)Wire.read();
    registerNumber = Wire.read();
}

void setup() {
  Serial.begin(115200);

  Wire.begin(I2C_ADDR);
  Wire.onRequest(onI2CRequest);
  Wire.onReceive(onI2CReceive);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  pinMode(INTERRUPT_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), handleInterrupt, FALLING);

  digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
}

void loop() {
  potByte = readPotByte();
  
  digitalWrite(LED_BUILTIN, HIGH);  // turn the LED on (HIGH is the voltage level)
  //delay(100);                      // wait for a second
  //digitalWrite(LED_BUILTIN, LOW);   // turn the LED off by making the voltage LOW
  //delay(100);                      // wait for a second

  //digitalWrite(LED_PIN, state);
}
