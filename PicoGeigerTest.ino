#include <Wire.h>

#define GEIGER_1_INTERRUPT_PIN 26
const uint8_t i2cAddr2 = 0x70;
volatile int geigerCount = 0;   // modified in each ISR
int copyCount = 0;

volatile uint8_t registerNumber = 0; // modified in onReceive event


void setup() {

  pinMode(LED_BUILTIN, OUTPUT);
  // Enable interrupts and associate a GPIO pin with a ISR
  pinMode(GEIGER_1_INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(GEIGER_1_INTERRUPT_PIN),ISR_geiger1, FALLING);


  Wire1.setSDA(2);
  Wire1.setSCL(3);
  //Wire1.begin(i2cAddr1);
  Wire1.begin(i2cAddr2);

  Wire1.onReceive(onReceive);
  Wire1.onRequest(I2CRequest);
}

void loop() {

  // Turn LED on and off 
  digitalWrite(LED_BUILTIN, HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN, LOW);
  delay(1000);


}


void ISR_geiger1() {
  // Hardware disables interrupt per event
  geigerCount++;
}


void I2CRequest() {


  if (registerNumber == 2) {

    // Disable interrupts
    noInterrupts();
    copyCount = geigerCount;
    // Clear Counts
    geigerCount = 0;
    Wire1.write(copyCount);
     // Enable interrupts
    interrupts();
  
  }
}

void onReceive(int numBytes) {


  while (Wire1.available()) {
    registerNumber = Wire1.read();
  }
}
