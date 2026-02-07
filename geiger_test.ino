#include <DFRobot_Geiger.h>
#if defined ESP32
#define detect_pin D3
#else
#define detect_pin 26 // Must configure to appropriate rp2040 board pin; gpio 26 is read from pico
#endif

// WILL NOT BE GPIO 26 USE ANOTHER PIN FOR MORE GEIGERS
/*!
   @brief Constructor
   @param pin   External interrupt pin
*/
DFRobot_Geiger  geiger(detect_pin);

void setup()
{
  Serial.begin(115200);
  //Start counting, enable external interrupt
  geiger.start();
}

void loop() {
  //Start counting, enable external interrupt
  //geiger.start();
  delay(3000);
  //Pause the count, turn off the external interrupt trigger, the CPM and radiation intensity values remain in the state before the pause
  //geiger.pause();
  //Get the current CPM, if it has been paused, the CPM is the last value before the pause
  //Predict CPM by falling edge pulse within 3 seconds, the error is ±3CPM
  Serial.println(geiger.getCPM());
  //Get the current nSv/h, if it has been paused, nSv/h is the last value before the pause
  Serial.println(geiger.getnSvh());
  //Get the current μSv/h, if it has been paused, the μSv/h is the last value before the pause
  Serial.println(geiger.getuSvh());
}