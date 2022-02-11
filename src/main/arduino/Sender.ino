#include<Wire.h>

#define SENSOR_PIN1 2
#define SENSOR_PIN2 3
#define SENSOR_PIN3 4
#define SENSOR_PIN4 5
#define SENSOR_PIN5 6
#define SENSOR_PIN6 7
#define SENSOR_PIN7 8
#define SENSOR_PIN8 9
#define NUM_OF_SENSORS 2
int pinArray[NUM_OF_SENSORS] = {SENSOR_PIN1, SENSOR_PIN2};
byte valArray[NUM_OF_SENSORS];

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin LED_BUILTIN as an output.
  Wire.begin(8);
  Wire.onRequest(sendDigital);
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT);
  for (int i = 0; i < NUM_OF_SENSORS; i++)
  {
     pinMode(pinArray[i], INPUT_PULLUP);
  }
}
// the loop function reads and puts sensor data from pinArray into valArray
void loop() {
  for (int i = 0; i < NUM_OF_SENSORS; i++)
  {
    valArray[i] = digitalRead(pinArray[i]);
    Serial.println(valArray[i]);
  }
  delay(500);
}

// Sends valArray data to the reciever when reciever asks for the data
void sendDigital()
  {
    Wire.write(valArray, 2);
  }
