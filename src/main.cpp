
#include <Arduino.h>

// #define LED_BUILTIN 2
// #define BUTTON_PIN 4

// void setup() {
//   pinMode(LED_BUILTIN, OUTPUT);
//   pinMode(BUTTON_PIN, INPUT);
// }

// void loop() {
//   int state = digitalRead(BUTTON_PIN);

//   if (state == HIGH) { 
//     digitalWrite(LED_BUILTIN, HIGH);
//     delay(500);

//   } else {         
//     digitalWrite(LED_BUILTIN, LOW);
//     delay(500);

//   }
// }


#define LED_PIN 2
#define BUTTON_PIN 4

void setup() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT);
  Serial.begin(115200);
}

void loop() {
  int State = digitalRead(BUTTON_PIN);

  if (State == HIGH) { 
    digitalWrite(LED_PIN, HIGH);
    delay(250);
    digitalWrite(LED_PIN, LOW);
    delay(250);
    Serial.println("Blinking");
} else {
    digitalWrite(LED_PIN, LOW);
    Serial.println("OFF");
  }
}