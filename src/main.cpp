
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


// #define LED_PIN 2
// #define BUTTON_PIN 4

// void setup() {
//   pinMode(LED_PIN, OUTPUT);
//   pinMode(BUTTON_PIN, INPUT);
//   Serial.begin(115200);
// }

// void loop() {
//   int State = digitalRead(BUTTON_PIN);

//   if (State == HIGH) { 
//     analogWrite(LED_PIN, HIGH);
//     delay(250);
//     analogWrite(LED_PIN, LOW);
//     delay(250);
//     Serial.println("Blinking");
// } else {
//     analogWrite(LED_PIN, LOW);
//     Serial.println("OFF");
//   }
// }


#define LED_BUILTIN 2  
#define BUTTON_PIN 4

int dutyCycle = 0; 


void setup() {
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);      
}

void loop() {
  int state = digitalRead(BUTTON_PIN);
  Serial.print("State: ");
  Serial.println(state);

  if (state == HIGH) { 
    // Fade up
     for (; dutyCycle <= 255; dutyCycle++) {
          analogWrite(LED_BUILTIN, dutyCycle); 
                delay(30);  
                
                if (digitalRead(BUTTON_PIN) == LOW) break;
    }
  } else if (state == LOW) { 
    // Fade down
    for (;dutyCycle >= 0; dutyCycle--){ 
            analogWrite(LED_BUILTIN, dutyCycle);   
            delay(100);  

                  if (digitalRead(BUTTON_PIN) == HIGH) break;
    }
  }
}
