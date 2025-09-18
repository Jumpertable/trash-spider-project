
#include <Arduino.h>

// // #define LED_BUILTIN 2
// // #define BUTTON_PIN 4

// // void setup() {
// //   pinMode(LED_BUILTIN, OUTPUT);
// //   pinMode(BUTTON_PIN, INPUT);
// // }

// // void loop() {
// //   int state = digitalRead(BUTTON_PIN);

// //   if (state == HIGH) { 
// //     digitalWrite(LED_BUILTIN, HIGH);
// //     delay(500);

// //   } else {         
// //     digitalWrite(LED_BUILTIN, LOW);
// //     delay(500);

// //   }
// // }


// // #define LED_PIN 2
// // #define BUTTON_PIN 4

// // void setup() {
// //   pinMode(LED_PIN, OUTPUT);
// //   pinMode(BUTTON_PIN, INPUT);
// //   Serial.begin(115200);
// // }

// // void loop() {
// //   int State = digitalRead(BUTTON_PIN);

// //   if (State == HIGH) { 
// //     analogWrite(LED_PIN, HIGH);
// //     delay(250);
// //     analogWrite(LED_PIN, LOW);
// //     delay(250);
// //     Serial.println("Blinking");
// // } else {
// //     analogWrite(LED_PIN, LOW);
// //     Serial.println("OFF");
// //   }
// // }


// #define LED_BUILTIN 2  
// #define BUTTON_PIN 4

// int dutyCycle = 0; 


// void setup() {
//   pinMode(BUTTON_PIN, INPUT);
//   pinMode(LED_BUILTIN, OUTPUT);
//   Serial.begin(115200);      
// }

// void loop() {
//   int state = digitalRead(BUTTON_PIN);
//   Serial.print("State: ");
//   Serial.println(state);

//   if (state == HIGH) { 
//     // Fade up
//      for (; dutyCycle <= 255; dutyCycle++) {
//           analogWrite(LED_BUILTIN, dutyCycle); 
//                 delay(30);  
                
//                 if (digitalRead(BUTTON_PIN) == LOW) break;
//     }
//   } else if (state == LOW) { 
//     // Fade down
//     for (;dutyCycle >= 0; dutyCycle--){ 
//             analogWrite(LED_BUILTIN, dutyCycle);   
//             delay(100);  

//                   if (digitalRead(BUTTON_PIN) == HIGH) break;
//     }
//   }
// }

#define LED_PIN 2
#define SW1_PIN 4
#define SW2_PIN 16

const int PWM_freq = 1000;        // 1000 Hz
const int PWM_resolution = 8;     // 8-bit: (0–255)
const int ledChannel = 0;         // set the PWM channel

int dutyCycle = 0;

void setup() {
  pinMode(SW1_PIN, INPUT); 
  pinMode(SW2_PIN, INPUT); 
  pinMode(LED_PIN, OUTPUT);


  ledcSetup(ledChannel, PWM_freq, PWM_resolution);
  ledcAttachPin(LED_PIN, ledChannel);

  Serial.begin(115200);
}

void loop() {
  int SW1 = digitalRead(SW1_PIN);
  int SW2 = digitalRead(SW2_PIN);
  Serial.print("SW1: ");
  Serial.println(SW1);
  
  Serial.print("SW2: ");
  Serial.println(SW2);

  if (SW1 == HIGH) {  
    Serial.println("SW1 pressed, fade upppp");
    for (; dutyCycle <= 255; dutyCycle++) {
      ledcWrite(ledChannel, dutyCycle);
      delay(10);

    }
  } 
  else if (SW2 == HIGH) {  
    Serial.println("SW2 pressed, fade dooooown");
    for (; dutyCycle >= 0; dutyCycle--) {
      ledcWrite(ledChannel, dutyCycle);
      delay(10);

      if (digitalRead(SW1) == HIGH) break;
    }
  }
}
