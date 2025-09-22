
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

// #include <Arduino.h>

// #define LED_PIN 2
// #define SW1_PIN 4
// #define SW2_PIN 5

// const int PWM_freq = 1000;        // 1 kHz
// const int PWM_resolution = 8;     // 8-bit (0–255)
// const int ledChannel = 0;         // PWM channel

// int dutyCycle = 0;

// void setup() {
//   pinMode(SW1_PIN, INPUT);
//   pinMode(SW2_PIN, INPUT);  
//   pinMode(LED_PIN, OUTPUT);

//   ledcSetup(ledChannel, PWM_freq, PWM_resolution);
//   ledcAttachPin(LED_PIN, ledChannel);
// }

// void loop() {
//   int SW1 = digitalRead(SW1_PIN);
//   int SW2 = digitalRead(SW2_PIN);

//   if (SW1 == HIGH && dutyCycle < 255) {   // fade up
//     while(dutyCycle<255){
//     dutyCycle++;
//     ledcWrite(ledChannel, dutyCycle);
//     delay(1);
//     }
//   }

//   if (SW2 == HIGH && dutyCycle > 0) {  
//     while(dutyCycle>0){   // fade down
//     dutyCycle--;
//     ledcWrite(ledChannel, dutyCycle);
//     delay(1);
//     }
//   }
// }

#define PWM 2
int dutyCycle = 127;  // start at 50%

const int PWM_freq = 1000;      // 1000 Hz
const int PWM_resolution = 8;   // 8-bit (0–255)
const int ledChannel = 0;       // PWM channel

#define SW1 26  
#define SW2 27 

void setup() {
  pinMode(PWM, OUTPUT);
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);

  ledcSetup(ledChannel, PWM_freq, PWM_resolution);
  ledcAttachPin(PWM, ledChannel);

  ledcWrite(ledChannel, dutyCycle);
}

void loop() { 
  if (digitalRead(SW1) == HIGH) {
    dutyCycle -= 10;
    if (dutyCycle < 0) dutyCycle = 0;
    ledcWrite(ledChannel, dutyCycle);
    delay(200);  
  }

 
  if (digitalRead(SW2) == HIGH) {
    dutyCycle += 10;
    if (dutyCycle > 255) dutyCycle = 255;
    ledcWrite(ledChannel, dutyCycle);
    delay(200);  
  }
}
