
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
#include <Arduino.h>

// #define LED_PIN 2
// #define SW1_PIN 4
// #define SW2_PIN 5

// const int PWM_freq = 5000;        // 5kHz
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
//     delay(4);
//     }
//   }

//   if (SW2 == HIGH && dutyCycle > 0) {  
//     while(dutyCycle>0){   // fade down
//     dutyCycle--;
//     ledcWrite(ledChannel, dutyCycle);
//     delay(4);
//     }
//   }
// }

// #define PWM_PIN 2         
// #define SW1 26             
// #define SW2 27             

// const int PWM_freq = 1000;      // 1kHz
// const int PWM_resolution = 8;   // 8-bit resolution (0-255)
// const int ledChannel = 0;       // PWM channel

// void setup() {
//   pinMode(SW1, INPUT);
//   pinMode(SW2, INPUT);

//   ledcSetup(ledChannel, PWM_freq, PWM_resolution);
//   ledcAttachPin(PWM_PIN, ledChannel);
// }

// void loop() {
//   while (1) {
//     bool s1 = (digitalRead(SW1) == HIGH);  
//     bool s2 = (digitalRead(SW2) == HIGH);

//     int duty;
//     if (s1 && !s2) {
//       duty = (9 * 255) / 100;
//     } else if (s2 && !s1) {
//       duty = (91 * 255) / 100;
//     } else {
//       duty = (50 * 255) / 100;
//     }

//     ledcWrite(ledChannel, duty);
//   }
// }

// #define PWM_PIN 2
// #define SW1 4
// #define SW2 5

// const int PWM_resolution = 8;
// const int ledChannel = 0;

// void setup() {
//   pinMode(SW1, INPUT);
//   pinMode(SW2, INPUT);

//   ledcSetup(ledChannel, 1000, PWM_resolution);
//   ledcAttachPin(PWM_PIN, ledChannel);
//   ledcWrite(ledChannel, 128);
// }

// void loop() {
//   bool s1 = (digitalRead(SW1) == HIGH);
//   bool s2 = (digitalRead(SW2) == HIGH);

//   int freq = 1000;
//   int duty = 128;   // 255 divided by 2 50%

//   if (s1) {
//     freq = 9000;    // 9 times 1kHz
//   } else if (s2) {
//     freq = 18000;   // 9 times 2kHz
//   }

//   ledcSetup(ledChannel, freq, PWM_resolution);
//   ledcWrite(ledChannel, duty);

//   delay(50);
// }


// const int sensorPin = 34;   
// const int ledPin = 2;     

// const int ledChannel = 0;  
// const int PWM_freq = 5000;  
// const int PWM_resolution = 8; 

// int ADCValue = 0;
// float voltage = 0.0;
// int ledBrightness = 0;

// void setup() {
//   Serial.begin(9600);

//   ledcSetup(ledChannel, PWM_freq, PWM_resolution);
//   ledcAttachPin(ledPin, ledChannel);
// }

// void loop() {
//   ADCValue = analogRead(sensorPin);
//   voltage = (ADCValue * 3.3) / 4095.0;
//   ledBrightness = map(ADCValue, 0, 4095, 50, 255);

//   ledcWrite(ledChannel, ledBrightness);

//   Serial.print("ADC Value = ");
//   Serial.println(ADCValue);

//   Serial.print("Voltage = ");
//   Serial.print(voltage, 2);
//   Serial.println(" V");

//   Serial.print("LED Brightness (PWM) = ");
//   Serial.println(ledBrightness);

//   Serial.println(">--------------------------<");

//   delay(1000);
// }

// #define PWM_PIN 2
// #define SW1 4
// #define SW2 5

// const int PWM_resolution = 8;
// const int ledChannel = 0;

// void setup() {
//   pinMode(SW1, INPUT);
//   pinMode(SW2, INPUT);

//   ledcSetup(ledChannel, 1000, PWM_resolution);
//   ledcAttachPin(PWM_PIN, ledChannel);
//   ledcWrite(ledChannel, 128);
// }

// void loop() {
//   bool s1 = (digitalRead(SW1) == HIGH);
//   bool s2 = (digitalRead(SW2) == HIGH);

//   int freq = 1000;
//   int duty = 128;   // 255 divided by 2, 50%

//   if (s1) {
//     freq = 9000;    // 9 times 1kHz
//   } else if (s2) {
//     freq = 18000;   // 9 times 2kHz
//   }

//   ledcSetup(ledChannel, freq, PWM_resolution);
//   ledcAttachPin(PWM_PIN, ledChannel);
//   ledcWrite(ledChannel, duty);

//   delay(100);
// }

// const uint8_t InterruptPin = 12;
// bool Request;
// volatile int counter = 0;

// void IRAM_ATTR isr() {
//   Request = true;
//   counter++;
// }

// void setup() {
//   Serial.begin(9600);
//   pinMode(InterruptPin, INPUT);
//   attachInterrupt(InterruptPin, isr, RISING);
// }

// void loop() {
//   if (Request) {
//     Serial.print("Counter: ");
//     Serial.println(counter);
//     Serial.println("Interrupt Request Received!");
//     Request = false;
//   }
// }

//1
// #define IN1 25    
// #define IN2 33   

// #define BTN_CW 16   
// #define BTN_CCW 17  

// const int PWM_freq = 1000;         
// const int PWM_resolution = 8;     
// const int PWM_Channel0 = 0;         
// const int PWM_Channel1 = 1;       
// const int PMW_Value = 255;       

// void setup() {
//   Serial.begin(9600);

//   pinMode(BTN_CW, INPUT);
//   pinMode(BTN_CCW, INPUT);

//   ledcSetup(PWM_Channel0, PWM_freq, PWM_resolution);
//   ledcAttachPin(IN1, PWM_Channel0);
  
//   ledcSetup(PWM_Channel1, PWM_freq, PWM_resolution);
//   ledcAttachPin(IN2, PWM_Channel1);
// }

// void loop() {
//   bool CW = digitalRead(BTN_CW) == HIGH;
//   bool CCW = digitalRead(BTN_CCW) == HIGH;

//   if (CW && !CCW) {
//     ledcWrite(PWM_Channel0, PMW_Value);
//     ledcWrite(PWM_Channel1, 0);        
//     Serial.println("Clockwise");
//   } else if (CCW && !CW) {
//     ledcWrite(PWM_Channel0, 0);        
//     ledcWrite(PWM_Channel1, PMW_Value); 
//     Serial.println("Counterclockwise");
//   } else {
//     ledcWrite(PWM_Channel0, 0);
//     ledcWrite(PWM_Channel1, 0);
//     Serial.println("Motor Stopped");
//   }
//   delay(100); 
// }

//2
// #define IN1 25   
// #define IN2 33    

// #define BTN_CW 16   
// #define BTN_CCW 17  

// #define POT_PIN 34 

// const int PWM_freq = 1000;         
// const int PWM_resolution = 8;      
// const int PWMChannel0 = 0;        
// const int PWMChannel1 = 1;        

// void setup() {
//   Serial.begin(9600);

//   pinMode(BTN_CW, INPUT_PULLUP);
//   pinMode(BTN_CCW, INPUT_PULLUP);

//   ledcSetup(PWMChannel0, PWM_freq, PWM_resolution);
//   ledcAttachPin(IN1, PWMChannel0);
  
//   ledcSetup(PWMChannel1, PWM_freq, PWM_resolution);
//   ledcAttachPin(IN2, PWMChannel1);

// }

// void loop() {
//   bool cwPressed = digitalRead(BTN_CW) == LOW;
//   bool ccwPressed = digitalRead(BTN_CCW) == LOW;

//   int potValue = analogRead(POT_PIN);
//   int pwmValue = map(potValue, 0, 4095, 0, 255);

//   if (cwPressed && !ccwPressed) {
//     ledcWrite(PWMChannel0, pwmValue);
//     ledcWrite(PWMChannel1, 0);
//     Serial.print("Clockwise, Speed: ");
//     Serial.println(pwmValue);
//   } else if (ccwPressed && !cwPressed) {
//     ledcWrite(PWMChannel0, 0);
//     ledcWrite(PWMChannel1, pwmValue);
//     Serial.print("Counterclockwise, Speed: ");
//     Serial.println(pwmValue);
//   } else {
//     ledcWrite(PWMChannel0, 0);
//     ledcWrite(PWMChannel1, 0);
//     Serial.println("Motor Stopped");
//   }

//   delay(100);
// }



//4

// #include <Adafruit_Sensor.h>
// #include <DHT.h>
// #include <DHT_U.h>

// #define IN1 25            // Motor IN1 (PWM)
// #define IN2 33            // Motor IN2 (PWM)
// #define POT_PIN 34        // Potentiometer input
// #define DHT_PIN 14        // DHT22 data pin
// #define LED_PIN 13        // LED output

// #define DHTTYPE DHT22
// DHT dht(DHT_PIN, DHTTYPE);

// const int PWM_freq = 1000;       
// const int PWM_resolution = 8;    
// const int PWMChannel0 = 0;    
// const int PWMChannel1 = 1;      

// #define TEMP_THRESHOLD 27.0

// void setup() {
//   Serial.begin(9600);

//   dht.begin();

//   pinMode(LED_PIN, OUTPUT);

//   ledcSetup(PWMChannel0, PWM_freq, PWM_resolution);
//   ledcAttachPin(IN1, PWMChannel0);

//   ledcSetup(PWMChannel1, PWM_freq, PWM_resolution);
//   ledcAttachPin(IN2, PWMChannel1);
// }

// void loop() {
//   float temperature = dht.readTemperature(); 
//   if (isnan(temperature)) {
//     Serial.println("Failed to read from DHT sensor!");
//     delay(2000);
//     return;
//   }

//   int potValue = analogRead(POT_PIN);     
//   int pwmValue = map(potValue, 0, 4095, 0, 255);
//   pwmValue = constrain(pwmValue, 0, 255);

//   Serial.print("Temperature: ");
//   Serial.print(temperature);
//   Serial.print(" °C, PWM Speed: ");
//   Serial.println(pwmValue);

//   if (temperature > TEMP_THRESHOLD) {
//     digitalWrite(LED_PIN, HIGH);               
//     ledcWrite(PWMChannel0, pwmValue);           
//     ledcWrite(PWMChannel1, 0);                  
//     Serial.println("Motor ON, LED ON");
//   } else {
//     digitalWrite(LED_PIN, LOW);                 
//     ledcWrite(PWMChannel0, 0);                  
//     ledcWrite(PWMChannel1, 0);                  
//     Serial.println("Motor OFF, LED OFF");
//   }

//   delay(500); 
// }



//3
// #include <Arduino.h>
// #include <DHT.h>

// #define POT_PIN 34   // Potentiometer (ADC)
// #define DHT_PIN 14   // DHT22 Sensor
// #define LED_PIN 13   // LED for temperature check

// #define DHTTYPE DHT22

// DHT dht(DHT_PIN, DHTTYPE);

// void setup() {
//   Serial.begin(9600);
//   pinMode(LED_PIN, OUTPUT);
//   dht.begin();
// }

// void loop() {
//   // Read potentiometer (not really used yet, but kept in case)
//   int potValue = analogRead(POT_PIN);
//   int pwmValue = map(potValue, 0, 4095, 0, 255);
//   pwmValue = constrain(pwmValue, 0, 255);

//   // Read temperature
//   float temperature = dht.readTemperature();

//   if (isnan(temperature)) {
//     Serial.println("Failed to read from DHT sensor!");
//   } else {
//     Serial.print("Temperature: ");
//     Serial.print(temperature);
//     Serial.println(" °C");

//     if (temperature < 28.0) {
//       digitalWrite(LED_PIN, HIGH);
//       Serial.println("LED ON (Temperature < 28°C)");
//     } else {
//       digitalWrite(LED_PIN, LOW);
//       Serial.println("LED OFF (Temperature >= 28°C)");
//     }
//   }

//   delay(500);
// }















#include <Arduino.h>
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include <IRremote.h>

//lid
#define TRIG 5
#define ECHO 18

//Servo
#define SERVO_PIN1 4
#define SERVO_PIN2 19

//IR receiver
#define PIN_RECEIVER 13

//motor
#define MOTOR_LEFT_FORWARD  25
#define MOTOR_LEFT_BACKWARD 26
#define MOTOR_RIGHT_FORWARD 32
#define MOTOR_RIGHT_BACKWARD 33

//led
#define L_LED 23
#define R_LED 27

//lcd
LiquidCrystal_I2C lcd(0x27, 16, 2);

Servo lidServo1;
Servo lidServo2;
long lidDistance;


//ir
bool powerState = false;

// void forward();
// void turnLeft();
// void turnRight();
// void stopMotors();

// #define L_CHANNEL 0
// #define R_CHANNEL 1

//ultra-sonic functions
long readUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000);
  if (duration == 0) return -1;
  long dist = duration * 0.034 / 2; //cm
  return dist;
}

//Motor functions
void moveForward() {
  digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
}

void turnRight() {
  digitalWrite(MOTOR_LEFT_FORWARD, HIGH);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, HIGH);
}

void turnLeft() {
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_FORWARD, HIGH);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
}

void stopRobot() {
  digitalWrite(MOTOR_LEFT_FORWARD, LOW);
  digitalWrite(MOTOR_LEFT_BACKWARD, LOW);
  digitalWrite(MOTOR_RIGHT_FORWARD, LOW);
  digitalWrite(MOTOR_RIGHT_BACKWARD, LOW);
}

void setup() {
  Serial.begin(115200);

  //Servo
  lidServo1.attach(SERVO_PIN1);     // first servo
  lidServo2.attach(SERVO_PIN2); 
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  //////IR reicver
  IrReceiver.begin(PIN_RECEIVER, ENABLE_LED_FEEDBACK);

  // Motors
  pinMode(MOTOR_LEFT_FORWARD, OUTPUT);
  pinMode(MOTOR_LEFT_BACKWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD, OUTPUT);

  //led
  pinMode(L_LED, OUTPUT);
  pinMode(R_LED, OUTPUT);

   //lcd
  lcd.init();
  lcd.backlight();
  lcd.print("Ready, Freddie!");


  // ledcAttachPin(L_PWM, L_CHANNEL);
  // ledcAttachPin(R_PWM, R_CHANNEL);
  // ledcSetup(L_CHANNEL, 1000, 8); // 1kHz, 8-bit
  // ledcSetup(R_CHANNEL, 1000, 8);
}

void loop() {
  //lid
  lidDistance = readUltrasonic(TRIG, ECHO);
  Serial.print("Distance: ");
  Serial.print(lidDistance);
  Serial.println(" cm");

  if (lidDistance > 0 && lidDistance <= 30) {
    lidServo1.write(90);//open
    Serial.println("Open");
  } else {
    lidServo1.write(0);//close
    Serial.println("Closed");
  }

  if (lidDistance > 0 && lidDistance <= 30) {
    lidServo2.write(90);//open
    Serial.println("Open");
  } else {
    lidServo2.write(0);//close
    Serial.println("Closed");
  }

  //ir
if (IrReceiver.decode()) {
    uint16_t cmd = IrReceiver.decodedIRData.command;
    Serial.print("Command: ");
    Serial.println(cmd);

  switch (cmd) {

   case 162: //power
    powerState = !powerState; 
    lcd.clear();
    if (powerState) {
    lcd.print("Power On!");
  } else {
    lcd.print("Power Off!");
  }
    break;

    case 2:
      moveForward(); //plus
      digitalWrite(R_LED, HIGH);
      digitalWrite(L_LED, HIGH);
      lcd.clear();
      lcd.print("Moving Forward");
      break;

    case 144:
      turnRight(); //next. >>
      digitalWrite(R_LED, LOW);
      digitalWrite(L_LED, HIGH);
      lcd.clear();
      lcd.print("Turning Right");
      break;

    case 224:
      turnLeft();//prev. <<
      digitalWrite(R_LED, HIGH);
      digitalWrite(L_LED, LOW);
      lcd.clear();
      lcd.print("Turning Left");
      break;

    case 168:
      stopRobot();//play |>
      digitalWrite(R_LED, LOW);
      digitalWrite(L_LED, LOW);
      lcd.clear();
      lcd.print("Stop!");
      break;

    case 66:
    lcd.clear();
    lcd.setCursor(4, 0);
    lcd.print("A_____A");
    lcd.setCursor(2, 1);
    lcd.print(">(\*\" ^ \"*\)<");
    break;

    default: //any
      lcd.clear();
      lcd.print("Cmd: ");
      lcd.print(cmd);
      break;
  }
  IrReceiver.resume(); 
}


  delay(200);
}


  // //follow line
  // int l_Val = digitalRead(L_SENSOR);
  // int r_Val = digitalRead(R_SENSOR);

  // if (l_Val == HIGH && r_Val == HIGH) {
  //   forward();
  // } 
  // else if (l_Val == LOW && r_Val == HIGH) {
  //   turnLeft();
  // } 
  // else if (l_Val == HIGH && r_Val == LOW) {
  //   turnRight(); 
  // } 
  // else {
  //   stopMotors();
  // }