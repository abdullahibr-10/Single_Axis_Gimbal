//INclude Libraries
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// MOTOR variables
int ena = 25;
int in1 = 26;
int in2 = 27;

// Motor encoder output pulses per 360 degree revolution (measured manually)
#define ENC_COUNT_REV 226
#define tickPerDegree 226/360;
 
// Encoder output to Arduino Interrupt pin. Tracks the pulse count.
#define ENC_IN_RIGHT_A 5 // yellow wire
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_RIGHT_B 18 // green wire
 
// True = Forward; False = Reverse
boolean Direction_right = true;
 
// Keep track of the number of right wheel pulses
volatile long right_wheel_pulse_count = 0;

// For PID control
double sensed_output, control_signal;
int setpoint = 113;
double Kp = 25; //proportional gain
double Ki = 0.01; //integral gain
double Kd = 25; //derivative gain
int T = 10; //sample time in milliseconds (ms)
unsigned long last_time;
double total_error, last_error;
int max_control = 255;
int min_control = -255;
float dead_zone = 65;

// local varables
bool IN1 = HIGH;
int state = 0;
float mult = 0.1;
int num = 260;
int changeSetPoint = 0;

//MPU variables
Adafruit_MPU6050 mpu;

double vel = 0;
double pos = 0;

// Bluetooth
#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#if !defined(CONFIG_BT_SPP_ENABLED)
#error Serial Bluetooth not available or not enabled. It is only available for the ESP32 chip.
#endif

BluetoothSerial SerialBT;
char command;


void setup() {
 
  // Open the serial port at 9600 bps
  Serial.begin(9600); 
 
  // Set pin states of the encoder
  pinMode(ENC_IN_RIGHT_A, INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B, INPUT);

  pinMode(ena, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
 
  // Every time the pin goes high, this is a pulse
  attachInterrupt(ENC_IN_RIGHT_A, right_wheel_pulse, RISING);

  // Bluetooth
  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");

  // MPU 6050 interface
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

}
 
void loop() {

  if (SerialBT.available()) {
    command = SerialBT.read();
    SerialBT.println(command);
  }

  switch (command) {
    case 'r':
      state = 1;
      right_wheel_pulse_count = 0;
      pos = 0;
      break;
    case 's':
      state = 0;
      break;
    case 'p':
      Kp += mult;
      SerialBT.print("Kp = ");
      SerialBT.println(Kp);
      break;
    case 'i':
      Ki += mult;
      SerialBT.print("Ki = ");
      SerialBT.println(Ki);
      break;
    case 'd':
      Kd += mult;
      SerialBT.print("Kd = ");
      SerialBT.println(Kd);
      break;
    case 'P':
      Kp -= mult;
      SerialBT.print("Kp = ");
      SerialBT.println(Kp);
      break;
    case 'I':
      Ki -= mult;
      SerialBT.print("Ki = ");
      SerialBT.println(Ki);
      break;
    case 'D':
      Kd -= mult;
      SerialBT.print("Kd = ");
      SerialBT.println(Kd);
      break;
    case 'g':
      SerialBT.print("Kp = ");
      SerialBT.println(Kp);
      SerialBT.print("Ki = ");
      SerialBT.println(Ki);
      SerialBT.print("Kd = ");
      SerialBT.println(Kd);
      break;
    case 'o':
      SerialBT.println(sensed_output);
      break;
    case 'c':
      SerialBT.println(control_signal);
      break;
    case 'm':
      mult = mult / 10;
      SerialBT.println(mult);
      break;
    case 'M':
      mult = mult * 10;
      SerialBT.println(mult);
      break;
    case 'e':
      num -= 5;
      state = 2;
      SerialBT.println(num);
      break;
    case 'E':
      num += 5;
      state = 2;
      SerialBT.println(num);
      break;
    case 'x':
      if (changeSetPoint == 0) {
        changeSetPoint = 1;
        SerialBT.println("Set Point set to MPU readings");
      } else {
        changeSetPoint = 0;
        SerialBT.print("Set Point set to fixed point: ");
        SerialBT.println(setpoint);
      }
      break;
    default:
      break;
  }

// SETPOINT CONDITIONALS
    if (changeSetPoint == 1) {
        setpoint = getMPUpos() * tickPerDegree;
      }
    else {
        setpoint = 113;
      }
//STATE CONDITIONALS
    if (state == 1) {
      sensed_output = right_wheel_pulse_count;
      PID_Control();
//    
      if (control_signal < 0){
          IN1 = LOW;
        }
      else {
          IN1 = HIGH;
      }
      digitalWrite(in1, IN1);
      digitalWrite(in2, !IN1);
      analogWrite(ena, abs(control_signal));
      delay(12);
      Serial.println((sensed_output/113)*400);
    }
    
    else if (state == 2 ) {
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
    
      analogWrite(ena, num);
      }
    else {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
    
      analogWrite(ena, 0);
     }

//CHECKING
//     Serial.println(right_wheel_pulse_count);
//     Serial.println(' ');
//     Serial.println(control_signal);
//    getMPUpos();
  
}

//SOME VARIABLES
int flag =0;
int dir = 1;

void PID_Control(){

  unsigned long current_time = millis(); //returns the number of milliseconds passed since the Arduino started running the program
  
  int delta_time = current_time - last_time; //delta time interval 
  
  if (delta_time >= T){
  
    int error = setpoint - sensed_output;
    total_error += error; //accumalates the error - integral term
    if (total_error >= max_control) total_error = max_control;
    else if (total_error <= min_control) total_error = min_control;
    
    
    double delta_error = error - last_error; //difference of error for derivative term
    
    control_signal = Kp*error + (Ki*T)*total_error + (Kd/T)*delta_error; //PID control compute

//    if (error < 0) {
//        dir = -1;
//      }
//   else {
//      dir = 1;
//    }

    
    if (control_signal >= max_control) control_signal = max_control;
    else if (control_signal <= min_control) control_signal = min_control;

//    control_signal = control_signal * dir;
//    Serial.println(error);
    
    last_error = error;
    last_time = current_time;
//    delay(10);
  } 
}

// Increment the number of pulses by 1
void right_wheel_pulse() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
    right_wheel_pulse_count++;
  }
  else {
    right_wheel_pulse_count--;
  }
}

double getMPUpos () {
    /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  vel = (g.gyro.x - 0.0145);
  pos += vel * 0.01 * (180 / PI);
//  Serial.println(pos);
//  delay(10);

  return pos;
}
