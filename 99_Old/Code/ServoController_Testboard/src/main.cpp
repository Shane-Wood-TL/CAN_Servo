#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h> 

void receiveEvent();

class serialValues{
  public:
  double goalAngle = 135;
  double Kp = 0;
  double Ki = 0;
  double Kd = 0;
  serialValues(double goalAngleI, double KpI, double KiI, double KdI){
    goalAngle = goalAngleI;
    Kp = KpI;
    Ki = KiI;
    Kd = KdI;
  }
  
};

serialValues parseData(String input);

#define PWMIN p29
#define TempPin p26
#define OUT1 p6
#define OUT2 p7
#define CurrentIN p28
#define MPSleep p8
#define I2cSlaveAddress 0x2F
#define PotValue p27

#define Vref 3.3

MbedI2C i2c(p2,p3);

float Vo;
float R1 = 10000;
float logR2, R2, T, Tc;
double c1 = 0.00009076800317960867, c2 = 0.0003820237749737886, c3 = -3.107248132974333e-7;

float goalAngle = 90;
float current;
bool increasing = true;

struct receivedAngle {
  float position;
};
//PID values
double Kp =1.8;
double Ki = 0.05;
double Kd = 0.1;

// Declare a variable to hold the received data
receivedAngle receivedAngleD;

serialValues receivedSerialData(goalAngle, Kp,Ki,Kd);



double setPoint;
double potInput;
double pidOutput;

PID motorPID(&potInput, &pidOutput, (&receivedSerialData.goalAngle),*(&receivedSerialData.Kp),*(&receivedSerialData.Ki),*(&receivedSerialData.Kd),DIRECT);

void setup() {
  pinMode(PWMIN, INPUT);
  pinMode(TempPin, INPUT);
  pinMode(PotValue, INPUT);
  pinMode(CurrentIN, INPUT);

  pinMode(OUT1, OUTPUT);
  pinMode(OUT2, OUTPUT);
  pinMode(MPSleep, OUTPUT);
  digitalWrite(MPSleep, HIGH);


  // Start the Wire library as a slave with the specified address
  //Wire.begin(I2cSlaveAddress);  // Replace 0x42 with the I2C address you want to assign to the slave

  // Register the event handler for receiving data
  //Wire.onReceive(receiveEvent);

  Serial.begin(9600);
  digitalWrite(OUT1,LOW);
  digitalWrite(OUT2,LOW);

  motorPID.SetMode(AUTOMATIC);
  motorPID.SetOutputLimits(-255,255);
  //motorPID.SetSampleTime(20);
  analogReadResolution(10);
}

void loop() {
 float atPos = analogRead(PotValue);
 setPoint = goalAngle;
//Serial.println(atPos);
 potInput = map(atPos, 0,1023,-255,255);

 motorPID.Compute();



 if (pidOutput > 0){
    analogWrite(OUT1,0);
    analogWrite(OUT2,abs(pidOutput));
  }else if (pidOutput < 0){
    analogWrite(OUT2,0);
    analogWrite(OUT1,abs(pidOutput));
  }
    //temp
  Vo = analogRead(26);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Tc = T - 273.15;

  //Current
  current = analogRead(28);
  float voltage = (current / 1023.0) * Vref;  // Convert analog reading to voltage
  float currentF = voltage / 0.2;  // Calculate current using the sensor's sensitivity

  Serial.print("Core Temp: ");
  float tempCore = analogReadTemp();
  Serial.print(analogReadTemp());
  Serial.print("Temp: ");
  Serial.print(Tc);
  Serial.print(" Current: ");
  Serial.println(currentF);

  if (Serial.available() > 0) {
          //     // Read the incoming data into a string
               String input = Serial.readStringUntil('\n');
               receivedSerialData = parseData(input);
          }
}




serialValues parseData(String input) {
  serialValues receivedValues(0,0,0,0);
  const int numFloats = 4;
  double values[numFloats];
  // Create an index for storing float values
  int index = 0;
  
  // Split the input string using a delimiter (assuming comma-separated values)
  int startIndex = 0;
  int endIndex = input.indexOf(',');

  while (endIndex != -1 && index < numFloats) {
    // Convert the substring to a float and store it in the values array
    values[index] = input.substring(startIndex, endIndex).toDouble();
    index++;

    // Update the indices for the next substring
    startIndex = endIndex + 1;
    endIndex = input.indexOf(',', startIndex);
  }

  // Handle the last value after the last comma
  if (index < numFloats) {
    values[index] = input.substring(startIndex).toDouble();  // This handles the last value in the string
  }

  receivedValues.goalAngle = values[0];
  receivedValues.Kp = values[1];
  receivedValues.Ki = values[2];
  receivedValues.Kd = values[3];
  motorPID.SetTunings(receivedValues.Kp ,receivedValues.Ki,receivedValues.Kd);

  return receivedValues;
}







  
  // put your main code here, to run repeatedly:
  //digitalWrite(OUT1, HIGH);
  // digitalWrite(OUT2,LOW);

  // //digitalWrite(OUT1, HIGH);
  // analogWrite(OUT1,255);
  // //digitalWrite(MPSleep, HIGH);
  





//float currentAngle = map(atPos, 0,1023,0,270);

// Serial.println(atPos);
//   if (atPos < goalAngle){
//     analogWrite(OUT1,0);
//     analogWrite(OUT2,255);
//   }else if (atPos > goalAngle){
//     analogWrite(OUT2,0);
//     analogWrite(OUT1,255);
//   }



void receiveEvent() {
  // // Ensure that the received data matches the size of the struct
  // if (Wire.available() >= sizeof(receivedAngle)) {
  //   // Read the received data into the struct
  //   Wire.readBytes((uint8_t*)&receivedAngleD, sizeof(receivedAngle));

  //   // Process the received data (you can modify this part based on your needs)
  //   Serial.println("Received Data:");
  //   Serial.print("Value 1: ");
  //   Serial.println(receivedAngleD.position);
  // }
  return;
}





   //digitalWrite(OUT2, LOW);
   //digitalWrite(OUT1, HIGH);
   //analogWrite(OUT1, 255);

// Serial.print("Angle:");
// Serial.print(currentAngle);
// Serial.print("Pot in:");
// Serial.print(potInput);
// Serial.print("Pid out: ");
// Serial.println(pidOutput);



    // Serial.println("test");
    // analogWrite(OUT2, 255);
    // digitalWrite(OUT1, LOW);

    //Serial.println(analogRead(PotValue));
    // delay(3000); // Delay for 1 second
    // Serial.println("off");
    // digitalWrite(OUT1, LOW);  
    // analogWrite(OUT2, 0);
    // delay(2000); // Delay for 1 second

    // digitalWrite(OUT2, LOW);  
    // analogWrite(OUT1, 255);
    // delay(3000); // Delay for 1 second
    //  Serial.println("test1");
  //  //delay(1000); // Delay for 1 second
  //      Serial.println("off1");
  //   digitalWrite(OUT1, LOW);  
  //   analogWrite(OUT2, 0);
  //   delay(2000); // Delay for 1 second
  //Position
// if (goalAngle < 160 and increasing) {
//     goalAngle++;
// } else if (goalAngle >= 160) {
//     increasing = false;
// } else if (goalAngle > 20 and !increasing) {
//     goalAngle--;
// } else if (goalAngle <= 20) {
//     increasing = true;
// }



  // //PWM input
  // int pwmValue = pulseIn(PWMIN, HIGH);  // Read PWM signal duration in microseconds
  // Serial.print("PWM Value: ");
  // Serial.println(pwmValue);