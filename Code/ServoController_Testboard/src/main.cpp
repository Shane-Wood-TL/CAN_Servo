#include <Arduino.h>
#include <Wire.h>

void receiveEvent();


#define PWMIN A3
#define TempPin A0
#define OUT1 5
#define OUT2 6
#define CurrentIN A2
#define MPSleep 9
#define I2cSlaveAddress 0x2F
#define PotValue A1

#define Vref 5


int Vo;
float R1 = 10000;
float logR2, R2, T, Tc;
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

float goalAngle = 90;
float current;
bool increasing = true;

struct receivedAngle {
  float position;
};

// Declare a variable to hold the received data
receivedAngle receivedAngleD;


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

}

void loop() {
  // put your main code here, to run repeatedly:
  //digitalWrite(OUT1, HIGH);
  // digitalWrite(OUT2,LOW);

  // //digitalWrite(OUT1, HIGH);
  // analogWrite(OUT1,255);
  // //digitalWrite(MPSleep, HIGH);



  // temp
  // Vo = analogRead(TempPin);
  // R2 = R1 * (1023.0 / (float)Vo - 1.0);
  // logR2 = log(R2);
  // T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  // Tc = T - 273.15;
  // Serial.println(Tc);

  //Current
  // current = analogRead(CurrentIN);
  // float voltage = (current / 1023.0) * Vref;  // Convert analog reading to voltage
  // float currentF = voltage / 0.2;  // Calculate current using the sensor's sensitivity
  // Serial.println(currentF);

  float atPos =analogRead(PotValue);
  atPos = map(atPos, 0,1023,0,360);
  Serial.println(map(atPos, 0,1023,0,360));

  if (atPos < goalAngle){
    digitalWrite(OUT2,LOW);
    analogWrite(OUT1,60);
  }else if (atPos > goalAngle){
    analogWrite(OUT2,60);
    digitalWrite(OUT1,LOW);
  }else{
    digitalWrite(OUT1, HIGH);
    digitalWrite(OUT2, HIGH);
  }
  //Position
  if (goalAngle < 160 and increasing){
    goalAngle++;
  }
  else if (goalAngle >= 155 or goalAngle <= 15){
    increasing = !(increasing);
  }else if(goalAngle > 15 and !(increasing)){
    goalAngle--;
  }

  // //PWM input
  // int pwmValue = pulseIn(PWMIN, HIGH);  // Read PWM signal duration in microseconds
  // Serial.print("PWM Value: ");
  // Serial.println(pwmValue);


}



void receiveEvent() {
  // Ensure that the received data matches the size of the struct
  if (Wire.available() >= sizeof(receivedAngle)) {
    // Read the received data into the struct
    Wire.readBytes((uint8_t*)&receivedAngleD, sizeof(receivedAngle));

    // Process the received data (you can modify this part based on your needs)
    Serial.println("Received Data:");
    Serial.print("Value 1: ");
    Serial.println(receivedAngleD.position);
  }
}