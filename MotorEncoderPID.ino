/** Hybrid servomotor using dc motor + encoder + PID
 *  
 *  Both libraries can be installed directly from the Library Manager
 *  Arduino PID Library by Brett Beauregard
 *  Encoder Library by Paul Stoffregen
 *  
 *  MIT License 2018 CRCibernetica
*/

#include <Encoder.h>
#include <PID_v1.h>
#define SPD_PIN 6
#define DIR_PIN 11
#define SET_PIN 7

Encoder myEnc(2, 3);

double Setpoint, Input, Output;

double Kp = 0.25, Ki = 0.15, Kd = 0.03;
PID posPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

void stopMotor() {
  digitalWrite(DIR_PIN, LOW);
  analogWrite(SPD_PIN, 0);
}

void drive(int control) {
  if (control > 0) {
    digitalWrite(DIR_PIN, HIGH);
  } else {
    digitalWrite(DIR_PIN, LOW);
  }
  analogWrite(SPD_PIN, abs(control));
}


void setup() {
  Serial.begin(9600);
  Serial.println("Starting...");
  pinMode(DIR_PIN, OUTPUT);
  pinMode(SET_PIN, INPUT_PULLUP);
  stopMotor();
  posPID.SetOutputLimits(-255, 255);
  posPID.SetMode(AUTOMATIC);

}

long oldPosition  = -999;

void loop() {
  if (digitalRead(7) == 1) {
    Setpoint = 1000;
  } else {
    Setpoint = 0;
  }

  long newPosition = myEnc.read();
  Input = newPosition;
  Serial.print(Input);
  Serial.print("\t");
  posPID.Compute();
  Serial.println(Output);
  drive(Output);

  //  if (newPosition != oldPosition) {
  //    oldPosition = newPosition;
  //    Serial.println(newPosition);
  //  }
}
