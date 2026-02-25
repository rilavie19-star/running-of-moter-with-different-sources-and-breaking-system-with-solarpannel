#include <LiquidCrystal.h>
//#define IR_sensor 2
const int irSensorPin = 2;         // IR sensor pin
unsigned long lastTime = 0;        // Variable to store the last time the sensor was triggered
unsigned long rpm = 0;             // Variable to store the RPM value
volatile unsigned long count = 0;  // Variable to count the number of interruptions
unsigned int sampleCount = 0;      // Variable to count the number of samples
unsigned long totalRPM = 0;        // Variable to store the total RPM for averaging

const int VIN = A0;      // define the Arduino pin A0 as voltage input (V in)
const float VCC = 5.04;  // supply voltage
const int MODEL = 0;     // enter the model (see above list)
#include <Robojax_AllegroACS_Current_Sensor.h>
float MotorCurrent = 0;
float PreviousCurrent = 0;
Robojax_AllegroACS_Current_Sensor robojax(MODEL, VIN);
//int pid;
//long milisec = millis(); // calculate time in milliseconds
//long Time= milisec/1000; // convert milliseconds to seconds
unsigned long millisBefore = 0;
unsigned long BTime = 0;
unsigned long UCTime = 0;
unsigned long PVTime = 0;
volatile int objects;
//int x = 10000;
const int UCSwitch = 1;           // relay 2
const int PVSwitch = 0;           // relay 3
const int speedcontrolPort = 11;  //Driver circuit
const int SelectCharge = 12;      // (Relay5)active low to charge the  battery in regenerative mode , active High to charge ultracapacitor  in genertion action
const int SwitchAction = 10;      // (Relay 4 +ve and Relay 4 -ve) ACTIVE LOW FOR MOTOR ACTION , ACTIVE HIGH FOR GENERTION ACTION
const int AllowBattery = 9;       // (Relay 1) active HIGH if Battery voltage is sufficient to drive motor.
float Battery = A1;               // to check the level of v of battery
float ultraC = A2;                // to check the level of UC
float Solar = A3;                 // to check the level of PV
int Speed = A5;                   // acclerator for motor
const int GeneratorPin = 13;      //turn on relay to opearate relay
//float MotorPower = 0;// motor input power is equals to motor supply power which may be battery or uc or PV .
float BatteryPower = 0;
float UcPower = 0;
float PVPower = 0;
float AngularV = 0;  //  angular velocity of wheel
float Tm = 0;        // Motor torque or load torque
float Eb = 0;        // Back emf of motor
float Pm = 0;        // Mechanical power of motor
float Ra = 4.2;      // Motor Armature resistance is 4.2 ohms
float BatteryValue = 0;
float ultraCValue = 0;
float SolarValue = 0;
float BatteryVoltage = 0;
float ultraCVoltage = 0;
float SolarVoltage = 0;
float SoCB = 0;
float SoCUC = 0;
float SoCSol = 0;
int SpeedControl = 0;
int SpeedControl1 = 0;
int SpeedControl2 = 0;
int SpeedMin = 0;
float BatteryEnergy = 0;
float UltraCEnergy = 0;
float PVEnergy = 0;


//int threshhold_value=100;
const int rs = 3, en = 4, d4 = 5, d5 = 6, d6 = 7, d7 = 8;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  Serial.begin(9600);
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Energy Mgmt System");
  delay(1000);
  pinMode(irSensorPin, INPUT);  // Set IR sensor pin as input
  attachInterrupt(digitalPinToInterrupt(irSensorPin), isr, FALLING);

  millisBefore = millis();
  //pinMode(driverckt,OUTPUT);
  //lcd.setCursor(6,0);// upper line sixth colomn
  // lcd.print("Amp");
  // lcd.setCursor(13,0);// upper line 13th colomn
  //lcd.print("RPM");

  //pinMode(MotorCurrent, INPUT);
  pinMode(speedcontrolPort, OUTPUT);
  pinMode(AllowBattery, OUTPUT);
  pinMode(SelectCharge, OUTPUT);
  digitalWrite(SelectCharge, LOW);
  pinMode(SwitchAction, OUTPUT);
  digitalWrite(SwitchAction, LOW);  // for motor action, HIGH for regenerative mode
  pinMode(UCSwitch, OUTPUT);
  pinMode(PVSwitch, OUTPUT);
  pinMode(GeneratorPin, OUTPUT);
  delay(100);
  digitalWrite(AllowBattery, HIGH);
  digitalWrite(SwitchAction, LOW);
}

void loop() {

  unsigned long currentTime = millis();  // Get the current time
  unsigned milisec = millis();
  unsigned Batmilisec = millis();  //time alloted for battert energy
  delay(1);
  unsigned UCmilisec = millis();  // time alloted for uc energy
  unsigned PVmilisec = millis();  // time alloted for Pv energy
  BatteryValue = analogRead(Battery);
  ultraCValue = analogRead(ultraC);
  SolarValue = analogRead(Solar);
  delay(1);
  float BatteryVoltage = BatteryValue * (11.30 / 1023.0);
  float SoCB = map(BatteryVoltage, 0.0, 11.30, 0.0, 100.00);  // consider full charge as 11.8

  float ultraCVoltage = ultraCValue * (10.60 / 1023.0);
  float SoCUC = map(ultraCVoltage, 0.0, 10.60, 0.0, 100.00);
  delay(1);
  float SolarVoltage = SolarValue * (10.0 / 1023.0);
  float SoCSol = map(SolarVoltage, 0.0, 10.0, 0.0, 100.00);

  if (SoCB <= 79.0) {
    digitalWrite(AllowBattery, LOW);
    digitalWrite(SwitchAction, LOW);
  }

  delay(1);
  SpeedControl = analogRead(Speed);
  // analogWrite(speedcontrolPort, SpeedControl/4);
  SpeedControl1 = map(SpeedControl, 0, 1023, 255, 0);  // 255,0
  SpeedControl2 = constrain(SpeedControl1, 0, 255);
  //newly added algo *********

  MotorCurrent = (robojax.getCurrentAverage(300));
  //analogWrite(speedcontrolPort, SpeedControl/4);

  if (SpeedControl2 - 2 > SpeedMin) {
    delay(10);
    Regenerative();
  }
  delay(1000);
  //i.e. SpeedControl2= 126 so SpeedControl = 128, 126>128 True
  if (SpeedControl2 <= SpeedMin) {
    delay(10);
    //Serial.println(  "Generative Action" );
    analogWrite(speedcontrolPort, SpeedControl2);
    Motoring_action();
  }

  SpeedMin = SpeedControl2;

  delay(10);
  //digitalWrite(AllowBattery, HIGH);// Enable battery to drive the motor
  // delay(1);
  // digitalWrite(SwitchAction, LOW);// Enable motor action
  // lcd.setCursor(0, 1);// lower line 0th colomn
  // lcd.print(SoCB,0);
  // simple linear regression model (motoring action)
  // Linear regression equation (motorcurrent = m*irreading+b)here b is previous current
  //Serial.print(  "MotorCurrent ");
  //Serial.println(  MotorCurrent );
  if (MotorCurrent >= PreviousCurrent) {
    Motoring_action();
  }

  if (MotorCurrent + .02 < PreviousCurrent) {
    Regenerative();
  }
  PreviousCurrent = MotorCurrent;
  delay(100);

  // rpm calculation


  // lcd.setCursor(13, 1);
  //lcd.print(rpm/100);
  //Source Power calculations
  float BatteryPower = (BatteryVoltage * MotorCurrent);  // in watt
  //delay(1);
  float UcPower = (ultraCVoltage * MotorCurrent);  // in watt
  //delay(1);
  float PVPower = (SolarVoltage * MotorCurrent);      // in watt
                                                      //*************** calculations  for Torque due to Battery as a supply source with back emf and angular diplacement********
  float Eb = (BatteryVoltage - (MotorCurrent * Ra));  // Back Emf Eb = Vs-(Ia*Ra)
  delay(1);
  float Pm = (Eb * MotorCurrent);  //Mechanical power out  Pm = Eb*Ia
  delay(1);
  //float AngularV =(2*22*rpm)/(7*60); // Convert Linear velocity into Angular velocity
  //delay(1);
  //float Tm = Pm/AngularV; // Pm = Tm * W(omega) so Tm = Pm/w

  delay(1);

  //Display all parameters on Serial Monitor
  //Serial.print("BV:");
  Serial.print(BatteryVoltage);
  Serial.print(",");
  delay(1);
  //Serial.print("SoCB:");
  Serial.print(SoCB);
  Serial.print(",");
  delay(1);
  //Serial.print("UCV:");
  Serial.print(ultraCVoltage);
  Serial.print(",");
  delay(1);
  //Serial.println(MotorCurrent);
  //Serial.print("SoCUC:");
  Serial.print(SoCUC);
  Serial.print(",");
  delay(1);
  //Serial.print("SV:");
  Serial.print(SolarVoltage);
  Serial.print(",");
  delay(1);
  //Serial.print("SoCSol:");
  Serial.print(SoCSol);
  Serial.print(",");
  delay(1);
  //Serial.print("MC:");
  Serial.print(MotorCurrent);
  Serial.print(",");
  delay(1);
  //Serial.print("BP:");
  Serial.print(BatteryPower);
  Serial.print(",");
  delay(1);
  //Serial.print("UP:");
  Serial.print(UcPower);
  Serial.print(",");
  delay(1);
  //Serial.print("PP:");
  Serial.print(PVPower);
  Serial.print(",");
  delay(1);
  //Serial.print("LT:");
  //Serial.print(Tm);
  //Serial.print(",");
  delay(1);
  if (currentTime - lastTime >= 1000) {
    // Average the RPM over 100 samples
    calculateRPM();
    if (sampleCount >= 0) {
      rpm = totalRPM / sampleCount;
      //Serial.print("Average RPM: ");
      Serial.print(rpm);
      Serial.print(",");
      float AngularV = (2 * 22 * rpm) / (7 * 60);  // Convert Linear velocity into Angular velocity
      delay(1);
      float Tm = Pm / AngularV;  // Pm = Tm * W(omega) so Tm = Pm/w
      Serial.print(Tm);
      Serial.print(",");
    }

    // Reset variables for next iteration
    totalRPM = 0;
    sampleCount = 0;
    lastTime = currentTime;
    count = 0;
    rpm = 0;
  }
  //Serial.print("Speed:");
  //Serial.print(rpm);
  //Serial.print(",");
  //Serial.print("\n ");
  delay(1001);
  if (Batmilisec - BTime >= 1000) {
    float BatteryEnergy = BatteryPower / 3600;  // Battery Energy in KWH
    BTime = Batmilisec;
    //Serial.print("BE:");
    Serial.print(BatteryEnergy, 4);
    Serial.print(",");
  }
  delay(10);
  if (UCmilisec - UCTime >= 1000) {
    float UltraCEnergy = UcPower / 3600;
    UCTime = UCmilisec;
    //Serial.print("UE:");
    Serial.print(UltraCEnergy, 4);
    Serial.print(",");
  }
  delay(10);
  if (PVmilisec - PVTime >= 1000) {
    float PVEnergy = PVPower / 3600;  // PV energy in KWH
    PVTime = PVmilisec;
    //Serial.print("PE:");
    Serial.print(PVEnergy, 4);
  }
  delay(10);
  //Serial.print(",");
  
}


void Motoring_action() {
  lcd.setCursor(0, 1);
  lcd.print("Motoring  Mode  ");
  digitalWrite(GeneratorPin, LOW);
  digitalWrite(AllowBattery, HIGH);
  digitalWrite(SwitchAction, LOW);
  delay(1);
  if (MotorCurrent >= 0.40 && MotorCurrent <= 0.60) {
    digitalWrite(UCSwitch, HIGH);
  }


  // APPLY FULL LOAD AND ADDING ALL SOURCES TOGETHER
  if (MotorCurrent >= 0.61) {
    digitalWrite(UCSwitch, HIGH);
    digitalWrite(PVSwitch, HIGH);
  } else {
    digitalWrite(UCSwitch, LOW);
    digitalWrite(PVSwitch, LOW);
  }
  //return;
}
void Regenerative() {
  digitalWrite(AllowBattery, LOW);
  digitalWrite(UCSwitch, LOW);
  digitalWrite(PVSwitch, LOW);
  digitalWrite(SwitchAction, HIGH);  // switch to generation action
  digitalWrite(GeneratorPin, HIGH);
  delay(1);
  lcd.setCursor(0, 1);
  lcd.print("Generating Mode");
  if (SoCB <= 80.0) {
    digitalWrite(SelectCharge, LOW);
  }
  if (SoCB >= 80.3) {
    digitalWrite(SelectCharge, HIGH);
  } else {
    digitalWrite(SelectCharge, LOW);
  }
  //return;
}
void isr() {
  count++;
}
void calculateRPM() {
  rpm = (count * 60) / 30;  // Assuming 2 pulses per revolution
  totalRPM += rpm;
  sampleCount++;
}
