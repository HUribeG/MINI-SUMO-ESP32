/*

Created by: HG
06/08/23 19:00:00

V1.5

*/
#include <Arduino.h>

/* 1 Para pruebas    0 para competencia */
#define MODE 0

#define PWMB 10
#define BIN2 12
#define BIN1 11
#define STBY 6
#define AIN1 7
#define AIN2 8
#define PWMA 9

#define NUM_JF40SEN 5
uint8_t sm[NUM_JF40SEN] = {18, 17, 16, 15, 14}; // Sensor Matrix

#define QREL 2
#define QRER 3
#define GO 5
#define RDY 4

// Interrrupt vars
volatile byte QREL_STATE = LOW;
volatile byte QRER_STATE = LOW;

// Millis
int16_t exectime = 500;
int32_t inittime = 0;

// Control Vars
int position = 0;
int last_post = 0;
int error = 0;
int last_error = 0;
int derivative = 0;
int uk = 0;

// Control Konstants
#define setpoint 40;

// Control Gains
int bwd = 200;
int vel = 100;
float kp = 6;
float kd = 1;

int numertor = 0;
int denomtor = 0;
int sensor_reads[NUM_JF40SEN];

void find_pos()
{

  for (uint8_t i = 0; i < NUM_JF40SEN; i++)
  {
    sensor_reads[i] = digitalRead(sm[i]);
  }

  numertor = (0 * sensor_reads[0] + 20 * sensor_reads[1] + 40 * sensor_reads[2] + 60 * sensor_reads[3] + 80 * sensor_reads[4]);
  denomtor = (sensor_reads[0] + sensor_reads[1] + sensor_reads[2] + sensor_reads[3] + sensor_reads[4]);
  position = numertor / denomtor;

  if (last_post <= 20 && position == -1)
    position = 0;
  if (last_post >= 60 && position == -1)
    position = 80;
  last_post = position;
}

/// Proportional-Derivative Control
void PDC()
{

  error = position - setpoint;
  derivative = error - last_error;
  uk = (kp * error) + (kd * derivative);
  last_error = error;
  if (uk > 255)
    uk = 255;
  else if (uk < -255)
    uk = -255;
}

void motors(int left, int right)
{

  if (left >= 0)
  {
    digitalWrite(BIN2, HIGH);
    digitalWrite(BIN1, LOW);
  }
  else
  {
    digitalWrite(BIN2, LOW);
    digitalWrite(BIN1, HIGH);
    left = left * (-1);
  }
  analogWrite(PWMB, left);

  if (right >= 0)
  {
    digitalWrite(AIN2, HIGH);
    digitalWrite(AIN1, LOW);
  }
  else
  {
    digitalWrite(AIN2, LOW);
    digitalWrite(AIN1, HIGH);
    right = right * (-1);
  }
  analogWrite(PWMA, right);
}

void QREL_ISR()
{
  QREL_STATE = HIGH;
}

void QRER_ISR()
{
  QRER_STATE = HIGH;
}

void setup()
{

#if MODE
  Serial.begin(9600);
#endif

  pinMode(QREL, INPUT);
  pinMode(QRER, INPUT);
  pinMode(sm[0], INPUT);
  pinMode(sm[1], INPUT);
  pinMode(sm[2], INPUT);
  pinMode(sm[3], INPUT);
  pinMode(sm[4], INPUT);
  pinMode(GO, INPUT);
  pinMode(RDY, INPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(STBY, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMA, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(QREL), QREL_ISR, FALLING);

  digitalWrite(13, HIGH);
  delay(1000);
  /*while(!digitalRead(GO));
  digitalWrite(STBY, HIGH);
  digitalWrite(13, LOW);*/
}

void loop()
{

#if MODE
  while (true)
  {
    digitalWrite(STBY, LOW);
    if (QREL_STATE == HIGH || QRER_STATE == HIGH)
    {

      if (inittime == 0)
      {
        inittime = millis();
      }
      if ((millis() - inittime) <= exectime)
      {
        motors(-bwd, -bwd + 50);
      }
      else
      {
        QREL_STATE = LOW;
        QRER_STATE = LOW;
        inittime = 0;
      }
    }
    else
    {
      find_pos();
      PDC();
      (uk < 0) ? motors(vel, vel + uk) : motors(vel - uk, vel);
    }

    Serial.print("Position: ");
    Serial.print(position);
    Serial.print(" Error: ");
    Serial.print(error);
    Serial.print(" PID out: ");
    Serial.print(uk);
    Serial.print(" || QRER: ");
    Serial.print(digitalRead(QRER));
    Serial.print(" QREL: ");
    Serial.print(digitalRead(QREL));
    Serial.print(" || QREL_STATE: ");
    Serial.print(QREL_STATE);
    Serial.print(" QRER_STATE: ");
    Serial.print(QRER_STATE);
    Serial.print(" || Init time: ");
    Serial.print(inittime);
    Serial.print(" Diference times:");
    Serial.println(millis() - inittime);

    if (digitalRead(GO))
    {
      break;
    }
  }

#else
  while (digitalRead(GO))
  {
    digitalWrite(STBY, HIGH);
    if (QREL_STATE == HIGH || QRER_STATE == HIGH)
    {

      if (inittime == 0)
      {
        inittime = millis();
      }
      if ((millis() - inittime) <= exectime)
      {
        motors(-bwd, -bwd + 200);
      }
      else
      {
        QREL_STATE = LOW;
        QRER_STATE = LOW;
        inittime = 0;
      }
    }
    else
    {
      find_pos();
      if(position == 40){
        motors(255, 255);
      }
      else{
        PDC();
        (uk < 0) ? motors(vel, vel + uk) : motors(vel - uk, vel);
      }
      
    }

    if (!digitalRead(GO))
    {
      digitalWrite(STBY, LOW);
      break;
    }
  }
#endif
}
