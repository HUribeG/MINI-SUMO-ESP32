
uint8_t sm[5] = {14, 15, 16, 17, 18}; //Sensor Matrix
#define QREL 2
#define QRER 3


void setup() {

  pinMode(QREL, INPUT);
  pinMode(QRER, INPUT);

  pinMode(sm[0], INPUT);
  pinMode(sm[1], INPUT);
  pinMode(sm[2], INPUT);
  pinMode(sm[3], INPUT);
  pinMode(sm[4], INPUT);
  
  Serial.begin(9600);
}

void loop() {
  
  Serial.print("SII: ");
  Serial.print(digitalRead(sm[4]));
  Serial.print(" SIC: ");
  Serial.print(digitalRead(sm[3]));
  Serial.print(" SC: ");
  Serial.print(digitalRead(sm[2]));
  Serial.print(" SDC: ");
  Serial.print(digitalRead(sm[1]));
  Serial.print(" SDD: ");
  Serial.println(digitalRead(sm[0]));
  Serial.print(" QREL: ");
  Serial.print(digitalRead(QREL));
  Serial.print(" QRER: ");
  Serial.println(digitalRead(QRER));
  delay(300);
}
