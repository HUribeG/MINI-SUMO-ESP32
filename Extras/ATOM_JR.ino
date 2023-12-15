
#include <Arduino.h>

///Created By: HG
#define PWMA 9
#define AIN2 7
#define AIN1 8
#define BIN1 12
#define BIN2 11
#define PWMB 10
#define STBY 6
#define pin_sd 2 
#define pin_si 3


int valor_sd = 0;
int valor_si = 0;
uint8_t Sensores[5] = {14, 15, 16, 17, 18};

uint8_t LecturaDig[5];
int SumaP, Suma, Pos, Poslast;

float KP = 1.25;
float KD = 1;
int Vcru = 0;

int error=0, error_ant=0, derivada=0, PWMot=0;


void setup() {
 Serial.begin(9600);
  /////// Sensores ///////
    DDRC &=~ (1<<DDC0); //Sensor 1
    DDRC &=~ (1<<DDC1); //Sensor 2
    DDRC &=~ (1<<DDC2); //Sensor 3
    DDRC &=~ (1<<DDC3); //Sensor 4
    DDRC &=~ (1<<DDC4); //Sensor 5

    DDRD &=~ (1<<DDD2); //Sensor Piso 1
    DDRD &=~ (1<<DDD3); //Sensor Piso 2

pinMode(PWMA, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
 
 ////// RRT ////
    DDRD &=~ (1<<DDD5);//Pin 5

 
}

void loop() {
// Serial.print("parado");

 while(PIND & (1<<DDD5)){

  valor_sd = digitalRead(pin_sd);
  //Serial.print(valor_sd);
  //Serial.print(" ");
 valor_si = digitalRead(pin_si);
 //Serial.println(valor_si);
// Serial.print("activado");
  ///Retrocede si encuentra borde///
  /*if(!valor_sd){
   Serial.println("retrocediendo der");
    Mover(-100, -100);
    _delay_ms(800);
    Mover(-100, 100);
    _delay_ms(300);
  }
   else if(!valor_si){
   Serial.println("retrocediendo izqM");
    Mover(-100, -100);
    _delay_ms(800);
    Mover(100, -100);
    _delay_ms(300);
   }
      */
    
 // else{
   digitalWrite(STBY, HIGH);
 //Serial.println("avanzando");
    //// Control PD////
    int posicion=Deteccion();
  // Serial.println(posicion);
    error = -posicion + 200;
   //Serial.println(error);
    derivada = error - error_ant;
    PWMot = ((KP * error) + (KD * derivada));
    error_ant = error;
    //Serial.println(KP*error);
    Mover(Vcru - PWMot, Vcru + PWMot);
   if(error == 0)Mover(255, 255);
    
    

    if (!(PIND & (1<<PIND5))){
     digitalWrite(STBY, LOW);
      break;
    }
    
//  }
 }
}

int Deteccion(void){
  for(uint8_t i=0; i<5; i++){
    LecturaDig[i] = digitalRead(Sensores[i]);
  }
  SumaP = (400*LecturaDig[0] + 300*LecturaDig[1] + 200*LecturaDig[2] + 100* LecturaDig[3] + 0*LecturaDig[4]);
  Suma  = (LecturaDig[0] + LecturaDig[1] + LecturaDig[2] + LecturaDig[3] + LecturaDig[4]);
  Pos = (SumaP/Suma);

  if(Poslast <= 100  && Pos == -1)Pos=0;
  if(Poslast >= 400  && Pos == -1)Pos=400;
  Poslast = Pos;
  return Pos;
  //Serial.println(Pos);
}

void Mover(int izq, int der){//0 hasta 255    0 hasta -255

  izq=constrain(izq,-255,255);
  der=constrain(der,-255,255);
  ////////////////motor LEFT "IZQUIERDO" ////////////////////////
  if(izq>=0){
    digitalWrite(AIN1,HIGH);
    digitalWrite(AIN2,LOW);
  }
  else{
    digitalWrite(AIN1,LOW);
    digitalWrite(AIN2,HIGH);
    izq=izq*(-1);
  }
  analogWrite(PWMA,izq);
  
  ////////////////motor RIGHT "DERECHO" ////////////////////////
  if(der>=0){
    digitalWrite(BIN1,HIGH);
    digitalWrite(BIN2,LOW);
  }
  else{
    digitalWrite(BIN1,LOW);
    digitalWrite(BIN2,HIGH);
    der=der*(-1);
  }
  analogWrite(PWMB,der);
  
}
