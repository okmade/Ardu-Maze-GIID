/*Robot Laberinto Tipo Seguidor de Linea con sesores QTR-8RC y Algoritmo de Mano Izquierda
 * 
 * Lectura Sensores: 
 *      0= NEGRO
 *      1= BLANCO
 * 
 * Tipologia de Bifurcaciones:
 * 
 * Tipologia de Resolucion:
 *     LBR = B
 *     LBS = R
 *     RBL = B
 *     SBL = R
 *     SBS = B
 *     LBL = S
*/

#include <QTRSensors.h>

#define NUM_SENSORS   6                                         // Numero de Sensores a Usar
#define TIMEOUT       2000
#define EMITTER_PIN   A4                                        // Pin de LEDON, Encendido de Sensores
#define HIST          300                                       // Valor de fluctuacion de sensores

#define BUTTON_MODO1  A0                                        // Button Seleccionador de Modo Aprendizaje
#define BUTTON_MODO2  A1                                        // Button Seleccionador de Modo Solucion
#define MODO_APREN    1
#define MODO_SOL      2
#define LEDMODO       13                                        // Led Indicador de Aprendizaje Laberinto(Led Parpadeo por 2 Seg)  
                                                                // o Aprendido Laberinto (Led Continuo por 2 Seg)
#define DEBUG         1                                         // Entrar en modo Depuracion

QTRSensorsRC qtrrc((unsigned char[]) {2,4,7,10,11,12},           //Pines donde van conectados los sensores
 NUM_SENSORS, TIMEOUT, EMITTER_PIN); 
unsigned int sensorValues[NUM_SENSORS];

unsigned int modo = 0;
unsigned int sensores = 0;
unsigned int estado_a=0;
unsigned int estado_b=0;
char letras[100];
int pos = 0;
int posmov = 0;
int error = 0;

// INICIO DECLARACION DE FUNCIONES
void leersensores();
void laberinto();
void bifurcacion();
void procesardatos();
void motor_adelante();
void motor_izquierda(boolean sist);
void motor_derecha(boolean sist);
void motor_parar();
void girar_izquierda();
void girar_derecha();
// FIN DECLARACION DE FUNCIONES


void setup(){
  if (DEBUG){
    Serial.begin(9600);    
  }
  pinMode(BUTTON_MODO1,INPUT_PULLUP);
  pinMode(BUTTON_MODO2,INPUT_PULLUP);
  pinMode(LEDMODO,OUTPUT);
  delay(2000);
}

void loop(){
  if ((digitalRead(BUTTON_MODO1)==HIGH)&&(digitalRead(BUTTON_MODO2)==LOW)){
    while(digitalRead(BUTTON_MODO2)==LOW){}
    if (DEBUG){
      Serial.println("MODO APRENDIZAJE");
    }
    modo = MODO_APREN;
    pos=0;
    letras[pos]=' ';
    error = 0;
    for (int i=0;i<5;i++){
      digitalWrite(LEDMODO,HIGH);
      delay(200);
      digitalWrite(LEDMODO,LOW);
      delay(200);
    }
    girar_izquierda();
    while ((letras[pos] != 'T') && error == 0){                                  //Si ya llego al Final Acabar
      laberinto();
      procesardatos();
    }
    motor_parar();
  }else if ((digitalRead(BUTTON_MODO1)==LOW)&&(digitalRead(BUTTON_MODO2)==HIGH)){
    while(digitalRead(BUTTON_MODO1)==LOW){}
    if (DEBUG){
      Serial.println("MODO SOLUCION");
    }
    modo = MODO_SOL;
    posmov = 0;
    error = 0;
    digitalWrite(LEDMODO,HIGH);
    delay(2000);
    digitalWrite(LEDMODO,LOW);
    delay(100);
    girar_izquierda();
    while ((letras[posmov] != 'T') && error == 0){                               //Si ya llego al Final Acabar
      laberinto();
    }
    motor_parar();
    if (error != 1){
      Serial.print("LA SOLUCION ES: ");
      Serial.println(letras);
    }
  }
  if (error == 1){
    if (DEBUG){
      Serial.println("ERROR ENCONTRADO");
    }
    for (int i=0;i<3;i++){
      motor_izquierda(1);
      delay(3000);
      motor_derecha(1);
      delay(3000);
    }
    while(true){
      digitalWrite(LEDMODO,HIGH);
      delay(1000);
      digitalWrite(LEDMODO,LOW);
      delay(1000);
    }
  }
}

void laberinto(){
  leersensores();
  switch (sensores) {                                           // Determinar dependiendo de las posibilidades
    case B110011:                                               // Adelante
      motor_adelante();
      break;
    case B100111:                                               // Correcion a Izquierda de Linea Recta
    case B001111:
    case B110111:
    case B101111:
    case B011111:
      motor_izquierda(0);
      break;
    case B111001:                                               // Correcion a Derecha de Linea Recta
    case B111100:
    case B111011:
    case B111101:
    case B111110:
      motor_derecha(0);
      break;
    case B111111:                                               // Posible 'B'
    case B111000:                                               // Posible 'S' |- u Obligatorio R
    case B110000:                                               // Posible 'S' |- u Obligatorio R
    //case B100000:                                             // Posible 'S' |- u Obligatorio R       Mirar en TEST
    case B000000:                                               // Posible 'L' T o llegada de Final
    case B000111:                                               // Posible 'L' -| u Obligatorio L
    case B000011:                                               // Posible 'L' -| u Obligatorio L
    //case B000001:                                             // Posible 'L' -| u Obligatorio L      Mirar en TEST
      bifurcacion();
      break;
  }
}

void bifurcacion(){
  estado_a=sensores;
  motor_adelante();
  delay(100);
  leersensores();
  estado_b = sensores;
  if ((estado_a == B000000)&&(estado_b == B000000)){            // Llego al Final del Laberinto 
    motor_adelante();
    delay(100);
    motor_parar();
    if (modo == MODO_APREN){
      if (DEBUG){
        Serial.println("FINAL ENCONTRADO");
      }
      letras[pos] = 'T';
      for (int i=0;i<5;i++){
        digitalWrite(LEDMODO,HIGH);
        delay(500);
        digitalWrite(LEDMODO,LOW);
        delay(500);
      }
    }else if (modo == MODO_SOL){
      if (letras[posmov]=='T'){
        if (DEBUG){
          Serial.println("LLEGO AL FINAL");
        }
        for (int i=0;i<5;i++){
          digitalWrite(LEDMODO,HIGH);
          delay(500);
          digitalWrite(LEDMODO,LOW);
          delay(500);
        }
      }else{
        error = 1;
        if (DEBUG){
          Serial.println("ERROR: SE ESPERABA LLEGAR A FINAL");
        }
      }
    }
  }else if ((estado_a == B111111)&&(estado_b == B111111)){      // Bifurcacion Tipo U = B
    if (modo == MODO_APREN){
      if (DEBUG){
        Serial.println("VUELTA EN U ENCONTRADA");
      }
      girar_izquierda();
      letras[pos] = 'B';
      pos++;
    }else if (modo == MODO_SOL){
      if (DEBUG){
        Serial.println("ERROR: VUELTA EN U NO EXISTE EN MODO SOLUCION");
      }
      error = 1;
    }
  }else if (((estado_a == B000000)&&(estado_b == B111111))||    // Bifurcacion Tipo T = L
           ((estado_a == B000000)&&((estado_b == B100111)||     // Bifurcacion Tipo + = L
                                    (estado_b == B110011)||
                                    (estado_b == B111001)))||
           (((estado_a == B000111)||                            // Bifurcacion Tipo -| = L
             (estado_a == B000011))&&((estado_b == B100111)||
                                     (estado_b == B110011)||
                                     (estado_b == B111001)))){
    if (modo == MODO_APREN){
      if (DEBUG){
        Serial.println("BIFURCACION TIPO L ENCONTRADA");
      }
      girar_izquierda();
      letras[pos] = 'L';
      pos++;
    }else if (modo == MODO_SOL){
      if (letras[posmov]=='L'){
        if (DEBUG){
          Serial.println("VUELTA L EJECUTADA");
        }
        girar_izquierda();
        posmov++;
      }else if (letras[posmov]=='S'){
        if (DEBUG){
          Serial.println("VUELTA S EJECUTADA");
        }
        motor_adelante();
        posmov++;
      }else if (letras[posmov]=='R'){
        if (DEBUG){
          Serial.println("VUELTA R EJECUTADA");
        }
        girar_derecha();
        posmov++;
      }else{
        if (DEBUG){
          Serial.println("ERROR: NO EJECUTADA");
        }
        error = 1;
      }
    }
  }else if (((estado_a == B111000)||                            // Bifurcacion Tipo |- = S
             (estado_a == B110000))&&((estado_b == B100111)||
                                     (estado_b == B110011)||
                                     (estado_b == B111001))){
    if (modo == MODO_APREN){
      if (DEBUG){
        Serial.println("BIFURCACION TIPO S ENCONTRADA");
      }
      motor_adelante();
      letras[pos] = 'S';
      pos++;
    }else if (modo == MODO_SOL){
      if (letras[posmov]=='S'){
        if (DEBUG){
          Serial.println("VUELTA S EJECUTADA");
        }
        motor_adelante();
        posmov++;
      }else if (letras[posmov]=='R'){
        if (DEBUG){
          Serial.println("VUELTA R EJECUTADA");
        }
        girar_derecha();
        posmov++;
      }else{
        if (DEBUG){
          Serial.println("ERROR: NO EJECUTADA");
        }
        error = 1;
      }
    }
  }else if (((estado_a == B000111)||                            // Bifurcacion Obligatorio L
             (estado_a == B000011))&&((estado_b == B111111))){
    if (DEBUG){
      Serial.println("BIFURCACION OBLIGATORIA L");
    }
    girar_izquierda();
  }else if (((estado_a == B111000)||                            // Bifurcacion Obligatorio R
             (estado_a == B110000))&&((estado_b == B111111))){
    if (DEBUG){
      Serial.println("BIFURCACION OBLIGATORIA R");
    }
    girar_derecha();
  }
}

void procesardatos(){
  if (pos>=3){
    if (letras[pos-2]=='B'){
      if (((letras[pos-3]=='L')&&(letras[pos-1]=='R'))||
          ((letras[pos-3]=='R')&&(letras[pos-1]=='L'))||
          ((letras[pos-3]=='S')&&(letras[pos-1]=='S'))){
        letras[pos-3] = 'B';
        letras[pos-2] = ' ';
        letras[pos-1] = ' ';
        pos = pos - 2;
      }
      if (((letras[pos-3]=='L')&&(letras[pos-1]=='S'))||
          ((letras[pos-3]=='S')&&(letras[pos-1]=='L'))){
        letras[pos-3] = 'R';
        letras[pos-2] = ' ';
        letras[pos-1] = ' ';
        pos = pos - 2;
      }
      if ((letras[pos-3]=='L')&&(letras[pos-1]=='L')){
        letras[pos-3] = 'S';
        letras[pos-2] = ' ';
        letras[pos-1] = ' ';
        pos = pos - 2;
      }
    }
  }
}

void girar_izquierda(){
  motor_izquierda(1);
  delay(50);
  while(sensores != B110011){
    leersensores();
  }
  motor_adelante();
}

void girar_derecha(){
  motor_derecha(1);
  delay(50);
  while(sensores != B110011){
    leersensores();
  }
  motor_adelante();
}

void motor_adelante(){
  analogWrite(5,0);
  analogWrite(3,200);
  analogWrite(9,0);
  analogWrite(6,200);
}

void motor_izquierda(boolean sist){                             //sist = 0 modo normal, sist = 1 modo sobre su propio eje
  analogWrite(9,0);
  analogWrite(6,200);
  if (sist == 1){
    analogWrite(5,200);
    analogWrite(3,0);
  }else{
    analogWrite(5,0);
    analogWrite(3,50);
  }
}

void motor_derecha(boolean sist){                               //sist = 0 modo normal, sist = 1 modo sobre su propio eje
  if (sist == 1){
    analogWrite(9,200);
    analogWrite(6,0);
  }else{
    analogWrite(9,0);
    analogWrite(6,50);
  }
  analogWrite(5,0);
  analogWrite(3,200);
}

void motor_parar(){
  analogWrite(9,0);
  analogWrite(6,0);
  analogWrite(5,0);
  analogWrite(3,0);
}

void leersensores(){                                            //Determinar dependiendo de cuantos sensores utiliza
  qtrrc.read(sensorValues);
  sensores = 0;
  for (int i=0;i<NUM_SENSORS;i++){
    if (sensorValues[i]<HIST)
      bitClear(sensores,i);
    else
      bitSet(sensores,i);
  }
  if (DEBUG){
    Serial.print("SENSORES: ");
    Serial.println(sensores,BIN);
  }
}
