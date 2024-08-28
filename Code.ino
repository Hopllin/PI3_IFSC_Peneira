#include "HX711.h"
#include "Nextion.h" 
#include <modbus.h>
#include <modbusDevice.h>
#include <modbusRegBank.h>
#include <modbusSlave.h>

//habilitar códigos
#define modb true
#define next true
#define tela true
#define hx true
#define rle true

modbusDevice regBank;
modbusSlave slave;

word peso1;
word peso2;
word peso3;
int fc1;
int fc2;

// Definições dos fins de curso e ponte H

int FC1 = 10; //baixo
int FC2 = 11; //alto
int IN1 = 6; //cuidado na ligação 
int IN2 = 7; //cuidado na ligação 
char cstr1[16];
char cstr2[16];
char cstr3[16];
char cstr4[16];
char cstr5[16];



// Definições HX711 para cada pino
#define CELULA_DT A0
#define CELULA_SCK A1
#define CELULA_DT_1 A2
#define CELULA_SCK_1 A3
#define CELULA_DT_2 A4
#define CELULA_SCK_2 A5

// Criando um objeto HX711 para cada módulo
HX711 Balanca1;
HX711 Balanca2;
HX711 Balanca3;

// Fator de calibração da Balança
float fator_calib1 = -45000;
float fator_calib2 = -44000;
float fator_calib3 = -42000;

#if tela == true
  // Definições da tela da IHM
  NexText tStatus = NexText(0, 3, "tStatus");// Página 0, id 1, nome do objeto tStatus
  NexText tRPM   = NexText(0, 5, "tRPM");   // Página 0, id 1, nome do objeto tRPM
  NexText tPeso1 = NexText(0, 10, "tPeso1"); // Página 0, id 1, nome do objeto tPeso1
  NexText tPeso2 = NexText(0, 11, "tPeso2"); // Página 0, id 1, nome do objeto tPeso2
  NexText tPeso3 = NexText(0, 12, "tPeso3"); // Página 0, id 1, nome do objeto tPeso3
  NexPage Principal = NexPage(0, 0, "Principal"); //Instanciando objeto pagina como numero da pagina, id e nome
  NexPage Aviso = NexPage(0, 0, "Aviso"); //Instanciando objeto pagina como numero da pagina, id e nome
  
  NexTouch *nex_listen_list[] = 
{
    &tStatus,
    &tRPM,
    &tPeso1,
    &tPeso2,    
    &tPeso3,
    &Principal,
    &Aviso,
    NULL
};
#endif

//Parâmetros do cabeamento da IHM e definições
//RX pino D15 fio azul
//TX pino D14 fio amarelo

int Liga = 36;
int Desliga = 34;
int SobeRPM = 32;
int DesceRPM = 30;
int SobeInc = 28;
int DesceInc = 26;
bool b1 = LOW;
bool b2 = LOW;
bool lb1 = LOW;
bool lb2 = LOW;
int RPM = 0;


void setup() {
//modbus config
  #if modb == true
    //Set Slave ID
    regBank.setId(1); 

    //Analog Input registers
    regBank.add(30001);
    regBank.add(30002);
    regBank.add(30003);
    //digital input
    regBank.add(20004);
    regBank.add(20005);

    slave._device = &regBank;
    //Set BaudRate
    slave.setBaud(9600);
  #endif

  #if next == true    
    nexInit();
    //Principal.show();
  #endif

   //Inicie o setup do HX711
  Balanca1.begin(CELULA_DT, CELULA_SCK);
  Balanca2.begin(CELULA_DT_1, CELULA_SCK_1);
  Balanca3.begin(CELULA_DT_2, CELULA_SCK_2);
  
  //Ajustando a escala e tarando a balanca
  Balanca1.set_scale(fator_calib1);
  Balanca2.set_scale(fator_calib2);
  Balanca3.set_scale(fator_calib3);
  Balanca1.tare();
  Balanca2.tare();
  Balanca3.tare();

  //Definicoes das portas do modulo rele
  pinMode(IN1, OUTPUT);                  // definições das portas IN1 e IN2 como portas de saidas 
  pinMode(IN2, OUTPUT);  
  pinMode(FC1, INPUT_PULLUP);           // definições das portas fimdeCurso1 e 2 como portas de entrada 
  pinMode(FC2, INPUT_PULLUP);
  pinMode(SobeInc, INPUT_PULLUP);          
  pinMode(DesceInc, INPUT_PULLUP);
  pinMode(Liga, INPUT);           // Definir liga e desliga como INPUTS
  pinMode(Desliga, INPUT);

}

void loop() {

  peso1 = Balanca1.get_units(3)*1000;
  if(peso1 <= 0){peso1 = 0}
  peso2 = Balanca2.get_units(3)*1000;
  if(peso2 <= 0){peso2 = 0}
  peso3 = Balanca3.get_units(3)*1000;
  if(peso3 <= 0){peso3 = 0}
  
  /*Loop do liga desliga para o motor trifasico
  if(digitalRead(Liga == HIGH && Desliga == LOW){
    Liga o motor
    }
    else(digitalRead(Low == HIGH && Desliga == LOW)){
      }
    */
    
  #if next == true
    
    //Loop da nextion pegando valores de peso
    itoa(peso1, cstr1, 10);
    tPeso1.setText(cstr1);

    itoa(peso2, cstr2, 10);
    tPeso2.setText(cstr2);
    
    itoa(peso3, cstr3, 10);
    tPeso3.setText(cstr3);

    if(digitalRead(SobeRPM) == HIGH){
      RPM += 10;
    }

    if(digitalRead(DesceRPM) == HIGH){
      RPM -= 10;
    }

    itoa(RPM, cstr4, 10);
    tRPM.setText(cstr4);
    
  #endif

  if(digitalRead(FC2) == HIGH || digitalRead(FC1) == HIGH){
    Aviso.show();
  }

    b1 = digitalRead(FC1);
    b2 = digitalRead(FC2);

  // compare the buttonState to its previous state
  if (b1 != lb1 || b2 != lb2) {
    // if the state has changed, increment the counter
    if (b1 == LOW && b2 == LOW) {
      Principal.show();
    }
  }
  // save the current state as the last state, for next time through the loop
  lb1 = b1;
  lb2 = b2;
  
  #if rle == true
      //Loop do fim de curso 
      if(digitalRead(FC2) == HIGH && digitalRead(SobeInc) == LOW){ 
        while(digitalRead(FC2) == HIGH){
          digitalWrite(IN1, LOW);
          digitalWrite(IN2, HIGH);
        }
        digitalWrite(IN2, LOW);
      } 
      if(digitalRead(FC1) == HIGH && digitalRead(DesceInc) == LOW){
        
        while(digitalRead(FC1) == HIGH){
          digitalWrite(IN2, LOW);
          digitalWrite(IN1, HIGH);
        }
        digitalWrite(IN1, LOW);
      }
  #endif
        
  #if modb == true
  
    byte fc1 = digitalRead(10);
    byte fc2 = digitalRead(11);
    fc1 = 1;
    fc2 = 0;
    
    //guarda valores analogicos nos registradores 
    regBank.set(30001, (word) peso1); 
    regBank.set(30002, (word) peso2); 
    regBank.set(30003, (word) peso3); 
    
    //guarda valores digitais nos registradores
    if(fc1 >= 1)
      regBank.set(20004, 1);
    if(fc1 <=0 )
      regBank.set(20004, 0);
    if(fc2 >= 1)
      regBank.set(20004, 1);
    if(fc2 <=0 )
      regBank.set(20004, 0);
    
    //roda o slave
    slave.run();  
  
  #endif
}
