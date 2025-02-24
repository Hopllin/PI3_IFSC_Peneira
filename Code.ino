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
#define controle true

modbusDevice regBank;
modbusSlave slave;

word peso1;
word peso2;
word peso3;
int fc1;
int fc2;

int supervisa = 0;
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

//Variáveis controlador
float tempo, tempo_01, volta, RPM, pulso, pwm, Kp, ki, b1, b2, T, Ti, Freq;
float Ta = 20.0;
float e, r, y, e_1, e_2, u, u_1, tp;
volatile unsigned long counter = 0;
float cont_rpm[50];
int i=0;
int k=0;
float media_rpm=0;

// Definições HX711 para cada pino
#define CELULA_DT A1
#define CELULA_SCK A0
#define CELULA_DT_1 A3
#define CELULA_SCK_1 A2
#define CELULA_DT_2 A5
#define CELULA_SCK_2 A4

// Criando um objeto HX711 para cada módulo
HX711 Balanca1;
HX711 Balanca2;
HX711 Balanca3;

// Fator de calibração da Balança
float fator_calib1 = 43500;
float fator_calib2 = -44000;
float fator_calib3 = -45000;

#if tela == true
  // Definições da tela da IHM
  NexText tStatus = NexText(0, 3, "tStatus");// Página 0, id 1, nome do objeto tStatus
  NexText tRPM = NexText(0, 5, "tRPM");   // Página 0, id 1, nome do objeto tRPM
  NexText tPeso1 = NexText(0, 10, "tPeso1"); // Página 0, id 1, nome do objeto tPeso1
  NexText tPeso2 = NexText(0, 11, "tPeso2"); // Página 0, id 1, nome do objeto tPeso2
  NexText tPeso3 = NexText(0, 12, "tPeso3"); // Página 0, id 1, nome do objeto tPeso3
  NexPage Principal = NexPage(0, 0, "Principal"); //Instanciando objeto pagina como numero da pagina, id e nome
  NexPage Aviso = NexPage(0, 0, "Aviso"); //Instanciando objeto pagina como numero da pagina, id e nome
  
  NexTouch *nex_listen_list[] = {
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
int SobeRPM = 37;
int DesceRPM = 35;
int SobeInc = 31;
int DesceInc = 33;
bool x1 = LOW;
bool x2 = LOW;
bool lb1 = LOW;
bool lb2 = LOW;
int RPM_ref = 1100;

void supervisiona(){
  
 #if hx == true
    peso1 = Balanca1.get_units(3)*1000;
    if(peso1 <= 0){peso1 = 0;}
    peso2 = Balanca2.get_units(3)*1000;
    if(peso2 <= 0){peso2 = 0;}
    peso3 = Balanca3.get_units(3)*1000;
    if(peso3 <= 0){peso3 = 0;}  
  #endif
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

  #endif
    
    itoa(RPM_ref, cstr4, 10);
    tRPM.setText(cstr4);

 #if modb == true
    
    //guarda valores analogicos nos registradores 
    regBank.set(30001, (word) peso1); 
    regBank.set(30002, (word) peso2); 
    regBank.set(30003, (word) peso3); 
    regBank.set(30006, RPM);
    
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

  
  #endif

}

void ai0() {
  counter++;
}

void conf_Timer() 
{ noInterrupts(); //Desativa interrupções enquanto configura  
  
    TCCR4A = 0;  // Zero o registrador de controle A
    TCCR4B = 0;  // Zera o registrador de controle B
    TCNT4 = 0;   // Zera o contador

   // ((16MHz/(Prescaler x Freq desejada)) - 1)
   //Tempo desejado = 20ms -> Freq. Desej. = 1/0,020 = 50Hz
   //Prescaler = 64 -> OCR4A = ((16MHz/(64 x 50)) - 1) = 4999
   //Valor OCR4A válido pq é menor q 65535 que é o máximo pro timer de 16 bits
   
    OCR4A = 4999;  //Valor para 20ms
    
    TCCR4B |= (1 << WGM12); // Ativa o modo CTC
    TCCR4B |= (1 << CS11) | (1 << CS10);  // Configura o prescaler 64
    
    TIMSK4 |= (1 << OCIE4A); // Habilita a interrupção do Timer 4

  interrupts(); // Reativa interrupções
}

//Interrupção do Timer 4 (executada a cada 20ms)
ISR(TIMER4_COMPA_vect) //Código é executado a cada 20ms
{  
  unsigned long tp = counter; //Lê o valor atual de counter
  counter = 0; // Reseta counter para a próxima medição
  
  // Calculo do RPM correto
  RPM = ((100.0 * tp) / Ta);
  cont_rpm[i]=RPM;
  i++;
  if(i==49)
  i=0;

  tempo = millis();

  // Controle PI
  e = r - RPM;  //cálculo do erro entte referência e RPM medido

  b1 = Kp * (1.0 + (T / (2.0 * Ti)));
  b2 = Kp * (-1.0 + (T / (2.0 * Ti)));

  u = u_1 + (b1 * e) + (b2 * e_1);

  // Atualização do erro anterior
  e_1 = e;
  u_1 = u;

  //Anti-WindUp
  u_1 = u;
  e_2 = e_1;
  e_1 = e;

  //Saturação do sinal de controle
  if (u < 0) u = 0.0;
  if (u >= 10) u = 10.0;

  // Conversão para PWM (escala de 0 a 255)
  pwm = (u * 255) / 10;

  //Saturação do PWM
  if (pwm < 100) pwm = 0;
  if (pwm >= 255) pwm = 255;
}

void setup() {
//modbus config
  #if modb == true
    //Set Slave ID
    regBank.setId(1); 

    //Analog Input registers
    regBank.add(30001);
    regBank.add(30002);
    regBank.add(30003);
    regBank.add(30006);
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

  //Setup controlador
  pinMode(2, INPUT);  // Entrada do canal A do encoder
  pinMode(9, OUTPUT);        //Saida PWM para motor

  conf_Timer();              //chama a função de configuração do timer

  pinMode(0, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(0), ai0, FALLING);

  // Inicialização de variáveis
  pulso = 600.0;
  volta = 1.0 / pulso;
  RPM = 0.0;
  e = e_1 = u_1 = u = pwm = 0;

  r = RPM_ref;  //Referência de RPM
  
  // Parâmetros do controlador PI
  Kp = 0.01;                //0,1/0.274/0.28476;(overshoot)//0956;//0.00241353771099636//0,00556;
  Ti = 7.5;                 //5.5;//2.314//0,1;
  T = 0.02;

}

void loop() {
    byte fc1 = digitalRead(10);
    byte fc2 = digitalRead(11);

  supervisiona();

  if(supervisa == 10){}
 /*
    if(digitalRead(SobeRPM) == HIGH){
      RPM_ref += 100;
    }

    if(digitalRead(DesceRPM) == HIGH){
      RPM_ref -= 100;
    }
    if(RPM_ref <=800 ){RPM_ref = 800;}
   */ 
    r = RPM_ref;

  if(digitalRead(FC2) == HIGH || digitalRead(FC1) == HIGH){
    Aviso.show();
  }

  x1 = digitalRead(FC1);
  x2 = digitalRead(FC2);


  // save the current state as the last state, for next time through the loop
  lb1 = x1;
  lb2 = x2;
  // compare the buttonState to its previous state
  if (x1 != lb1 || x2 != lb2) {
    // if the state has changed, increment the counter
    if (x1 == LOW && x2 == LOW) {
      Principal.show();
    }
  }
  
  #if rle == true
    //Loop do fim de curso 
    if(digitalRead(SobeInc) == HIGH && digitalRead(FC1) == LOW){
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    }

    if(digitalRead(DesceInc) == HIGH && digitalRead(FC2) == LOW){
      digitalWrite(IN2, LOW);
      digitalWrite(IN1, HIGH);
    }
    if(digitalRead(DesceInc) == LOW && digitalRead(SobeInc) == LOW){
      digitalWrite(IN2, LOW);
      digitalWrite(IN1, LOW);
    }
    if(digitalRead(FC1) == HIGH){digitalWrite(IN2, LOW);}
    if(digitalRead(FC2) == HIGH){digitalWrite(IN1, LOW);}
    

  #endif

  #if controle == true
    // Atualiza os vlaores no Serial Monitor a cada 1 segundo
    if (millis() - tempo_01 >= 500.0) 
    {   
      for(k=0;k<50;k++){
        media_rpm+=cont_rpm[k];
      }
      media_rpm=media_rpm/50;
      /*
      Serial.print("RPM: ");
      Serial.println(RPM);
      Serial.print("Referência: ");
      Serial.println(r);
      Serial.print("Sinal de controle: ");
      Serial.println(u);
      Serial.print("PWM: ");
      Serial.println(pwm);
      Serial.println("--------------");
      */
      tempo_01 = millis();

    }

      // Aplica o PWM ao motor
      analogWrite(9, pwm);
    #endif

  supervisa +=1;
  slave.run();  

}
