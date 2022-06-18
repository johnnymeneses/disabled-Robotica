/*==============================================================================
Autor:        Johnny Meneses [bilbo_vader@outlook.com]
Descrição:    Lógica para ligar uma LED

Lista de Componentes
[1] Arduino Uno R3
[1] Red LED
[1] 220 Ω Resistor


 *===========================================================================*/  


#include <Arduino.h>
#define LED 4


int estadoBotao;


void setup()
{
    pinMode(LED, OUTPUT);    

}

void loop()
{
	
   	digitalWrite(LED,HIGH);
  	delay(1000);
   	digitalWrite(LED,LOW);
  	delay(1000);
    
}