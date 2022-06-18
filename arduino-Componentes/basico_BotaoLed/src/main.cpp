/*=============================================================================
Autor:        Johnny Meneses [bilbo_vader@outlook.com]
Descrição:    Lógica para ligar uma LED com um Pushbutton

Lista de Componentes
[1] Arduino Uno R3
[1] Red LED
[1] 220 Ω Resistor
[1] Pushbutton

 *============================================================================*/  


#include <Arduino.h>

#define LED 4
#define botaoliga 10

int estadoBotao;


void setup()
{
    Serial.begin(9600);
    pinMode(LED, OUTPUT);    
  	pinMode(botaoliga,INPUT_PULLUP);
}

void loop()
{
	
  	estadoBotao = digitalRead(botaoliga);
  	
  	if(estadoBotao==0)
    {
    	digitalWrite(LED,HIGH);
      Serial.println("Liguei a Luz");
      
    }else
    {
    	digitalWrite(LED,LOW);
    }


}