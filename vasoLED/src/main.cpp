
#include <Arduino.h>
#define ledVermelha 7
#define ledVerde 8


#define pinSensorA A0
#define pinSensorD 10



void setup()
{
    pinMode(ledVermelha, OUTPUT);    
    pinMode(ledVerde, OUTPUT);    
    
    pinMode(pinSensorD, INPUT);
    Serial.begin(9600);

}


void aguaOK()
{
    digitalWrite(ledVerde,HIGH);
    digitalWrite(ledVermelha,LOW);
}

void aguaNOK()
{
    digitalWrite(ledVerde,LOW );
    digitalWrite(ledVermelha,HIGH);
}


void testa()
{
    digitalWrite(ledVerde,LOW );
    digitalWrite(ledVermelha,HIGH);
    delay(50);
    digitalWrite(ledVerde, HIGH );
    digitalWrite(ledVermelha,LOW);
    delay(50);
}



void validaDigital()
{
	Serial.print("Digital:");
  
  if (digitalRead(pinSensorD)) {
     Serial.print("SEM UMIDADE ");
     aguaNOK();
  } else {
     Serial.print("COM UMIDADE ");
     aguaOK();
   
  }
}

void validaAnalogico()
{
Serial.print("  Analogico:");
  Serial.print(analogRead(pinSensorA)); 
  Serial.print("  ");

  Serial.print("  Atuador:");
  if (analogRead(pinSensorA) > 150) {
     Serial.println("SOLENOIDE LIGADO");
     aguaNOK();
     delay(500);
  } else {
    Serial.println("SOLENOIDE DESLIGADO");
    aguaOK();
    delay(500);

  }

}

void loop()
{

validaAnalogico();

   
}

