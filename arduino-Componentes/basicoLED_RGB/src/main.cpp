/*=============================================================================
Autor:        Johnny Meneses [bilbo_vader@outlook.com]
Descrição:    Lógica para ligar uma LED RGB, incluindo várias cores e uma animação

Lista de Componentes
[1] Arduino Uno R3
[1] Red RGB
[3] 330 Ω Resistor

 *===========================================================================*/  





#include <Arduino.h>


const int azul = 5;
const int verde = 6;
const int vermelho = 7;



void setup()
{
  Serial.begin(9600);  
 
   pinMode(azul, OUTPUT);
   pinMode(verde, OUTPUT);
   pinMode(vermelho, OUTPUT);
  

  
}


/*------------- RGB CODE -------------*/

void rgbAnalog (int luzVermelha, int luzVerde, int luzAzul)
{
  analogWrite(vermelho, luzVermelha);
  analogWrite(verde, luzVerde);
  analogWrite(azul, luzAzul);  
}


//Cor Branca


void rgbDesligado()
{
  rgbAnalog(0,0,0);
};


	void rgbBranco()
	{
	  rgbAnalog(255,255,255);
	}


void rgbVerde()
{
  rgbAnalog(0, 255, 0); // Green
}

void rgbVermelho()
{
  rgbAnalog(255, 0, 0); // Red
}

void rgbAzul()
{
  rgbAnalog(0, 0, 255); // Blue
}


void rgbRaspberry()
{
  rgbAnalog(255, 255, 125); // Raspberry
}

void rgbCyan()
{
  
  rgbAnalog(0, 255, 255); // Cyan
}

void rgbMagenta()
{
  rgbAnalog(255, 0, 255); // Magenta
}

void rgbYellow()
{
  rgbAnalog(255, 255, 0); // Yellow
}



void rgbStar()
{
  rgbBranco();
  delay(50);
  rgbDesligado();
  
  rgbVerde();
  delay(50);
  rgbDesligado();
  
  rgbVermelho();
  delay(50);
  rgbDesligado();
  
  rgbAzul();
  delay(50);
  rgbDesligado();
  
  
  rgbRaspberry();
  delay(50);
  rgbDesligado();
  
  
  rgbCyan();
  delay(50);
  rgbDesligado();
  
  
  rgbMagenta();
  delay(50);
  rgbDesligado();

  rgbYellow();
  delay(50);
  rgbDesligado();
  
 
}



void loop()
{

    rgbDesligado();
    delay(1000);
    rgbBranco();
    delay(1000);
    rgbVerde();
    delay(1000);
    rgbVermelho();
    delay(1000);
    rgbAzul();
    delay(1000);
    rgbRaspberry();
    delay(1000);
    rgbCyan();
    delay(1000);
    rgbMagenta();
    delay(1000);
    rgbYellow();
    delay(1000);
    rgbStar();
    delay(3000);
 
}

