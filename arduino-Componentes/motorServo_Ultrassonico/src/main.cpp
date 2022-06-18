/*=============================================================================

Autor:        Johnny Meneses [bilbo_vader@outlook.com]
Descrição:    Micro Servo movimentado pelo resultado do Sensor Ultrasônico 


Lista de Componentes
[1] Arduino Uno R3
[1] Ultrasonic Distance Sensor
[1] Positional Micro Servo


 *===========================================================================*/  

#include <Arduino.h>
#include <Servo.h>
#include "Ultrasonic.h" //Inclusão da Biblioteca - Sensor Ultrasônico



const int echoPin = 6; //PINO DIGITAL UTILIZADO PELO HC-SR04 ECHO(RECEBE)
const int trigPin = 7; //PINO DIGITAL UTILIZADO PELO HC-SR04 TRIG(ENVIA)
Ultrasonic ultrasonic(trigPin,echoPin); //INICIALIZANDO OS PINOS DO ARDUINO

int distancia;


#define SERVO 9 // Porta Digital 6 PWM
Servo s; // Variável Servo
int pos; // Posição Servo


void setupSensor()
{
  pinMode(echoPin, INPUT); //Pino de Entrada (Recebe)
  pinMode(trigPin, OUTPUT); //Pino de Saída (Envia)

}

void setupMotor()
{
  s.attach(SERVO);
  Serial.begin(9600);
  s.write(0); // Inicia motor posição zero
}


void setup()
{
  Serial.begin(9600);
  setupSensor();
  setupMotor();
}



void acionaSensor()
{
   //Serial.print(ultrasonic.read()); // Prints the distance on the default unit (centimeters)
  distancia = ultrasonic.read();
  
  delay(1000);

}
 
void acionaMotor()
{
  for(pos = 0; pos < 90; pos++)
  {
    s.write(pos);
  delay(15);
  }
delay(500);
  for(pos = 90; pos >= 0; pos--)
  {
    s.write(pos);
    delay(15);
  }
}


void loop()
{
  
 
  distancia = ultrasonic.read();
  Serial.print("Pode ir ");
  delay(500);
  
  if(distancia < 10)
  {
     for(pos = 0; pos < 90; pos++)
    {
        s.write(pos);
        delay(15);
    }
    Serial.print("ESQUERDA");

  }
  
 

 


}

