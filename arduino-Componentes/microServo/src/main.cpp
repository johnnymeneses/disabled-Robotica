/*=============================================================================
Autor:        Johnny Meneses [bilbo_vader@outlook.com]
Descrição:    Lógica para movimentar um micro Servo

Lista de Componentes
[1] Arduino Uno R3
[1] Positional Micro Servo


*/

#include <Arduino.h>
#include <Servo.h>


 
#define SERVO 9 // Porta Digital 6 PWM
 
Servo s; // Variável Servo
int pos; // Posição Servo
 
void setup ()
{
  s.attach(SERVO);
  Serial.begin(9600);
  s.write(0); // Inicia motor posição zero
}
 
void loop()
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