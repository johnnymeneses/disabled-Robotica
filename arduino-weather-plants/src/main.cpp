#include <Arduino.h>

//Sensor de Humidade da planta 
#define pino_sinal_analogico A0
#define pino_led_vermelho 5
#define pino_led_verde 7
int valor_analogico;


//Sensor de Temperatura do ambiente
#include "DHT.h"
#define DHT11Pin 2
#define DHTType DHT11

//OLED
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define botaoliga 10
int estadoBotao;

DHT HT(DHT11Pin,DHTType);
float humi;
float tempC;
float tempF;


//OLED define
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);



void setupComponentes(){
  pinMode(pino_sinal_analogico, INPUT);
  pinMode(pino_led_vermelho, OUTPUT);
  pinMode(pino_led_verde, OUTPUT);
}

void apagaleds()
{
  digitalWrite(pino_led_vermelho, LOW);
  digitalWrite(pino_led_verde, LOW);
}

void leituraPlanta(){

//Le o valor do pino A0 do sensor
  valor_analogico = analogRead(pino_sinal_analogico);
 
  //Mostra o valor da porta analogica no serial monitor
  Serial.print("Porta analogica: ");
  Serial.print(valor_analogico);
 
  //Solo umido, acende o led verde
  if (valor_analogico > 0 && valor_analogico < 400)
  {
    Serial.println(" Status: Solo umido");
    apagaleds();
    digitalWrite(pino_led_verde, HIGH);
  }
 
  //Solo seco, acende led vermelho
  if (valor_analogico > 800 && valor_analogico < 1024)
  {
    Serial.println(" Status: Solo seco");
    apagaleds();
    digitalWrite(pino_led_vermelho, HIGH);
  }
  delay(100);

}


//Sensor de Temperatura
void setupTemperatura(){

  //For DHT11
  HT.begin();
  //For OLED I2C
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }
  display.display(); //Display logo
  delay(1000); 
  display.clearDisplay();
  
}


void chamaTemperatura(){
   delay(1000);
 humi = HT.readHumidity();
 tempC = HT.readTemperature();
 tempF = HT.readTemperature(true);

 Serial.print("Humidity:");
 Serial.print(humi,0);
 Serial.print("%");
 Serial.print(" Temperature:");
 Serial.print(tempC,1);
 Serial.print("C ~ ");
 Serial.print(tempF,1);
 Serial.println("F");

 display.clearDisplay();
 oledDisplayHeader();
 

 oledDisplay(3,5,28,humi,"%");
 oledDisplay(2,70,16,tempC,"C");
 oledDisplay(2,70,44,tempF,"F");
 
 display.display();
}


void oledDisplayHeader(){
 display.setTextSize(1);
 display.setTextColor(WHITE);
 display.setCursor(0, 0);
 display.print("Umidade");
 display.setCursor(60, 0);
 display.print("Temperatura");
}

void oledDisplay(int size, int x,int y, float value, String unit){
 int charLen=12;
 int xo=x+charLen*3.2;
 int xunit=x+charLen*3.6;
 int xval = x; 
 display.setTextSize(size);
 display.setTextColor(WHITE);
 
 if (unit=="%"){
   display.setCursor(x, y);
   display.print(value,0);
   display.print(unit);
 } else {
   if (value>99){
    xval=x;
   } else {
    xval=x+charLen;
   }
   display.setCursor(xval, y);
   display.print(value,0);
   display.drawCircle(xo, y+2, 2, WHITE);  // print degree symbols (  )
   display.setCursor(xunit, y);
   display.print(unit);
 }
 
}


void setup()
{
  Serial.begin(9600);
  setupComponentes();
   pinMode(botaoliga,INPUT_PULLUP);
  setupTemperatura();
  
}
 
void loop()
{
  leituraPlanta();
  display.clearDisplay();
 chamaTemperatura();

}
 

