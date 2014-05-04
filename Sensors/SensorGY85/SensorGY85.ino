#include <Wire.h>
#define MPU6050_I2C_ADDRESS 0x68

void setup()
{
   Wire.begin();        // подключение к шине i2c
   Serial.begin(9600);  // запуск последовательного порта
}

void loop()
{
   Wire.requestFrom(MPU6050_I2C_ADDRESS, 6);    // запрос 6 байт от слейва #2

   while(Wire.available())    // пока есть, что читать
   { 
     char c = Wire.read();    // получаем байт (как символ)
     Serial.print(c);         // печатает в порт
   }

   delay(500);
}
