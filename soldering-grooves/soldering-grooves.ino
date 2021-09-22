// Прошивка контроллера Arduino Uno, который управляет шаговым двигателем и распределителем.
// Паяет углубления для пилочек.
// На драйвере двигателя установить 800 имп/об

#include <EEPROM.h>            // подключаем библиотеку для работы с памятью
#include <stepperQ.h>          // подключаем библиотеку для работы с ШД
#include "sav_button.h"        // подключаем библиотеку для работы с кнопками
#include "Wire.h"              // подключаем библиотеку для работы с I2C
#include "LiquidCrystal_I2C.h" // подключаем библиотеку для работы с LCD

#define FW 1.0          	// Версия прошивки.
#define STEPPERREV 800 		// Установленное значение на драйвере имп/об.
#define CLK_ENC 3       	// Номер пина для подключения энкодера настроек
#define DT_ENC 4        	// Номер пина для подключения энкодера настроек
#define KEY_ENC 5       	// Номер пина для подключения кнопки энкодера настроек

void setup()
{
  Serial.begin(115200);
  // выводим на дисплей версию прошивки.
  Serial.print("Версия прошивки:");
  Serial.println(FW, 1);
}
void loop()
{

}