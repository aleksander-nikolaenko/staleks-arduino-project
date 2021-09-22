// Прошивка контроллера Arduino Uno, который управляет шаговым двигателем и распределителем.
// Паяет углубления для пилочек.
// На драйвере двигателя установить 800 имп/об

#include <EEPROM.h>            // подключаем библиотеку для работы с памятью
#include <stepperQ.h>          // подключаем библиотеку для работы с ШД
#include "sav_button.h"        // подключаем библиотеку для работы с кнопками
#include "Wire.h"              // подключаем библиотеку для работы с I2C
#include "LiquidCrystal_I2C.h" // подключаем библиотеку для работы с LCD

#define FW 1.0         // Версия прошивки.
#define STEPPERREV 800 // Установленное значение на драйвере имп/об.
#define CLK_ENC 3      // Номер пина для подключения энкодера настроек
#define DT_ENC 4       // Номер пина для подключения энкодера настроек
#define KEY_ENC 5      // Номер пина для подключения кнопки энкодера настроек

#define pin_but_Start 7      // Кнопка пуск
#define pin_but_Stop 8       // Кнопка стоп
#define pin_but_Home 2       // Кнопка привязка
#define pin_but_Feed 6       // Кнопка подача
#define pin_but_Soldering 10 // Кнопка пайка
#define pin_Sensor_Home 8    // Датчик привязки

#define pin_out_Soldering 9 // Выход включение распределителя подачи материала
#define pin_en_Motor A0     // En драйвера мотора
#define pin_pulse_Motor A1  // Pulse драйвера мотора
#define pin_dir_Motor A2    // Dir драйвера мотора

int debug = 1;     // переменная для вывода информации отладки в последовательный порт ((debug = 1 включено) (debug = 0 отключено) )
int alarm = 1;     // переменная для фиксации смены состояния аварий
int st = 0;        // переменная переключения режимов работы контроллера
int e = 0;         // переменная переключения экранов LCD
int SetPar_st = 0; // переменная переключения состояния при установке параметров

volatile int encCnt = 0; // переменная счета срабатываний энкодера

float SetLenght = 100.0;           // переменная длинны протяжки в mm
float SetSpeed = 10.0;             // переменная скорость протяжки в м/мин
float SetLenghtHome = 10.0;        // переменная длинны привязки mm
float SetTimeoutSoldering = 500.0; // переменная задержка пайки в ms
float SetTimeoutFeed = 100.0;      // переменная задержка после пайки в ms
float SetQuantity = 10.0;          // переменная количество изготавливаемых изделий

int speed_set = 1000.0;  // переменная для задания скорости работы имп/сек
int accel_set = 30000.0; // переменная для задания ускорения двигателя имп/сек^2

float RemainQuantity = 0.0;    // переменная для подсчета оставшегося количества изготавливаемых изделий
long SetLenghtInPulse = 0;     // переменная для задания длины в импульсах
long SetLenghtHomeInPulse = 0; // переменная для задания длины на которую нужно отьехать после привязки

//pin-номер пина
//50-таймаут дребезга
//1000-время длинного нажатия кнопки
// 0 - врямя перевода кнопки в генерацию серии нажатий. По умолсанию отключено
// 0 - время между кликами в серии. По умолчанию 500 мс. Если tm3 = 0 то не работает
SButton BUT_ENC(KEY_ENC, 50, 1000, 0, 0);
SButton Start(pin_but_Start, 50, 1000, 0, 0);

//создаём объект lcd адрес I2C 0x27 или 0x3f
LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup()
{
  Serial.begin(115200);
  // выводим на дисплей версию прошивки.
  Serial.print("Версия прошивки:");
  Serial.println(FW, 1);
  // lcd инициализация
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Выставляем режимы работы пинов
  pinMode(pin_Sensor_Home, INPUT_PULLUP);

  pinMode(pin_but_Start, INPUT_PULLUP);
  pinMode(pin_but_Stop, INPUT_PULLUP);
  pinMode(pin_but_Soldering, INPUT_PULLUP);
  pinMode(pin_but_Feed, INPUT_PULLUP);

  pinMode(pin_en_Motor, OUTPUT);
  digitalWrite(pin_en_Motor, 0); //отключение мотора
  pinMode(pin_pulse_Motor, OUTPUT);
  digitalWrite(pin_pulse_Motor, 1); //начальное состояие pulse
  pinMode(pin_dir_Motor, OUTPUT);
  digitalWrite(pin_dir_Motor, 1); //начальное состояие dir

  pinMode(pin_out_Soldering, OUTPUT);
  digitalWrite(pin_out_Soldering, 0); //отключение цилиндра паяльника
  delay(50);
  // инициализируем прерывание для энкодера настроек
  attachInterrupt(digitalPinToInterrupt(CLK_ENC), enc, FALLING);
  delay(50);
  // выводим на дисплей версию прошивки.
  lcd.setCursor(4, 0);
  lcd.print("Soldering");
  lcd.setCursor(6, 2);
  lcd.print("FW: ");
  lcd.print(FW, 1);
  delay(2000);
  // записываем в память дефолтные значения если значения не записаны
  if (EEPROM.read(200) != 32)
  {
    EEPROM.put(0, SetLenght);
    EEPROM.put(4, SetSpeed);
    EEPROM.put(8, SetLenghtHome);
    EEPROM.put(12, SetTimeoutFeed);
    EEPROM.put(16, SetTimeoutSoldering);
    EEPROM.put(20, SetQuantity);

    EEPROM.write(200, 32);
  }
  // читаем значения из памяти
  readEE();

  // инициализируем оставшееся количество деталей которые нужно изготовить
  RemainQuantity = SetQuantity; // обновление оставшегося количества изготавливаемых изделий
                                // вычисляем установленную длинну в импульсах
  SetLenghtInPulse = SetLenght * (STEPPERREV / 265.0);
  // вычисляем длинну в импульсах на которую нужно отъехать от датчика
  SetLenghtHomeInPulse = SetLenghtHome * (STEPPERREV / 265.0);
  //вычисляем скорость в имп/сек
  speed_set = (SetSpeed * (STEPPERREV / 265.0) * 1000) / 60;
  // инициализация шагового двигателя
  stepperq.init(pin_dir_Motor, pin_pulse_Motor);
  stepperq.setMaxSpeed(speed_set);
  stepperq.setAcceleration(accel_set);
  stepperq.setCurrentPosition(0);
}

// Чтение параметров из памяти EEPROM
void readEE()
{
  EEPROM.get(0, SetLenght);
  EEPROM.get(4, SetSpeed);
  EEPROM.get(8, SetLenghtHome);
  EEPROM.get(12, SetTimeoutFeed);
  EEPROM.get(26, SetTimeoutSoldering);
  EEPROM.get(20, SetQuantity);
}
// Прерывание срабатывание энкодера
void enc()
{
  if (digitalRead(CLK_ENC) == digitalRead(DT_ENC))
    encCnt++;
  else
    encCnt--;
}

// Обновление дисплея в соответствии с выбранным экраном для отображения
void Display()
{
  switch (e)
  {
  case 0:
    lcd.clear();
    lcd.setCursor(4, 0);
    lcd.print("Wait mode");
    lcd.setCursor(0, 1);
    lcd.print("sQt:");
    lcd.print(SetQuantity, 0);
    lcd.setCursor(8, 1);
    lcd.print("rQt:");
    lcd.print(RemainQuantity, 0);
    break;
  case 1:
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Parameter mode");
    lcd.setCursor(0, 1);
    lcd.print("Lenght:");
    lcd.print(SetLenght, 0);
    lcd.print(" mm");
    break;
  case 2:
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Parameter mode");
    lcd.setCursor(0, 1);
    lcd.print("WorkSpeed:");
    lcd.print(SetSpeed, 0);
    lcd.print(" M/min");
    break;

  case 3:
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Parameter mode");
    lcd.setCursor(0, 1);
    lcd.print("LenghtHome:");
    lcd.print(SetLenghtHome, 0);
    lcd.print(" mm");
    break;
  case 4:
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Parameter mode");
    lcd.setCursor(0, 1);
    lcd.print("TimFeed:");
    lcd.print(SetTimeoutFeed, 0);
    lcd.print(" ms");
    break;
  case 5:
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Parameter mode");
    lcd.setCursor(0, 2);
    lcd.print("TimSoldering:");
    lcd.print(SetTimeoutSoldering, 0);
    lcd.print(" ms");
    break;
  case 6:
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Parameter mode");
    lcd.setCursor(0, 2);
    lcd.print("Quantity:");
    lcd.print(SetQuantity, 0);
    break;

  case 10:
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Homing mode");

    break;

  case 11:
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Homing OK");
    lcd.setCursor(3, 1);
    lcd.print("Wait start");
    break;

  case 12:
    lcd.clear();
    lcd.setCursor(4, 0);
    lcd.print("Work mode");
    lcd.setCursor(0, 1);
    lcd.print("sQt:");
    lcd.print(SetQuantity, 0);
    lcd.setCursor(8, 1);
    lcd.print("rQt:");
    lcd.print(RemainQuantity, 0);
    break;

  case 15:
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Stop mode");
    lcd.setCursor(0, 1);
    lcd.print("sQt:");
    lcd.print(SetQuantity, 0);
    lcd.setCursor(8, 1);
    lcd.print("rQt:");
    lcd.print(RemainQuantity, 0);
    break;
  }
}

// Установка параметров
void SettingParameters()
{
  switch (SetPar_st)
  {
  //Читаем кнопку энкодера если долгое нажатие переходим в режим установки параметров
  case 0:
    switch (BUT_ENC.Loop())
    {
    case SB_LONG_CLICK:
      SetPar_st = 1;
      st = 100;
      encCnt = 0;
      e = 1;     // Задание номера экрана для отображения
      Display(); //обновление дисплея
      break;
    }
    break;

    // Просмотр параметров вращением энкодера
  case 1:
    switch (BUT_ENC.Loop())
    {
      // переход в режим установки параметра
    case SB_CLICK:
      SetPar_st = SetPar_st + e;
      break;
      // Выход из режима установка параметров
    case SB_LONG_CLICK:
      SetPar_st = 0;
      st = 0; // Ручной режим ожидание кнопок
      encCnt = 0;
      e = 0;     // Задание номера экрана для отображения
      Display(); //обновление дисплея
      break;
    }
    // вращение энкодера изменяет номер просматриваемого параметра
    if (encCnt != 0)
    {
      e += encCnt;
      encCnt = 0;
      if (e > 6)
        e = 6; // Задание номера экрана для отображения
      else if (e < 1)
        e = 1;   // Задание номера экрана для отображения
      Display(); //обновление дисплея
    }
    break;

    // установка длины плиты
  case 2:
    lcd.setCursor(15, 1);
    lcd.print("<<");
    switch (BUT_ENC.Loop())
    {
    // выход из режима установки без сохранения
    case SB_CLICK:
      SetLenghtInPulse = SetLenght * (STEPPERREV / 265.0);
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
    // выход из режима установки с сохранением в память
    case SB_LONG_CLICK:
      EEPROM.put(0, SetLenght);
      SetLenghtInPulse = SetLenght * (STEPPERREV / 265.0);
      SetPar_st = 1;
      encCnt = 0;
      st = 0;    // Ручной режим ожидание кнопок
      Display(); //обновление дисплея
      break;
    }
    // вращение изменяет значение длинны
    if (encCnt != 0)
    {
      SetLenght += encCnt * 0.5;
      encCnt = 0;
      if (SetLenght > 300)
        SetLenght = 300;
      else if (SetLenght < 0.5)
        SetLenght = 0.5;
      Display(); //обновление дисплея
      lcd.setCursor(15, 1);
      lcd.print("<<");
    }
    break;
  
    // установка скорости
    case 3: 
    lcd.setCursor(15, 1);
    lcd.print("<<");
    switch (BUT_ENC.Loop())
    {
      // выход из режима установки без сохранения
    case SB_CLICK:
      speed_set = (SetSpeed * (STEPPERREV / 265.0) * 1000) / 60;
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
      // выход из режима установки с сохранением в память
    case SB_LONG_CLICK:
      EEPROM.put(4, SetSpeed);
      speed_set = (SetSpeed * (STEPPERREV / 265.0) * 1000) / 60;
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
    }
    // вращение изменяет значение скорости ручного
    if (encCnt != 0)
    {
      SetSpeed += encCnt * 1;
      encCnt = 0;
      if (SetSpeed > 30)
        SetSpeed = 30;
      else if (SetSpeed < 1)
        SetSpeed = 1;
      Display(); //обновление дисплея
      lcd.setCursor(15, 1);
      lcd.print("<<");
    }
    break;
// Установка отъезда при привязке
  case 4:
    lcd.setCursor(15, 1);
    lcd.print("<<");
    switch (BUT_ENC.Loop())
    {
      // выход из режима установки без сохранения
    case SB_CLICK:
      SetLenghtHomeInPulse = SetLenghtHome * (STEPPERREV / 265.0);
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
      // выход из режима установки с сохранением в память
    case SB_LONG_CLICK:
      EEPROM.put(8, SetLenghtHome);
      SetLenghtHomeInPulse = SetLenghtHome * (STEPPERREV / 265.0);
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
    }
    // вращение изменяет значение длины за которую выполняется подача
    if (encCnt != 0)
    {
      SetLenghtHome += encCnt * 1;
      encCnt = 0;
      if (SetLenghtHome > 500)
        SetLenghtHome = 500;
      else if (SetLenghtHome < 0)
        SetLenghtHome = 0;
      Display(); //обновление дисплея
      lcd.setCursor(15, 1);
      lcd.print("<<");
    }
    break;

// установка задержки перед подачей
  case 5:
    lcd.setCursor(15, 1);
    lcd.print("<<");
    switch (BUT_ENC.Loop())
    {
      // выход из режима установки без сохранения
    case SB_CLICK:
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
      // выход из режима установки с сохранением в память
    case SB_LONG_CLICK:
      EEPROM.put(12, SetTimeoutFeed);
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
    }
    // вращение изменяет значение таймаута
    if (encCnt != 0)
    {
      SetTimeoutFeed += encCnt * 10;
      encCnt = 0;
      if (SetTimeoutFeed > 1000)
        SetTimeoutFeed = 1000;
      else if (SetTimeoutFeed < 0)
        SetTimeoutFeed = 0;
      Display(); //обновление дисплея
      lcd.setCursor(15, 1);
      lcd.print("<<");
    }
    break;

// установка задержки пайки
  case 6:
    lcd.setCursor(0, 1);
    lcd.print("<<");
    switch (BUT_ENC.Loop())
    {
      // выход из режима установки без сохранения
    case SB_CLICK:
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
      // выход из режима установки с сохранением в память
    case SB_LONG_CLICK:
      EEPROM.put(16, SetTimeoutSoldering);
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
    }
    // вращение изменяет значение таймаута
    if (encCnt != 0)
    {
      SetTimeoutSoldering += encCnt * 100;
      encCnt = 0;
      if (SetTimeoutSoldering > 5000)
        SetTimeoutSoldering = 5000;
      else if (SetTimeoutSoldering < 0)
        SetTimeoutSoldering = 0;
      Display(); //обновление дисплея
      lcd.setCursor(15, 1);
      lcd.print("<<");
    }
    break;
    
// установка количества
  case 7:
    lcd.setCursor(15, 1);
    lcd.print("<<");
    switch (BUT_ENC.Loop())
    {
      // выход из режима установки без сохранения
    case SB_CLICK:
      RemainQuantity = SetQuantity; // обновление оставшегося количества изготавливаемых изделий
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
      // выход из режима установки с сохранением в память
    case SB_LONG_CLICK:
      EEPROM.put(20, SetQuantity);
      RemainQuantity = SetQuantity; // обновление оставшегося количества изготавливаемых изделий
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
    }
    // вращение изменяет значение количества изделий которые нужно изготовить
    if (encCnt != 0)
    {
      SetQuantity += encCnt * 1;
      encCnt = 0;
      if (SetQuantity > 100)
        SetQuantity = 100;
      else if (SetQuantity < 0)
        SetQuantity = 0;
      Display(); //обновление дисплея
      lcd.setCursor(0, 1);
      lcd.print("<<");
    }
    break;

  }
}

void loop()
{
}
