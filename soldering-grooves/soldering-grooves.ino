// Прошивка контроллера Arduino Uno, который управляет шаговым двигателем и распределителем.
// Паяет углубления для пилочек.
// На драйвере двигателя установить 800 имп/об

#include <EEPROM.h>            // подключаем библиотеку для работы с памятью
#include <stepperQ.h>          // подключаем библиотеку для работы с ШД
#include "sav_button.h"        // подключаем библиотеку для работы с кнопками
#include "Wire.h"              // подключаем библиотеку для работы с I2C
#include "LiquidCrystal_I2C.h" // подключаем библиотеку для работы с LCD

#define FW 1.0           // Версия прошивки.
#define STEPPERREV 800   // Установленное значение на драйвере имп/об.
#define DISTPERREV 185.0 //Значение длаины материала протягиваемого за оборот
#define CLK_ENC 3        // Номер пина для подключения энкодера настроек
#define DT_ENC 4         // Номер пина для подключения энкодера настроек
#define KEY_ENC 5        // Номер пина для подключения кнопки энкодера настроек

#define pin_but_Start 6 // Кнопка пуск
#define pin_but_Stop 7  // Кнопка стоп
#define pin_but_Feed 10 // Кнопка подача
// #define pin_but_Soldering 10   // Кнопка пайка
#define pin_Sensor_Home 8      // Датчик привязки
#define pin_Sensor_Soldering 2 // Кнопка привязка

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

uint32_t tm_delay = 0; // переменные для задержки через millis
uint16_t dt_delay = 0; // переменные для задержки через millis
int delay_init = 1;    // переменная разрешения инициализации таймера

float SetLenght = 100.0;            // переменная длинны протяжки в mm
float SetHomeSpeed = 5.0;           // переменная скорость привязки в м/мин
float SetWorkSpeed = 10.0;          // переменная скорость протяжки в м/мин
float SetLenghtHome = 100.0;        // переменная длинны привязки mm
float SetTimeoutSoldering = 1000.0; // переменная задержка пайки в ms
float SetTimeoutFeed = 100.0;       // переменная задержка после пайки в ms
float SetQuantity = 10.0;           // переменная количество изготавливаемых изделий

int speed_work = 1000.0;       // переменная для задания скорости работы имп/сек
int accel_set = 30000.0;       // переменная для задания ускорения двигателя имп/сек^2
int speed_home = 1000.0;       // переменная для задания скорости привязки имп/сек
float RemainQuantity = 0.0;    // переменная для подсчета оставшегося количества изготавливаемых изделий
long SetLenghtInPulse = 0;     // переменная для задания длины в импульсах
long SetLenghtHomeInPulse = 0; // переменная для задания длины на которую нужно отьехать после привязки
long TargetPosition = 0;       // переменная позиции в которую нужно доехать

//pin-номер пина
//50-таймаут дребезга
//1000-время длинного нажатия кнопки
// 0 - врямя перевода кнопки в генерацию серии нажатий. По умолсанию отключено
// 0 - время между кликами в серии. По умолчанию 500 мс. Если tm3 = 0 то не работает
SButton BUT_ENC(KEY_ENC, 50, 500, 0, 0);
SButton Feed(pin_but_Feed, 50, 500, 0, 0);

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
  // pinMode(pin_but_Soldering, INPUT_PULLUP);
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
  lcd.setCursor(1, 0);
  lcd.print("Solder groove");
  lcd.setCursor(4, 1);
  lcd.print("FW: ");
  lcd.print(FW, 1);
  delay(2000);
  // записываем в память дефолтные значения если значения не записаны
  if (EEPROM.read(200) != 32)
  {
    EEPROM.put(0, SetLenght);
    EEPROM.put(4, SetHomeSpeed);
    EEPROM.put(8, SetWorkSpeed);
    EEPROM.put(12, SetLenghtHome);
    EEPROM.put(16, SetTimeoutFeed);
    EEPROM.put(20, SetTimeoutSoldering);
    EEPROM.put(24, SetQuantity);

    EEPROM.write(200, 32);
  }
  // читаем значения из памяти
  readEE();

  // инициализируем оставшееся количество деталей которые нужно изготовить
  RemainQuantity = SetQuantity;
  // вычисляем установленную длинну в импульсах
  SetLenghtInPulse = SetLenght * (STEPPERREV / DISTPERREV);
  // вычисляем длинну в импульсах на которую нужно отъехать от датчика
  SetLenghtHomeInPulse = SetLenghtHome * (STEPPERREV / DISTPERREV);
  //вычисляем скорость привязки в имп/сек
  speed_home = (SetHomeSpeed * (STEPPERREV / DISTPERREV) * 1000) / 60;
  //вычисляем скорость работы в имп/сек
  speed_work = (SetWorkSpeed * (STEPPERREV / DISTPERREV) * 1000) / 60;
  // инициализация шагового двигателя
  stepperq.init(pin_dir_Motor, pin_pulse_Motor);
  stepperq.setMaxSpeed(speed_work);
  stepperq.setAcceleration(accel_set);
  stepperq.setCurrentPosition(0);
}

// Чтение параметров из памяти EEPROM
void readEE()
{
  EEPROM.get(0, SetLenght);
  EEPROM.get(4, SetHomeSpeed);
  EEPROM.get(8, SetWorkSpeed);
  EEPROM.get(12, SetLenghtHome);
  EEPROM.get(16, SetTimeoutFeed);
  EEPROM.get(20, SetTimeoutSoldering);
  EEPROM.get(24, SetQuantity);
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
    lcd.setCursor(2, 0);
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
    lcd.print(SetLenght, 1);
    lcd.print("mm");
    break;
  case 2:
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Parameter mode");
    lcd.setCursor(0, 1);
    lcd.print("HSpeed:");
    lcd.print(SetHomeSpeed, 0);
    lcd.print(" M/min");
    break;
  case 3:
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Parameter mode");
    lcd.setCursor(0, 1);
    lcd.print("WSpeed:");
    lcd.print(SetWorkSpeed, 0);
    lcd.print(" M/min");
    break;

  case 4:
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Parameter mode");
    lcd.setCursor(0, 1);
    lcd.print("LenHome:");
    lcd.print(SetLenghtHome, 1);
    lcd.print("mm");
    break;
  case 5:
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Parameter mode");
    lcd.setCursor(0, 1);
    lcd.print("TimFeed:");
    lcd.print(SetTimeoutFeed, 0);
    lcd.print(" ms");
    break;
  case 6:
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Parameter mode");
    lcd.setCursor(0, 1);
    lcd.print("TimSolder:");
    lcd.print(SetTimeoutSoldering, 0);
    lcd.print("ms");
    break;
  case 7:
    lcd.clear();
    lcd.setCursor(1, 0);
    lcd.print("Parameter mode");
    lcd.setCursor(0, 1);
    lcd.print("Quantity:");
    lcd.print(SetQuantity, 0);
    break;

  case 9:
    lcd.clear();
    lcd.setCursor(2, 0);
    lcd.print("Return mode");
    break;

  case 10:
    lcd.clear();
    lcd.setCursor(2, 0);
    lcd.print("Homing mode");
    break;

  case 11:
    lcd.clear();
    lcd.setCursor(3, 0);
    lcd.print("Homing OK");
    lcd.setCursor(3, 1);
    lcd.print("Wait Start");
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

  case 13:
    lcd.clear();
    lcd.setCursor(2, 0);
    lcd.print("Feed mode");
    break;

  case 14:
    lcd.clear();
    lcd.setCursor(2, 0);
    lcd.print("Solder mode");
    break;

  case 15:
    lcd.clear();
    lcd.setCursor(3, 0);
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
      if (e > 7)
        e = 7; // Задание номера экрана для отображения
      else if (e < 1)
        e = 1;   // Задание номера экрана для отображения
      Display(); //обновление дисплея
    }
    break;

  // установка длины
  case 2:
    lcd.setCursor(14, 1);
    lcd.print("<<");
    switch (BUT_ENC.Loop())
    {
    // выход из режима установки без сохранения
    case SB_CLICK:
      SetLenghtInPulse = SetLenght * (STEPPERREV / DISTPERREV);
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
    // выход из режима установки с сохранением в память
    case SB_LONG_CLICK:
      EEPROM.put(0, SetLenght);
      SetLenghtInPulse = SetLenght * (STEPPERREV / DISTPERREV);
      SetPar_st = 1;
      encCnt = 0;
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
      lcd.setCursor(14, 1);
      lcd.print("<<");
    }
    break;

  // установка скорости привязки
  case 3:
    lcd.setCursor(14, 1);
    lcd.print("<<");
    switch (BUT_ENC.Loop())
    {
    // выход из режима установки без сохранения
    case SB_CLICK:
      speed_home = (SetHomeSpeed * (STEPPERREV / DISTPERREV) * 1000) / 60;
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
    // выход из режима установки с сохранением в память
    case SB_LONG_CLICK:
      EEPROM.put(4, SetHomeSpeed);
      speed_home = (SetHomeSpeed * (STEPPERREV / DISTPERREV) * 1000) / 60;
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
    }
    // вращение изменяет значение скорости ручного
    if (encCnt != 0)
    {
      SetHomeSpeed += encCnt * 1;
      encCnt = 0;
      if (SetHomeSpeed > 30)
        SetHomeSpeed = 30;
      else if (SetHomeSpeed < 1)
        SetHomeSpeed = 1;
      Display(); //обновление дисплея
      lcd.setCursor(14, 1);
      lcd.print("<<");
    }
    break;

  // установка скорости работы
  case 4:
    lcd.setCursor(14, 1);
    lcd.print("<<");
    switch (BUT_ENC.Loop())
    {
    // выход из режима установки без сохранения
    case SB_CLICK:
      speed_work = (SetWorkSpeed * (STEPPERREV / DISTPERREV) * 1000) / 60;
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
    // выход из режима установки с сохранением в память
    case SB_LONG_CLICK:
      EEPROM.put(8, SetWorkSpeed);
      speed_work = (SetWorkSpeed * (STEPPERREV / DISTPERREV) * 1000) / 60;
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
    }
    // вращение изменяет значение скорости ручного
    if (encCnt != 0)
    {
      SetWorkSpeed += encCnt * 1;
      encCnt = 0;
      if (SetWorkSpeed > 30)
        SetWorkSpeed = 30;
      else if (SetWorkSpeed < 1)
        SetWorkSpeed = 1;
      Display(); //обновление дисплея
      lcd.setCursor(14, 1);
      lcd.print("<<");
    }
    break;

  // Установка отъезда при привязке
  case 5:
    lcd.setCursor(14, 1);
    lcd.print("<<");
    switch (BUT_ENC.Loop())
    {
    // выход из режима установки без сохранения
    case SB_CLICK:
      SetLenghtHomeInPulse = SetLenghtHome * (STEPPERREV / DISTPERREV);
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
    // выход из режима установки с сохранением в память
    case SB_LONG_CLICK:
      EEPROM.put(12, SetLenghtHome);
      SetLenghtHomeInPulse = SetLenghtHome * (STEPPERREV / DISTPERREV);
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
    }
    // вращение изменяет значение длины за которую выполняется подача
    if (encCnt != 0)
    {
      SetLenghtHome += encCnt * 0.5;
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
  case 6:
    lcd.setCursor(14, 1);
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
      EEPROM.put(16, SetTimeoutFeed);
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
    }
    // вращение изменяет значение таймаута
    if (encCnt != 0)
    {
      SetTimeoutFeed += encCnt * 100;
      encCnt = 0;
      if (SetTimeoutFeed > 10000)
        SetTimeoutFeed = 10000;
      else if (SetTimeoutFeed < 0)
        SetTimeoutFeed = 0;
      Display(); //обновление дисплея
      lcd.setCursor(14, 1);
      lcd.print("<<");
    }
    break;

  // установка задержки пайки
  case 7:
    lcd.setCursor(14, 1);
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
      EEPROM.put(20, SetTimeoutSoldering);
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
      lcd.setCursor(14, 1);
      lcd.print("<<");
    }
    break;

  // установка количества
  case 8:
    lcd.setCursor(14, 1);
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
      EEPROM.put(24, SetQuantity);
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
      lcd.setCursor(14, 1);
      lcd.print("<<");
    }
    break;
  }
}
// Ожидание нажатия кнопок
void ManualWaitButton()
{
  // кнопка стоп нажата
  if (digitalRead(pin_but_Stop) == 1)
  {
    digitalWrite(pin_en_Motor, 1); // включение S-on драйвера
    if (debug)
      Serial.println("StopButon");
    st = 15; // переход в режим стоп кнопка нажата
    alarm = 0;
  }
  // кнопка start нажата
  else if (digitalRead(pin_but_Start) == 0)
  {

    st = 2;
    e = 10;
    Display();
    if (debug)
      Serial.println("StartButton");
  }

  // ни одна из кнопок не нажата и экран не обновлен после перехода в этот режим
  else if (alarm == 1)
  {
    digitalWrite(pin_en_Motor, 0); // включение S-on драйвера
    if (debug)
      Serial.println("Wait PressButton");
    e = 0;     // Задание номера экрана для отображения
    Display(); //обновление дисплея
    alarm = 0;
  }

  // кнопка feed нажата
  switch (Feed.Loop())
  {
  //  Кроткое нажатие запускает ручную подачу
  case SB_CLICK:
    st = 4;
    e = 13;
    Display();
    if (debug)
      Serial.println("ManFeed");
    break;
  //  Долгое  нажатие запускает ручную пайку
  case SB_LONG_CLICK:
    st = 6;
    e = 14;
    Display();
    digitalWrite(pin_out_Soldering, 1);
    if (debug)
      Serial.println("ManSoldering");
    break;
  }
}
// Стоп установки
void StopMechanism()
{
  if (debug)
    Serial.println("StopButon");
  stepperq.stop();
  digitalWrite(pin_out_Soldering, 0); //отключение цилиндра паяльника
  delay(2000);
  st = 0; // Режим ожидание кнопок
}
void loop()
{
  switch (st)
  {
  //  Режим ожидание кнопок
  case 0:
    SettingParameters(); // Провверка кнопки энкодера если нажата переход в режим установки параметров
    ManualWaitButton();  // Проверка состояния кнопок если что-то нажато переход в соответствующий режим
    break;
  // Установки движения привязки авто режима
  case 2:
    if (debug)
      Serial.println("StartAuto");
    stepperq.setCurrentPosition(0);
    TargetPosition = 4000000;
    stepperq.setMaxSpeed(speed_home); // задаем скорость привязки
    if (debug)
      Serial.print("Speed = ");
    if (debug)
      Serial.println(speed_home);
    st = 21; // Старт движения вперед ручной режим
    break;

  // Установки ручной прокрутки
  case 4:
    stepperq.setCurrentPosition(0);
    TargetPosition = SetLenghtInPulse; // Задаем первую точку пайки
    stepperq.setMaxSpeed(speed_work);  // задаем скорость движения
    if (debug)
      Serial.print("Speed = ");
    if (debug)
      Serial.println(speed_work);
    if (debug)
      Serial.print("MoveTo = ");
    if (debug)
      Serial.println(TargetPosition);
    stepperq.moveTo(TargetPosition);
    stepperq.start();
    st = 5;
    break;

  // Ручное перемещение на заданную длинну
  case 5:
    if (digitalRead(pin_but_Stop) == 0)
    {
      // ожидание пока не доедет до позиции
      if (stepperq.currentPosition() == TargetPosition)
      {
        if (debug)
          Serial.println("EndFeed");
        st = 0;    //переход в режим ожидания кнопок
        alarm = 1; // переменная для переключения экрана
        Display();
      }
    }
    else
    {
      StopMechanism();
    }
    break;
  // Ручная пайка
  case 6:
    if (digitalRead(pin_but_Stop) == 0)
    {
      // условие если сработал датчик опускания паяльникка
      if (digitalRead(pin_Sensor_Soldering) == 0)
      {
        if (delay_init == 1)
        {
          tm_delay = millis();
          delay_init = 0;
        }
        dt_delay = millis() - tm_delay;
        if (dt_delay >= SetTimeoutSoldering)
        {
          delay_init = 1;
          digitalWrite(pin_out_Soldering, 0); // отключение цилиндра паяльникка
          while (digitalRead(pin_but_Feed) == 0)
            delay(10);
          st = 0;    //переход в режим ожидания кнопок
          alarm = 1; // переменная для переключения экрана
          Display(); // обновление дисплея
          if (debug)
            Serial.println("EndSoldering");
        }
      }
    }
    else
    {
      delay_init = 1;
      StopMechanism();
    }

    break;

  // Старт движения привязки
  case 21:
    if (debug)
      Serial.println("StartMove");
    if (debug)
      Serial.print("MoveTo = ");
    if (debug)
      Serial.println(TargetPosition);
    stepperq.moveTo(TargetPosition);
    stepperq.start();
    st = 22; // Движение до датчика
    break;

  // Движение до датчика
  case 22:
    if (digitalRead(pin_but_Stop) == 0)
    {
      // ожидание пока не сработает датчик Home плиты или кнопка стоп
      if (digitalRead(pin_Sensor_Home) == 0)
      {
        stepperq.stop();
        if (debug)
          Serial.println("SensorHomeOk");
        delay(2000);
        stepperq.setCurrentPosition(0);
        TargetPosition = SetLenghtHomeInPulse;
        st = 23; // отъезд на заданное расстояние
      }
    }
    else
    {
      StopMechanism();
    }
    break;

  // Старт движения отъезда
  case 23:
    if (digitalRead(pin_but_Stop) == 0)
    {
      if (debug)
        Serial.println("MoveHomePosition");
      if (debug)
        Serial.print("MoveToHome = ");
      if (debug)
        Serial.println(TargetPosition);
      stepperq.setCurrentPosition(0);
      stepperq.moveTo(TargetPosition);
      stepperq.start();
      st = 24; // ожидание окончания движения
    }
    else
    {
      StopMechanism();
    }
    break;

  // ожидание окончания движения
  case 24:
    if (digitalRead(pin_but_Stop) == 0)
    {
      // ожидание пока не доедет до позиции Home
      if (stepperq.currentPosition() == TargetPosition)
      {
        if (debug)
          Serial.println("HomeOk");
        e = 11;
        Display();
        delay(2000);
        stepperq.setCurrentPosition(0);
        st = 25; // Ожидание кнопки старт
      }
    }
    else
    {
      StopMechanism();
    }
    break;

  // Ожидание кнопки старт
  case 25:
    if (digitalRead(pin_but_Stop) == 0)
    {
      if (digitalRead(pin_but_Start) == 0)
      {
        if (debug)
          Serial.println("StartCicle");
        e = 12;
        Display();
        st = 250; // Начало рабочего цикла
      }
    }
    else
    {
      StopMechanism();
    }
    break;

  // Начало рабочего цикла
  case 250:
    if (digitalRead(pin_but_Stop) == 0)
    {

      if (digitalRead(pin_but_Start) == 0)
      {
        if (debug)
          Serial.println("Cicle");
        e = 12;
        Display();
        st = 251; // Установки начала движения
      }
    }
    else
    {
      StopMechanism();
    }
    break;

  // Установки начала движения
  case 251:
    stepperq.setCurrentPosition(0);
    RemainQuantity = SetQuantity;      // обновление оставшегося количества изготавливаемых изделий
    TargetPosition = SetLenghtInPulse; // Задаем первую точку пайки
    stepperq.setMaxSpeed(speed_work);  // задаем скорость движения
    if (debug)
      Serial.print("Speed = ");
    if (debug)
      Serial.println(speed_work);
    st = 252; // Старт цикла движения
    break;

  // Старт цикла движения
  case 252:
    if (debug)
      Serial.println("StartCicle");
    if (debug)
      Serial.print("MoveTo = ");
    if (debug)
      Serial.println(TargetPosition);
    stepperq.moveTo(TargetPosition);
    stepperq.start();
    st = 253; // Движение на заданную длину
    break;

  // Движение на заданную длину
  case 253:
    if (digitalRead(pin_but_Stop) == 0)
    {
      // ожидание пока не доедет до позиции Home
      if (stepperq.currentPosition() == TargetPosition)
      {
        if (debug)
          Serial.println("AutoSoldering");
        TargetPosition = stepperq.currentPosition() + SetLenghtInPulse; // вычисление позиции следующей пайки
        digitalWrite(pin_out_Soldering, 1);                             // включение паяльника
        st = 254;                                                       // Завершение пайки
      }
    }
    else
    {
      StopMechanism();
    }
    break;

  // Завершение пайки
  case 254:
    if (digitalRead(pin_but_Stop) == 0)
    {
      // условие если сработал датчик опускания паяльникка
      if (digitalRead(pin_Sensor_Soldering) == 0)
      {
        if (delay_init == 1)
        {
          tm_delay = millis();
          delay_init = 0;
        }
        dt_delay = millis() - tm_delay;
        if (dt_delay >= SetTimeoutSoldering)
        {
          delay_init = 1;
          digitalWrite(pin_out_Soldering, 0); // отключение цилиндра паяльникка
          while (digitalRead(pin_Sensor_Soldering) == 0)
            delay(10);
          delay(SetTimeoutFeed);               // Задержка на поднятие паяльника (перед началом следующего движения)
          RemainQuantity = RemainQuantity - 1; // вычисление оставшегося количества изготавливаемых изделий
          Display();                           // обновление дисплея
          // условие оставшееся количество изготавливаемых изделий равно нулю
          if (RemainQuantity == 0)
          {
            if (debug)
              Serial.println("AutoPause");
            st = 255; //Возврат на исходную
            e = 9;
            TargetPosition = -(SetLenghtHomeInPulse * 2 + SetLenghtInPulse * SetQuantity);
            Display();                    //обновление дисплея
            RemainQuantity = SetQuantity; // обновление оставшегося количества изготавливаемых изделий
          }
          // оставшееся количество изготавливаемых изделий не равно нулю
          else
          {
            st = 252; // Старт цикла движения
          }
        }
      }
    }
    else
    {
      delay_init = 1;
      StopMechanism();
    }
    break;

  // Старт цикла движения
  case 255:
    if (debug)
      Serial.println("StartReturn");
    if (debug)
      Serial.print("MoveTo = ");
    if (debug)
      Serial.println(TargetPosition);
    stepperq.moveTo(TargetPosition);
    stepperq.start();
    st = 256; // Окончание движения возврата
    break;

  // Окончание движения возврата
  case 256:
    if (digitalRead(pin_but_Stop) == 0)
    {
      // ожидание пока не доедет до позиции Home
      if (stepperq.currentPosition() == TargetPosition)
      {
        if (debug)
          Serial.println("EndSoldering");
        st = 0;
        e = 0;
        alarm = 1;
        Display();
      }
    }
    else
    {
      StopMechanism();
    }
    break;

  // Режим стоп кнопка нажата
  case 15:
    // условие если кнопка стоп не нажата
    if (digitalRead(pin_but_Stop) == 0)
    {
      RemainQuantity = SetQuantity; // обновление оставшегося количества изготавливаемых изделий
      st = 0;                       // Ручной режим ожидание кнопок
    }
    // кнопка стоп нажата
    else
    {
      if (alarm == 0)
      {
        alarm = 1;
        e = 15;    // Задание номера экрана для отображения
        Display(); //обновление дисплея
      }
      SettingParameters(); // Провверка кнопки энкодера если нажата переход в режим установки параметров
    }
    break;

  // Режим настройки параметров
  case 100:
    SettingParameters(); // Провверка кнопки энкодера если нажата переход в режим установки параметров
    break;
  }
}
