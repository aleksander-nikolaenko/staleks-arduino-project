//Для контроллер платы управления линии поклейки минеральной пилочки
//На серводрайвере установить 1000 имп/об.

#include <EEPROM.h>            // подключаем библиотеку для работы с памятью
#include <stepperQ.h>          // подключаем библиотеку для работы с ШД
#include "sav_button.h"        // подключаем библиотеку для работы с кнопками
#include "Wire.h"              // подключаем библиотеку для работы с I2C
#include "LiquidCrystal_I2C.h" // подключаем библиотеку для работы с LCD

#define FW 2.1          // Версия прошивки.
#define STEPPERREV 1000 //Установленное значение серводрайвере имп/об.
#define CLK_ENC 2       // Номер пина для подключения энкодера настроек
#define DT_ENC 4        // Номер пина для подключения энкодера настроек
#define KEY_ENC 6       // Номер пина для подключения кнопки энкодера настроек

#define pin_SensorKnife 3 // Датчик цилидра гильотины
#define pin_SensorFeed 5  // Датчик цилиндра подачи

#define pin_but_Start 7  // Кнопка пуск
#define pin_but_Pause 9  // Кнопка пауза
#define pin_but_Stop 11  // Кнопка стоп
#define pin_but_Fwd 13   // Кнопка промотка вперед
#define pin_but_Rev 17   // Кнопка промотка назад
#define pin_but_Feed 19  // Кнопка подача плиты
#define pin_but_Knife 23 // Кнопка удар гильотины

#define pin_son_Motor 30   // Son драйвера мотра
#define pin_pulse_Motor 26 // Pulse драйвера мотора
#define pin_dir_Motor 24   // Dir драйвера мотора
#define pin_alm_Motor 22   // Alarm драйвера мотора

#define pin_out_Feed 8   // Выход включение распределителя подачи материала
#define pin_out_Knife 10 // Выход включение распределителя гильотины

int debug = 1;     // переменная для вывода информации отладки в последовательный порт ((debug = 1 включено) (debug = 0 отключено) )
int alarm = 1;     // переменная для фиксации смены состояния аварий
int st = 0;        // переменная переключения режимов работы контроллера
int e = 0;         // переменная переключения экранов LCD
int SetPar_st = 0; // переменная переключения состояния при установке параметров

uint32_t tm_disp = 0; // переменные для обновления дисплея в режиме fwd
uint16_t dt_disp = 0; // переменные для обновления дисплея в режиме fwd

float SetLenght = 1000.0;    // переменная длинны протяжки в mm
float SetWorkSpeed = 10.0;   // переменная скорость протяжки в м/мин
float SetManSpeed = 10.0;    // переменная скорость протяжки в м/мин
float SetLenghtFeed = 0.0;   // переменная длинны включения подачи mm
float SetTimeoutFeed = 0.0;  // переменная задержка перед подачей в ms
float SetTimeoutKnife = 0.0; // переменная задержка после реза в ms
float SetQuantity = 100.0;   // переменная количество изготавливаемых изделий

volatile int encCnt = 0; // переменная счета срабатываний энкодера

int work_speed_set = 1000.0;     // переменная для задания скорости работы имп/сек в авто режиме
int man_speed_set = 1000.0;      // переменная для задания скорости работы имп/сек в ручном режиме
int accel_set = 50000.0;         // переменная для задания ускорения двигателя имп/сек^2
float UsedLenght = 0.0;          // переменная для подсчета длинны используемого материала
float MakeParts = 0.0;           // переменная для подсчета количества изготовленных деталей
long TargetPositionFeed = 0;     // переменная позиции в которой должен подаваться материал в ручном режиме
long TargetPositionFeedAuto = 0; // переменная позиции в которой должен подаваться материал в авто режиме
long TargetPositionKnife = 0;    // переменная позиции в которой должена включаться гильотина
long TargetPositionFwd = 0;      // переменная позиции до которой на паузе в авто режиме можно промотать вперед
long TargetPositionRev = 0;      // переменная позиции до которой на паузе в авто режиме можно промотать назад
long StartPosition = 0;          // переменная позиции с которой началась работа (позиция первой подачи плиты)
long SetLenghtInPulse = 0;       // переменная для задания длины плиты в импульсах
long SetLenghtFeedInPulse = 0;   // переменная для задания длины за сколько подается плита в импульсах (для авто режима без остановок)
float PulsePerM = 0;             // переменная количество импульсов на метр (для подсчета длины в метрах)

float RemainQuantity = 0.0; // переменная для подсчета оставшегося количества изготавливаемых изделий
String Cmd;                 // переменная для приема команд с другого контроллера

//pin-номер пина
//50-таймаут дребезга
//1000-время длинного нажатия кнопки
// 0 - врямя перевода кнопки в генерацию серии нажатий. По умолсанию отключено
// 0 - время между кликами в серии. По умолчанию 500 мс. Если tm3 = 0 то не работает
SButton BUT_ENC(KEY_ENC, 50, 1000, 0, 0);
SButton Start(pin_but_Start, 50, 1000, 0, 0);

//создаём объект lcd адрес I2C 0x27 или 0x3f
LiquidCrystal_I2C lcd(0x27, 20, 4);

// Настройка контроллера код который выполняется при загрузке
void setup()
{
  Serial.begin(115200);
  // выводим на дисплей версию прошивки.
  Serial.print("Версия прошивки:");
  Serial.println(FW, 1);
  Serial3.begin(9600);
  Serial3.print("STOP\n"); // передаем контроллеру подмотки команду стоп

  // lcd инициализация
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Выставляем режимы работы пинов
  pinMode(pin_SensorKnife, INPUT_PULLUP);
  pinMode(pin_SensorFeed, INPUT_PULLUP);

  pinMode(pin_but_Start, INPUT_PULLUP);
  pinMode(pin_but_Pause, INPUT_PULLUP);
  pinMode(pin_but_Stop, INPUT_PULLUP);

  pinMode(pin_but_Fwd, INPUT_PULLUP);
  pinMode(pin_but_Rev, INPUT_PULLUP);
  pinMode(pin_but_Knife, INPUT_PULLUP);
  pinMode(pin_but_Feed, INPUT_PULLUP);

  pinMode(pin_alm_Motor, INPUT_PULLUP);
  pinMode(pin_son_Motor, OUTPUT);
  digitalWrite(pin_son_Motor, 0);
  pinMode(pin_pulse_Motor, OUTPUT);
  digitalWrite(pin_pulse_Motor, 0);
  pinMode(pin_dir_Motor, OUTPUT);
  digitalWrite(pin_dir_Motor, 0);

  pinMode(pin_out_Feed, OUTPUT);
  pinMode(pin_out_Knife, OUTPUT);
  digitalWrite(pin_out_Feed, 0);  //отключение цилиндра подачи плиты
  digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
  delay(50);
  // инициализируем прерывание для энкодера настроек
  attachInterrupt(digitalPinToInterrupt(CLK_ENC), enc, FALLING);
  delay(50);
  // выводим на дисплей версию прошивки.
  lcd.setCursor(1, 1);
  lcd.print("Mineral Controller");
  lcd.setCursor(6, 2);
  lcd.print("FW: ");
  lcd.print(FW, 1);
  delay(2000);
  // записываем в память дефолтные значения если значения не записаны
  if (EEPROM.read(200) != 32)
  {
    EEPROM.put(0, SetLenght);
    EEPROM.put(4, SetWorkSpeed);
    EEPROM.put(8, SetManSpeed);
    EEPROM.put(12, SetLenghtFeed);
    EEPROM.put(16, SetTimeoutFeed);
    EEPROM.put(20, SetTimeoutKnife);
    EEPROM.put(24, SetQuantity);

    EEPROM.write(200, 32);
  }
  // читаем значения из памяти
  readEE();
  // инициализируем оставшееся количество деталей которые нужно изготовить
  RemainQuantity = SetQuantity; // обновление оставшегося количества изготавливаемых изделий
  // вычисляем установленную длинну в импульсах
  SetLenghtInPulse = ((SetLenght / 1000.0) * 74900.0 * STEPPERREV) / 10000.0;
  // вычисляем длинну в импульсах за которую происходит подача плиты
  SetLenghtFeedInPulse = ((SetLenghtFeed / 1000) * 74900 * STEPPERREV) / 10000;
  // вычисляем количество импульсов на 1 метр
  PulsePerM = (74900 * STEPPERREV) / 10000;
  //вычисляем скорости в имп/сек
  man_speed_set = (SetManSpeed * (74900 * STEPPERREV) / 10000) / 60;
  work_speed_set = (SetWorkSpeed * (74900 * STEPPERREV) / 10000) / 60;
  // инициализация шагового двигателя
  stepperq.init(pin_dir_Motor, pin_pulse_Motor);
  stepperq.setMaxSpeed(man_speed_set);
  stepperq.setAcceleration(accel_set);
  stepperq.setCurrentPosition(0);
}
// Бесконечный цикл микроконтроллера
void loop()
{
  switch (st)
  {
  // Ручной режим ожидание кнопок
  case 0:
    SettingParameters(); // Провверка кнопки энкодера если нажата переход в режим установки параметров
    ManualWaitButton();  // Проверка состояния кнопок если что-то нажато переход в соответствующий режим
    break;

  // Режим стоп авария сервомотора
  case 10:
    // условие если нет сигнала авария серводвигателя
    if (digitalRead(pin_alm_Motor) == 0)
    {
      RemainQuantity = SetQuantity; // обновление оставшегося количества изготавливаемых изделий
      st = 0;                       // Ручной режим ожидание кнопок
    }
    // есть сигнал авария серводвигателя
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

  // Режим стоп кнопка нажата
  case 11:
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
        e = 16;    // Задание номера экрана для отображения
        Display(); //обновление дисплея
      }
      SettingParameters(); // Провверка кнопки энкодера если нажата переход в режим установки параметров
    }
    break;
  // Режим пауза авария подмотки лайнера
  case 12:
    // если нажата кнопка энкодера настроек
    if (digitalRead(KEY_ENC) == 0)
    {
      if (debug)
        Serial.println("AutoPause");
      st = 70;   // переход в  режим пауза
      e = 11;    // Задание номера экрана для отображения
      Display(); //обновление дисплея
    }
    // кнопка энкодера настроек не нажата
    else
    {
      if (alarm == 0)
      {
        alarm = 1;
        e = 17;    // Задание номера экрана для отображения
        Display(); //обновление дисплея
      }
    }
    break;

  // Установки для ручного движения вперед
  case 20:
    if (debug)
      Serial.println("StartManual");
    StartPosition = stepperq.currentPosition();
    TargetPositionFeed = stepperq.currentPosition() + SetLenghtInPulse; // вычисляем позицию следующей подачи плиты в ручном режиме
    stepperq.setMaxSpeed(man_speed_set);
    if (debug)
      Serial.print("ManSpeed = ");
    if (debug)
      Serial.println(SetManSpeed, 0);
    Serial3.println(SetManSpeed, 0); // передаем контроллеру подмотки заданную скорость работы
    st = 21;                         // Старт движения вперед ручной режим
    break;
  // Старт движения вперед ручной режим
  case 21:
    if (debug)
      Serial.println("StartFwd");
    stepperq.moveTo(TargetPositionFeed);
    stepperq.start();
    Serial3.print("START\n"); // передаем контроллеру подмотки команду старт
    st = 22;                  // Движение вперед ручной режим
    break;

  // Движение вперед ручной режим ожидание паузы или конца плиты
  case 22:
    // условие если нет сигнала авария сервомотора и кнопка стоп не нажата
    if ((digitalRead(pin_alm_Motor) == 0) && (digitalRead(pin_but_Stop) == 0))
    {
      // условие если кнопка движение вперед нажата
      if (digitalRead(pin_but_Fwd) == 0)
      {
        UpdUsedLenght(); // обновление на экране длинны использованного материала
        // условие если текущая позиция равна позиции подачи плиты
        if (stepperq.currentPosition() == TargetPositionFeed)
        {
          if (debug)
            Serial.println("StartFeed");
          UsedLenght = stepperq.currentPosition() / PulsePerM;
          lcd.setCursor(4, 0);
          lcd.print(UsedLenght, 2);
          st = 24;                       // Завершение подачи плиты
          digitalWrite(pin_out_Feed, 1); // включение цилиндра подачи плиты
          Serial3.print("STOP\n");       // передаем контроллеру подмотки команду стоп
        }
      }
      // кнопка движение вперед не нажата
      else
      {
        stepperq.stop();
        Serial3.print("STOP\n"); // передаем контроллеру подмотки команду стоп
        if (debug)
          Serial.println("ManualPause");
        st = 23; // Пауза ручного режима
      }
    }
    // есть сигнал либо  кнопка стоп нажата либо сигнал авария сервомотора
    else
    {
      stepperq.stop();
      digitalWrite(pin_out_Feed, 0);  //отключение цилиндра подачи плиты
      digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
      st = 0;                         // Ручной режим ожидание кнопок
      Serial3.print("STOP\n");        // передаем контроллеру подмотки команду стоп
      if (debug)
        Serial.println("EndManual");
    }
    break;

  // Пауза ручного режима ожидание движения вперед или назад или переход в авто режим
  case 23:
    // условие если нет сигнала авария сервомотора и кнопка стоп не нажата
    if ((digitalRead(pin_alm_Motor) == 0) && (digitalRead(pin_but_Stop) == 0))
    {
      UpdUsedLenght(); // обновление на экране длинны использованного материала
      // условие если кнопка движение вперед нажата
      if (digitalRead(pin_but_Fwd) == 0)
      {
        st = 21; // Старт движения вперед ручной режим
      }
      // условие если кнопка движение назад нажата
      if (digitalRead(pin_but_Rev) == 0)
      {
        st = 30; // Старт движения назад ручной режим
      }
      // условие если кнопка старт нажата
      if (digitalRead(pin_but_Start) == 0)
      {
        st = 60;   // Переход в режим авто
        e = 10;    // Задание номера экрана для отображения
        Display(); //обновление дисплея
      }
    }
    // есть сигнал либо кнопка стоп нажата либо сигнал авария сервомотора
    else
    {
      stepperq.stop();
      digitalWrite(pin_out_Feed, 0);  //отключение цилиндра подачи плиты
      digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
      st = 0;                         // Ручной режим ожидание кнопок
      Serial3.print("STOP\n");        // передаем контроллеру подмотки команду стоп
      if (debug)
        Serial.println("EndManual");
    }
    break;
  // Завершение подачи плиты ручной режим обновление дисплея выход в опрос кнопок
  case 24:
    // условие если нет сигнала авария сервомотора и кнопка стоп не нажата
    if ((digitalRead(pin_alm_Motor) == 0) && (digitalRead(pin_but_Stop) == 0))
    {
      // условие если датчик подачи плиты сработал
      if (digitalRead(pin_SensorFeed) == 0)
      {
        digitalWrite(pin_out_Feed, 0); //отключение цилиндра подачи плиты
        st = 0;                        // Ручной режим ожидание кнопок
        e = 0;                         // Задание номера экрана для отображения
        Display();                     //обновление дисплея
        if (debug)
          Serial.println("EndFeed");
        if (debug)
          Serial.println("EndManual");
      }
    }
    // есть сигнал либо кнопка стоп нажата либо сигнал авария сервомотора
    else
    {
      stepperq.stop();
      digitalWrite(pin_out_Feed, 0);  //отключение цилиндра подачи плиты
      digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
      st = 0;                         // Ручной режим ожидание кнопок
      Serial3.print("STOP\n");        // передаем контроллеру подмотки команду стоп
      if (debug)
        Serial.println("EndManual");
    }
    break;

  // Старт движения назад ручной режим
  case 30:
    if (debug)
      Serial.println("StartRev");
    TargetPositionRev = stepperq.currentPosition() - ((stepperq.currentPosition() - StartPosition) % SetLenghtInPulse); // вычисляем позицию до которой можно двигаться обратно(начало текущей плиты)
    stepperq.moveTo(TargetPositionRev);
    stepperq.start();
    st = 31; // Движение назад ручной режим
    break;

  // Движение назад ручной режим ожидание паузы или конца движения
  case 31:
    // условие если нет сигнала авария сервомотора и кнопка стоп не нажата
    if ((digitalRead(pin_alm_Motor) == 0) && (digitalRead(pin_but_Stop) == 0))
    {
      // условие если кнопка движение назад нажата
      if (digitalRead(pin_but_Rev) == 0)
      {
        UpdUsedLenght(); // обновление на экране длинны использованного материала
        // условие если текущая позиция равна позиции до которой можно двигаться назад
        if (stepperq.currentPosition() == TargetPositionRev)
        {
          if (debug)
            Serial.println("EndMotionRev");
          UsedLenght = stepperq.currentPosition() / PulsePerM;
          lcd.setCursor(4, 0);
          lcd.print(UsedLenght, 2);
          st = 32; // Ожидание движения вперед ручной режим
        }
      }
      // кнопка движение назад не нажата
      else
      {
        stepperq.stop();
        Serial3.print("STOP\n"); // передаем контроллеру подмотки команду стоп
        if (debug)
          Serial.println("ManualPause");
        st = 23; // Пауза ручного режима
      }
    }
    // есть сигнал либо кнопка стоп нажата либо сигнал авария сервомотора
    else
    {
      stepperq.stop();
      digitalWrite(pin_out_Feed, 0);  //отключение цилиндра подачи плиты
      digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
      st = 0;                         // Ручной режим ожидание кнопок
      Serial3.print("STOP\n");        // передаем контроллеру подмотки команду стоп
      if (debug)
        Serial.println("EndManual");
    }
    break;

  // Ожидание движения вперед ручной режим
  case 32:
    // условие если нет сигнала авария сервомотора и кнопка стоп не нажата
    if ((digitalRead(pin_alm_Motor) == 0) && (digitalRead(pin_but_Stop) == 0))
    {
      // условие если кнопка движение вперед нажата
      if (digitalRead(pin_but_Fwd) == 0)
      {
        st = 21; // Старт движения вперед ручной режим
      }
    }
    // есть сигнал либо кнопка стоп нажата либо сигнал авария сервомотора
    else
    {
      stepperq.stop();
      digitalWrite(pin_out_Feed, 0);  //отключение цилиндра подачи плиты
      digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
      st = 0;                         // Ручной режим ожидание кнопок
      Serial3.print("STOP\n");        // передаем контроллеру подмотки команду стоп
      if (debug)
        Serial.println("EndManual");
    }
    break;

  // Подача плиты ручной режим
  case 40:
    lcd.setCursor(0, 2);
    lcd.print("       FEED          ");
    if (debug)
      Serial.println("ManualFeed");
    // ожидание пока не сработает датчик подачи плиты или кнопка стоп
    while ((digitalRead(pin_SensorFeed) == 1) && (digitalRead(pin_but_Stop) == 0))
      ;
    digitalWrite(pin_out_Feed, 0); //отключение цилиндра подачи плиты
    // ожидание пока кнопка ручная подача плиты не будет отпущена
    while (digitalRead(pin_but_Feed) == 0)
      delay(10);

    st = 0;    // Ручной режим ожидание кнопок
    Display(); //обновление дисплея
    break;

  // Включение гильотины ручной режим
  case 50:
    lcd.setCursor(0, 2);
    lcd.print("      KNIFE          ");
    if (debug)
      Serial.println("ManualKnife");
    // задержка пока не сработает датчик гильотины или кнопка стоп
    while ((digitalRead(pin_SensorKnife) == 1) && (digitalRead(pin_but_Stop) == 0))
      ;
    digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
    // ожидание пока кнопка ручное включение гильотины не будет отпущена
    while (digitalRead(pin_but_Knife) == 0)
      delay(10);

    st = 0;
    Display(); //обновление дисплея
    break;

  //Автоматический режим настройка движения
  case 60:
    if (debug)
      Serial.println("Auto");
    RemainQuantity = SetQuantity;                                        // обновление оставшегося количества изготавливаемых изделий
    TargetPositionKnife = stepperq.currentPosition() + SetLenghtInPulse; // вычисление позиции в которой необходимо включить гильотину
    stepperq.setMaxSpeed(work_speed_set);
    // уловие подача плиты без остановки
    if (SetTimeoutFeed == 0)
    {
      TargetPositionFeedAuto = TargetPositionFeed - SetLenghtFeedInPulse; // вычисление позиции в которой необходимо включить подачу плиты
      stepperq.moveTo(TargetPositionKnife);
    }
    // подача плиты с задержкой
    else
    {
      TargetPositionFeedAuto = TargetPositionFeed; // вычисление позиции в которой необходимо включить подачу плиты
      stepperq.moveTo(TargetPositionFeedAuto);
    }

    if (debug)
      Serial.print("Speed = ");
    if (debug)
      Serial.println(SetWorkSpeed, 0);
    Serial3.println(SetWorkSpeed, 0); // передаем контроллеру подмотки заданную скорость работы
    st = 61;                          //Автоматический режим старт движения
    break;

  //Автоматический режим старт движения
  case 61:
    if (debug)
      Serial.println("StartAuto");
    stepperq.start();
    Serial3.print("START\n"); // передаем контроллеру подмотки команду старт
    st = 62;
    break;

  //Автоматический режим движение проверка аварий автоподача плиты ожидание окончания движения или паузы
  case 62:
    // условие если нет сигнала авария серводвигателя
    if (digitalRead(pin_alm_Motor) == 0)
    {
      // условие если кнопка стоп не нажата
      if (digitalRead(pin_but_Stop) == 0)
      {
        WaitStopLainer(); // ожидание от контроллера подмотки сигнала стоп
        UpdUsedLenght();  // обновление на экране длинны использованного материала
        // условие если кнопка пауза нажата
        if (digitalRead(pin_but_Pause) == 0)
        {
          if (debug)
            Serial.println("AutoPause");
          st = 70;                        //Пауза авто режима
          e = 11;                         // Задание номера экрана для отображения
          Display();                      //обновление дисплея
          digitalWrite(pin_out_Feed, 0);  //отключение цилиндра подачи плиты
          digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
          stepperq.stop();
          Serial3.print("STOP\n"); // передаем контроллеру подмотки команду стоп
          stepperq.setMaxSpeed(man_speed_set);
          if (debug)
            Serial.print("Speed = ");
          if (debug)
            Serial.println(SetManSpeed, 0);
        }
        // условие если текущая позиция равна позиции подачи плиты
        if (stepperq.currentPosition() == TargetPositionFeedAuto)
        {
          if (debug)
            Serial.println("AutoFeed");
          st = 63;                                                    // Завершение подачи плиты авто режим
          TargetPositionFeed = TargetPositionFeed + SetLenghtInPulse; // вычисляем позицию следующей подачи плиты
          digitalWrite(pin_out_Feed, 1);                              // включение цилиндра подачи плиты
          Serial3.print("STOP\n");                                    // передаем контроллеру подмотки команду стоп
        }
        // условие если текущая позиция равна позиции включения гильотины
        if (stepperq.currentPosition() == TargetPositionKnife)
        {
          if (debug)
            Serial.println("AutoKnife");
          st = 64;                                                      // Завершение реза гильотины авто режим
          TargetPositionKnife = TargetPositionKnife + SetLenghtInPulse; // вычисляем следующую позицию включения гильотины
          digitalWrite(pin_out_Knife, 1);
          Serial3.print("STOP\n"); // передаем контроллеру подмотки команду стоп
        }
      }
      // кнопка стоп нажата
      else
      {
        stepperq.stop();
        digitalWrite(pin_out_Feed, 0);  //отключение цилиндра подачи плиты
        digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
        st = 0;                         // Ручной режим ожидание кнопок
        Serial3.print("STOP\n");        // передаем контроллеру подмотки команду стоп
        if (debug)
          Serial.println("EndAuto");
      }
    }
    // есть сигнал авария серводвигателя
    else
    {
      stepperq.stop();
      digitalWrite(pin_out_Feed, 0);  //отключение цилиндра подачи плиты
      digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
      st = 0;                         // Ручной режим ожидание кнопок
      Serial3.print("STOP\n");        // передаем контроллеру подмотки команду стоп
      if (debug)
        Serial.println("EndAuto");
    }
    break;

  // Завершение подачи плиты авто режим
  case 63:
    // условие если нет сигнала авария сервомотора и кнопка стоп не нажата
    if ((digitalRead(pin_alm_Motor) == 0) && (digitalRead(pin_but_Stop) == 0))
    {
      // условие если датчик подачи плиты сработал
      if (digitalRead(pin_SensorFeed) == 0)
      {
        stepperq.moveTo(TargetPositionKnife);
        digitalWrite(pin_out_Feed, 0); //отключение цилиндра подачи плиты
        delay(SetTimeoutFeed);         // Задержка на поднятие ножа (перед началом следующего движения)
        st = 61;                       //Автоматический режим старт движения
        if (debug)
          Serial.println("EndFeed");
      }
    }
    // есть сигнал либо кнопка стоп нажата либо сигнал авария сервомотора
    else
    {
      stepperq.stop();
      digitalWrite(pin_out_Feed, 0);  //отключение цилиндра подачи плиты
      digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
      st = 0;                         // Ручной режим ожидание кнопок
      Serial3.print("STOP\n");        // передаем контроллеру подмотки команду стоп
      if (debug)
        Serial.println("EndAuto");
    }
    break;
  // завершение удара ножа
  case 64:
    // условие если нет сигнала авария сервомотора и кнопка стоп не нажата
    if ((digitalRead(pin_alm_Motor) == 0) && (digitalRead(pin_but_Stop) == 0))
    {
      // условие если датчик ножа сработал
      if (digitalRead(pin_SensorKnife) == 0)
      {
        // условие подача плиты без остановки
        if (SetTimeoutFeed == 0)
        {
          TargetPositionFeedAuto = TargetPositionFeed - SetLenghtFeedInPulse; // вычисление позиции в которой нужно включить подачу плиты
          stepperq.moveTo(TargetPositionKnife);
        }
        // подача плиты с задержкой
        else
        {
          TargetPositionFeedAuto = TargetPositionFeed; // вычисление позиции в которой нужно включить подачу плиты
          stepperq.moveTo(TargetPositionFeedAuto);
        }
        digitalWrite(pin_out_Knife, 0);      // отключение цилиндра гильотины
        delay(SetTimeoutKnife);              // Задержка на поднятие ножа (перед началом следующего движения)
        MakeParts = MakeParts + 1;           // вычисление количества изготовленных изделий
        RemainQuantity = RemainQuantity - 1; // вычисление оставшегося количества изготавливаемых изделий
        lcd.setCursor(16, 0);
        lcd.print(MakeParts, 0);
        lcd.setCursor(16, 3);
        lcd.print(RemainQuantity, 0);
        lcd.print(" ");
        // условие оставшееся количество изготавливаемых изделий равно нулю
        if (RemainQuantity == 0)
        {
          if (debug)
            Serial.println("AutoPause");
          st = 70;                        //Пауза авто режима
          e = 11;                         // Задание номера экрана для отображения
          Display();                      //обновление дисплея
          RemainQuantity = SetQuantity;   // обновление оставшегося количества изготавливаемых изделий
          digitalWrite(pin_out_Feed, 0);  //отключение цилиндра подачи плиты
          digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
          stepperq.stop();
          Serial3.print("STOP\n"); // передаем контроллеру подмотки команду стоп
          stepperq.setMaxSpeed(man_speed_set);
          if (debug)
            Serial.print("Speed = ");
          if (debug)
            Serial.println(SetManSpeed, 0);
        }
        // оставшееся количество изготавливаемых изделий не равно нулю
        else
        {
          st = 61; //Автоматический режим старт движения
        }
      }
    }
    // есть сигнал либо кнопка стоп нажата либо сигнал авария сервомотора
    else
    {
      stepperq.stop();
      digitalWrite(pin_out_Feed, 0);  //отключение цилиндра подачи плиты
      digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
      st = 0;                         // Ручной режим ожидание кнопок
      Serial3.print("STOP\n");        // передаем контроллеру подмотки команду стоп
      if (debug)
        Serial.println("EndAuto");
    }
    break;

  // Пауза авто режима ожидание движения вперед или назад или переход в авто режим
  case 70:
    // условие если нет сигнала авария серводвигателя
    if (digitalRead(pin_alm_Motor) == 0)
    {
      // условие если кнопка стоп не нажата
      if (digitalRead(pin_but_Stop) == 0)
      {
        UpdUsedLenght(); // обновление на экране длинны использованного материала
        // условие если кнопка движение вперед нажата
        if (digitalRead(pin_but_Fwd) == 0)
        {
          st = 71; // Переход в режим движение вперед на паузе
        }
        // условие если кнопка движение назад нажата
        if (digitalRead(pin_but_Rev) == 0)
        {
          st = 72; // Переход в режим движение назад на паузе
        }
        // условие если кнопка старт нажата
        if (digitalRead(pin_but_Start) == 0)
        {
          st = 61;   //Автоматический режим старт движения
          e = 10;    // Задание номера экрана для отображения
          Display(); //обновление дисплея
          stepperq.setMaxSpeed(work_speed_set);
          if (debug)
            Serial.print("Speed = ");
          if (debug)
            Serial.println(SetWorkSpeed, 0);
          // условие подача плиты без остановки
          if (SetTimeoutFeed == 0)
          {
            TargetPositionFeedAuto = TargetPositionFeed - SetLenghtFeedInPulse; // вычисление позиции в которой нужно включить подачу плиты
            stepperq.moveTo(TargetPositionKnife);
          }
          // подача плиты с задержкой
          else
          {
            TargetPositionFeedAuto = TargetPositionFeed;
            // условие позиция включения гильотины больше позиции включения подачи
            if (TargetPositionKnife > TargetPositionFeed)
            {
              stepperq.moveTo(TargetPositionFeedAuto); // двигаться в позицию подачи плиты
            }
            // позиция включения гильотины меньше больше позиции включения подачи
            else
            {
              stepperq.moveTo(TargetPositionKnife); // двигаться в позицию включения гильотины
            }
          }
        }
      }
      // кнопка стоп нажата
      else
      {
        stepperq.stop();
        digitalWrite(pin_out_Feed, 0);  //отключение цилиндра подачи плиты
        digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
        st = 0;                         // Ручной режим ожидание кнопок
        Serial3.print("STOP\n");        // передаем контроллеру подмотки команду стоп
        if (debug)
          Serial.println("EndAuto");
      }
    }
    // есть сигнал авария серводвигателя
    else
    {
      stepperq.stop();
      digitalWrite(pin_out_Feed, 0);  //отключение цилиндра подачи плиты
      digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
      st = 0;                         // Ручной режим ожидание кнопок
      Serial3.print("STOP\n");        // передаем контроллеру подмотки команду стоп
      if (debug)
        Serial.println("EndAuto");
    }
    break;

  // Старт движения вперед на паузе
  case 71:
    if (debug)
      Serial.println("MoveFwd");
    TargetPositionFwd = TargetPositionFeed - (100 * 0.001 * 74900 * STEPPERREV * 0.0001); // вычисление позиции до которой можно двигаться вперед(за 100мм перед следующей подачей)
    stepperq.moveTo(TargetPositionFwd);
    stepperq.start();
    Serial3.print("START\n"); // передаем контроллеру подмотки команду старт
    st = 711;                 // Движение вперед на паузе
    break;

  // Движение вперед на паузе ожидание реза гильотины и конца движения
  case 711:
    // условие если нет сигнала авария серводвигателя
    if (digitalRead(pin_alm_Motor) == 0)
    {
      // условие если кнопка стоп не нажата
      if (digitalRead(pin_but_Stop) == 0)
      {
        // условие если кнопка движение вперед нажата
        if (digitalRead(pin_but_Fwd) == 0)
        {
          UpdUsedLenght(); // обновление на экране длинны использованного материала
          // условие текущая позиция равна конечной позиции движения вперед на паузе
          if (stepperq.currentPosition() == TargetPositionFwd)
          {
            if (debug)
              Serial.println("EndMoveFwd");
            st = 712; // Ожидание движения в обратную сторону
            lcd.setCursor(0, 2);
            lcd.print("    END MOVING    ");
            Serial3.print("STOP\n"); // передаем контроллеру подмотки команду стоп
          }
          // условие текущая позиция равна позиции включения гильотины
          if (stepperq.currentPosition() == TargetPositionKnife)
          {
            if (debug)
              Serial.println("AutoKnife");
            st = 713;                                                            // Завершение удара ножа на паузе
            TargetPositionKnife = stepperq.currentPosition() + SetLenghtInPulse; // вычисление позиции следующего включения гильотины
            digitalWrite(pin_out_Knife, 1);                                      // включение гильотины
            Serial3.print("STOP\n");                                             // передаем контроллеру подмотки команду стоп
          }
        }
        // кнопка движение вперед не нажата
        else
        {
          if (debug)
            Serial.println("AutoPause");
          st = 70; //Пауза авто режима
          stepperq.stop();
          Serial3.print("STOP\n"); // передаем контроллеру подмотки команду стоп
        }
      }
      // кнопка стоп нажата
      else
      {
        stepperq.stop();
        digitalWrite(pin_out_Feed, 0);  //отключение цилиндра подачи плиты
        digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
        st = 0;                         // Ручной режим ожидание кнопок
        Serial3.print("STOP\n");        // передаем контроллеру подмотки команду стоп
        if (debug)
          Serial.println("EndAuto");
      }
    }
    // есть сигнал авария серводвигателя
    else
    {
      stepperq.stop();
      digitalWrite(pin_out_Feed, 0);  //отключение цилиндра подачи плиты
      digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
      st = 0;                         // Ручной режим ожидание кнопок
      Serial3.print("STOP\n");        // передаем контроллеру подмотки команду стоп
      if (debug)
        Serial.println("EndAuto");
    }
    break;

  // Ожидание движения в обратную сторону
  case 712:
    // условие если нет сигнала авария серводвигателя
    if (digitalRead(pin_alm_Motor) == 0)
    {
      // условие если кнопка стоп не нажата
      if (digitalRead(pin_but_Stop) == 0)
      {
        // условие если кнопка движение назад нажата
        if (digitalRead(pin_but_Rev) == 0)
        {
          if (debug)
            Serial.println("AutoPause");
          st = 70;
          Display(); //обновление дисплея
        }
      }
      // кнопка стоп нажата
      else
      {
        stepperq.stop();
        digitalWrite(pin_out_Feed, 0);  //отключение цилиндра подачи плиты
        digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
        st = 0;                         // Ручной режим ожидание кнопок
        Serial3.print("STOP\n");        // передаем контроллеру подмотки команду стоп
        if (debug)
          Serial.println("EndAuto");
      }
    }
    // есть сигнал авария сервомотора
    else
    {
      stepperq.stop();
      digitalWrite(pin_out_Feed, 0);  //отключение цилиндра подачи плиты
      digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
      st = 0;                         // Ручной режим ожидание кнопок
      Serial3.print("STOP\n");        // передаем контроллеру подмотки команду стоп
      if (debug)
        Serial.println("EndAuto");
    }
    break;

  // Завершение удара ножа на паузе
  case 713:
    // условие если нет сигнала авария сервомотора и кнопка стоп не нажата
    if ((digitalRead(pin_alm_Motor) == 0) && (digitalRead(pin_but_Stop) == 0))
    {
      // условие если сработал датчик опускания гильотины
      if (digitalRead(pin_SensorKnife) == 0)
      {
        digitalWrite(pin_out_Knife, 0);      // отключение цилиндра гильотины
        st = 711;                            // Движение вперед на паузе ожидание реза гильотины и конца движения
        delay(SetTimeoutKnife);              // Задержка на поднятие ножа (перед началом следующего движения)
        MakeParts = MakeParts + 1;           // вычисление количества изготовленных изделий
        RemainQuantity = RemainQuantity - 1; // вычисление оставшегося количества изготавливаемых изделий
        lcd.setCursor(16, 0);
        lcd.print(MakeParts, 0);
        lcd.setCursor(16, 3);
        lcd.print(RemainQuantity, 0);
        // условие оставшееся количество изготавливаемых изделий равно нулю
        if (RemainQuantity == 0)
        {
          RemainQuantity = SetQuantity; // обновление оставшегося количества изготавливаемых изделий
        }
      }
    }
    // есть сигнал либо кнопка стоп нажата либо сигнал авария сервомотора
    else
    {
      stepperq.stop();
      digitalWrite(pin_out_Feed, 0);  //отключение цилиндра подачи плиты
      digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
      st = 0;                         // Ручной режим ожидание кнопок
      Serial3.print("STOP\n");        // передаем контроллеру подмотки команду стоп
      if (debug)
        Serial.println("EndAuto");
    }
    break;

  // Старт движения назад на паузе
  case 72:
    if (debug)
      Serial.println("MoveRev");
    TargetPositionRev = stepperq.currentPosition() - ((stepperq.currentPosition() - StartPosition) % SetLenghtInPulse); // вычисление позиции до которой можно двигаться назад(позиция подачи текущей плиты)
    stepperq.moveTo(TargetPositionRev);
    stepperq.start();
    st = 721;
    break;

  // Движение назад на паузе ожидание  конца движения
  case 721:
    // условие если нет сигнала авария серводвигателя
    if (digitalRead(pin_alm_Motor) == 0)
    {
      // условие если кнопка стоп не нажата
      if (digitalRead(pin_but_Stop) == 0)
      {
        // условие если кнопка движение назад нажата
        if (digitalRead(pin_but_Rev) == 0)
        {
          UpdUsedLenght(); // обновление на экране длинны использованного материала
          // условие текущая позиция равна позиции до которой можно двигаться назад
          if (stepperq.currentPosition() == TargetPositionRev)
          {
            if (debug)
              Serial.println("EndMoveRev");
            st = 722; // ожидание движения в обратную сторону
            lcd.setCursor(0, 2);
            lcd.print("    END MOVING    ");
          }
        }
        // кнопка движения назад не нажата
        else
        {
          if (debug)
            Serial.println("AutoPause");
          st = 70;
          stepperq.stop();
        }
      }
      // кнопка стоп нажата
      else
      {
        stepperq.stop();
        digitalWrite(pin_out_Feed, 0);  //отключение цилиндра подачи плиты
        digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
        st = 0;                         // Ручной режим ожидание кнопок
        Serial3.print("STOP\n");        // передаем контроллеру подмотки команду стоп
        if (debug)
          Serial.println("EndAuto");
      }
    }
    // есть сигнал аварии серводвигателя
    else
    {
      stepperq.stop();
      digitalWrite(pin_out_Feed, 0);  //отключение цилиндра подачи плиты
      digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
      st = 0;                         // Ручной режим ожидание кнопок
      Serial3.print("STOP\n");        // передаем контроллеру подмотки команду стоп
      if (debug)
        Serial.println("EndAuto");
    }
    break;

  // Ожидание движения в обратную сторону
  case 722:
    // условие если нет сигнала авария серводвигателя
    if (digitalRead(pin_alm_Motor) == 0)
    {
      // условие если кнопка стоп не нажата
      if (digitalRead(pin_but_Stop) == 0)
      {
        // условие если кнопка движение вперед нажата
        if (digitalRead(pin_but_Fwd) == 0)
        {
          if (debug)
            Serial.println("AutoPause");
          st = 70;
          Display(); //обновление дисплея
        }
      }
      // кнопка стоп нажата
      else
      {
        stepperq.stop();
        digitalWrite(pin_out_Feed, 0);  //отключение цилиндра подачи плиты
        digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
        st = 0;                         // Ручной режим ожидание кнопок
        Serial3.print("STOP\n");        // передаем контроллеру подмотки команду стоп
        if (debug)
          Serial.println("EndAuto");
      }
    }
    // есть сигнал авария серводвигателя
    else
    {
      stepperq.stop();
      digitalWrite(pin_out_Feed, 0);  //отключение цилиндра подачи плиты
      digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
      st = 0;                         // Ручной режим ожидание кнопок
      Serial3.print("STOP\n");        // передаем контроллеру подмотки команду стоп
      if (debug)
        Serial.println("EndAuto");
    }
    break;
  // Режим
  case 100:
    SettingParameters(); // Провверка кнопки энкодера если нажата переход в режим установки параметров
    break;
  }
}
// Чтение параметров из памяти EEPROM
void readEE()
{
  EEPROM.get(0, SetLenght);
  EEPROM.get(4, SetWorkSpeed);
  EEPROM.get(8, SetManSpeed);
  EEPROM.get(12, SetLenghtFeed);
  EEPROM.get(16, SetTimeoutFeed);
  EEPROM.get(20, SetTimeoutKnife);
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
// Ожидание команды стоп от контроллера подмотки лайнера
void WaitStopLainer()
{
  while (Serial3.available() > 0)
  {
    char recieved = Serial3.read();
    Cmd += recieved;
    if (recieved == '\n')
    {

      if (Cmd == "STOP\n")
      {
        stepperq.stop();
        stepperq.setMaxSpeed(man_speed_set);
        digitalWrite(pin_out_Feed, 0);  //отключение цилиндра подачи плиты
        digitalWrite(pin_out_Knife, 0); // отключение цилиндра гильотины
        Serial3.print("STOP\n");        // передаем контроллеру подмотки команду стоп
        st = 12;                        // переход в режим стоп авария подмотки лайнера
        alarm = 0;
      }
      Cmd = "";
    }
  }
}
// Обновление дисплея в соответствии с выбранным экраном для отображения
void Display()
{
  switch (e)
  {
  case 0:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Len:");
    lcd.print(UsedLenght, 2);
    lcd.setCursor(12, 0);
    lcd.print("Qty:");
    lcd.print(MakeParts, 0);
    lcd.setCursor(5, 1);
    lcd.print("Manual mode");
    lcd.setCursor(1, 2);
    lcd.print("SetSpeed:");
    lcd.print(SetManSpeed, 0);
    lcd.print(" M/min");
    lcd.setCursor(0, 3);
    lcd.print("sQty:");
    lcd.print(SetQuantity, 0);
    lcd.setCursor(11, 3);
    lcd.print("rQty:");
    lcd.print(RemainQuantity, 0);
    break;
  case 1:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Len:");
    lcd.print(UsedLenght, 2);
    lcd.setCursor(12, 0);
    lcd.print("Qty:");
    lcd.print(MakeParts, 0);
    lcd.setCursor(3, 1);
    lcd.print("Parameter mode");
    lcd.setCursor(0, 2);
    lcd.print("Lenght:");
    lcd.print(SetLenght, 0);
    lcd.print(" mm");
    lcd.setCursor(0, 3);
    lcd.print("sQty:");
    lcd.print(SetQuantity, 0);
    lcd.setCursor(11, 3);
    lcd.print("rQty:");
    lcd.print(RemainQuantity, 0);
    break;
  case 2:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Len:");
    lcd.print(UsedLenght, 2);
    lcd.setCursor(12, 0);
    lcd.print("Qty:");
    lcd.print(MakeParts, 0);
    lcd.setCursor(3, 1);
    lcd.print("Parameter mode");
    lcd.setCursor(0, 2);
    lcd.print("WorkSpeed:");
    lcd.print(SetWorkSpeed, 0);
    lcd.print(" M/min");
    lcd.setCursor(0, 3);
    lcd.print("sQty:");
    lcd.print(SetQuantity, 0);
    lcd.setCursor(11, 3);
    lcd.print("rQty:");
    lcd.print(RemainQuantity, 0);
    break;
  case 3:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Len:");
    lcd.print(UsedLenght, 2);
    lcd.setCursor(12, 0);
    lcd.print("Qty:");
    lcd.print(MakeParts, 0);
    lcd.setCursor(3, 1);
    lcd.print("Parameter mode");
    lcd.setCursor(0, 2);
    lcd.print("ManSpeed:");
    lcd.print(SetManSpeed, 0);
    lcd.print(" M/min");
    lcd.setCursor(0, 3);
    lcd.print("sQty:");
    lcd.print(SetQuantity, 0);
    lcd.setCursor(11, 3);
    lcd.print("rQty:");
    lcd.print(RemainQuantity, 0);
    break;
  case 4:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Len:");
    lcd.print(UsedLenght, 2);
    lcd.setCursor(12, 0);
    lcd.print("Qty:");
    lcd.print(MakeParts, 0);
    lcd.setCursor(3, 1);
    lcd.print("Parameter mode");
    lcd.setCursor(0, 2);
    lcd.print("LenghtFeed:");
    lcd.print(SetLenghtFeed, 0);
    lcd.print(" mm");
    lcd.setCursor(0, 3);
    lcd.print("sQty:");
    lcd.print(SetQuantity, 0);
    lcd.setCursor(11, 3);
    lcd.print("rQty:");
    lcd.print(RemainQuantity, 0);
    break;
  case 5:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Len:");
    lcd.print(UsedLenght, 2);
    lcd.setCursor(12, 0);
    lcd.print("Qty:");
    lcd.print(MakeParts, 0);
    lcd.setCursor(3, 1);
    lcd.print("Parameter mode");
    lcd.setCursor(0, 2);
    lcd.print("TimeoutFeed:");
    lcd.print(SetTimeoutFeed, 0);
    lcd.print(" ms");
    lcd.setCursor(0, 3);
    lcd.print("sQty:");
    lcd.print(SetQuantity, 0);
    lcd.setCursor(11, 3);
    lcd.print("rQty:");
    lcd.print(RemainQuantity, 0);
    break;
  case 6:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Len:");
    lcd.print(UsedLenght, 2);
    lcd.setCursor(12, 0);
    lcd.print("Qty:");
    lcd.print(MakeParts, 0);
    lcd.setCursor(3, 1);
    lcd.print("Parameter mode");
    lcd.setCursor(0, 2);
    lcd.print("TimeoutKnife:");
    lcd.print(SetTimeoutKnife, 0);
    lcd.print(" ms");
    lcd.setCursor(0, 3);
    lcd.print("sQty:");
    lcd.print(SetQuantity, 0);
    lcd.setCursor(11, 3);
    lcd.print("rQty:");
    lcd.print(RemainQuantity, 0);
    break;
  case 7:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Len:");
    lcd.print(UsedLenght, 2);
    lcd.setCursor(12, 0);
    lcd.print("Qty:");
    lcd.print(MakeParts, 0);
    lcd.setCursor(3, 1);
    lcd.print("Parameter mode");
    lcd.setCursor(0, 2);
    lcd.print("Quantity:");
    lcd.print(SetQuantity, 0);
    lcd.print(" ");
    lcd.setCursor(0, 3);
    lcd.print("sQty:");
    lcd.print(SetQuantity, 0);
    lcd.setCursor(11, 3);
    lcd.print("rQty:");
    lcd.print(RemainQuantity, 0);
    break;
  case 8:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Len:");
    lcd.print(UsedLenght, 2);
    lcd.setCursor(12, 0);
    lcd.print("Qty:");
    lcd.print(MakeParts, 0);
    lcd.setCursor(3, 1);
    lcd.print("Parameter mode");
    lcd.setCursor(0, 2);
    lcd.print("Zero Lenght:");
    lcd.print(UsedLenght, 2);
    lcd.print(" ");
    lcd.setCursor(0, 3);
    lcd.print("sQty:");
    lcd.print(SetQuantity, 0);
    lcd.setCursor(11, 3);
    lcd.print("rQty:");
    lcd.print(RemainQuantity, 0);
    break;
  case 9:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Len:");
    lcd.print(UsedLenght, 2);
    lcd.setCursor(12, 0);
    lcd.print("Qty:");
    lcd.print(MakeParts, 0);
    lcd.setCursor(3, 1);
    lcd.print("Parameter mode");
    lcd.setCursor(0, 2);
    lcd.print("Zero Parts:");
    lcd.print(MakeParts, 0);
    lcd.print(" ");
    lcd.setCursor(0, 3);
    lcd.print("sQty:");
    lcd.print(SetQuantity, 0);
    lcd.setCursor(11, 3);
    lcd.print("rQty:");
    lcd.print(RemainQuantity, 0);
    break;

  case 10:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Len:");
    lcd.print(UsedLenght, 2);
    lcd.setCursor(12, 0);
    lcd.print("Qty:");
    lcd.print(MakeParts, 0);
    lcd.setCursor(2, 1);
    lcd.print("Automatic mode");
    lcd.setCursor(1, 2);
    lcd.print("SetSpeed:");
    lcd.print(SetWorkSpeed, 0);
    lcd.print(" M/min");
    lcd.setCursor(0, 3);
    lcd.print("sQty:");
    lcd.print(SetQuantity, 0);
    lcd.setCursor(11, 3);
    lcd.print("rQty:");
    lcd.print(RemainQuantity, 0);
    break;
  case 11:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Len:");
    lcd.print(UsedLenght, 2);
    lcd.setCursor(12, 0);
    lcd.print("Qty:");
    lcd.print(MakeParts, 0);
    lcd.setCursor(5, 1);
    lcd.print("Pause mode");
    lcd.setCursor(1, 2);
    lcd.print("SetSpeed:");
    lcd.print(SetManSpeed, 0);
    lcd.print(" M/min");
    lcd.setCursor(0, 3);
    lcd.print("sQty:");
    lcd.print(SetQuantity, 0);
    lcd.setCursor(11, 3);
    lcd.print("rQty:");
    lcd.print(RemainQuantity, 0);
    break;

  case 15:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Len:");
    lcd.print(UsedLenght, 2);
    lcd.setCursor(12, 0);
    lcd.print("Qty:");
    lcd.print(MakeParts, 0);
    lcd.setCursor(5, 1);
    lcd.print("Stop mode");
    lcd.setCursor(4, 2);
    lcd.print("ALARM SERVO");
    lcd.setCursor(0, 3);
    lcd.print("sQty:");
    lcd.print(SetQuantity, 0);
    lcd.setCursor(11, 3);
    lcd.print("rQty:");
    lcd.print(RemainQuantity, 0);
    break;
  case 16:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Len:");
    lcd.print(UsedLenght, 2);
    lcd.setCursor(12, 0);
    lcd.print("Qty:");
    lcd.print(MakeParts, 0);
    lcd.setCursor(5, 1);
    lcd.print("Stop mode");
    lcd.setCursor(3, 2);
    lcd.print("ALARM BUTTON");
    lcd.setCursor(0, 3);
    lcd.print("sQty:");
    lcd.print(SetQuantity, 0);
    lcd.setCursor(11, 3);
    lcd.print("rQty:");
    lcd.print(RemainQuantity, 0);
    break;
  case 17:
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Len:");
    lcd.print(UsedLenght, 2);
    lcd.setCursor(12, 0);
    lcd.print("Qty:");
    lcd.print(MakeParts, 0);
    lcd.setCursor(5, 1);
    lcd.print("Stop mode");
    lcd.setCursor(4, 2);
    lcd.print("ALARM LINER");
    lcd.setCursor(0, 3);
    lcd.print("sQty:");
    lcd.print(SetQuantity, 0);
    lcd.setCursor(11, 3);
    lcd.print("rQty:");
    lcd.print(RemainQuantity, 0);
    break;
  }
}
// Ожидание нажатия кнопки в ручном режиме
void ManualWaitButton()
{
  // есть сигнал авария сервомотор
  if (digitalRead(pin_alm_Motor) == 1)
  {
    digitalWrite(pin_son_Motor, 0); // выключение S-on драйвера
    if (debug)
      Serial.println("alm_servo");
    st = 10; // переход в режим стоп авария сервомотора
    alarm = 0;
  }
  // кнопка стоп нажата
  else if (digitalRead(pin_but_Stop) == 1)
  {
    digitalWrite(pin_son_Motor, 0); // включение S-on драйвера
    if (debug)
      Serial.println("alm_buton");
    st = 11; // переход в режим стоп кнопка нажата
    alarm = 0;
  }
  // кнопка движение вперед нажата
  else if (digitalRead(pin_but_Fwd) == 0)
  {
    st = 20; // переход в режим ручной протяжки
  }
  // кнопка подача плиты нажата
  else if (digitalRead(pin_but_Feed) == 0)
  {
    digitalWrite(pin_out_Feed, 1); // включение цилиндра подачи плиты
    if (debug)
      Serial.println("Feed");
    st = 40; // переход в режим рез ножом
  }
  // кнопка включение гильотины нажата
  else if (digitalRead(pin_but_Knife) == 0)
  {
    digitalWrite(pin_out_Knife, 1); // включение цилиндра гильотины
    if (debug)
      Serial.println("Knife");
    st = 50; // переход в режим рез ножом
  }
  // ни одна из кнопок не нажата и экран не обновлен после перехода в этот режим
  else if (alarm == 1)
  {
    digitalWrite(pin_son_Motor, 1); // включение S-on драйвера
    if (debug)
      Serial.println("Wait Button");
    e = 0;     // Задание номера экрана для отображения
    Display(); //обновление дисплея
    alarm = 0;
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
      if (e > 9)
        e = 9; // Задание номера экрана для отображения
      else if (e < 1)
        e = 1;   // Задание номера экрана для отображения
      Display(); //обновление дисплея
    }
    break;
    // установка длины плиты
  case 2:
    lcd.setCursor(0, 1);
    lcd.print("Set");
    switch (BUT_ENC.Loop())
    {
    // выход из режима установки без сохранения
    case SB_CLICK:
      SetLenghtInPulse = ((SetLenght / 1000) * 74900 * STEPPERREV) / 10000;
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
    // выход из режима установки с сохранением в память
    case SB_LONG_CLICK:
      EEPROM.put(0, SetLenght);
      SetLenghtInPulse = ((SetLenght / 1000) * 74900 * STEPPERREV) / 10000;
      SetPar_st = 1;
      encCnt = 0;
      st = 0;    // Ручной режим ожидание кнопок
      Display(); //обновление дисплея
      break;
    }
    // вращение изменяет значение длинны
    if (encCnt != 0)
    {
      SetLenght += encCnt * 1;
      encCnt = 0;
      if (SetLenght > 3000)
        SetLenght = 3000;
      else if (SetLenght < 10)
        SetLenght = 10;
      Display(); //обновление дисплея
      lcd.setCursor(0, 1);
      lcd.print("Set");
    }
    break;

  case 3:
    lcd.setCursor(0, 1);
    lcd.print("Set");
    switch (BUT_ENC.Loop())
    {
      // выход из режима установки без сохранения
    case SB_CLICK:
      work_speed_set = (SetWorkSpeed * (74900 * STEPPERREV) / 10000) / 60;
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
      // выход из режима установки с сохранением в память
    case SB_LONG_CLICK:
      EEPROM.put(4, SetWorkSpeed);
      work_speed_set = (SetWorkSpeed * (74900 * STEPPERREV) / 10000) / 60;
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
    }
    // вращение изменяет значение рабочей скорости
    if (encCnt != 0)
    {
      SetWorkSpeed += encCnt * 1;
      encCnt = 0;
      if (SetWorkSpeed > 40)
        SetWorkSpeed = 40;
      else if (SetWorkSpeed < 1)
        SetWorkSpeed = 1;
      Display(); //обновление дисплея
      lcd.setCursor(0, 1);
      lcd.print("Set");
    }
    break;

  case 4:
    lcd.setCursor(0, 1);
    lcd.print("Set");
    switch (BUT_ENC.Loop())
    {
      // выход из режима установки без сохранения
    case SB_CLICK:
      man_speed_set = (SetManSpeed * (74900 * STEPPERREV) / 10000) / 60;
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
      // выход из режима установки с сохранением в память
    case SB_LONG_CLICK:
      EEPROM.put(8, SetManSpeed);
      man_speed_set = (SetManSpeed * (74900 * STEPPERREV) / 10000) / 60;
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
    }
    // вращение изменяет значение скорости ручного
    if (encCnt != 0)
    {
      SetManSpeed += encCnt * 1;
      encCnt = 0;
      if (SetManSpeed > 30)
        SetManSpeed = 30;
      else if (SetManSpeed < 1)
        SetManSpeed = 1;
      Display(); //обновление дисплея
      lcd.setCursor(0, 1);
      lcd.print("Set");
    }
    break;

  case 5:
    lcd.setCursor(0, 1);
    lcd.print("Set");
    switch (BUT_ENC.Loop())
    {
      // выход из режима установки без сохранения
    case SB_CLICK:
      SetLenghtFeedInPulse = ((SetLenghtFeed / 1000) * 74900 * STEPPERREV) / 10000;
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
      // выход из режима установки с сохранением в память
    case SB_LONG_CLICK:
      EEPROM.put(12, SetLenghtFeed);
      SetLenghtFeedInPulse = ((SetLenghtFeed / 1000) * 74900 * STEPPERREV) / 10000;
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
    }
    // вращение изменяет значение длины за которую выполняется подача
    if (encCnt != 0)
    {
      SetLenghtFeed += encCnt * 1;
      encCnt = 0;
      if (SetLenghtFeed > 500)
        SetLenghtFeed = 500;
      else if (SetLenghtFeed < 0)
        SetLenghtFeed = 0;
      Display(); //обновление дисплея
      lcd.setCursor(0, 1);
      lcd.print("Set");
    }
    break;

  case 6:
    lcd.setCursor(0, 1);
    lcd.print("Set");
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
      SetTimeoutFeed += encCnt * 10;
      encCnt = 0;
      if (SetTimeoutFeed > 1000)
        SetTimeoutFeed = 1000;
      else if (SetTimeoutFeed < 0)
        SetTimeoutFeed = 0;
      Display(); //обновление дисплея
      lcd.setCursor(0, 1);
      lcd.print("Set");
    }
    break;

  case 7:
    lcd.setCursor(0, 1);
    lcd.print("Set");
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
      EEPROM.put(20, SetTimeoutKnife);
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
    }
    // вращение изменяет значение таймаута
    if (encCnt != 0)
    {
      SetTimeoutKnife += encCnt * 10;
      encCnt = 0;
      if (SetTimeoutKnife > 1000)
        SetTimeoutKnife = 1000;
      else if (SetTimeoutKnife < 0)
        SetTimeoutKnife = 0;
      Display(); //обновление дисплея
      lcd.setCursor(0, 1);
      lcd.print("Set");
    }
    break;

  case 8:
    lcd.setCursor(0, 1);
    lcd.print("Set");
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
      if (SetQuantity > 200)
        SetQuantity = 200;
      else if (SetQuantity < 0)
        SetQuantity = 0;
      Display(); //обновление дисплея
      lcd.setCursor(0, 1);
      lcd.print("Set");
    }
    break;

  case 9:
    lcd.setCursor(0, 1);
    lcd.print("Set");
    switch (BUT_ENC.Loop())
    {
      // выход из режима установки без сохранения
    case SB_CLICK:
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
      // обнуление длины использованного материала
    case SB_LONG_CLICK:
      UsedLenght = 0;
      stepperq.setCurrentPosition(0);
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
    }
    break;

  case 10:
    lcd.setCursor(0, 1);
    lcd.print("Set");
    switch (BUT_ENC.Loop())
    {
      // выход из режима установки без сохранения
    case SB_CLICK:
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
      // обнуление количества изготовленных изделий
    case SB_LONG_CLICK:
      MakeParts = 0;
      SetPar_st = 1;
      encCnt = 0;
      Display(); //обновление дисплея
      break;
    }
    break;
  }
}
// обновление на экране длинны использованного материала
void UpdUsedLenght()
{
  dt_disp = millis() - tm_disp;
  if (dt_disp >= 100)
  {
    UsedLenght = stepperq.currentPosition() / PulsePerM;
    lcd.setCursor(4, 0);
    lcd.print(UsedLenght, 2);
    tm_disp = millis();
  }
}

/*


*/
