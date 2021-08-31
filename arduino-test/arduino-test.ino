// Тестовая прошивка Arduino Uno мигание светодиодом на 13 ноге плюс вывод в Serial


// the setup routine runs once when you press reset:
int debug = 0;

void setup()
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);

  Serial.println("Hello world");
  pinMode(13, OUTPUT);
}
// the loop routine runs over and over again forever:
void loop()
{
  if (debug)
    Serial.println("Led on");

  digitalWrite(13, 1);

  delay(1000);

  if (debug)
    Serial.println("Led off");

  digitalWrite(13, 0);

  delay(1000);
}
