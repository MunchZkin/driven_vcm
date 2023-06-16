unsigned long previousMillis1 = 0;  // Previous time for task1
unsigned long previousMillis2 = 0;  // Previous time for task2
unsigned long previousMillis3 = 0;  // Previous time for task3

const unsigned long interval1 = 1;  // Task1 interval in milliseconds (1 kHz)
const unsigned long interval2 = 10; // Task2 interval in milliseconds (100 Hz)
const unsigned long interval3 = 1000; // Task3 interval in milliseconds (1 Hz)

unsigned long taskTime1 = 0;  // Execution time for task1
unsigned long taskTime2 = 0;  // Execution time for task2
unsigned long taskTime3 = 0;  // Execution time for task3

unsigned long overrun1 = 0;  // Number of task1 overruns
unsigned long overrun2 = 0;  // Number of task2 overruns
unsigned long overrun3 = 0;  // Number of task3 overruns

unsigned long overrunsPerSecond = 0;  // Total overruns per second
unsigned long previousSecond = 0;  // Previous second count

const int ledPin =  LED_BUILTIN;// the number of the LED pin
int ledState = LOW; 

void task1()
{
//  1khz task
}

void task2()
{
//  100hz task
}

void task3()
{
  statusLED();
}

void statusLED()
{
  if (ledState == LOW) {
    ledState = HIGH;
  }
  else {
    ledState = LOW;
  }
  digitalWrite(ledPin, ledState);
}

void setup()
{
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  
}

void loop()
{
  unsigned long currentMillis = millis();

  // Task1 (1 kHz)
  if (currentMillis - previousMillis1 >= interval1)
  {
    previousMillis1 = currentMillis;
    unsigned long startTask1 = millis();
    task1();
    taskTime1 = millis() - startTask1;

    if (taskTime1 > interval1)
    {
      overrun1++;
    }
  }

  // Task2 (100 Hz)
  if (currentMillis - previousMillis2 >= interval2)
  {
    previousMillis2 = currentMillis;
    unsigned long startTask2 = millis();
    task2();
    taskTime2 = millis() - startTask2;

    if (taskTime2 > interval2)
    {
      overrun2++;
    }
  }

  // Task3 (1 Hz)
  if (currentMillis - previousMillis3 >= interval3)
  {
    previousMillis3 = currentMillis;
    unsigned long startTask3 = millis();
    task3();
    taskTime3 = millis() - startTask3;

    if (taskTime3 > interval3)
    {
      overrun3++;
    }
  }

  // Calculate overruns per second
  unsigned long currentSecond = millis() / 1000;
  if (currentSecond != previousSecond)
  {
    overrunsPerSecond = overrun1 + overrun2 + overrun3;
    overrun1 = 0;
    overrun2 = 0;
    overrun3 = 0;
    previousSecond = currentSecond;
    Serial.print(taskTime1);
    Serial.print(", ");
    Serial.print(taskTime2);
    Serial.print(", ");
    Serial.println(taskTime3);
    
  }
}
