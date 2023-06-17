const int ledPin = LED_BUILTIN;            // Digital BuiltIn status LED pin
const int throttle_Pin = A0;               // Analog throttle input pin
const int brake_Pin = 2;                   // Digital brake input pin
const int reverse_Pin = 3;                 // Digital reverse input pin
const int pwmPin = 9;                      // PWM output pin
const int dirPin = 8;                      // Direction output pin
const int temperatureSensor1_Pin = A1;     // Analog input pin for temperature sensor 1
const int temperatureSensor2_Pin = A2;     // Analog input pin for temperature sensor 2
const int voltageSensor1_Pin = A3;         // Analog input pin for voltage sensor 1
const int voltageSensor2_Pin = A4;         // Analog input pin for voltage sensor 2
const int currentSensor_Pin = A5;          // Analog input pin for current sensor
const int motorSpeedSensor_Pin = 6;        // Digital input pin for motor speed sensor
const int wheelSpeedSensor_Pin = 7;        // Digital input pin for wheel speed sensor

unsigned long previousMillis1 = 0;         // Previous time for task1
unsigned long previousMillis2 = 0;         // Previous time for task2
unsigned long previousMillis3 = 0;         // Previous time for task3

const unsigned long interval1 = 1;        // Task1 interval in milliseconds (1000 Hz)
const unsigned long interval2 = 10;       // Task2 interval in milliseconds (100 Hz)
const unsigned long interval3 = 1000;     // Task3 interval in milliseconds (1 Hz)

unsigned long taskTime1 = 0;               // Execution time for task1
unsigned long taskTime2 = 0;               // Execution time for task2
unsigned long taskTime3 = 0;               // Execution time for task3

unsigned long overrun1 = 0;                // Number of task1 overruns
unsigned long overrun2 = 0;                // Number of task2 overruns
unsigned long overrun3 = 0;                // Number of task3 overruns

unsigned long overrunsPerSecond = 0;       // Total overruns per second
unsigned long previousSecond = 0;          // Previous second count

int ledState = LOW;                        // Led status 
int motor_out = 0;                         // Motor PWM duty cycle             

float filterFactor = 0.2;                  // Filter factor (0.0 - 1.0)

bool brake_signalState = false;            // State of brake signal
bool reverse_latchState = false;           // State of the reverse signal

void setup()
{
  Serial.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(dirPin, INPUT);
  pinMode(throttle_Pin, INPUT);
  pinMode(brake_Pin, INPUT);
  pinMode(reverse_Pin, INPUT);
  pinMode(temperatureSensor1_Pin, INPUT);
  pinMode(temperatureSensor2_Pin, INPUT);
  pinMode(voltageSensor1_Pin, INPUT);
  pinMode(voltageSensor2_Pin, INPUT);
  pinMode(currentSensor_Pin, INPUT);
  pinMode(motorSpeedSensor_Pin, INPUT);
  pinMode(wheelSpeedSensor_Pin, INPUT);
  
}

void task1()
{
  int v_throttle = analogRead(throttle_Pin);      // Read throttle input
  static float v_throttle_filtered = v_throttle;  // Initialize the filtered value
  v_throttle_filtered = lowPassFilter(v_throttle, v_throttle_filtered);
    
  // Motor control
  motor_out = v_throttle_filtered/255;
  analogWrite(pwmPin, motor_out);  // Set PWM duty cycle to maximum (255)
}

void task2()
{
  // Read temperature sensor measurements
  float temperature1 = analogRead(temperatureSensor1_Pin);
  float temperature2 = analogRead(temperatureSensor2_Pin);

  // Read voltage sensor measurements
  float voltage1 = analogRead(voltageSensor1_Pin);
  float voltage2 = analogRead(voltageSensor2_Pin);

  // Read current sensor measurement
  float current = analogRead(currentSensor_Pin);

  // Read wheel speed sensor measurements
  float motorSpeed = analogRead(motorSpeedSensor_Pin);
  float wheelSpeed = analogRead(wheelSpeedSensor_Pin);

  float power = (voltage1 + voltage2) * current;

}

void task3()
{
  if (ledState == LOW) {
    ledState = HIGH;
  }
  else {
    ledState = LOW;
  }
  digitalWrite(ledPin, ledState);
}

float lowPassFilter(float input, float outputPrev)
{
  float output = (input * filterFactor) + (outputPrev * (1 - filterFactor));
  return output;
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
  }
}
