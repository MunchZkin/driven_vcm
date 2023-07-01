const int ledPin = LED_BUILTIN;             // Digital BuiltIn status LED pin
const int throttle_Pin = A0;                // Analog throttle input pin
const int brake_Pin = 2;                    // Digital brake input pin
const int reverse_Pin = 3;                  // Digital reverse input pin
const int pwmPin = 9;                       // PWM output pin
const int dirPin = 8;                       // Direction output pin
const int powerButtonPin = 4;               // Digital input pin for turn-on button
const int hvRelayPin = 5;                   // Digital output pin for HV relay
const int temperatureSensor1_Pin = A1;      // Analog input pin for temperature sensor 1
const int temperatureSensor2_Pin = A2;      // Analog input pin for temperature sensor 2
const int voltageSensor1_Pin = A3;          // Analog input pin for voltage sensor 1
const int voltageSensor2_Pin = A4;          // Analog input pin for voltage sensor 2
const int currentSensor_Pin = A5;           // Analog input pin for current sensor
const int motorSpeedSensor_Pin = 6;         // Digital input pin for motor speed sensor
const int wheelSpeedSensor_Pin = 7;         // Digital input pin for wheel speed sensor

unsigned long previousMillis1 = 0;          // Previous time for task1
unsigned long previousMillis2 = 0;          // Previous time for task2
unsigned long previousMillis3 = 0;          // Previous time for task3

const unsigned long interval1 = 1;          // Task1 interval in milliseconds (1000 Hz)
const unsigned long interval2 = 10;         // Task2 interval in milliseconds (100 Hz)
const unsigned long interval3 = 1000;       // Task3 interval in milliseconds (1 Hz)

unsigned long taskTime1 = 0;                // Execution time for task1
unsigned long taskTime2 = 0;                // Execution time for task2
unsigned long taskTime3 = 0;                // Execution time for task3

unsigned long overrun1 = 0;                 // Number of task1 overruns
unsigned long overrun2 = 0;                 // Number of task2 overruns
unsigned long overrun3 = 0;                 // Number of task3 overruns

unsigned long overrunsPerSecond = 0;        // Total overruns per second
unsigned long previousSecond = 0;           // Previous second count

unsigned long debounceDelay = 1000;         // Debounce period in milliseconds
unsigned long reverse_lastDebounceTime = 0; // Time of the last button state change
unsigned long turnOnDuration = 3000;        // Turn-on duration in milliseconds
unsigned long turnOnStartTime = 0;          // Start time for turn-on duration

int ledState = LOW;                         // Led status 
int motor_out = 0;                          // Motor PWM duty cycle             

float filterFactor = 0.8;                   // Filter factor (0.0 - 1.0)

bool brakePressed = false;                  // State of brake signal
bool turnOnButtonPressed = false;           // Flag to indicate turn-on button press
bool hvRelayState = false;                  // State of HV relay
bool reverse_State = false;                 // State of brake signal
bool reverse_buttonState = false;           // Initial state of the button
bool reverse_lastButtonState = false;       // Previous state of the button

float throttle_input = 0.0;                 // PID input based on throttle
float power_input = 0.0;                    // PID input based on power consumption
float thermal_input = 0.0;                  // PID input based on motor thermals
float motor_output = 0.0;                   // PID output (0-255)
float power_threshold = 100;                // Watt Soft limit for power
float temperature_threshold = 80;           // °C   Hard limit for temperature

// Floats for resistor values in divider (in ohms)
float ref_voltage = 5.0;
float R1 = 30000.0;
float R2 = 7500.0;

float error = 0.0;                          // internal variable

// structure for PID 
struct PIDController {
  float previousValue;
  float integral;
  float previousError;
  float Kp;
  float Ki;
  float Kd;
};

PIDController throttle_pid;
PIDController temperature_pid;
PIDController power_pid;

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
  pinMode(powerButtonPin, INPUT);
  pinMode(hvRelayPin, OUTPUT);
  digitalWrite(hvRelayPin, LOW);  // Initialize HV relay as OFF
  initializePID(throttle_pid, 1.0, 0.2, 0.5);
  initializePID(power_pid, 0.5, 0.1, 0.3);
  initializePID(temperature_pid, 0.8, 0.3, 0.6);
  
}

void task1()
{
  int v_throttle = analogRead(throttle_Pin);      // Read throttle input
  if (v_throttle <= 50) {
    v_throttle = 0;
  }
  static float v_throttle_filtered = v_throttle;  // Initialize the filtered value
  v_throttle_filtered = lowPassFilter(v_throttle, v_throttle_filtered);
  throttle_input = calculatePID(throttle_pid, (v_throttle_filtered/1024), 0.0);
      
  // Motor control
  // motor_output  = (throttle_input * power_input * thermal_input)*255;
  motor_output = (v_throttle_filtered/1024)*255;
  analogWrite(pwmPin, motor_output);  // Set PWM duty cycle to maximum (255)
}

void task2()
{
  // Reverse Condition
  int reverse_reading = digitalRead(reverse_Pin);
  // Check if button state has changed
  if (reverse_reading != reverse_lastButtonState) {
    reverse_lastDebounceTime = millis();
  }
  // Check if debounce period has passed
  if ((millis() - reverse_lastDebounceTime) > debounceDelay) {
    // Update button state if debounce period has passed
    if (reverse_reading != reverse_buttonState) {
      reverse_buttonState = reverse_reading;

      // Toggle reverse switch state when button is pressed
      if (reverse_buttonState == HIGH) {
        reverse_State = !reverse_State;
        digitalWrite(dirPin, reverse_State);
      }
    }
  }
  reverse_lastButtonState = reverse_reading;

  // Read temperature sensor measurements
  float temperature1 = analogRead(temperatureSensor1_Pin);
  float temperature2 = analogRead(temperatureSensor2_Pin);
  throttle_input = calculatePID(temperature_pid, temperature2, temperature_threshold);

  // Read voltage sensor measurements
  float voltage1 = analogRead(voltageSensor1_Pin);
  voltage1 = ((voltage1 * ref_voltage) / 1024.0)/ (R2/(R1+R2));
  float voltage2 = analogRead(voltageSensor2_Pin);
  voltage2 = ((voltage2 * ref_voltage) / 1024.0)/ (R2/(R1+R2));
  
  // Read current sensor measurement  
  float adc = analogRead(currentSensor_Pin);     //Read current sensor values
  float currentVoltage = adc * 5 / 1023.0;
  float current = (currentVoltage - 2.5) / 0.185;
  
  // Read wheel speed sensor measurements
  float motorSpeed = analogRead(motorSpeedSensor_Pin);
  float wheelSpeed = analogRead(wheelSpeedSensor_Pin);
  
  voltage2 = 0.0;
  float power = (voltage1 + voltage2) * current;
}

void task3()
{
  // Arming HV system
  bool brakeState = digitalRead(brake_Pin);
  // Check if the brake is pressed and latch the brakePressed flag
  if (brakeState == LOW && !brakePressed) {
    brakePressed = true;
  }

  // Read the state of the turn-on button pin
  bool turnOnButtonState = digitalRead(powerButtonPin);

  // Check if the turn-on button is pressed and start the turn-on timer
  if (turnOnButtonState == LOW && !turnOnButtonPressed) {
    turnOnButtonPressed = true;
    turnOnStartTime = millis();
  }

  // Check if the brake is released and reset the brakePressed flag
  if (brakeState == HIGH && brakePressed) {
    brakePressed = false;
  }

  // Check if the turn-on button is released and reset the turnOnButtonPressed flag
  if (turnOnButtonState == HIGH && turnOnButtonPressed) {
    turnOnButtonPressed = false;
  }

  // Check if the turn-on duration has elapsed and turn on the HV relay
  if (turnOnButtonPressed && (millis() - turnOnStartTime >= turnOnDuration)) {
    hvRelayState = true;
    digitalWrite(hvRelayPin, HIGH);
  }

  // Check if the HV relay is ON and the brake is pressed to turn it off
  if (hvRelayState && brakePressed) {
    hvRelayState = false;
    digitalWrite(hvRelayPin, LOW);
  }
  
  // status LED
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

void initializePID(PIDController& pid, float Kp, float Ki, float Kd) {
  pid.previousValue = 0.0;
  pid.integral = 0.0;
  pid.previousError = 0.0;
  pid.Kp = Kp;
  pid.Ki = Ki;
  pid.Kd = Kd;
}

float calculatePID(PIDController& pid, float currentValue, float setpoint) {
  // Error calculation
  if (setpoint == 0.0){
    error = currentValue - pid.previousValue;
  } else {
    error = setpoint - currentValue;
  }
  
  // Proportional term
  float P = pid.Kp * error;

  // Integral term (approximation using accumulated error)
  pid.integral += pid.Ki * error;

  // Derivative term (approximation using difference in error)
  float derivative = pid.Kd * (error - pid.previousError);
  pid.previousError = error;

  // Calculate the PID output
  float output = P + pid.integral + derivative;

  // Update previous value for the next iteration
  pid.previousValue = currentValue;

  if (output >= 1){
    output = 1;
  } else if(output <= 0){
    output = 0;
  }

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
