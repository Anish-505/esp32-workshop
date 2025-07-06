# esp32-workshop#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);  // I2C address 0x27, 16 column, 2 row LCD

#define trig_A 13
#define echo_A 12
#define trig_B 14
#define echo_B 27

// Shared variables
volatile bool detectedA = false;
volatile bool detectedB = false;
volatile unsigned long t1 = 0, t2 = 0;

float sensor_distance = 0.20; // meters
float speed = 0.0;
float time_diff = 0.0;

// Function to get distance from an ultrasonic sensor
float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000); // 20 ms timeout
  float distance = (duration * 0.0343) / 2;
  return distance;
}

// Task for Sensor A
void sensorATask(void* pvParameters) {
  while (true) {
    if (!detectedA) {
      float dA = getDistance(trig_A, echo_A);
      if (dA < 10.0) {
        t1 = micros();
        detectedA = true;
        Serial.println("Sensor A Triggered");

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Sensor A: Obj");
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// Task for Sensor B
void sensorBTask(void* pvParameters) {
  while (true) {
    if (detectedA && !detectedB) {
      float dB = getDistance(trig_B, echo_B);
      if (dB < 10.0) {
        t2 = micros();
        detectedB = true;
        Serial.println("Sensor B Triggered");

        time_diff = (t2 - t1) / 1000000.0;  // convert µs to seconds
        speed = sensor_distance / time_diff;

        // Display results
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Speed: ");
        lcd.print(speed, 2);
        lcd.print(" m/s");

        lcd.setCursor(0, 1);
        lcd.print("T: ");
        lcd.print(time_diff, 3);
        lcd.print(" s");

        delay(3000);

        // Reset
        detectedA = false;
        detectedB = false;

        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Ready...");
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Ultrasonic SPD");

  pinMode(trig_A, OUTPUT);
  pinMode(echo_A, INPUT);
  pinMode(trig_B, OUTPUT);
  pinMode(echo_B, INPUT);

  delay(2000);
  lcd.clear();
  lcd.print("Ready...");

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(sensorATask, "Sensor A Task", 2048, NULL, 1, NULL, 0); // Core 0
  xTaskCreatePinnedToCore(sensorBTask, "Sensor B Task", 2048, NULL, 1, NULL, 1); // Core 1
}

void loop() {
  // Nothing here, everything runs in RTOS tasks
}

speedometer using esp32 and ultrasonic sensors using RTos
