#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <DHT.h>

#define DHTPIN 15
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);
LiquidCrystal_I2C lcd(0x27, 16, 2); // 0x27 is the default I2C address

// Create a FreeRTOS Queue to pass data safely between tasks
QueueHandle_t tempQueue;

// TASK 1: Read the sensor every 500ms
void TaskReadSensor(void *pvParameters) {
  for(;;) {
    float temp = dht.readTemperature(); // Read temperature in Celsius
    
    if (!isnan(temp)) {
      // Send the reading to the queue
      xQueueSend(tempQueue, &temp, portMAX_DELAY);
    }
    
    // Non-blocking FreeRTOS delay for 500ms
    vTaskDelay(pdMS_TO_TICKS(500)); 
  }
}

// TASK 2: Process the data and output to LCD/UART
void TaskProcessAndDisplay(void *pvParameters) {
  float temp;
  float sum = 0;
  float readings[10] = {0}; // Circular buffer for moving average filter
  int count = 0;
  int index = 0;
  
  for(;;) {
    // Wait until data is received from the queue
    if (xQueueReceive(tempQueue, &temp, portMAX_DELAY) == pdPASS) {
      
      // Algorithm: Basic Statistical Analysis (Moving Average Filter)
      sum = sum - readings[index];
      readings[index] = temp;
      sum = sum + temp;
      index = (index + 1) % 10;
      if (count < 10) count++;
      float avgTemp = sum / count;

      // UART Output (Serial Monitor)
      Serial.print("Raw Temp: ");
      Serial.print(temp);
      Serial.print(" C | Filtered Avg: ");
      Serial.println(avgTemp);

      // I2C Output (LCD Screen)
      lcd.setCursor(0, 0);
      lcd.print("Raw: ");
      lcd.print(temp, 1);
      lcd.print("C  ");
      
      lcd.setCursor(0, 1);
      lcd.print("Avg: ");
      lcd.print(avgTemp, 1);
      lcd.print("C  ");
    }
  }
}

void setup() {
  Serial.begin(115200); // Initialize UART
  dht.begin();          // Initialize DHT22
  lcd.init();           // Initialize I2C LCD
  lcd.backlight();

  // Initialize the Queue to hold up to 10 float values
  tempQueue = xQueueCreate(10, sizeof(float));

  // Initialize FreeRTOS Tasks
  xTaskCreate(TaskReadSensor, "ReadSensor", 2048, NULL, 1, NULL);
  xTaskCreate(TaskProcessAndDisplay, "ProcessDisplay", 2048, NULL, 1, NULL);
}

void loop() {
  // Empty. FreeRTOS scheduler handles all execution via tasks.
}
