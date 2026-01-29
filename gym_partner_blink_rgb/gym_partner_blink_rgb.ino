// File : gym_partner_blinking_rgb.ino
// Date : July 8, 2024

#include <platform_config.h>
#include <platform_config_default.h>
#include <vl53l5cx_api.h>
#include <vl53l5cx_buffers.h>
#include <vl53l5cx_class.h>
#include <vl53l5cx_plugin_detection_thresholds.h>
#include <vl53l5cx_plugin_motion_indicator.h>
#include <vl53l5cx_plugin_xtalk.h>
#include <X-NUCLEO-53L5A1.h>
#include <Wire.h>
#include <Arduino.h>
#include <FspTimer.h>  // For real-time task scheduling

#include "NanoEdgeAI.h"
#include "knowledge.h"

#define LPN_PIN             5
#define I2C_RST_PIN         3
#define SENSOR_DATA_RATE    7   // Ranging frequency 7Hz, can go up to 15Hz
#define SENSOR_FRAME_RESOLUTION 64
#define SENSOR_FRAMES       16

#define Blinking_Frequency 2  // Blink every 0.5 seconds (2 Hz)

// ToF sensor
VL53L5CX myImager(&Wire, LPN_PIN, I2C_RST_PIN);
VL53L5CX_ResultsData measurementData;

uint8_t imageResolution = 0;    
uint8_t vl53l5cx_data_ready = 0;
static uint16_t neai_ptr = 0;

// RGB LED
int redPin = 9;
int greenPin = 10;
int bluePin = 11;
volatile bool ledState = true;

// NanoEdge AI
float input_user_buffer[DATA_INPUT_USER * AXIS_NUMBER]; // Buffer of input values 1024*1
float output_class_buffer[CLASS_NUMBER]; // Buffer of class probabilities
uint16_t id_class = 0;

const char *id2class[CLASS_NUMBER + 1] = { // Buffer for mapping class id to class name
  "",
  "arm_stretching",
  "nobody",
  "shoulders",
  "squat",
  "standing",
};

volatile int current_class = 0;

void fill_buffer(float sample_buffer[]) {
  while (neai_ptr < SENSOR_FRAMES) {
    myImager.vl53l5cx_check_data_ready(&vl53l5cx_data_ready);
    if (vl53l5cx_data_ready == true) {
      if (myImager.vl53l5cx_get_ranging_data(&measurementData) == false) { // Read distance data into the neai buffer
        for (uint16_t i = 0; i < SENSOR_FRAME_RESOLUTION; i++) {
          input_user_buffer[i + (SENSOR_FRAME_RESOLUTION * neai_ptr)] = (float)measurementData.distance_mm[i];
        }
      }
      neai_ptr++;
    }
  }
  // Reset pointer
  neai_ptr = 0;
}

// Timer for real-time scheduling
FspTimer led_timer;

void led_timer_callback(timer_callback_args_t __attribute((unused)) *p_args) {
  blink_LED();   // We can write the function body here, it's better for the stack
}

void blink_LED() {
  ledState = !ledState;
  switch (current_class) {
    case 1:  // arm_stretching --> Purple
      analogWrite(redPin, ledState ? 0xFF : 0x00);
      analogWrite(greenPin, 0x00);
      analogWrite(bluePin, ledState ? 0xFA : 0x00);
      break;
    case 2:  // nobody --> Red
      analogWrite(redPin, ledState ? 0xFF : 0x00);
      analogWrite(greenPin, 0x00);
      analogWrite(bluePin, 0x00);
      break;
    case 3:  // shoulders --> Yellow
      analogWrite(redPin, ledState ? 0xFF : 0x00);
      analogWrite(greenPin, ledState ? 0x90 : 0x00);
      analogWrite(bluePin, 0x00);
      break;
    case 4:  // squat --> Green
      analogWrite(redPin, 0x00);
      analogWrite(greenPin, ledState ? 0xFF : 0x00);
      analogWrite(bluePin, 0x00);
      break;
    case 5:  // standing --> Blue
      analogWrite(redPin, 0x00);
      analogWrite(greenPin, 0x00);
      analogWrite(bluePin, ledState ? 0xFF : 0x00);
      break;
  }
}

bool beginLedTimer(float rate) {
  uint8_t timer_type = GPT_TIMER;
  int8_t tindex = FspTimer::get_available_timer(timer_type);
  if (tindex < 0) {
    tindex = FspTimer::get_available_timer(timer_type, true);
  }
  if (tindex < 0) {
    return false;
  }
  FspTimer::force_use_of_pwm_reserved_timer();

  Serial.print("LED Timer Index = ");
  Serial.println(tindex);

  if (!led_timer.begin(TIMER_MODE_PERIODIC, timer_type, tindex, rate, 0.0f, led_timer_callback)) {
    return false;
  }
  if (!led_timer.setup_overflow_irq()) {
    return false;
  }
  if (!led_timer.open()) {
    return false;
  }
  if (!led_timer.start()) {
    return false;
  }
  return true;
}

void setup() {
  // Setup code to run once:
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, LOW);

  Serial.begin(115200);
  delay(100);
  if (!beginLedTimer(Blinking_Frequency)) { // Blink every 1/Blinking_Frequency seconds 
    Serial.println("Failed to start LED timer");
  }
  Wire.begin();          // This resets to 100kHz I2C
  Wire.setClock(400000); // Sensor has max I2C freq of 400kHz
  myImager.begin();

  if (myImager.init_sensor()) {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1);
  }
  myImager.vl53l5cx_set_resolution(SENSOR_FRAME_RESOLUTION);
  myImager.vl53l5cx_set_ranging_frequency_hz(SENSOR_DATA_RATE); // Ranging frequency
  myImager.vl53l5cx_get_resolution(&imageResolution); // Query sensor for current resolution - either 4x4 or 8x8
  myImager.vl53l5cx_start_ranging();

  enum neai_state error_code = neai_classification_init(knowledge);  // Initializing
  if (error_code != NEAI_OK) {
    // This happens if the knowledge does not correspond to the library or if the library works into a not supported board.
    Serial.println(F("The knowledge does not correspond to the library or the library works into a not supported board"));
  }
}

void loop() {
  // Main code:
  fill_buffer(input_user_buffer);
  neai_classification(input_user_buffer, output_class_buffer, &id_class);
  current_class = id_class;
  Serial.println(id_class);
  Serial.println(id2class[id_class]);
}
