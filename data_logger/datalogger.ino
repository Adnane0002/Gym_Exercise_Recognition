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

#define LPN_PIN     5
#define I2C_RST_PIN 3
#define SENSOR_DATA_RATE  7    //Ranging frequency 7Hz, it can go up to 15Hz
#define SENSOR_FRAME_RESOLUTION  64  //all 64 pads
#define SENSOR_FRAMES  16


VL53L5CX myImager(&Wire, LPN_PIN, I2C_RST_PIN);
VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 bytes of RAM


uint8_t imageResolution = 0;          //Used to correctly print output
uint8_t vl53l5cx_data_ready = 0;


static uint16_t neai_ptr = 0;
float neai_buffer[SENSOR_FRAME_RESOLUTION * SENSOR_FRAMES];   // Buffer of input values



void setup() {    
  Serial.begin(115200);
  delay(100);
  Wire.begin();          //This resets to 100kHz I2C
  Wire.setClock(400000); //Sensor has max I2C freq of 400kHz 
  myImager.begin();
  
  if(myImager.init_sensor())
  {
    Serial.println(F("Sensor not found - check your wiring. Freezing"));
    while (1);
  }
  myImager.vl53l5cx_set_resolution(SENSOR_FRAME_RESOLUTION);              
  myImager.vl53l5cx_set_ranging_frequency_hz(SENSOR_DATA_RATE);     //Ranging frequency 
  myImager.vl53l5cx_get_resolution(&imageResolution); //Query sensor for current resolution - either 4x4 or 8x8
  myImager.vl53l5cx_start_ranging();
}

void loop() {  
     
  while(neai_ptr < SENSOR_FRAMES) {
    myImager.vl53l5cx_check_data_ready(&vl53l5cx_data_ready);    
    if (vl53l5cx_data_ready == true) 
    {
      if(myImager.vl53l5cx_get_ranging_data(&measurementData) == false) //Read distance data into the neai buffer
      {
      for (uint16_t i = 0; i < SENSOR_FRAME_RESOLUTION; i++) {
        neai_buffer[i + (SENSOR_FRAME_RESOLUTION * neai_ptr)] = (float) measurementData.distance_mm[i];
      }
      }
      neai_ptr++;
    }
  }
  //Reset pointer
   neai_ptr = 0;


  for (uint16_t i = 0; i < (uint16_t) (SENSOR_FRAME_RESOLUTION * SENSOR_FRAMES); i++) {
  Serial.print((String) neai_buffer[i] + " ");
  }
  Serial.print("\n");

}
