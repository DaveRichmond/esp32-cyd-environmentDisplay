#include <Arduino.h>

#include <freertos/semphr.h>

// Including this library will also include lvgl.h and TFT_eSPI.h 
#include <LVGL_CYD.h>

#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_AHTX0.h>

const auto BMP_SCL = 22;
const auto BMP_SDA = 27;
//const auto BMP_VCC = 27;
const auto SEALEVELPRESSURE_HPA = (1013.25);


Adafruit_BMP280 bmp(&Wire);
Adafruit_AHTX0 aht;

typedef struct {
    bool valid;
    float pressure;
    float temperature;
    float humidity;
} Environment ;
Environment env{false, 0.0, 0.0, 0.0};
SemaphoreHandle_t env_mutex;

lv_obj_t *temperature_label = NULL;
lv_obj_t *pressure_label = NULL;
lv_obj_t *humidity_label = NULL;

void env_task(void *params){
    while(1){
        if(xSemaphoreTake(env_mutex, (TickType_t)10) == pdTRUE){
            env.valid = true;
            env.temperature = bmp.readTemperature();
            env.pressure = bmp.readPressure();

            sensors_event_t aht_temp, aht_humidity;
            aht.getEvent(&aht_humidity, &aht_temp);

            env.humidity = aht_humidity.relative_humidity;

            Serial.println(String("Temperatures: bmp:") + String(env.temperature) + String(" aht:") + String(aht_temp.temperature));
            xSemaphoreGive(env_mutex);
        } else {
            Serial.println("Failed to get mutex");
        }
        
        if(env.valid){
            // Serial.print(F("Temperature = "));
            // Serial.print(env.temperature);
            // Serial.println(" *C");

            // Serial.print(F("Pressure = "));
            // Serial.print(env.pressure);
            // Serial.println(" Pa");

            // Serial.println();
        } 
        
        vTaskDelay(100 * portTICK_PERIOD_MS);
    }
}

void setup() {
  
    // Start Serial and lvgl, set everything up for this device
    // Will also do Serial.begin(115200), lvgl.init(), set up the touch driver
    // and the lvgl timer.
    LVGL_CYD::begin(USB_LEFT);

    // pinMode(BMP_VCC, OUTPUT);
    // digitalWrite(BMP_VCC, 1);

    Wire.setPins(BMP_SDA, BMP_SCL);
    Serial.println(F("BMP280 test"));
    unsigned status;
    //status = bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID);
    status = bmp.begin();
    if (!status) {
        Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                        "try a different address!"));
        Serial.print("SensorID was: 0x"); Serial.println(bmp.sensorID(),16);
        Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
        Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
        Serial.print("        ID of 0x60 represents a BME 280.\n");
        Serial.print("        ID of 0x61 represents a BME 680.\n");
        while (1) delay(10);
    }
    Serial.println("AHT20 starting");
    if(!aht.begin()){
        Serial.println("Failed to find AHT Sensor");
        while(1) delay(10);
    }

    Serial.println("setup GUI");

    lv_obj_t *layout = lv_obj_create(lv_scr_act());
    lv_obj_set_size(layout, 320, 240);
    lv_obj_align(layout, LV_ALIGN_CENTER, 0, 0); 
    lv_obj_set_flex_flow(layout, LV_FLEX_FLOW_COLUMN);

    lv_obj_set_style_text_font(layout, &lv_font_montserrat_16, 0);

    // Create single button in center of active screen
    lv_obj_t * button = lv_button_create(layout);
    lv_obj_t * button_label = lv_label_create(button);
    lv_label_set_text(button_label, "Oh yeah?");

    // Set a callback to print to serial port when button is clicked.
    //   For those unfamiliar: The "[]() ->" thing is an "anonymous function". It lets
    //   you define a function in place instead pointing to one named elsewhere.
    lv_obj_add_event_cb(button, [](lv_event_t * e) -> void {
        Serial.println("Hell yeah!");
    }, LV_EVENT_CLICKED, NULL);

    lv_obj_t *row, *label;
    
    row = lv_obj_create(layout);
    lv_obj_set_size(row, 280, 50);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    label = lv_label_create(row);
    lv_label_set_text(label, "Temperature");
    temperature_label = lv_label_create(row);
    lv_label_set_text(temperature_label, "Pending");

    row = lv_obj_create(layout);
    lv_obj_set_size(row, 280, 50);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    label = lv_label_create(row);
    lv_label_set_text(label, "Pressure");
    pressure_label = lv_label_create(row);
    lv_label_set_text(temperature_label, "Pending");

   
    row = lv_obj_create(layout);
    lv_obj_set_size(row, 280, 50);
    lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
    label = lv_label_create(row);
    lv_label_set_text(label, "Humidity");
    humidity_label = lv_label_create(row);
    lv_label_set_text(temperature_label, "Pending");

    Serial.println("Start environment montioring");
    env_mutex = xSemaphoreCreateMutex();
    xTaskCreate(env_task, "environment", 2048, NULL, 1, NULL);

}

void loop() {

    // lvgl needs this to be called in a loop to run the interface
    lv_task_handler();

    if(xSemaphoreTake(env_mutex, 10) == pdTRUE){
        if(env.valid){
            if(temperature_label != NULL){
                Serial.println("Setting temperature text");
                String s = String(env.temperature) + String("°C");
                lv_label_set_text(temperature_label, s.c_str());
            }
            if(pressure_label != NULL){
                Serial.println("Setting pressure text");
                String s = String(env.pressure/1000) + String("khPa");
                lv_label_set_text(pressure_label, s.c_str());
            }
            if(humidity_label != NULL){
                String s = String(env.humidity) + String("%rh");
                lv_label_set_text(humidity_label, s.c_str());
            }
        }
        xSemaphoreGive(env_mutex);
    }
}