/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <cmath>

#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_flash.h"
#include "esp_log.h"
#include <driver/gpio.h> // GPIO pins
#include <driver/adc.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <BLEBeacon.h>
#include <nvs.h>
#include <nvs_flash.h>

#include <Preferences.h>

#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "sdkconfig.h"
#include "esp_sntp.h"
#include <string>

#include <ArduinoJson.h>
#include <EEPROM.h>

Preferences stopplate_preferences_store;
#define MAX_CLOCK_SYNC_RETRY_TIME 50
#define TIME_BETWEEN_HIT 0.05

/* #region DEBUG TAG BLOCK */
static const char *MIC_DEBUG_TAG = "Mic Sensor";
static const char *BLE_NOTIFY_DEBUG_TAG = "BLE | Notify";
static const char *LED_DEBUG_TAG = "LED";
static const char *BLE_SERVER_DEBUG_TAG = "BLE | Server";
static const char *BLE_DEVICE_DEBUG_TAG = "BLE | Device";
static const char *BLE_ADVERTISING_DEBUG_TAG = "BLE | Advertising";
static const char *BLE_GATT_DEBUG_TAG = "BLE | GATT";
static const char *CLOCK_DEBUG_TAG = "CLOCK";
static const char *PPREFERENCES_DEBUG_TAG = "PPREFERENCES";
static const char *JSON_DEBUG_TAG = "JSON";
/* #endregion */

enum waveform_type
{
    Sine,
    Square,
    Sawtooth,
    Triangle,
};
std::string waveform_obj[] = {
    "sine",
    "square",
    "sawtooth",
    "triangle",

};
enum EStopplatePreferences
{
    IndicatorLightUpDuration,
    CountdownRandomTimeMin,
    CountdownRandomTimeMax,
    BuzzerDuration,
    BuzzerFrequency,
    BuzzerWaveform,
};

void load_preferences();
/* #region GPIO BLOCK */
#define ORANGE_LED GPIO_NUM_4
#define BLUE_LED GPIO_NUM_2
#define ANALOG_MIC ADC1_CHANNEL_4
#define DIGITAL_MIC ADC1_CHANNEL_5

void init_gpio(void)
{
    gpio_set_direction(ORANGE_LED, GPIO_MODE_OUTPUT);
    gpio_set_direction(BLUE_LED, GPIO_MODE_OUTPUT);

    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(DIGITAL_MIC, ADC_ATTEN_DB_0);
    adc1_config_channel_atten(ANALOG_MIC, ADC_ATTEN_DB_0);
}
void init_gpio_level(void)
{
    gpio_set_level(ORANGE_LED, 1);
    gpio_set_level(BLUE_LED, 1);
}
/* #endregion */

/* #region LED BLOCK */
bool led_state = true;
void toggle_led(void)
{
    gpio_set_level(ORANGE_LED, led_state);
    gpio_set_level(BLUE_LED, !led_state);
    led_state = !led_state;
    ESP_LOGI(LED_DEBUG_TAG, "LED toggled");
}
/* #endregion */

/* #region BLE CALLBACK BLOCK */
void sync_time_with_ble(void);
void set_time_double(double sec);
double get_time(void);
double time_init = get_time();

BLEServer *pServer;
BLEService *pService;
BLECharacteristic *startSignalCharacteristic;
BLECharacteristic *stopSignalCharacteristic;
BLECharacteristic *timeSyncRequestCharacteristic;
BLECharacteristic *timeSyncWriteCharacteristic;
BLECharacteristic *settingCharacteristic;
BLECharacteristic *testingCharacteristic;
bool bleConnected = false;

double start_time = 0.0;

void notify_ble(void)
{
    stopSignalCharacteristic->setValue(std::to_string(get_time() - start_time));
    stopSignalCharacteristic->notify();
}

class stopSignalCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        ESP_LOGI(BLE_GATT_DEBUG_TAG, "stop signal write value raw: %s", pCharacteristic->getData());
        ESP_LOGI(BLE_GATT_DEBUG_TAG, "stop signal write value: %s", pCharacteristic->getValue().c_str());
    }
};
class startSignalCallbacks : public BLECharacteristicCallbacks
{
    void onNotify(BLECharacteristic *pCharacteristic)
    {
        start_time = std::stod(pCharacteristic->getValue());
    }
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        start_time = std::stod(pCharacteristic->getValue());
        ESP_LOGI(BLE_GATT_DEBUG_TAG, "current time: %s", std::to_string(get_time()).c_str());
        ESP_LOGI(BLE_GATT_DEBUG_TAG, "start signal write value raw: %s", pCharacteristic->getData());
        ESP_LOGI(BLE_GATT_DEBUG_TAG, "start signal write value: %s", pCharacteristic->getValue().c_str());
    }
};

int clock_sync_retry_count;
void notify_perform_clock_sync()
{
    if (clock_sync_retry_count > MAX_CLOCK_SYNC_RETRY_TIME)
    {
        return;
    }
    clock_sync_retry_count++;
    time_init = get_time();
    ESP_LOGI(CLOCK_DEBUG_TAG, "sync_time_with_ble begin");
    timeSyncRequestCharacteristic->setValue("");
    timeSyncRequestCharacteristic->notify();
    ESP_LOGI(CLOCK_DEBUG_TAG, "Start sync request");
}
class timeSyncRequestCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        clock_sync_retry_count = 0;
        notify_perform_clock_sync();
    }
};
class timeSyncWriteCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        try {
            double time_received = std::stod(pCharacteristic->getValue());
            double time_end = get_time();
            double time_interval = time_end - time_init;
            double time_final = (time_received) + ((time_interval) / 2) /* /2 divide by 2 is essential for cristian clock syncronize alg. but idk why it will decrease the accuracy. */;
            set_time_double(time_final);
            double error = std::fabs(get_time() - time_final);
            ESP_LOGI(CLOCK_DEBUG_TAG, "time sync reply with: %s", std::to_string(time_received).c_str());
            ESP_LOGI(CLOCK_DEBUG_TAG, "time_start: %s", std::to_string(time_init).c_str());
            ESP_LOGI(CLOCK_DEBUG_TAG, "time_end: %s", std::to_string(time_end).c_str());
            ESP_LOGI(CLOCK_DEBUG_TAG, "time_interval: %s", std::to_string(time_interval).c_str());
            ESP_LOGI(CLOCK_DEBUG_TAG, "time_received: %s", std::to_string(time_received).c_str());
            ESP_LOGI(CLOCK_DEBUG_TAG, "time_final: %s", std::to_string(time_final).c_str());
            ESP_LOGI(CLOCK_DEBUG_TAG, "error: %s", std::to_string(error).c_str());
            ESP_LOGI(BLE_GATT_DEBUG_TAG, "time sync write value raw: %s", pCharacteristic->getData());
            ESP_LOGI(BLE_GATT_DEBUG_TAG, "time sync write value: %s", pCharacteristic->getValue().c_str());
            if (error > 0.1 && clock_sync_retry_count < MAX_CLOCK_SYNC_RETRY_TIME)
            {
                ESP_LOGI(CLOCK_DEBUG_TAG, "retry count: %d", clock_sync_retry_count);
                // retry until error < 0.1
                notify_perform_clock_sync();
            }
        } catch (std::exception &e) {
            ESP_LOGI(CLOCK_DEBUG_TAG, "time sync write value error: %s", e.what());
            return;
        }
    }
};
class settingCallbacks : public BLECharacteristicCallbacks
{
    void onWrite(BLECharacteristic *pCharacteristic)
    {
        ESP_LOGI(BLE_GATT_DEBUG_TAG, "setting write value: %s", pCharacteristic->getValue().c_str());

        StaticJsonDocument<200> json_doc;
        DeserializationError json_error;
        char json_output[250];
        memset(json_output, '\0', sizeof(json_output));
        strcpy(json_output, pCharacteristic->getValue().c_str());
        json_error = deserializeJson(json_doc, json_output);
        ESP_LOGI(JSON_DEBUG_TAG, "%s", json_error.c_str());
        if (!json_error)
        {
            int32_t indicator_light_up_duration = json_doc["indicator_light_up_duration"];
            float_t countdown_random_time_min = json_doc["countdown_random_time_min"];
            float_t countdown_random_time_max = json_doc["countdown_random_time_max"];
            int32_t buzzer_duration = json_doc["buzzer_duration"];
            int32_t buzzer_frequency = json_doc["buzzer_frequency"];
            int32_t buzzer_waveform = json_doc["buzzer_waveform"];

            ESP_LOGI(JSON_DEBUG_TAG, "indicator_light_up_duration: %i,countdown_random_time_min: %f,countdown_random_time_max: %f,buzzer_duration: %i,buzzer_frequency: %i,buzzer_waveform: %i", indicator_light_up_duration, countdown_random_time_min, countdown_random_time_max, buzzer_duration, buzzer_frequency, buzzer_waveform);
            stopplate_preferences_store.begin("settings", false);
            stopplate_preferences_store.putInt(std::to_string(EStopplatePreferences::IndicatorLightUpDuration).c_str(), indicator_light_up_duration);
            stopplate_preferences_store.putFloat(std::to_string(EStopplatePreferences::CountdownRandomTimeMin).c_str(), countdown_random_time_min);
            stopplate_preferences_store.putFloat(std::to_string(EStopplatePreferences::CountdownRandomTimeMax).c_str(), countdown_random_time_max);
            stopplate_preferences_store.putInt(std::to_string(EStopplatePreferences::BuzzerDuration).c_str(), buzzer_duration);
            stopplate_preferences_store.putInt(std::to_string(EStopplatePreferences::BuzzerFrequency).c_str(), buzzer_frequency);
            stopplate_preferences_store.putInt(std::to_string(EStopplatePreferences::BuzzerWaveform).c_str(), buzzer_waveform);
            stopplate_preferences_store.end();
            load_preferences();
        }
    }
};

/* #endregion */

/* #region BLE BLOCK */
#define DEVICE_NAME "1"
#define SERVICE_UUID "7A0247E7-8E88-409B-A959-AB5092DDB03E"
#define START_SIGNAL_CHARACTERISTIC_UUID "3C224D84-566D-4F13-8B1C-2117021FF1A2"
#define STOP_SIGNAL_CHARACTERISTIC_UUID "57B92756-3DF4-4038-B825-FC8E1C2FDB5B"
#define TIME_SYNC_REQUEST_CHARACTERISTIC_UUID "840A0941-55E9-44E4-BFFF-1C3C27BF6AF0"
#define TIME_SYNC_WRITE_CHARACTERISTIC_UUID "E0832E3E-F1E1-43B1-9569-109E2770E3ED"
#define SETTING_CHARACTERISTIC_UUID "798F2478-4C44-417F-BB6E-EE2A826CC17C"

class StopplateBLEServerCallback : public BLEServerCallbacks
{
    void onConnect(BLEServer *pServer)
    {
        bleConnected = true;
        ESP_LOGI(BLE_SERVER_DEBUG_TAG, "Connected");
    };

    void onDisconnect(BLEServer *pServer)
    {
        bleConnected = false;
        ESP_LOGI(BLE_SERVER_DEBUG_TAG, "Disconnected");

        // Restart advertising to be visible and connectable again
        BLEAdvertising *pAdvertising;
        pAdvertising = pServer->getAdvertising();
        pAdvertising->start();
        ESP_LOGI(BLE_ADVERTISING_DEBUG_TAG, "Advertising restart due to device disconnected");
    }
};

void init_characteristic()
{
    stopSignalCharacteristic = pService->createCharacteristic(
        STOP_SIGNAL_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    stopSignalCharacteristic->setCallbacks(new stopSignalCallbacks());
    stopSignalCharacteristic->addDescriptor(new BLE2902());

    timeSyncRequestCharacteristic = pService->createCharacteristic(
        TIME_SYNC_REQUEST_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_NOTIFY |
            BLECharacteristic::PROPERTY_READ);
    timeSyncRequestCharacteristic->setCallbacks(new timeSyncRequestCallbacks());
    timeSyncRequestCharacteristic->addDescriptor(new BLE2902());

    timeSyncWriteCharacteristic = pService->createCharacteristic(
        TIME_SYNC_WRITE_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_WRITE |
            BLECharacteristic::PROPERTY_WRITE_NR |
            BLECharacteristic::PROPERTY_NOTIFY |
            BLECharacteristic::PROPERTY_READ);
    timeSyncWriteCharacteristic->setCallbacks(new timeSyncWriteCallbacks());
    timeSyncWriteCharacteristic->addDescriptor(new BLE2902());

    startSignalCharacteristic = pService->createCharacteristic(
        START_SIGNAL_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_WRITE);
    startSignalCharacteristic->setCallbacks(new startSignalCallbacks());
    startSignalCharacteristic->addDescriptor(new BLE2902());

    settingCharacteristic = pService->createCharacteristic(
        SETTING_CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_WRITE);
    settingCharacteristic->setCallbacks(new settingCallbacks());
    settingCharacteristic->addDescriptor(new BLE2902());
}

void init_ble()
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_LOGI(BLE_DEVICE_DEBUG_TAG, "started nvs_flash_init");
        ret = nvs_flash_init();
        ESP_LOGI(BLE_DEVICE_DEBUG_TAG, "nvs_flash_init initialized");
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(BLE_DEVICE_DEBUG_TAG, "started to init_ble");
    BLEDevice::init(DEVICE_NAME);
    ESP_LOGI(BLE_DEVICE_DEBUG_TAG, "BLE device initialized");

    pServer = BLEDevice::createServer();
    ESP_LOGI(BLE_SERVER_DEBUG_TAG, "BLE server created");

    pServer->setCallbacks(new StopplateBLEServerCallback());
    pService = pServer->createService(SERVICE_UUID);
    ESP_LOGI(BLE_SERVER_DEBUG_TAG, "BLE service created");

    init_characteristic();

    pService->start();
    ESP_LOGI(BLE_SERVER_DEBUG_TAG, "BLE service started");

    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    ESP_LOGI(BLE_ADVERTISING_DEBUG_TAG, "BLE advertising getted");

    pAdvertising->addServiceUUID(SERVICE_UUID);
    ESP_LOGI(BLE_ADVERTISING_DEBUG_TAG, "BLE advertising uuid setted");

    pAdvertising->setScanResponse(false);
    pAdvertising->setMinPreferred(0x0); // set value to 0x00 to not advertise this parameter
    BLEDevice::startAdvertising();
    ESP_LOGI(BLE_ADVERTISING_DEBUG_TAG, "BLE advertising started");
}
/* #endregion */

/* #region ESP CORE BLOCK */
double int_to_decimal(int num)
{
    int numDigits = std::to_string(num).length();
    double decimal = static_cast<double>(num) / std::pow(10, numDigits);

    return decimal;
}
double get_time(void)
{
    struct timeval tv_now;
    gettimeofday(&tv_now, NULL);
    // long int time_us = tv_now.tv_sec * 1000000L + tv_now.tv_usec;
    int time_us = tv_now.tv_usec * 0.01;
    int time_sec = tv_now.tv_sec * 1;
    double final_time = time_sec + time_us * 0.0001;
    ESP_LOGI(CLOCK_DEBUG_TAG, "Get time: %f", final_time);
    return final_time;
}
void set_time(int sec, int usec)
{
    timeval current = {
        .tv_sec = sec,  /* seconds */
        .tv_usec = usec /* and microseconds */
    };
    timezone zone = {.tz_minuteswest = 0,
                     .tz_dsttime = 0};

    settimeofday(&current, &zone);
    ESP_LOGI(CLOCK_DEBUG_TAG, "Set time: %d.%d", sec, usec);
};
void set_time_double(double sec)
{
    int rounded = std::floor(sec);
    timeval current = {
        .tv_sec = rounded,                      /* seconds */
        .tv_usec = (suseconds_t)(sec - rounded) /* and microseconds */
    };
    timezone zone = {.tz_minuteswest = 0,
                     .tz_dsttime = 0};

    settimeofday(&current, &zone);
    ESP_LOGI(CLOCK_DEBUG_TAG, "Set time: %f", sec);
};

void load_preferences()
{
    stopplate_preferences_store.begin("settings", false);
    StaticJsonDocument<200> json_doc_v;
    char json_output_v[250];
    json_doc_v["indicator_light_up_duration"] = stopplate_preferences_store.getInt(std::to_string(EStopplatePreferences::IndicatorLightUpDuration).c_str());
    json_doc_v["countdown_random_time_min"] = stopplate_preferences_store.getFloat(std::to_string(EStopplatePreferences::CountdownRandomTimeMin).c_str(), 0);
    json_doc_v["countdown_random_time_max"] = stopplate_preferences_store.getFloat(std::to_string(EStopplatePreferences::CountdownRandomTimeMax).c_str(), 0);
    json_doc_v["buzzer_duration"] = stopplate_preferences_store.getInt(std::to_string(EStopplatePreferences::BuzzerDuration).c_str());
    json_doc_v["buzzer_frequency"] = stopplate_preferences_store.getInt(std::to_string(EStopplatePreferences::BuzzerFrequency).c_str());
    json_doc_v["buzzer_waveform"] = stopplate_preferences_store.getInt(std::to_string(EStopplatePreferences::BuzzerWaveform).c_str());
    serializeJson(json_doc_v, json_output_v);
    ESP_LOGI(JSON_DEBUG_TAG, "json_output: %s", json_output_v);
    settingCharacteristic->setValue(json_output_v);
    stopplate_preferences_store.end();
}

void esp_loop(void);
extern "C" void app_main()
{
    init_gpio();
    init_gpio_level();

    init_ble();

    set_time(1695827472, 100);

    // nvs_flash_erase(); // erase the NVS partition and...
    // nvs_flash_init();  // initialize the NVS partition.

    load_preferences();

    timeSyncRequestCharacteristic->setValue("tea");
    while (1)
    {
        esp_loop();
    }
}

double previous_hit = 0;

void esp_loop(void)
{
    int analog_mic_val = adc1_get_raw(ANALOG_MIC);
    int digital_mic_val = adc1_get_raw(DIGITAL_MIC);
    if (analog_mic_val >= 1200)
    {
        if (get_time() - previous_hit >= TIME_BETWEEN_HIT)
        {

            ESP_LOGI(MIC_DEBUG_TAG, "Triggered");
            ESP_LOGI(MIC_DEBUG_TAG, "analog_mic_val: %i, digital_mic_val: %i,", analog_mic_val, digital_mic_val);
            toggle_led();
            notify_ble();
            previous_hit = get_time();
        }
    }

    vTaskDelay(10);
}
/* #endregion */
