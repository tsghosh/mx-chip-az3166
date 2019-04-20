// Copyright (c) Microsoft. All rights reserved.
// Licensed under the MIT license. 

#include "HTS221Sensor.h"
#include "LIS2MDLSensor.h"
#include "LSM6DSLSensor.h"
#include "AzureIotHub.h"
#include "Arduino.h"
#include "parson.h"
#include "config.h"
#include "RGB_LED.h"

#define RGB_LED_BRIGHTNESS 32

DevI2C *i2c;
HTS221Sensor *sensor;
static RGB_LED rgbLed;
static int interval = INTERVAL;
static float humidity;
static float temperature;

LIS2MDLSensor *lis2mdl;
LSM6DSLSensor *lsm6dsl;

int getInterval()
{
    return interval;
}

void blinkLED()
{
    rgbLed.turnOff();
    rgbLed.setColor(RGB_LED_BRIGHTNESS, 0, 0);
    delay(500);
    rgbLed.turnOff();
}

void blinkSendConfirmation()
{
    rgbLed.turnOff();
    rgbLed.setColor(0, 0, RGB_LED_BRIGHTNESS);
    delay(500);
    rgbLed.turnOff();
}

void parseTwinMessage(DEVICE_TWIN_UPDATE_STATE updateState, const char *message)
{
    JSON_Value *root_value;
    root_value = json_parse_string(message);
    if (json_value_get_type(root_value) != JSONObject)
    {
        if (root_value != NULL)
        {
            json_value_free(root_value);
        }
        LogError("parse %s failed", message);
        return;
    }
    JSON_Object *root_object = json_value_get_object(root_value);

    double val = 0;
    if (updateState == DEVICE_TWIN_UPDATE_COMPLETE)
    {
        JSON_Object *desired_object = json_object_get_object(root_object, "desired");
        if (desired_object != NULL)
        {
            val = json_object_get_number(desired_object, "interval");
        }
    }
    else
    {
        val = json_object_get_number(root_object, "interval");
    }
    if (val > 500)
    {
        interval = (int)val;
        LogInfo(">>>Device twin updated: set interval to %d", interval);
    }
    json_value_free(root_value);
}

void SensorInit()
{
    i2c = new DevI2C(D14, D15);
    sensor = new HTS221Sensor(*i2c);
    sensor->init(NULL);
    
    humidity = -1;
    temperature = -1000;

    lis2mdl = new LIS2MDLSensor(*i2c);
    // init
    lis2mdl->init(NULL);

    lsm6dsl = new LSM6DSLSensor(*i2c, D4, D5);
    // init
    lsm6dsl->init(NULL);
}



float readTemperature()
{
    sensor->reset();

    float temperature = 0;
    sensor->getTemperature(&temperature);

    return temperature;
}

float readHumidity()
{
    sensor->reset();

    float humidity = 0;
    sensor->getHumidity(&humidity);

    return humidity;
}

bool readAllSensorData(int messageId, char *payload)
{
    JSON_Value *root_value = json_value_init_object();
    JSON_Object *root_object = json_value_get_object(root_value);
    char *serialized_string = NULL;

    json_object_set_number(root_object, "messageId", messageId);

    float temperature = readTemperature();
    float humidity = readHumidity();
    bool temperatureAlert = false;
    json_object_set_number(root_object, "temperature", temperature);
    json_object_set_number(root_object, "humidity", humidity);


    int axes[3];

    lis2mdl->getMAxes(axes);
    
    json_object_set_number(root_object,"magmeter-X",axes[0]);
    json_object_set_number(root_object,"magmeter-Y",axes[1]);
    json_object_set_number(root_object,"magmeter-Z",axes[2]);
    int aAxes[3];
    int aRaws[3];
    float aSensitivity;

    lsm6dsl->enableAccelerator();
    lsm6dsl->getXAxes(aAxes);

    json_object_set_number(root_object,"accel-X",aAxes[0]);
    json_object_set_number(root_object,"accel-Y",aAxes[1]);
    json_object_set_number(root_object,"accel-Z",aAxes[2]);

    int gAxes[3];
    int gRaws[3];
    float gSensitivity;

    lsm6dsl->enableGyroscope();
    lsm6dsl->getGAxes(gAxes);

    json_object_set_number(root_object,"gyro-X",gAxes[0]);
    json_object_set_number(root_object,"gyro-Y",gAxes[1]);
    json_object_set_number(root_object,"gyro-Z",gAxes[2]);



    serialized_string = json_serialize_to_string_pretty(root_value);
    snprintf(payload, MESSAGE_MAX_LEN, "%s", serialized_string);
    json_free_serialized_string(serialized_string);
    json_value_free(root_value);
    return temperatureAlert;
}


bool readMessage(int messageId, char *payload, float *temperatureValue, float *humidityValue)
{
    JSON_Value *root_value = json_value_init_object();
    JSON_Object *root_object = json_value_get_object(root_value);
    
    char *serialized_string = NULL;

    json_object_set_number(root_object, "messageId", messageId);

    float t = readTemperature();
    float h = readHumidity();
        
    bool temperatureAlert = false;
    if(t != temperature)
    {
        temperature = t;
        *temperatureValue = t;
        json_object_set_number(root_object, "temperature", temperature);
    }
    if(temperature > TEMPERATURE_ALERT)
    {
        temperatureAlert = true;
    }
    
    if(h != humidity)
    {
        humidity = h;
        *humidityValue = h;
        json_object_set_number(root_object, "humidity", humidity);

        
    }

    
    int axes[3];

    lis2mdl->getMAxes(axes);
    
    json_object_set_number(root_object,"magmeter-X",axes[0]);
    json_object_set_number(root_object,"magmeter-Y",axes[1]);
    json_object_set_number(root_object,"magmeter-Z",axes[2]);
    

    int aAxes[3];
    int aRaws[3];
    float aSensitivity;

    lsm6dsl->enableAccelerator();
    lsm6dsl->getXAxes(aAxes);
    //lsm6dsl->getXSensitivity(&aSensitivity);
    //lsm6dsl->getXAxesRaw(aRaws);

    json_object_set_number(root_object,"accel-X",aAxes[0]);
    json_object_set_number(root_object,"accel-Y",aAxes[1]);
    json_object_set_number(root_object,"accel-Z",aAxes[2]);
    //json_object_set_number(root_object,"accel-sensitivity",aSensitivity);
    //json_object_set_number(root_object,"accel-raw-X",aRaw[0]);
    //json_object_set_number(root_object,"accel-raw-Y",aRaw[1]);
    //json_object_set_number(root_object,"accel-raw-Z",aRaw[2]);

    int gAxes[3];
    int gRaws[3];
    float gSensitivity;

    lsm6dsl->enableGyroscope();
    lsm6dsl->getGAxes(gAxes);
    //lsm6dsl->getGSensitivity(gSensitivity);
    //lsm6dsl->getGAxesRaw(gRaws);


    json_object_set_number(root_object,"gyro-X",gAxes[0]);
    json_object_set_number(root_object,"gyro-Y",gAxes[1]);
    json_object_set_number(root_object,"gyro-Z",gAxes[2]);
   // json_object_set_number(root_object,"gyro-sensitivity",gSensitivity);
   // json_object_set_number(root_object,"gyro-raw-X",gRaws[0]);
   // json_object_set_number(root_object,"gyro-raw-Y",gRaws[1]);
   // json_object_set_number(root_object,"gyro-raw-Z",gRaws[2]);
   
   
    serialized_string = json_serialize_to_string_pretty(root_value);

    snprintf(payload, MESSAGE_MAX_LEN, "%s", serialized_string);
    json_free_serialized_string(serialized_string);
    json_value_free(root_value);
    return temperatureAlert;
}

#if (DEVKIT_SDK_VERSION >= 10602)
void __sys_setup(void)
{
    // Enable Web Server for system configuration when system enter AP mode
    EnableSystemWeb(WEB_SETTING_IOT_DEVICE_CONN_STRING);
}
#endif