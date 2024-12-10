#include <stdio.h>
#include <string.h>

//Includes FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//Includes ESP-IDF
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_err.h"

//Biblioteca criada
#include "wifi.h"
#include "MQTT_lib.h"

#include "mpu6050.h"

#ifdef CONFIG_EXAMPLE_I2C_ADDRESS_LOW
#define ADDR MPU6050_I2C_ADDRESS_LOW
#else
#define ADDR MPU6050_I2C_ADDRESS_HIGH
#endif

static const char *TAG = "Exemplo MQTT Main";

void mpu6050_test(void *pvParameters)
{
    mpu6050_dev_t dev = { 0 };

    ESP_ERROR_CHECK(mpu6050_init_desc(&dev, ADDR, 0, CONFIG_EXAMPLE_SDA_GPIO, CONFIG_EXAMPLE_SCL_GPIO));

    while (1)
    {
        esp_err_t res = i2c_dev_probe(&dev.i2c_dev, I2C_DEV_WRITE);
        if (res == ESP_OK)
        {
            ESP_LOGI(TAG, "Found MPU60x0 device");
            break;
        }
        ESP_LOGE(TAG, "MPU60x0 not found");
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_ERROR_CHECK(mpu6050_init(&dev));

    ESP_LOGI(TAG, "Accel range: %d", dev.ranges.accel);
    ESP_LOGI(TAG, "Gyro range:  %d", dev.ranges.gyro);

    while (1)
    {
        float temp;
        mpu6050_acceleration_t accel = { 0 };
        mpu6050_rotation_t rotation = { 0 };

        ESP_ERROR_CHECK(mpu6050_get_temperature(&dev, &temp));
        ESP_ERROR_CHECK(mpu6050_get_motion(&dev, &accel, &rotation));

        // ESP_LOGI(TAG, "**********************************************************************");
        ESP_LOGI(TAG, "Acceleration: x=%.2f   y=%.2f   z=%.2f", accel.x, accel.y, accel.z);
        // ESP_LOGI(TAG, "Rotation:     x=%.4f   y=%.4f   z=%.4f", rotation.x, rotation.y, rotation.z);
        // ESP_LOGI(TAG, "Temperature:  %.1f", temp);

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void mqtt_task(void *pvParameters)
{
    // Inicializando NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();    //Inicializando o WiFi. Função da Lib criada, wifi.h
    ESP_LOGI(TAG, "WiFi foi inicializado!");

    mqtt_start();       //Iniciando conexão MQTT. Função da Lib criada, MQTT.h
    ESP_LOGI(TAG, "MQTT foi inicializado!");

    //Inscrevendo nos tópicos
    mqtt_subscribe("teste/william/teste", 0);
    // mqtt_subscribe("UFRN/Lab/Umidade", 0);

   while(1){
       int temp = esp_random() % 30;
       int umidade = esp_random() % 50;
       char temp_str[10], umidade_str[10];
       sprintf(temp_str, "%d", temp);
       sprintf(umidade_str, "%d", umidade);

       if(mqtt_connected()){
           mqtt_publish("teste/william/teste", temp_str, 0, 0);
           ESP_LOGI(TAG, "Tempetatura: %d", temp);
        //    mqtt_publish("UFRN/Lab/Umidade", umidade_str, 0, 0);
        //    ESP_LOGI(TAG, "Umidade: %d", umidade);
       }


       vTaskDelay(5000/portTICK_PERIOD_MS);
   }
}

void app_main(void)
{
    // task
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreate(mpu6050_test, "mpu6050_test", configMINIMAL_STACK_SIZE * 6, NULL, 2, NULL);
    xTaskCreate(mqtt_task, "mqtt_task", configMINIMAL_STACK_SIZE * 6, NULL, 2, NULL);

    vTaskDelay(5000/portTICK_PERIOD_MS);
}
