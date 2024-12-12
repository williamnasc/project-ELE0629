#include <stdio.h>
#include <string.h>

//Includes FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

//Includes ESP-IDF
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_err.h"

//Biblioteca criada
#include "wifi.h"
#include "MQTT_lib.h"

//Bibliotecas externas
#include "mpu6050.h"

#ifdef CONFIG_EXAMPLE_I2C_ADDRESS_LOW
#define ADDR MPU6050_I2C_ADDRESS_LOW
#else
#define ADDR MPU6050_I2C_ADDRESS_HIGH
#endif

QueueHandle_t queueTemperatura;
QueueHandle_t queueMPU;

static const char *TAG = "Exemplo MQTT Main";
static const char *TAG_TEMPERATURA = "Temperatura Task";
static const char *TAG_MPU = "MPU Task";

// Definição do struct 
struct Dados { 
    char *texto; 
    int temperatura;
    int umidade; 
    float pressao;
    float sensacao; 
};

void temperatura_task(void *pvParameters)
{
    //inicia a fila que salva os valores de temperatura
    queueTemperatura = xQueueCreate(3, sizeof(uint32_t));
    
    int count = 0;
    while (1) {
        count++;
        // Envia o valor de 'count' para a fila
        if (xQueueSend(queueTemperatura, &count, portMAX_DELAY) == pdPASS) {
            ESP_LOGI(TAG_TEMPERATURA, "Enviado para a fila: %d", count);
        } else {
            ESP_LOGE(TAG_TEMPERATURA, "Falha ao enviar para a fila");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));  // Aguarda 1 segundo
    }
}

void mpu6050_test(void *pvParameters)
{
    mpu6050_dev_t dev = { 0 };

    ESP_ERROR_CHECK(mpu6050_init_desc(&dev, ADDR, 0, CONFIG_EXAMPLE_SDA_GPIO, CONFIG_EXAMPLE_SCL_GPIO));

    while (1)
    {
        esp_err_t res = i2c_dev_probe(&dev.i2c_dev, I2C_DEV_WRITE);
        if (res == ESP_OK)
        {
            ESP_LOGI(TAG_MPU, "Found MPU60x0 device");
            break;
        }
        ESP_LOGE(TAG_MPU, "MPU60x0 not found");
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    ESP_ERROR_CHECK(mpu6050_init(&dev));

    ESP_LOGI(TAG_MPU, "Accel range: %d", dev.ranges.accel);
    ESP_LOGI(TAG_MPU, "Gyro range:  %d", dev.ranges.gyro);

    queueMPU = xQueueCreate(3, sizeof("000.000, 000.000, 000.000"));

    while (1)
    {
        float temp;
        mpu6050_acceleration_t accel = { 0 };
        mpu6050_rotation_t rotation = { 0 };

        ESP_ERROR_CHECK(mpu6050_get_temperature(&dev, &temp));
        ESP_ERROR_CHECK(mpu6050_get_motion(&dev, &accel, &rotation));

        ESP_LOGI(TAG_MPU, "Acceleration: x=%.2f   y=%.2f   z=%.2f", accel.x, accel.y, accel.z);
        
        // JUNTA OS DADOS DO MPU NUMA STRING
        char converted_value[50];
        sprintf(converted_value, "(%.2f, %.2f, %.2f)", accel.x, accel.y, accel.z);        

        if (xQueueSend(queueMPU, &converted_value, portMAX_DELAY) == pdPASS) {
            ESP_LOGI(TAG_MPU, "Enviado para a fila: %s", converted_value);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));  // Aguarda 1 segundo
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

    mqtt_subscribe("ELE0629/Weather/Teste", 0);
    mqtt_subscribe("ELE0629/Weather/Temperatura", 0);
    mqtt_subscribe("ELE0629/Weather/MPU", 0);

   while(1){
       int temp = esp_random() % 30;
       int umidade = esp_random() % 50;
       char temp_str[10], umidade_str[10];
       sprintf(temp_str, "%d", temp);
       sprintf(umidade_str, "%d", umidade);

       if(mqtt_connected()){
            
            // teste de um strut
            struct Dados pacote = {"Alice", 30, 30, 1.65, 1.65};
            pacote.texto = NULL;
            pacote.temperatura = NULL;


            // Espera receber um valor da fila de Temperatura
            char received_mpu[50];
            if (xQueueReceive(queueMPU, &received_mpu, portMAX_DELAY) == pdTRUE) {
                ESP_LOGI(TAG_TEMPERATURA, "Recebido do MPU: %s",  received_mpu);
                mqtt_publish("ELE0629/Weather/MPU", received_mpu, 0, 0);   
            }

            // Espera receber um valor da fila de Temperatura
            int received_temperatura;
            if (xQueueReceive(queueTemperatura, &received_temperatura, portMAX_DELAY) == pdTRUE) {
                ESP_LOGI(TAG_TEMPERATURA, "Recebido da fila temp: %d",  received_temperatura);
                pacote.temperatura = received_temperatura;   
            }
            
            //VERIFICA SE ESTA TODO MUNDO PREENCHIDO PRA ENVIAR
            if (pacote.temperatura != NULL){
                ESP_LOGI(TAG_TEMPERATURA, "Temperatura no pacote: %d",  pacote.temperatura);
                // CONVERTE O VALOR RECEBIDO PARA STRING
                char converted_value[50];
                sprintf(converted_value, "%d", pacote.temperatura);
                // PUBLICA NO BROKER
                mqtt_publish("ELE0629/Weather/Temperatura", converted_value, 0, 0);
            }
       }


       vTaskDelay(2000/portTICK_PERIOD_MS);
   }
}

void app_main(void)
{
    // task
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreate(mpu6050_test, "mpu6050_test", configMINIMAL_STACK_SIZE * 6, NULL, 2, NULL);
    xTaskCreate(temperatura_task, "temperatura_task", configMINIMAL_STACK_SIZE * 6, NULL, 2, NULL);
    xTaskCreate(mqtt_task, "mqtt_task", configMINIMAL_STACK_SIZE * 6, NULL, 2, NULL);

    vTaskDelay(5000/portTICK_PERIOD_MS);
}
