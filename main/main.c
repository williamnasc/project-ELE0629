#include <stdio.h>
#include <string.h>
#include <inttypes.h>

//Includes FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

//Includes ESP-IDF
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_random.h"
#include "esp_err.h"
#include "driver/gpio.h"
#include <esp_system.h>

//Biblioteca criada
#include "wifi.h"
#include "MQTT_lib.h"

//Bibliotecas externas
#include <bmp180.h>
#include <dht.h>

#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

#define SENSOR_TYPE DHT_TYPE_DHT11

#define DHT_GPIO 4

QueueHandle_t queueTemperatura;
QueueHandle_t queueUmidade;
QueueHandle_t queuePressao;

static const char *TAG = "MQTT Task";
static const char *TAG_TEMPERATURA = "Temperatura Task";
static const char *TAG_UMIDADE = "Umidade Task";
static const char *TAG_PRESSAO = "Pressao Task";

// Definição do struct 
struct Dados { 
    char *texto; 
    float temperatura;
    float umidade; 
    uint32_t pressao; 
};

void dht11_task(void *pvParameters)
{
    float temperatura, umidade;
    gpio_set_pull_mode(DHT_GPIO, GPIO_PULLUP_ONLY);
    
    //inicia a fila que salva os valores de temperatura
    queueTemperatura = xQueueCreate(1, sizeof(float));
    queueUmidade = xQueueCreate(1, sizeof(float));
    
    while (1)
    {
          
        if (dht_read_float_data(SENSOR_TYPE, DHT_GPIO, &umidade, &temperatura) == ESP_OK)
        {
            ESP_LOGI(TAG_TEMPERATURA, "Temperatura: %.2f C", temperatura);
            if (xQueueSend(queueTemperatura, &temperatura, portMAX_DELAY) != pdPASS)
            {
                ESP_LOGE(TAG_TEMPERATURA, "Falha ao enviar temperatura para a fila");
            }
            ESP_LOGI(TAG_UMIDADE, "Umidade: %.2f", umidade);
            if (xQueueSend(queueUmidade, &umidade, portMAX_DELAY) != pdPASS)
            {
                ESP_LOGE(TAG_UMIDADE, "Falha ao enviar umidade para a fila");
            }
        }
        else
        {
            ESP_LOGE(TAG_TEMPERATURA, "Falha ao ler sensor DHT11");
        }

        vTaskDelay(pdMS_TO_TICKS(2000));  // Aguarda 2 segundos
    }
}

void pressao_task(void *pvParameters)
{
    bmp180_dev_t dev;
    memset(&dev, 0, sizeof(bmp180_dev_t));

    ESP_ERROR_CHECK(bmp180_init_desc(&dev, 0, CONFIG_EXAMPLE_I2C_MASTER_SDA, CONFIG_EXAMPLE_I2C_MASTER_SCL));
    ESP_ERROR_CHECK(bmp180_init(&dev));

    //inicia a fila que salva os valores de temperatura
    queuePressao = xQueueCreate(1, sizeof(uint32_t));
    

    while (1) 
    {
        float temp;
        uint32_t pressure;

        bmp180_measure(&dev, &temp, &pressure, BMP180_MODE_STANDARD);

        if (xQueueSend(queuePressao, &pressure, portMAX_DELAY) == pdPASS) {
            ESP_LOGI(TAG_PRESSAO, "Enviado para a fila: %"PRIu32"", pressure);
        } else {
            ESP_LOGE(TAG_PRESSAO, "Falha ao enviar para a fila");
        }

        vTaskDelay(pdMS_TO_TICKS(2000));  // Aguarda 1 segundo
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

   while(1){
        // INICIA A ESTRUTA QUE JUNTA OS DADOS
        struct Dados pacote = {NULL, 0.0, 0.0, 0.0};

       if(mqtt_connected()){
            
            // ATUALIZA STATUS
            mqtt_publish("ELE0629/Weather/Status", "ON", 0, 0);

            // Espera receber um valor da fila de Temperatura
            float received_temperatura;
            if (xQueueReceive(queueTemperatura, &received_temperatura, portMAX_DELAY) == pdTRUE) {
                ESP_LOGI(TAG_TEMPERATURA, "Recebido da fila temp: %.2f",  received_temperatura);
                pacote.temperatura = received_temperatura;   
            }

            float received_umidade;
            if (xQueueReceive(queueUmidade, &received_umidade, portMAX_DELAY) == pdTRUE) {
                pacote.umidade = received_umidade;   
            }

            uint32_t received_pressao;
            if (xQueueReceive(queuePressao, &received_pressao, portMAX_DELAY) == pdTRUE) {
                pacote.pressao = received_pressao;   
            }

            
            //VERIFICA SE ESTA TODO MUNDO PREENCHIDO PRA ENVIAR
            if (pacote.temperatura != 0.0 && pacote.umidade != 0.0 && pacote.pressao != 0.0)
            {

                ESP_LOGI(TAG_TEMPERATURA, "Temperatura no pacote: %.2f",  pacote.temperatura);

                char json_string[50];
                sprintf(
                    json_string, 
                    "{Temp: %.2f °C, Umi: %.2f, Press: %"PRIu32" Pa}", 
                    pacote.temperatura, pacote.umidade, pacote.pressao
                );        

                mqtt_publish("ELE0629/Weather/Data", json_string, 0, 0);

                // RESETA OS VALORES DO STRUT
                pacote.texto = NULL;
                pacote.temperatura = 0.0;
                pacote.umidade = 0.0;
                pacote.pressao = 0.0;

            }
       }


       vTaskDelay(4000/portTICK_PERIOD_MS);
   }
}

void app_main(void)
{
    // task
    ESP_ERROR_CHECK(i2cdev_init());

    xTaskCreate(dht11_task, "dht11_task", configMINIMAL_STACK_SIZE * 6, NULL, 2, NULL);
    xTaskCreatePinnedToCore(pressao_task, "pressao_task", configMINIMAL_STACK_SIZE * 6, NULL, 2, NULL, APP_CPU_NUM);
    xTaskCreate(mqtt_task, "mqtt_task", configMINIMAL_STACK_SIZE * 6, NULL, 2, NULL);

    vTaskDelay(5000/portTICK_PERIOD_MS);
}
