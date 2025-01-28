#include <stdio.h>
#include <string.h>
#include <inttypes.h>

// Includes FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// Includes ESP-IDF
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_err.h"

// SPIFFS
#include "esp_spiffs.h"

// MQTT e Wi-Fi
#include "wifi.h"
#include "MQTT_lib.h"

// DHT e BMP180
#include <dht.h>
#include <bmp180.h>

// Configurações do DHT
#if defined(CONFIG_EXAMPLE_TYPE_DHT11)
#define SENSOR_TYPE DHT_TYPE_DHT11
#endif

// Configurações do BMP180
#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

// Tags para logging
static const char *TAG_SPIFFS = "SPIFFS";
static const char *TAG_MQTT = "MQTT";
static const char *TAG_DHT = "DHT";
static const char *TAG_PRESSAO = "BMP180";

// Filas para comunicação entre tarefas
static QueueHandle_t queueTemperatura;
static QueueHandle_t queueUmidade;
static QueueHandle_t queuePressao;

// Inicialização do SPIFFS
static esp_err_t init_spiffs() {
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG_SPIFFS, "Erro ao inicializar SPIFFS (%s)", esp_err_to_name(ret));
    }
    return ret;
}

// Salvar dados no SPIFFS
static void salvar_dados_spiffs(float temperatura, float umidade, uint32_t pressao) {
    FILE *arquivo = fopen("/spiffs/dados_sensores.csv", "a");
    if (arquivo == NULL) {
        ESP_LOGE(TAG_SPIFFS, "Erro ao abrir arquivo para escrita!");
        return;
    }

    fprintf(arquivo, "%.2f,%.2f,%" PRIu32 "\n", temperatura, umidade, pressao);
    fclose(arquivo);
    ESP_LOGI(TAG_SPIFFS, "Dados salvos no SPIFFS: Temp=%.2f°C, Umid=%.2f%%, Press= %" PRIu32 " Pa", temperatura, umidade, pressao);
}

// Tarefa para leitura do DHT11
void dht_task(void *pvParameters) {
    float temperatura, umidade;

#ifdef CONFIG_EXAMPLE_INTERNAL_PULLUP
    gpio_set_pull_mode(CONFIG_EXAMPLE_DATA_GPIO, GPIO_PULLUP_ONLY);
#endif

    queueTemperatura = xQueueCreate(1, sizeof(float));
    queueUmidade = xQueueCreate(1, sizeof(float));

    while (1) {
        if (dht_read_float_data(SENSOR_TYPE, CONFIG_EXAMPLE_DATA_GPIO, &umidade, &temperatura) == ESP_OK) {
            ESP_LOGI(TAG_DHT, "Temperatura: %.2f°C, Umidade: %.2f%%", temperatura, umidade);

            xQueueSend(queueTemperatura, &temperatura, portMAX_DELAY);
            xQueueSend(queueUmidade, &umidade, portMAX_DELAY);
        } else {
            ESP_LOGE(TAG_DHT, "Erro ao ler DHT11!");
        }
        vTaskDelay(pdMS_TO_TICKS(4000));
    }
}

// Tarefa para leitura do BMP180
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

        vTaskDelay(pdMS_TO_TICKS(4000)); 
    }
}

// Tarefa para salvar e enviar dados
void mqtt_spiffs_task(void *pvParameters) {
    float temperatura, umidade;
    uint32_t pressao;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wifi_init_sta();    // Inicializando o WiFi
    mqtt_start();       // Iniciando conexão MQTT

    while (1) {
        if (xQueueReceive(queueTemperatura, &temperatura, portMAX_DELAY) &&
            xQueueReceive(queueUmidade, &umidade, portMAX_DELAY) &&
            xQueueReceive(queuePressao, &pressao, portMAX_DELAY)) {
            
            // Salvar dados no SPIFFS
            salvar_dados_spiffs(temperatura, umidade, pressao);

            // Enviar dados via MQTT
            if (mqtt_connected()) {
                char msg_temp[50], msg_umid[50], msg_press[50];
                snprintf(msg_temp, sizeof(msg_temp), "Temp: %.2f °C", temperatura);
                snprintf(msg_umid, sizeof(msg_umid), "Umid: %.2f %%", umidade);
                snprintf(msg_press, sizeof(msg_press), "Press: %" PRIu32 " Pa", pressao);

                mqtt_publish("ELE0629/Weather/Temperature", msg_temp, 0, 0);
                mqtt_publish("ELE0629/Weather/Humidity", msg_umid, 0, 0);
                mqtt_publish("ELE0629/Weather/Pressure", msg_press, 0, 0);

                ESP_LOGI(TAG_MQTT, "Dados enviados via MQTT: %s, %s, %s", msg_temp, msg_umid, msg_press);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(4000));
    }
}

// Função principal
void app_main(void) {
    // Inicializar SPIFFS
    ESP_ERROR_CHECK(init_spiffs());

    ESP_ERROR_CHECK(i2cdev_init());

    // Criar tarefas
    xTaskCreate(dht_task, "dht_task", 2048, NULL, 5, NULL);
    xTaskCreatePinnedToCore(pressao_task, "pressao_task", configMINIMAL_STACK_SIZE * 6, NULL, 2, NULL, APP_CPU_NUM);
    xTaskCreate(mqtt_spiffs_task, "mqtt_spiffs_task", 4096, NULL, 5, NULL);
}
