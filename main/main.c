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

// DHT
#include <dht.h>

// Configuração do DHT
#if defined(CONFIG_EXAMPLE_TYPE_DHT11)
#define SENSOR_TYPE DHT_TYPE_DHT11
#endif

// Tags para logging
static const char *TAG_SPIFFS = "SPIFFS";
static const char *TAG_MQTT = "MQTT";
static const char *TAG_DHT = "DHT";

// Fila para comunicação entre tarefas
static QueueHandle_t queueTemperatura;
static QueueHandle_t queueUmidade;

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
static void salvar_dados_spiffs(float temperatura, float umidade) {
    FILE *arquivo = fopen("/spiffs/dados_sensores.csv", "a");
    if (arquivo == NULL) {
        ESP_LOGE(TAG_SPIFFS, "Erro ao abrir arquivo para escrita!");
        return;
    }

    fprintf(arquivo, "%.2f,%.2f\n", temperatura, umidade);
    fclose(arquivo);
    ESP_LOGI(TAG_SPIFFS, "Dados salvos no SPIFFS: Temp=%.2f°C, Umid=%.2f%%", temperatura, umidade);
}

// Ler dados do SPIFFS e contar leituras
static void ler_dados_spiffs() {
    FILE *arquivo = fopen("/spiffs/dados_sensores.csv", "r");
    if (arquivo == NULL) {
        ESP_LOGE(TAG_SPIFFS, "Erro ao abrir arquivo para leitura!");
        return;
    }

    ESP_LOGI(TAG_SPIFFS, "Lendo conteúdo do arquivo:");
    char linha[128];
    int contador = 0;

    while (fgets(linha, sizeof(linha), arquivo) != NULL) {
        printf("Leitura %d: %s", ++contador, linha);
    }
    fclose(arquivo);

    ESP_LOGI(TAG_SPIFFS, "Total de leituras realizadas: %d", contador);
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

// Tarefa para salvar e enviar dados
void mqtt_spiffs_task(void *pvParameters) {
    float temperatura, umidade;

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
            xQueueReceive(queueUmidade, &umidade, portMAX_DELAY)) {
            
            // Salvar dados no SPIFFS
            salvar_dados_spiffs(temperatura, umidade);

            // Enviar dados via MQTT
            if (mqtt_connected()) {
                char msg_temp[50], msg_umid[50];
                snprintf(msg_temp, sizeof(msg_temp), "Temp: %.2f °C", temperatura);
                snprintf(msg_umid, sizeof(msg_umid), "Umid: %.2f %%", umidade);

                mqtt_publish("ELE0629/Weather/Temperature", msg_temp, 0, 0);
                mqtt_publish("ELE0629/Weather/Humidity", msg_umid, 0, 0);

                ESP_LOGI(TAG_MQTT, "Dados enviados via MQTT: %s, %s", msg_temp, msg_umid);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(4000));
    }
}

// Tarefa para ler dados salvos no SPIFFS
void spiffs_read_task(void *pvParameters) {
    while (1) {
        ler_dados_spiffs();
        vTaskDelay(pdMS_TO_TICKS(100000));
    }
}

// Função principal
void app_main(void) {
    // Inicializar SPIFFS
    ESP_ERROR_CHECK(init_spiffs());

    // Criar tarefas
    xTaskCreate(dht_task, "dht_task", 2048, NULL, 5, NULL);
    xTaskCreate(mqtt_spiffs_task, "mqtt_spiffs_task", 4096, NULL, 5, NULL);
    xTaskCreate(spiffs_read_task, "spiffs_read_task", 4096, NULL, 5, NULL);
}
