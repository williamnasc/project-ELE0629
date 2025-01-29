#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdlib.h>

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
#include "driver/uart.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "driver/rtc_io.h"
#include <esp_system.h>

//Biblioteca criada
#include "wifi.h"
#include "MQTT_lib.h"

//Bibliotecas externas
#include <bmp180.h>
#include <dht.h>

//Configuração do bmp180
#ifndef APP_CPU_NUM
#define APP_CPU_NUM PRO_CPU_NUM
#endif

//Configuração do dht11 (tipo do sensor)
#if defined(CONFIG_EXAMPLE_TYPE_DHT11)
#define SENSOR_TYPE DHT_TYPE_DHT11
#endif
#if defined(CONFIG_EXAMPLE_TYPE_AM2301)
#define SENSOR_TYPE DHT_TYPE_AM2301
#endif
#if defined(CONFIG_EXAMPLE_TYPE_SI7021)
#define SENSOR_TYPE DHT_TYPE_SI7021
#endif

#define BOTAO 0
#define TIME_SLEEPING 5 // TEMPO EM SEGUNGOS
#define TIME_TO_UPDATE_MQTT_DATA 20000 // TEMPO EM MILISEGUNDOS

/*Prototipo de Metodos Auxiliares*/
void button_check();      
void config_sleep_and_button();
int64_t start_sleep();

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
    //Configuração do dht11 (pullup interno)
    #ifdef CONFIG_EXAMPLE_INTERNAL_PULLUP
        gpio_set_pull_mode(CONFIG_EXAMPLE_DATA_GPIO, GPIO_PULLUP_ONLY);
    #endif
    
    //inicia a fila que salva os valores de temperatura
    queueTemperatura = xQueueCreate(1, sizeof(float));
    queueUmidade = xQueueCreate(1, sizeof(float));
    
    while (1)
    {
          
        if (dht_read_float_data(SENSOR_TYPE, CONFIG_EXAMPLE_DATA_GPIO, &umidade, &temperatura) == ESP_OK)
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
    
    while(true){
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

    // xTaskCreate(dht11_task, "dht11_task", configMINIMAL_STACK_SIZE * 6, NULL, 2, NULL);
    // xTaskCreatePinnedToCore(pressao_task, "pressao_task", configMINIMAL_STACK_SIZE * 6, NULL, 2, NULL, APP_CPU_NUM);
    // TODO > COLOCAR A TAREFA DE CONECTIVIDADE PARA OUTRO CORE
    xTaskCreate(mqtt_task, "mqtt_task", configMINIMAL_STACK_SIZE * 6, NULL, 2, NULL);

    vTaskDelay(5000/portTICK_PERIOD_MS);
}

void button_check(){
    if (rtc_gpio_get_level(BOTAO) == 0)
    {
        // printf("Aguardando soltar o botão ... \n");
        do
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        } while (rtc_gpio_get_level(BOTAO) == 0);
    }
}

void config_sleep_and_button(){
    // Configuração do pino como entrada
    gpio_config_t config = {
        .pin_bit_mask = BIT64(BOTAO),               //gpio pin to use for wakeup
        .mode = GPIO_MODE_INPUT,                    //set gpio pin to input mode
        .pull_down_en = false,                      //disable pull down resistor
        .pull_up_en = false,                        //disable pull up resistor
        .intr_type = GPIO_INTR_DISABLE              //disable interrupt
    };
    gpio_config(&config);
    gpio_wakeup_enable(BOTAO, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();

    // Configurando o Sleep Timer (em microsegundos)
    // uint64_t tempo_sono = 2 * (60 * 1000000); // VALOR * (1 MINUTO)
    uint64_t tempo_sono = (TIME_SLEEPING * 1000000); // VALOR * (1 MINUTO)
    esp_sleep_enable_timer_wakeup(tempo_sono);
}

int64_t start_sleep() {
    printf("Entrando em modo Light Sleep\n");
    
    // Configura o modo sleep somente após completar a escrita na UART para finalizar o printf
    uart_wait_tx_idle_polling(CONFIG_ESP_CONSOLE_UART_NUM);

    int64_t tempo_antes_de_dormir = esp_timer_get_time();
    // Entra em modo Light Sleep
    esp_light_sleep_start();
    int64_t tempo_apos_acordar = esp_timer_get_time();
    
    int64_t sleep_time_ms = 0;
    sleep_time_ms = (tempo_apos_acordar - tempo_antes_de_dormir) / 1000;
    
    printf("Dormiu por %lld ms\n", sleep_time_ms);
    return sleep_time_ms;
}