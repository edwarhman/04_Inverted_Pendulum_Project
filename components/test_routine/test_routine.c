#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "system_status.h"
#include "test_routine.h"

static const char *TAG = "TEST_ROUTINE";
static TaskHandle_t test_task_handle = NULL;

static void test_routine_task(void *pvParameters) {
    ESP_LOGI(TAG, "Test routine iniciada. Esperando 5 segundos antes de comenzar...");
    
    // Esperar unos segundos iniciales para permitir estabilización del péndulo real
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // 1. Ir a 20 cm
    ESP_LOGI(TAG, "Inyectando perturbación: Setpoint = 0.2 m (20 cm)");
    status_set_ref_position(0.2f);
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // 2. Ir a -20 cm
    ESP_LOGI(TAG, "Inyectando perturbación: Setpoint = -0.2 m (-20 cm)");
    status_set_ref_position(-0.2f);
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    // 3. Regresar a 0 cm
    ESP_LOGI(TAG, "Fin de rutina. Setpoint = 0.0 m (0 cm)");
    status_set_ref_position(0.0f);
    
    // Detener la tarea (one-shot)
    test_task_handle = NULL;
    ESP_LOGI(TAG, "Tarea de test destruida.");
    vTaskDelete(NULL);
}

void test_routine_start(void) {
    if (test_task_handle == NULL) {
        xTaskCreate(test_routine_task, "Test_Routine", 2048, NULL, 4, &test_task_handle);
    } else {
        ESP_LOGI(TAG, "La rutina ya está en ejecución.");
    }
}
