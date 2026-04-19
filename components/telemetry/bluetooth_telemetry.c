// main/bluetooth_telemetry.c
#include "esp_bt.h"
#include "esp_bt_device.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_log.h"
#include "esp_spp_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include <ctype.h> // Para tolower/toupper
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>


#include "bluetooth_telemetry.h"
#include "button_handler.h"
#include "pid_controller.h"
#include "pulse_counter.h"
#include "state_space_controller.h"
#include "state_space_reducido.h"
#include "system_status.h"
#include "test_routine.h"

#define SPP_SERVER_NAME "SPP_SERVER"
#define CONFIG_BT_DEVICE_NAME "Pendulo_Invertido"

static const char *TAG = "BT_TELEM";
static uint32_t spp_handle = 0;
static bool spp_connected = false;
static bool telemetry_enabled = true;

// Buffer para comandos entrantes
static char cmd_buffer[128];
static int cmd_index = 0;

static void send_bt_response(const char *msg) {
  if (spp_connected && spp_handle != 0) {
    char resp[160];
    int len = snprintf(resp, sizeof(resp), "#%s\r\n", msg);
    esp_spp_write(spp_handle, len, (uint8_t *)resp);
  }
}

static void process_bt_command(char *line) {
  // Eliminar posibles espacios en blanco al final
  int len = strlen(line);
  while (len > 0 && isspace((unsigned char)line[len - 1])) {
    line[--len] = '\0';
  }

  if (len == 0)
    return;

  char *cmd = strtok(line, " ");
  char *val = strtok(NULL, " ");

  if (cmd == NULL)
    return;

  // Convertir comando a mayúsculas para facilitar comparación
  for (int i = 0; cmd[i]; i++)
    cmd[i] = toupper((unsigned char)cmd[i]);

  if (strcmp(cmd, "SETKP") == 0 && val != NULL) {
    float v = atof(val);
    pid_set_kp(v);
    char msg[64];
    snprintf(msg, sizeof(msg), "Sintonizado: Kp = %.4f", v);
    send_bt_response(msg);
  } else if (strcmp(cmd, "SETKI") == 0 && val != NULL) {
    float v = atof(val);
    pid_set_ki(v);
    char msg[64];
    snprintf(msg, sizeof(msg), "Sintonizado: Ki = %.4f", v);
    send_bt_response(msg);
  } else if (strcmp(cmd, "SETKD") == 0 && val != NULL) {
    float v = atof(val);
    pid_set_kd(v);
    char msg[64];
    snprintf(msg, sizeof(msg), "Sintonizado: Kd = %.4f", v);
    send_bt_response(msg);
  } else if (strcmp(cmd, "SETPOS") == 0 && val != NULL) {
    float v = atof(val);
    status_set_ref_position(v);
    char msg[64];
    snprintf(msg, sizeof(msg), "Setpoint Posicion: %.4f m", v);
    send_bt_response(msg);
  } else if (strcmp(cmd, "SETCONTROL") == 0 && val != NULL) {
    if (strcasecmp(val, "pid") == 0) control_switch_mode(MODE_PID);
    else if (strcasecmp(val, "iden") == 0) control_switch_mode(MODE_STATE_SPACE);
    else if (strcasecmp(val, "redu") == 0) control_switch_mode(MODE_STATE_SPACE_RED);
    else if (strcasecmp(val, "func") == 0) control_switch_mode(MODE_STATE_SPACE_FUNC);
    
    char msg[64];
    snprintf(msg, sizeof(msg), "Modo Control: %s", status_get_control_mode_str());
    send_bt_response(msg);
  } else if (strcmp(cmd, "ENABLE") == 0) {
    control_toggle_current();
    bool is_en = is_any_controller_enabled();
    char msg[64];
    snprintf(msg, sizeof(msg), "SISTEMA %s",
             is_en ? "HABILITADO" : "DESHABILITADO");
    send_bt_response(msg);
  } else if (strcmp(cmd, "TELE") == 0) {
    telemetry_enabled = !telemetry_enabled;
    char msg[64];
    snprintf(msg, sizeof(msg), "Telemetria %s",
             telemetry_enabled ? "INICIADA" : "DETENIDA");
    send_bt_response(msg);
  } else if (strcmp(cmd, "STATUS") == 0) {
    char msg[160];
    snprintf(
        msg, sizeof(msg),
        "STATUS: Mode=%s, Kp=%.4f, Ki=%.4f, Kd=%.4f, PosSP=%.4f, Enabled=%d, Tele=%d",
        status_get_control_mode_str(), pid_get_kp(), pid_get_ki(), pid_get_kd(),
        status_get_ref_position(), is_any_controller_enabled(), telemetry_enabled);
    send_bt_response(msg);
  } else if (strcmp(cmd, "CALIBRATE") == 0) {
    if (pid_is_enabled()) {
      send_bt_response("ERROR: Deshabilite PID antes de calibrar");
    } else {
      send_bt_response("Iniciando Calibracion...");
      button_handler_start_calibration();
      send_bt_response("Calibracion Finalizada");
    }
  } else if (strcmp(cmd, "TESTRUN") == 0) {
    if (!is_any_controller_enabled()) {
      send_bt_response("ERROR: Habilite el control antes de ejecutar el test");
    } else {
      send_bt_response("Iniciando Rutina de Prueba...");
      test_routine_start();
    }
  } else if (strcmp(cmd, "HELP") == 0) {
    send_bt_response("Comandos: ENABLE, TELE, STATUS, CALIBRATE, TESTRUN, SETK[P,I,D] "
                     "<val>, SETPOS <m>, SETCONTROL <pid|iden|redu|func>");
  } else {
    char msg[64];
    snprintf(msg, sizeof(msg), "Comando desconocido: %s", cmd);
    send_bt_response(msg);
  }
}

static void esp_spp_cb(esp_spp_cb_event_t event, esp_spp_cb_param_t *param) {
  switch (event) {
  case ESP_SPP_INIT_EVT:
    ESP_LOGI(TAG, "ESP_SPP_INIT_EVT");
    esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE, 0,
                      SPP_SERVER_NAME);
    break;
  case ESP_SPP_DISCOVERY_COMP_EVT:
    ESP_LOGI(TAG, "ESP_SPP_DISCOVERY_COMP_EVT");
    break;
  case ESP_SPP_OPEN_EVT:
    ESP_LOGI(TAG, "ESP_SPP_OPEN_EVT");
    break;
  case ESP_SPP_CLOSE_EVT:
    ESP_LOGI(TAG, "ESP_SPP_CLOSE_EVT");
    spp_connected = false;
    break;
  case ESP_SPP_START_EVT:
    ESP_LOGI(TAG, "ESP_SPP_START_EVT");
    esp_bt_gap_set_device_name(CONFIG_BT_DEVICE_NAME);
    esp_bt_gap_set_scan_mode(ESP_BT_CONNECTABLE, ESP_BT_GENERAL_DISCOVERABLE);
    break;
  case ESP_SPP_CL_INIT_EVT:
    ESP_LOGI(TAG, "ESP_SPP_CL_INIT_EVT");
    break;
  case ESP_SPP_DATA_IND_EVT:
    ESP_LOGI(TAG, "ESP_SPP_DATA_IND_EVT len=%d", param->data_ind.len);
    for (int i = 0; i < param->data_ind.len; i++) {
      char c = param->data_ind.data[i];
      if (c == '\n' || c == '\r') {
        if (cmd_index > 0) {
          cmd_buffer[cmd_index] = '\0';
          process_bt_command(cmd_buffer);
          cmd_index = 0;
        }
      } else if (cmd_index < sizeof(cmd_buffer) - 1) {
        cmd_buffer[cmd_index++] = c;
      }
    }
    break;
  case ESP_SPP_CONG_EVT:
    break;
  case ESP_SPP_WRITE_EVT:
    break;
  case ESP_SPP_SRV_OPEN_EVT:
    ESP_LOGI(TAG, "ESP_SPP_SRV_OPEN_EVT");
    spp_handle = param->srv_open.handle;
    spp_connected = true;
    break;
  case ESP_SPP_SRV_STOP_EVT:
    ESP_LOGI(TAG, "ESP_SPP_SRV_STOP_EVT");
    break;
  case ESP_SPP_UNINIT_EVT:
    ESP_LOGI(TAG, "ESP_SPP_UNINIT_EVT");
    break;
  default:
    break;
  }
}

static void bluetooth_telemetry_task(void *arg) {
  char packet[128];
  while (1) {
    if (spp_connected) {
      uint64_t time_ms = pid_get_run_time_ms();
      if (telemetry_enabled) {
        if (ss_is_enabled()) {
          // tiempo_ms, angulo, posicion, accion_control, velocidad, velocidad_angular
          float theta = ss_get_theta();
          float x_pos = ss_get_x_pos();
          float u_ctrl = ss_get_u_control(); // Control Action
          float x_dot = ss_get_x_dot();
          float theta_dot = ss_get_theta_dot_hat();
          
          int len = snprintf(packet, sizeof(packet), "%llu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
                             time_ms, theta, x_pos, u_ctrl, x_dot, theta_dot, status_get_ref_position());
          if (len > 0) {
            esp_spp_write(spp_handle, len, (uint8_t *)packet);
          }
        } else if (ss_red_is_enabled()) {
          // tiempo_ms, angulo, posicion, accion_control, velocidad, velocidad_angular
          float theta = ss_red_get_theta();
          float x_pos = ss_red_get_x_pos();
          float u_ctrl = ss_red_get_u_control(); // Control Action
          float x_dot = ss_red_get_x_dot();
          float theta_dot = ss_red_get_theta_dot_hat();
          
          int len = snprintf(packet, sizeof(packet), "%llu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
                             time_ms, theta, x_pos, u_ctrl, x_dot, theta_dot, status_get_ref_position());
          if (len > 0) {
            esp_spp_write(spp_handle, len, (uint8_t *)packet);
          }
        } else {
          // tiempo_ms, angulo, posicion, accion_control, velocidad, velocidad_angular
          // tiempo_ms, angulo, posicion, accion_control(acel), velocidad_carro, velocidad_angular
          float theta     = pulse_counter_get_angle_rad();
          float x_pos     = pid_get_car_position_m();
          float u_ctrl    = pid_get_acceleration();      // salida del PID de ángulo (m/s²)
          float vel       = pid_get_velocity();           // velocidad del carro (m/s)
          float theta_dot = pid_get_angular_velocity();  // velocidad angular (rad/s)

          int len = snprintf(packet, sizeof(packet), "%llu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\r\n",
                             time_ms, theta, x_pos, u_ctrl, vel, theta_dot, status_get_ref_position());
          if (len > 0) {
            esp_spp_write(spp_handle, len, (uint8_t *)packet);
          }
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50)); // Enviar cada 50 ms (20 Hz)
  }
}

void bluetooth_telemetry_init(void) {
  esp_err_t ret;

  // Liberamos memoria de BLE que no usaremos si es posible
  ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE);
  if (ret != ESP_OK) {
    ESP_LOGI(TAG,
             "No se pudo liberar memoria BLE (quizas no este habilitado): %s",
             esp_err_to_name(ret));
  }

  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
    ESP_LOGE(TAG, "Error inicializando controlador BT: %s",
             esp_err_to_name(ret));
    return;
  }

  if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
    ESP_LOGE(TAG, "Error habilitando BT clásico: %s", esp_err_to_name(ret));
    return;
  }

  esp_bluedroid_config_t bluedroid_cfg = BT_BLUEDROID_INIT_CONFIG_DEFAULT();
  if ((ret = esp_bluedroid_init_with_cfg(&bluedroid_cfg)) != ESP_OK) {
    ESP_LOGE(TAG, "Error inicializando bluedroid: %s", esp_err_to_name(ret));
    return;
  }

  if ((ret = esp_bluedroid_enable()) != ESP_OK) {
    ESP_LOGE(TAG, "Error habilitando bluedroid: %s", esp_err_to_name(ret));
    return;
  }

  if ((ret = esp_bt_gap_register_callback(NULL)) !=
      ESP_OK) { // No necesitamos GAP callback por ahora
    ESP_LOGE(TAG, "Error registrando GAP cb: %s", esp_err_to_name(ret));
  }

  if ((ret = esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_VARIABLE, 0, NULL)) != ESP_OK) {
    ESP_LOGE(TAG, "Error seteando el pin GAP: %s", esp_err_to_name(ret));
  }

  if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
    ESP_LOGE(TAG, "Error registrando SPP cb: %s", esp_err_to_name(ret));
    return;
  }

  esp_spp_cfg_t bt_spp_cfg = {
      .mode = ESP_SPP_MODE_CB,
      .enable_l2cap_ertm = true,
      .tx_buffer_size = 0,
  };
  if ((ret = esp_spp_enhanced_init(&bt_spp_cfg)) != ESP_OK) {
    ESP_LOGE(TAG, "Error inicializando SPP: %s", esp_err_to_name(ret));
    return;
  }

  // Tarea con poco tamaño de stack necesario
  xTaskCreate(bluetooth_telemetry_task, "bt_telemetry_task", 4096, NULL, 3,
              NULL);

  ESP_LOGI(TAG, "Bluetooth Inicializado exitosamente.");
}
