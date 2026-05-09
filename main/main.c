/* Standard C/C++ Libraries */
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h> // Cần cho asprintf
#include <string.h>

/* FreeRTOS Libraries */
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

/* ESP-IDF Core & System Libraries */
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_system.h"
#include "nvs.h"
#include "nvs_flash.h"

/* ESP-IDF Driver Libraries */
#include "driver/gpio.h"

/* ESP-IDF Network Libraries */
#include "cJSON.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_now.h"
#include "esp_wifi.h"
#include "mqtt_client.h"

/* Custom Component & Sensor Libraries (Giả định đã có) */
#include "RCSwitch.h"
#include "ds18b20.h"
#include "mq2_sensor.h"

// ============================
// --- CONFIGURATION ---
// ============================

// --- General ---
#define DEVICE_ID "TU_1_NHABEP"
#define BUZZ_PIN GPIO_NUM_15
#define LED_PIN GPIO_NUM_2
#define SENSOR_POLL_INTERVAL_MS 2000
#define DATA_PUBLISH_INTERVAL_MS 1000
#define ALARM_EVENT_QUEUE_LENGTH 8
#define GPIO_INTERRUPT_QUEUE_LENGTH 10
#define ESPNOW_HEARTBEAT_INTERVAL_MS 3000

// --- Wi-Fi & MQTT ---
#define WIFI_SSID "life"
#define WIFI_PASS "giangdaika"
#define MQTT_BROKER_URI CONFIG_PBL3_MQTT_BROKER_URI
#define MQTT_USERNAME CONFIG_PBL3_MQTT_USERNAME
#define MQTT_PASSWORD CONFIG_PBL3_MQTT_PASSWORD

// Tên topic được xây dựng dynamic
#define MQTT_TOPIC_DATA_FMT "sensor/%s/data"
#define MQTT_TOPIC_FIRE_FMT "sensor/%s/alert"
#define MQTT_TOPIC_COMMAND_FMT "sensor/%s/command"
#define MQTT_TOPIC_DEVICE_TELEMETRY_FMT "devices/%s/telemetry"
#define MQTT_TOPIC_DEVICE_ALARM_STATE_FMT "devices/%s/alarm/state"
#define MQTT_TOPIC_DEVICE_ALARM_EVENT_FMT "devices/%s/alarm/event"
#define MQTT_TOPIC_DEVICE_COMMAND_FMT "devices/%s/command"
#define MQTT_TOPIC_DEVICE_COMMAND_ACK_FMT "devices/%s/command/ack"
#define MQTT_TOPIC_DEVICE_STATUS_FMT "devices/%s/status"
#define MQTT_LWT_OFFLINE_MSG "{\"online\":false}"

// --- Sensor Thresholds ---
#define TEMP_FIRE_ON_THRESHOLD_C 45.0f
#define TEMP_FIRE_OFF_THRESHOLD_C 42.0f
#define TEMP_VALID_MIN_C 10.0f
#define TEMP_VALID_MAX_C 80.0f
#define TEMP_FILTER_ALPHA_PERCENT 35
#define TEMP_SPIKE_MAX_DELTA_C 6.0f
#define TEMP_SPIKE_CONFIRM_SAMPLES 2
#define GAS_FIRE_ON_THRESHOLD 80
#define GAS_FIRE_OFF_THRESHOLD 50
#define SENSOR_FIRE_CONFIRM_SAMPLES 3
#define SENSOR_CLEAR_CONFIRM_SAMPLES 3

// --- RF Remote Control ---
#define RF_RECEIVER_PIN GPIO_NUM_35
#define LEARN_BUTTON_PIN GPIO_NUM_18
#define DELETE_BUTTON_PIN GPIO_NUM_5
#define NVS_NAMESPACE "storage"
#define MAX_RF_CODES 10

// --- Manual Fire Control ---
#define MANUAL_ALARM_PIN GPIO_NUM_33
#define MANUAL_RESET_PIN GPIO_NUM_25

// ============================
// --- GLOBALS & TYPE DEFS ---
// ============================

static const char *TAG = "GATEWAY_FIRE_SYSTEM";
static const char *STATUS_TAG = "TRẠNG THÁI HỆ THỐNG";

// --- Network & MQTT Configuration (Static Arrays) ---
static char MQTT_TOPIC_DATA[64];
static char MQTT_TOPIC_FIRE[64];
static char MQTT_TOPIC_COMMAND[64];
static char MQTT_TOPIC_DEVICE_TELEMETRY[64];
static char MQTT_TOPIC_DEVICE_ALARM_STATE[64];
static char MQTT_TOPIC_DEVICE_ALARM_EVENT[64];
static char MQTT_TOPIC_DEVICE_COMMAND[64];
static char MQTT_TOPIC_DEVICE_COMMAND_ACK[64];
static char MQTT_TOPIC_DEVICE_STATUS[64];

// --- Network & ESP-NOW ---
// MAC Address của Tủ 2 (Peer) - Cần thay đổi nếu nạp cho Tủ 2
static const uint8_t PEER_MAC[6] = {0xA0, 0xA3, 0xB3, 0xA9, 0xE9, 0x34};

static uint8_t s_local_mac[6] = {0};
static esp_mqtt_client_handle_t mqtt_client = NULL;
static bool mqtt_connected = false;
static bool mqtt_started = false;
static uint8_t last_cmd_sent_espnow = 0xFF;

// --- System State Variables ---
static bool alarm_on_state = false; // Trạng thái báo cháy toàn cục (để hú còi)
static bool reported_local_fire_state =
    false; // Trạng thái báo cháy cục bộ đã gửi lên MQTT

// --- Individual Local Alarm Source States ---
static bool g_temp_gas_fire_state = false;
static bool g_temp_fire_state = false;
static bool g_gas_fire_state = false;
static bool g_rf_triggered_fire_state = false;
static bool g_manual_triggered_fire_state = false;
// [NEW] Biến kích hoạt từ Web
static bool g_web_triggered_fire_state = false;
static bool g_local_alarm_test_state = false;
// [NEW] Latched alarm: sensor fire không tự tắt cho đến khi RESET
static bool g_sensor_latched_fire = false;

// --- RF Control Globals ---
RCSWITCH_t rf_receiver;
unsigned long learned_rf_codes[MAX_RF_CODES] = {0};
int num_learned_codes = 0;
bool is_learning_mode = false;

// --- Data Structures ---
typedef struct {
  float temperature;
  int gas_level;
  bool combined_local_fire; // Tổng hợp các nguồn kích hoạt cục bộ
  bool remote_fire;         // Cảnh báo cháy từ thiết bị khác gửi tới
} sensor_state_t;

typedef struct __attribute__((packed)) {
  uint8_t cmd; // 0 = Safe, 1 = Fire detected
} espnow_payload_t;

typedef struct {
  bool active;
  bool temp_fire;
  bool gas_fire;
  bool rf_fire;
  bool manual_fire;
  bool web_fire;
  bool remote_fire;
  bool sensor_latched;
} alarm_event_t;

// --- Shared Resources ---
static sensor_state_t sensor_data;
static SemaphoreHandle_t data_mutex;
static SemaphoreHandle_t publish_timer_sem;
static QueueHandle_t alarm_event_queue;
static QueueHandle_t gpio_interrupt_queue;
static TimerHandle_t data_publish_timer;
static TaskHandle_t data_publish_task_handle;

// ============================
// --- FORWARD DECLARATIONS ---
// ============================
static void update_and_propagate_alarm_state(void);
static void publish_alarm_state_message(const alarm_event_t *event,
                                        bool retained);
static void publish_alarm_event_message(const alarm_event_t *event);
void init_nvs();
void init_rf_control_pins();
void init_manual_control_pins();
bool is_code_already_learned(unsigned long code_to_check);
void save_new_code(unsigned long new_code);
void load_codes_from_nvs();
void delete_all_codes_from_nvs();

// ============================
// --- HELPER FUNCTIONS ---
// ============================

// Snapshot trạng thái alarm hiện tại (phải gọi trong mutex)
static alarm_event_t snapshot_alarm_event(bool active) {
  return (alarm_event_t){
      .active = active,
      .temp_fire = g_temp_fire_state,
      .gas_fire = g_gas_fire_state,
      .rf_fire = g_rf_triggered_fire_state,
      .manual_fire = g_manual_triggered_fire_state,
      .web_fire = g_web_triggered_fire_state,
      .remote_fire = sensor_data.remote_fire,
      .sensor_latched = g_sensor_latched_fire,
  };
}

// Reset tất cả nguồn latched (phải gọi trong mutex)
static bool reset_all_latched_sources(void) {
  bool had_any = g_rf_triggered_fire_state || g_manual_triggered_fire_state ||
                 g_web_triggered_fire_state || g_local_alarm_test_state ||
                 g_sensor_latched_fire || sensor_data.remote_fire;
  g_rf_triggered_fire_state = false;
  g_manual_triggered_fire_state = false;
  g_web_triggered_fire_state = false;
  g_local_alarm_test_state = false;
  g_sensor_latched_fire = false;
  sensor_data.remote_fire = false;
  return had_any;
}

// ============================
// --- PERIPHERAL INITIALIZATION ---
// ============================

void init_nvs() {
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
      ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
}

void init_rf_control_pins() {
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask =
      (1ULL << LEARN_BUTTON_PIN) | (1ULL << DELETE_BUTTON_PIN);
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 1;
  gpio_config(&io_conf);
}

static void IRAM_ATTR gpio_isr_handler(void *arg) {
  gpio_num_t gpio_num = (gpio_num_t)(uint32_t)arg;
  BaseType_t higher_priority_task_woken = pdFALSE;

  if (gpio_interrupt_queue != NULL) {
    xQueueSendFromISR(gpio_interrupt_queue, &gpio_num,
                      &higher_priority_task_woken);
  }

  if (higher_priority_task_woken == pdTRUE) {
    portYIELD_FROM_ISR();
  }
}

void init_manual_control_pins() {
  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_NEGEDGE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask =
      (1ULL << MANUAL_ALARM_PIN) | (1ULL << MANUAL_RESET_PIN);
  io_conf.pull_down_en = 0;
  io_conf.pull_up_en = 1;
  ESP_ERROR_CHECK(gpio_config(&io_conf));
  ESP_ERROR_CHECK(gpio_isr_handler_add(MANUAL_ALARM_PIN, gpio_isr_handler,
                                       (void *)MANUAL_ALARM_PIN));
  ESP_ERROR_CHECK(gpio_isr_handler_add(MANUAL_RESET_PIN, gpio_isr_handler,
                                       (void *)MANUAL_RESET_PIN));
}

// ============================
// --- RF CONTROL ---
// ============================

bool is_code_already_learned(unsigned long code_to_check) {
  for (int i = 0; i < num_learned_codes; i++) {
    if (learned_rf_codes[i] == code_to_check)
      return true;
  }
  return false;
}

void save_new_code(unsigned long new_code) {
  if (num_learned_codes >= MAX_RF_CODES) {
    ESP_LOGE(TAG, "Cannot learn new code, storage is full!");
    return;
  }
  if (is_code_already_learned(new_code)) {
    ESP_LOGW(TAG, "Code %lu has already been learned.", new_code);
    return;
  }

  nvs_handle_t my_handle;
  if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle) != ESP_OK) {
    ESP_LOGE(TAG, "Error opening NVS handle!");
    return;
  }

  char key[20];
  snprintf(key, sizeof(key), "code_%d", num_learned_codes);

  if (nvs_set_u32(my_handle, key, new_code) == ESP_OK) {
    num_learned_codes++;
    if (nvs_set_i32(my_handle, "code_count", num_learned_codes) == ESP_OK) {
      nvs_commit(my_handle);
      ESP_LOGI(TAG, "Successfully saved new code %lu. Total codes: %d",
               new_code, num_learned_codes);
      load_codes_from_nvs();
    }
  }
  nvs_close(my_handle);
}

void load_codes_from_nvs() {
  nvs_handle_t my_handle;
  if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle) != ESP_OK)
    return;

  int32_t count = 0;
  esp_err_t err = nvs_get_i32(my_handle, "code_count", &count);
  if (err != ESP_OK || count <= 0) {
    num_learned_codes = 0;
  } else if (count > MAX_RF_CODES) {
    ESP_LOGW(TAG, "NVS code_count=%ld exceeds limit %d; clamping.", (long)count,
             MAX_RF_CODES);
    num_learned_codes = MAX_RF_CODES;
  } else {
    num_learned_codes = count;
  }

  if (num_learned_codes > 0) {
    ESP_LOGI(TAG, "Found %d learned RF codes. Loading...", num_learned_codes);
    for (int i = 0; i < num_learned_codes; i++) {
      char key[20];
      snprintf(key, sizeof(key), "code_%d", i);
      uint32_t temp_code = 0;
      nvs_get_u32(my_handle, key, &temp_code);
      learned_rf_codes[i] = temp_code;
      ESP_LOGI(TAG, "  -> Code %d: %lu", i, learned_rf_codes[i]);
    }
  } else {
    ESP_LOGI(TAG, "No learned RF codes found in NVS.");
  }
  nvs_close(my_handle);
}

void delete_all_codes_from_nvs() {
  nvs_handle_t my_handle;
  if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle) != ESP_OK)
    return;

  nvs_erase_all(my_handle);
  nvs_commit(my_handle);
  nvs_close(my_handle);

  memset(learned_rf_codes, 0, sizeof(learned_rf_codes));
  num_learned_codes = 0;
  ESP_LOGW(TAG, "DELETED ALL LEARNED RF CODES!");
}

// ============================
// --- ALARM CONTROL LOGIC ---
// ============================

static void send_fire_alert_espnow(uint8_t fire_flag, bool force) {
  if (!force && fire_flag == last_cmd_sent_espnow)
    return;
  espnow_payload_t tx_payload = {.cmd = fire_flag};
  if (esp_now_send(PEER_MAC, (uint8_t *)&tx_payload, sizeof(tx_payload)) ==
      ESP_OK) {
    ESP_LOGI(TAG, "Sent ESP-NOW message: {cmd: %d, force: %d}", fire_flag,
             force);
    last_cmd_sent_espnow = fire_flag;
  } else {
    ESP_LOGE(TAG, "Failed to send ESP-NOW message.");
  }
}

// --- HÀM CẬP NHẬT TRẠNG THÁI (Quan trọng) ---
static void update_and_propagate_alarm_state(void) {
  // Các cờ (Flag) để lưu hành động cần thực hiện sau khi thoát Mutex
  bool should_publish_mqtt_on = false;
  bool should_publish_mqtt_off = false;
  bool should_send_espnow = false;
  bool should_queue_alarm_event = false;
  bool is_mqtt_connected = false;
  int espnow_payload_val = 0;
  alarm_event_t pending_alarm_event = {0};

  // --- BẮT ĐẦU VÙNG TỚI HẠN ---
  if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
    // B1: Tính toán trạng thái CỤC BỘ (Local)
    // [UPDATE] Thêm g_sensor_latched_fire: alarm không tự tắt cho đến khi RESET
    bool new_combined_local_state =
        g_temp_gas_fire_state || g_sensor_latched_fire ||
        g_rf_triggered_fire_state || g_manual_triggered_fire_state ||
        g_web_triggered_fire_state;

    // B2: Nếu Local thay đổi -> Đánh dấu gửi ESP-NOW
    if (new_combined_local_state != sensor_data.combined_local_fire) {
      sensor_data.combined_local_fire = new_combined_local_state;
      should_send_espnow = true;
      espnow_payload_val = sensor_data.combined_local_fire ? 1 : 0;
    }

    is_mqtt_connected = mqtt_connected;

    // B3: Gửi MQTT Alert CHỈ KHI trạng thái cục bộ (Local) thay đổi
    if (new_combined_local_state != reported_local_fire_state) {
      reported_local_fire_state = new_combined_local_state;
      if (new_combined_local_state) {
        should_publish_mqtt_on = true;
        should_queue_alarm_event = true;
        pending_alarm_event = snapshot_alarm_event(true);
        ESP_LOGW(TAG, "LOCAL ALARM ACTIVATED! Publishing to MQTT.");
      } else {
        should_publish_mqtt_off = true;
        should_queue_alarm_event = true;
        pending_alarm_event = snapshot_alarm_event(false);
        ESP_LOGI(TAG, "LOCAL ALARM DEACTIVATED. Publishing to MQTT.");
      }
    }

    // B4: Tính toán trạng thái TOÀN CỤC (Global = Local OR Remote) ĐỂ BẬT
    // CÒI/ĐÈN
    bool is_global_fire_active =
        sensor_data.combined_local_fire || sensor_data.remote_fire;
    if (is_global_fire_active != alarm_on_state) {
      alarm_on_state = is_global_fire_active;
      ESP_LOGI(TAG, "Global Hardware Alarm State changed to: %s",
               alarm_on_state ? "ON" : "OFF");
    }

    xSemaphoreGive(data_mutex);
  }
  // --- KẾT THÚC VÙNG TỚI HẠN ---

  // --- THỰC HIỆN TÁC VỤ MẠNG (Không blocking Mutex) ---

  // 1. Gửi ESP-NOW
  if (should_send_espnow) {
    send_fire_alert_espnow(espnow_payload_val, false);
  }

  // 2. Push alarm transition detail to queue
  if (should_queue_alarm_event && alarm_event_queue != NULL) {
    if (xQueueSend(alarm_event_queue, &pending_alarm_event, 0) != pdTRUE) {
      ESP_LOGW(TAG, "Alarm event queue is full; dropping transition event.");
    }
  }

  // 3. Gửi MQTT
  if ((should_publish_mqtt_on || should_publish_mqtt_off) &&
      is_mqtt_connected) {
    char *msg;
    if (asprintf(&msg, "{\"alert\":%s}",
                 should_publish_mqtt_on ? "true" : "false") > 0) {
      esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_FIRE, msg, 0, 1, 0);
      free(msg);
    }
    publish_alarm_state_message(&pending_alarm_event, true);
    publish_alarm_event_message(&pending_alarm_event);
  }
}

// ============================
// --- ESP-NOW ---
// ============================

static void espnow_recv_cb(const esp_now_recv_info_t *info, const uint8_t *data,
                           int len) {
  if (memcmp(info->src_addr, s_local_mac, 6) == 0)
    return;
  if (len < sizeof(espnow_payload_t))
    return;

  const espnow_payload_t *rx_payload = (const espnow_payload_t *)data;
  bool new_remote_fire_state = (rx_payload->cmd == 1);
  ESP_LOGI(TAG, "ESP-NOW alert received from peer. Remote fire state: %s",
           new_remote_fire_state ? "ON" : "OFF");

  if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
    sensor_data.remote_fire = new_remote_fire_state;
    xSemaphoreGive(data_mutex);
  }
  update_and_propagate_alarm_state();
}

static esp_err_t espnow_init_and_setup(void) {
  ESP_ERROR_CHECK(esp_now_init());
  ESP_ERROR_CHECK(esp_now_register_recv_cb(espnow_recv_cb));

  esp_now_peer_info_t peer = {0};
  memcpy(peer.peer_addr, PEER_MAC, 6);
  peer.ifidx = ESP_IF_WIFI_STA;
  peer.encrypt = false;

  if (esp_now_add_peer(&peer) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to add ESP-NOW peer");
    return ESP_FAIL;
  }
  return ESP_OK;
}

static void espnow_heartbeat_task(void *pvParameters) {
  (void)pvParameters;

  while (1) {
    bool local_fire_snapshot = false;

    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
      local_fire_snapshot = sensor_data.combined_local_fire;
      xSemaphoreGive(data_mutex);
    }

    send_fire_alert_espnow(local_fire_snapshot ? 1 : 0, true);
    vTaskDelay(pdMS_TO_TICKS(ESPNOW_HEARTBEAT_INTERVAL_MS));
  }
}

// ============================
// --- MQTT & WIFI ---
// ============================

static const char *bool_json(bool value) { return value ? "true" : "false"; }

static bool mqtt_topic_matches(const char *topic, int topic_len,
                               const char *expected_topic) {
  return expected_topic != NULL && topic_len == strlen(expected_topic) &&
         strncmp(topic, expected_topic, topic_len) == 0;
}

static bool mqtt_payload_is_json_object(const char *data, int len) {
  for (int i = 0; i < len; i++) {
    if (!isspace((unsigned char)data[i])) {
      return data[i] == '{';
    }
  }
  return false;
}

static void copy_trimmed_payload(char *dst, size_t dst_size, const char *data,
                                 int len) {
  int start = 0;
  int end = len;

  while (start < end && isspace((unsigned char)data[start])) {
    start++;
  }
  while (end > start && isspace((unsigned char)data[end - 1])) {
    end--;
  }

  int copy_len = end - start;
  if (copy_len >= (int)dst_size) {
    copy_len = dst_size - 1;
  }
  if (copy_len > 0) {
    memcpy(dst, data + start, copy_len);
  }
  dst[copy_len] = '\0';
}

static void publish_mqtt_status(bool online) {
  if (mqtt_client == NULL || MQTT_TOPIC_DEVICE_STATUS[0] == '\0') {
    return;
  }

  const char *msg = online ? "{\"online\":true}" : MQTT_LWT_OFFLINE_MSG;
  esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_DEVICE_STATUS, msg, 0, 1, 1);
}

static void publish_mqtt_command_ack(const char *cmd_id, const char *action,
                                     const char *status, const char *reason) {
  if (mqtt_client == NULL || MQTT_TOPIC_DEVICE_COMMAND_ACK[0] == '\0' ||
      !mqtt_connected) {
    return;
  }

  char *msg = NULL;
  int len =
      asprintf(&msg,
               "{\"device_id\":\"%s\",\"cmd_id\":\"%s\",\"action\":\"%s\","
               "\"status\":\"%s\",\"reason\":\"%s\"}",
               DEVICE_ID, cmd_id ? cmd_id : "", action ? action : "",
               status ? status : "accepted", reason ? reason : "ok");
  if (len > 0) {
    esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_DEVICE_COMMAND_ACK, msg, 0,
                            1, 0);
    free(msg);
  }
}

static void publish_alarm_state_message(const alarm_event_t *event,
                                        bool retained) {
  if (mqtt_client == NULL || MQTT_TOPIC_DEVICE_ALARM_STATE[0] == '\0' ||
      event == NULL || !mqtt_connected) {
    return;
  }

  char *msg = NULL;
  int len = asprintf(&msg,
                     "{\"device_id\":\"%s\",\"alarm_active\":%s,"
                     "\"sources\":{\"temp\":%s,\"gas\":%s,\"rf\":%s,"
                     "\"manual\":%s,\"web\":%s,\"remote\":%s}}",
                     DEVICE_ID, bool_json(event->active),
                     bool_json(event->temp_fire), bool_json(event->gas_fire),
                     bool_json(event->rf_fire), bool_json(event->manual_fire),
                     bool_json(event->web_fire), bool_json(event->remote_fire));
  if (len > 0) {
    esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_DEVICE_ALARM_STATE, msg, 0,
                            1, retained ? 1 : 0);
    free(msg);
  }
}

static void publish_alarm_event_message(const alarm_event_t *event) {
  if (mqtt_client == NULL || MQTT_TOPIC_DEVICE_ALARM_EVENT[0] == '\0' ||
      event == NULL || !mqtt_connected) {
    return;
  }

  char *msg = NULL;
  int len =
      asprintf(&msg,
               "{\"device_id\":\"%s\",\"event\":\"%s\",\"alarm_active\":%s,"
               "\"sources\":{\"temp\":%s,\"gas\":%s,\"rf\":%s,"
               "\"manual\":%s,\"web\":%s,\"remote\":%s}}",
               DEVICE_ID, event->active ? "alarm_on" : "alarm_off",
               bool_json(event->active), bool_json(event->temp_fire),
               bool_json(event->gas_fire), bool_json(event->rf_fire),
               bool_json(event->manual_fire), bool_json(event->web_fire),
               bool_json(event->remote_fire));
  if (len > 0) {
    esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_DEVICE_ALARM_EVENT, msg, 0,
                            1, 0);
    free(msg);
  }
}

static void publish_current_alarm_state(void) {
  alarm_event_t event = {0};

  if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
    event = snapshot_alarm_event(sensor_data.combined_local_fire);
    xSemaphoreGive(data_mutex);
  }

  publish_alarm_state_message(&event, true);
}

static void parse_mqtt_command_payload(const char *data, int len, char *action,
                                       size_t action_size, char *cmd_id,
                                       size_t cmd_id_size, bool *bad_json) {
  action[0] = '\0';
  cmd_id[0] = '\0';
  *bad_json = false;

  if (!mqtt_payload_is_json_object(data, len)) {
    copy_trimmed_payload(action, action_size, data, len);
    return;
  }

  cJSON *root = cJSON_ParseWithLength(data, len);
  if (root == NULL) {
    *bad_json = true;
    return;
  }

  cJSON *action_item = cJSON_GetObjectItemCaseSensitive(root, "action");
  cJSON *cmd_id_item = cJSON_GetObjectItemCaseSensitive(root, "cmd_id");

  if (cJSON_IsString(action_item) && action_item->valuestring != NULL) {
    snprintf(action, action_size, "%s", action_item->valuestring);
  }
  if (cJSON_IsString(cmd_id_item) && cmd_id_item->valuestring != NULL) {
    snprintf(cmd_id, cmd_id_size, "%s", cmd_id_item->valuestring);
  }

  cJSON_Delete(root);
}

static void handle_mqtt_command(const char *data, int len) {
  char action[40];
  char cmd_id[48];
  bool bad_json = false;
  bool state_changed = false;
  bool should_update_alarm = false;
  bool should_publish_status = false;
  const char *ack_status = "accepted";
  const char *ack_reason = "ok";

  parse_mqtt_command_payload(data, len, action, sizeof(action), cmd_id,
                             sizeof(cmd_id), &bad_json);
  if (bad_json) {
    publish_mqtt_command_ack(cmd_id, "unknown", "rejected", "bad_json");
    return;
  }

  if (strcmp(action, "LED_ON") == 0 || strcmp(action, "ALARM_ON") == 0) {
    snprintf(action, sizeof(action), "%s", "system_alarm_on");
  } else if (strcmp(action, "LED_OFF") == 0 ||
             strcmp(action, "ALARM_OFF") == 0) {
    snprintf(action, sizeof(action), "%s", "system_alarm_off");
  } else if (strcmp(action, "alarm_test_on") == 0) {
    snprintf(action, sizeof(action), "%s", "local_alarm_test_on");
  } else if (strcmp(action, "alarm_test_off") == 0) {
    snprintf(action, sizeof(action), "%s", "local_alarm_test_off");
  }

  if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
    if (strcmp(action, "local_alarm_test_on") == 0) {
      if (!g_local_alarm_test_state) {
        g_local_alarm_test_state = true;
        state_changed = true;
      }
      should_publish_status = true;
      ack_reason = "local_alarm_test_enabled";
    } else if (strcmp(action, "local_alarm_test_off") == 0) {
      if (g_local_alarm_test_state) {
        g_local_alarm_test_state = false;
        state_changed = true;
      }
      should_publish_status = true;
      ack_reason = "local_alarm_test_cleared";
    } else if (strcmp(action, "system_alarm_on") == 0 ||
               strcmp(action, "alarm_on") == 0) {
      if (!g_web_triggered_fire_state) {
        g_web_triggered_fire_state = true;
        state_changed = true;
      }
      should_update_alarm = true;
      ack_reason = "system_alarm_enabled";
    } else if (strcmp(action, "system_alarm_off") == 0 ||
               strcmp(action, "alarm_off") == 0) {
      if (g_web_triggered_fire_state) {
        g_web_triggered_fire_state = false;
        state_changed = true;
      }
      should_update_alarm = true;
      ack_reason = "system_alarm_cleared";
    } else if (strcmp(action, "reset_latched_alarm") == 0) {
      state_changed = reset_all_latched_sources();
      should_update_alarm = true;
      should_publish_status = true;
      ack_reason = g_temp_gas_fire_state
                       ? "latched_sources_cleared_sensor_sources_still_active"
                       : "latched_sources_cleared";
    } else if (strcmp(action, "request_status") == 0) {
      should_publish_status = true;
      ack_reason = "status_requested";
    } else {
      ack_status = "rejected";
      ack_reason = "unsupported_action";
    }
    xSemaphoreGive(data_mutex);
  }

  if (strcmp(ack_status, "accepted") == 0) {
    ESP_LOGI(TAG, "MQTT command accepted: action=%s changed=%d reason=%s",
             action, state_changed, ack_reason);
  } else {
    ESP_LOGW(TAG, "MQTT command rejected: action=%s reason=%s", action,
             ack_reason);
  }

  publish_mqtt_command_ack(cmd_id, action, ack_status, ack_reason);

  if (should_update_alarm) {
    update_and_propagate_alarm_state();
  }
  if (should_publish_status) {
    publish_current_alarm_state();
    if (publish_timer_sem != NULL) {
      xSemaphoreGive(publish_timer_sem);
    }
  }
}

static void data_publish_timer_cb(TimerHandle_t xTimer) {
  (void)xTimer;
  if (publish_timer_sem != NULL) {
    xSemaphoreGive(publish_timer_sem);
  }
}

static void alarm_event_monitor_task(void *pvParameters) {
  (void)pvParameters;
  uint32_t alarm_event_count = 0;
  alarm_event_t event;

  while (1) {
    if (xQueueReceive(alarm_event_queue, &event, portMAX_DELAY) == pdTRUE) {
      alarm_event_count++;
      ESP_LOGI(TAG,
               "Alarm queue event #%lu: state=%s Src[T:%d G:%d RF:%d M:%d W:%d "
               "R:%d]",
               (unsigned long)alarm_event_count, event.active ? "ON" : "OFF",
               event.temp_fire, event.gas_fire, event.rf_fire,
               event.manual_fire, event.web_fire, event.remote_fire);
    }
  }
}

static void mqtt_event_handler(void *args, esp_event_base_t base,
                               int32_t event_id, void *event_data) {
  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
  if (event->event_id == MQTT_EVENT_CONNECTED) {
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
      mqtt_connected = true;
      xSemaphoreGive(data_mutex);
    }
    ESP_LOGI(TAG, "MQTT client connected. Subscribing to commands...");
    if (MQTT_TOPIC_COMMAND[0] != '\0') {
      esp_mqtt_client_subscribe(mqtt_client, MQTT_TOPIC_COMMAND, 1);
    }
    if (MQTT_TOPIC_DEVICE_COMMAND[0] != '\0') {
      esp_mqtt_client_subscribe(mqtt_client, MQTT_TOPIC_DEVICE_COMMAND, 1);
    }
    publish_mqtt_status(true);
    publish_current_alarm_state();
    if (data_publish_task_handle != NULL) {
      vTaskResume(data_publish_task_handle);
      xSemaphoreGive(publish_timer_sem);
    }
  } else if (event->event_id == MQTT_EVENT_DISCONNECTED) {
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
      mqtt_connected = false;
      xSemaphoreGive(data_mutex);
    }
    ESP_LOGW(TAG, "MQTT client disconnected.");
  } else if (event->event_id == MQTT_EVENT_DATA) {
    if (mqtt_topic_matches(event->topic, event->topic_len,
                           MQTT_TOPIC_COMMAND) ||
        mqtt_topic_matches(event->topic, event->topic_len,
                           MQTT_TOPIC_DEVICE_COMMAND)) {
      handle_mqtt_command(event->data, event->data_len);
    }
  }
}

static bool mqtt_credentials_need_update(void) {
  return MQTT_USERNAME[0] == '\0' || MQTT_PASSWORD[0] == '\0' ||
         strcmp(MQTT_USERNAME, "YOUR_HIVEMQ_USERNAME") == 0 ||
         strcmp(MQTT_PASSWORD, "YOUR_HIVEMQ_PASSWORD") == 0;
}

static esp_err_t mqtt_app_init(void) {
  snprintf(MQTT_TOPIC_DATA, sizeof(MQTT_TOPIC_DATA), MQTT_TOPIC_DATA_FMT,
           DEVICE_ID);
  snprintf(MQTT_TOPIC_FIRE, sizeof(MQTT_TOPIC_FIRE), MQTT_TOPIC_FIRE_FMT,
           DEVICE_ID);
  snprintf(MQTT_TOPIC_COMMAND, sizeof(MQTT_TOPIC_COMMAND),
           MQTT_TOPIC_COMMAND_FMT, DEVICE_ID);
  snprintf(MQTT_TOPIC_DEVICE_TELEMETRY, sizeof(MQTT_TOPIC_DEVICE_TELEMETRY),
           MQTT_TOPIC_DEVICE_TELEMETRY_FMT, DEVICE_ID);
  snprintf(MQTT_TOPIC_DEVICE_ALARM_STATE, sizeof(MQTT_TOPIC_DEVICE_ALARM_STATE),
           MQTT_TOPIC_DEVICE_ALARM_STATE_FMT, DEVICE_ID);
  snprintf(MQTT_TOPIC_DEVICE_ALARM_EVENT, sizeof(MQTT_TOPIC_DEVICE_ALARM_EVENT),
           MQTT_TOPIC_DEVICE_ALARM_EVENT_FMT, DEVICE_ID);
  snprintf(MQTT_TOPIC_DEVICE_COMMAND, sizeof(MQTT_TOPIC_DEVICE_COMMAND),
           MQTT_TOPIC_DEVICE_COMMAND_FMT, DEVICE_ID);
  snprintf(MQTT_TOPIC_DEVICE_COMMAND_ACK, sizeof(MQTT_TOPIC_DEVICE_COMMAND_ACK),
           MQTT_TOPIC_DEVICE_COMMAND_ACK_FMT, DEVICE_ID);
  snprintf(MQTT_TOPIC_DEVICE_STATUS, sizeof(MQTT_TOPIC_DEVICE_STATUS),
           MQTT_TOPIC_DEVICE_STATUS_FMT, DEVICE_ID);

  ESP_LOGI(TAG, "MQTT Legacy Data Topic: %s", MQTT_TOPIC_DATA);
  ESP_LOGI(TAG, "MQTT Telemetry Topic: %s", MQTT_TOPIC_DEVICE_TELEMETRY);
  ESP_LOGI(TAG, "MQTT Alarm State Topic: %s", MQTT_TOPIC_DEVICE_ALARM_STATE);
  ESP_LOGI(TAG, "MQTT Command Topic: %s", MQTT_TOPIC_DEVICE_COMMAND);
  ESP_LOGI(TAG, "MQTT Legacy Command Topic: %s", MQTT_TOPIC_COMMAND);
  if (mqtt_credentials_need_update()) {
    ESP_LOGE(TAG, "HiveMQ username/password are not configured yet.");
    ESP_LOGE(TAG, "Run: idf.py menuconfig -> PBL3 HiveMQ Configuration");
  }

  uint8_t mac[6];
  char client_id[48];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  snprintf(client_id, sizeof(client_id), "pbl3-%s-%02X%02X%02X", DEVICE_ID,
           mac[3], mac[4], mac[5]);

  const esp_mqtt_client_config_t mqtt_cfg = {
      .broker =
          {
              .address.uri = MQTT_BROKER_URI,
              .verification.crt_bundle_attach = esp_crt_bundle_attach,
          },
      .credentials =
          {
              .client_id = client_id,
              .username = MQTT_USERNAME,
              .authentication.password = MQTT_PASSWORD,
          },
      .session =
          {
              .last_will =
                  {
                      .topic = MQTT_TOPIC_DEVICE_STATUS,
                      .msg = MQTT_LWT_OFFLINE_MSG,
                      .qos = 1,
                      .retain = 1,
                  },
          },
  };
  ESP_LOGI(TAG, "MQTT Broker URI: %s", MQTT_BROKER_URI);
  ESP_LOGI(TAG, "MQTT Client ID: %s", client_id);

  mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
  if (mqtt_client == NULL) {
    ESP_LOGE(TAG, "Failed to initialize MQTT client.");
    return ESP_FAIL;
  }

  return esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID,
                                        mqtt_event_handler, NULL);
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t event_id, void *data) {
  if (event_id == WIFI_EVENT_STA_START ||
      event_id == WIFI_EVENT_STA_DISCONNECTED) {
    esp_wifi_connect();
  } else if (event_id == IP_EVENT_STA_GOT_IP) {
    ESP_LOGI(TAG, "Wi-Fi connected.");
    if (mqtt_client != NULL && !mqtt_started) {
      esp_err_t err = esp_mqtt_client_start(mqtt_client);
      if (err == ESP_OK) {
        mqtt_started = true;
        ESP_LOGI(TAG, "MQTT client started.");
      } else {
        ESP_LOGE(TAG, "Failed to start MQTT client: %s", esp_err_to_name(err));
      }
    }
  }
}

static void wifi_init_sta(void) {
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL));
  wifi_config_t wifi_cfg = {.sta = {.ssid = WIFI_SSID,
                                    .password = WIFI_PASS,
                                    .threshold.authmode = WIFI_AUTH_WPA2_PSK}};
  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_cfg));
  ESP_ERROR_CHECK(esp_wifi_start());
}

// ============================
// --- TASKS ---
// ============================

static bool filter_temperature_sample(float raw_temp, float *filtered_temp) {
  static bool initialized = false;
  static float filtered = 0.0f;
  static uint8_t spike_count = 0;

  if (raw_temp <= TEMP_VALID_MIN_C || raw_temp >= TEMP_VALID_MAX_C) {
    return false;
  }

  if (!initialized) {
    filtered = raw_temp;
    initialized = true;
    spike_count = 0;
    *filtered_temp = filtered;
    return true;
  }

  float delta = raw_temp - filtered;
  float abs_delta = (delta >= 0.0f) ? delta : -delta;
  if (abs_delta > TEMP_SPIKE_MAX_DELTA_C) {
    if (++spike_count < TEMP_SPIKE_CONFIRM_SAMPLES) {
      ESP_LOGW(TAG, "Ignored DS18B20 temperature spike: raw=%.2f filtered=%.2f",
               raw_temp, filtered);
      *filtered_temp = filtered;
      return true;
    }
    spike_count = 0;
  } else {
    spike_count = 0;
  }

  filtered +=
      (raw_temp - filtered) * ((float)TEMP_FILTER_ALPHA_PERCENT / 100.0f);
  *filtered_temp = filtered;
  return true;
}

static bool update_confirmed_sensor_state(bool *state, bool candidate_state,
                                          uint8_t *fire_candidate_count,
                                          uint8_t *clear_candidate_count) {
  if (candidate_state == *state) {
    *fire_candidate_count = 0;
    *clear_candidate_count = 0;
    return false;
  }

  uint8_t *candidate_count =
      candidate_state ? fire_candidate_count : clear_candidate_count;
  uint8_t confirm_samples = candidate_state ? SENSOR_FIRE_CONFIRM_SAMPLES
                                            : SENSOR_CLEAR_CONFIRM_SAMPLES;

  if (*candidate_count < confirm_samples) {
    (*candidate_count)++;
  }

  if (*candidate_count < confirm_samples) {
    return false;
  }

  *state = candidate_state;
  *fire_candidate_count = 0;
  *clear_candidate_count = 0;
  return true;
}

void temp_gas_sensor_task(void *pvParameters) {
  uint8_t temp_fire_candidate_count = 0;
  uint8_t temp_clear_candidate_count = 0;
  uint8_t gas_fire_candidate_count = 0;
  uint8_t gas_clear_candidate_count = 0;

  while (1) {
    float raw_temp = ds18b20_read_temp();
    float temp = 0.0f;
    bool temp_valid = filter_temperature_sample(raw_temp, &temp);
    int gas = mq2_read_value();

    if (!temp_valid) {
      ESP_LOGW(TAG, "Invalid DS18B20 temperature: %.2f", raw_temp);
      temp_fire_candidate_count = 0;
      temp_clear_candidate_count = 0;
    }

    bool temp_source_changed = false;
    bool gas_source_changed = false;
    bool temp_gas_state_changed = false;
    bool temp_gas_snapshot = false;
    bool temp_fire_snapshot = false;
    bool gas_fire_snapshot = false;

    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
      sensor_data.temperature = temp_valid ? temp : sensor_data.temperature;
      sensor_data.gas_level = gas;

      if (temp_valid) {
        bool temp_fire_candidate = g_temp_fire_state
                                       ? temp >= TEMP_FIRE_OFF_THRESHOLD_C
                                       : temp >= TEMP_FIRE_ON_THRESHOLD_C;
        temp_source_changed = update_confirmed_sensor_state(
            &g_temp_fire_state, temp_fire_candidate, &temp_fire_candidate_count,
            &temp_clear_candidate_count);
      }

      bool gas_fire_candidate = g_gas_fire_state ? gas >= GAS_FIRE_OFF_THRESHOLD
                                                 : gas >= GAS_FIRE_ON_THRESHOLD;
      gas_source_changed = update_confirmed_sensor_state(
          &g_gas_fire_state, gas_fire_candidate, &gas_fire_candidate_count,
          &gas_clear_candidate_count);

      bool new_temp_gas_fire_state = g_temp_fire_state || g_gas_fire_state;
      temp_gas_state_changed = temp_source_changed || gas_source_changed ||
                               new_temp_gas_fire_state != g_temp_gas_fire_state;
      if (temp_gas_state_changed) {
        g_temp_gas_fire_state = new_temp_gas_fire_state;
        // [FIX] Latch BÊN TRONG mutex — tránh race condition
        if (g_temp_gas_fire_state && !g_sensor_latched_fire) {
          g_sensor_latched_fire = true;
        }
      }

      temp_gas_snapshot = g_temp_gas_fire_state;
      temp_fire_snapshot = g_temp_fire_state;
      gas_fire_snapshot = g_gas_fire_state;
      xSemaphoreGive(data_mutex);
    }

    if (temp_gas_state_changed) {
      if (g_sensor_latched_fire) {
        ESP_LOGW(
            TAG,
            "SENSOR FIRE LATCHED! Will not auto-clear until manual RESET.");
      }
      ESP_LOGW(TAG,
               "Temp/Gas state: %s | temp_src=%d gas_src=%d temp=%.2f gas=%d",
               temp_gas_snapshot ? "DETECTED" : "CLEARED", temp_fire_snapshot,
               gas_fire_snapshot, temp_valid ? temp : raw_temp, gas);
      update_and_propagate_alarm_state();
    } else {
      ESP_LOGD(TAG, "Temp/Gas stable: temp_src=%d gas_src=%d temp=%.2f gas=%d",
               temp_fire_snapshot, gas_fire_snapshot,
               temp_valid ? temp : raw_temp, gas);
    }

    vTaskDelay(pdMS_TO_TICKS(SENSOR_POLL_INTERVAL_MS));
  }
}

void rf_control_task(void *pvParameters) {
  load_codes_from_nvs();

  while (1) {
    if (gpio_get_level(LEARN_BUTTON_PIN) == 0) {
      is_learning_mode = true;
      vTaskDelay(pdMS_TO_TICKS(500)); // Debounce
    }

    if (gpio_get_level(DELETE_BUTTON_PIN) == 0) {
      delete_all_codes_from_nvs();
      bool state_changed = false;
      if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        if (g_rf_triggered_fire_state) {
          g_rf_triggered_fire_state = false;
          state_changed = true;
        }
        xSemaphoreGive(data_mutex);
      }
      if (state_changed) {
        update_and_propagate_alarm_state();
      }
      vTaskDelay(pdMS_TO_TICKS(500)); // Debounce
    }

    if (available(&rf_receiver)) {
      unsigned long received_code = getReceivedValue(&rf_receiver);
      ESP_LOGI(TAG, "Received RF code: %lu", received_code);

      if (is_learning_mode) {
        save_new_code(received_code);
        is_learning_mode = false;
      } else {
        if (is_code_already_learned(received_code)) {
          ESP_LOGI(TAG, "Matching RF code found!");
          bool state_changed = false;
          if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
            if (!g_rf_triggered_fire_state) {
              g_rf_triggered_fire_state = true;
              state_changed = true;
            }
            xSemaphoreGive(data_mutex);
          }
          if (state_changed) {
            update_and_propagate_alarm_state();
          }
        }
      }
      resetAvailable(&rf_receiver);
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void manual_control_task(void *pvParameters) {
  (void)pvParameters;
  gpio_num_t triggered_gpio;

  while (1) {
    if (xQueueReceive(gpio_interrupt_queue, &triggered_gpio, portMAX_DELAY) !=
        pdTRUE) {
      continue;
    }

    // Debounce after the interrupt. Buttons use pull-up, so pressed = 0.
    vTaskDelay(pdMS_TO_TICKS(20));
    if (gpio_get_level(triggered_gpio) != 0) {
      continue;
    }

    if (triggered_gpio == MANUAL_ALARM_PIN) {
      bool state_changed = false;
      if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        if (!g_manual_triggered_fire_state) {
          g_manual_triggered_fire_state = true;
          state_changed = true;
        }
        xSemaphoreGive(data_mutex);
      }
      if (state_changed) {
        ESP_LOGW(TAG, "MANUAL ALARM TRIGGERED BY GPIO INTERRUPT!");
        update_and_propagate_alarm_state();
      }
    } else if (triggered_gpio == MANUAL_RESET_PIN) {
      ESP_LOGW(TAG, "MANUAL RESET ACTIVATED BY GPIO INTERRUPT! Clearing ALL "
                    "latched alarm states.");

      if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
        reset_all_latched_sources();
        xSemaphoreGive(data_mutex);
      }

      update_and_propagate_alarm_state();
      if (publish_timer_sem != NULL) {
        xSemaphoreGive(publish_timer_sem);
      }
    }

    while (gpio_get_level(triggered_gpio) == 0) {
      vTaskDelay(pdMS_TO_TICKS(50));
    }
  }
}

// --- TASK ĐIỀU KHIỂN CÒI/ĐÈN (CHUẨN BÁO CHÁY THẬT) ---
// Logic: Không phân biệt local/remote — cháy ở đâu cũng báo động toàn bộ
// Pattern: Temporal-3 (NFPA 72) — 3 xung ngắn rồi nghỉ
void alarm_control_task(void *pvParameters) {
  while (1) {
    bool is_alarm_active = false;
    bool is_local_test = false;

    // 1. Lấy trạng thái từ Mutex
    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
      // KHÔNG phân biệt local/remote — alarm_on_state = local OR remote
      is_alarm_active = alarm_on_state;
      is_local_test = g_local_alarm_test_state;
      xSemaphoreGive(data_mutex);
    }

    // 2. XỬ LÝ LOGIC

    if (is_alarm_active || is_local_test) {
      // --- BÁO ĐỘNG: Temporal-3 Pattern (NFPA 72) ---
      // 3 xung (0.5s ON, 0.5s OFF) → nghỉ 1.5s → lặp lại
      for (int pulse = 0; pulse < 3; pulse++) {
        gpio_set_level(LED_PIN, 1);
        gpio_set_level(BUZZ_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(500));

        gpio_set_level(LED_PIN, 0);
        gpio_set_level(BUZZ_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
      }
      // Nghỉ giữa các chu kỳ Temporal-3
      vTaskDelay(pdMS_TO_TICKS(1500));
    }
    // --- BÌNH THƯỜNG ---
    else {
      gpio_set_level(LED_PIN, 0);
      gpio_set_level(BUZZ_PIN, 0);
      vTaskDelay(pdMS_TO_TICKS(250));
    }
  }
}

void data_publish_task(void *pv) {
  char *msg = NULL;

  while (1) {
    if (publish_timer_sem != NULL) {
      xSemaphoreTake(publish_timer_sem, portMAX_DELAY);
    } else {
      vTaskDelay(pdMS_TO_TICKS(DATA_PUBLISH_INTERVAL_MS));
    }

    float current_temp = 0.0f;
    int current_gas = 0;
    bool is_global_alert_active = false;
    bool is_gas_high = false;
    bool is_mqtt_connected = false;
    bool temp_fire_snapshot = false;
    bool gas_fire_snapshot = false;
    bool rf_fire_snapshot = false;
    bool manual_fire_snapshot = false;
    bool web_fire_snapshot = false;
    bool remote_fire_snapshot = false;
    bool local_test_snapshot = false;
    bool sensor_latched_snapshot = false;
    bool local_fire_snapshot = false;

    if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
      is_mqtt_connected = mqtt_connected;
      current_temp = sensor_data.temperature;
      current_gas = sensor_data.gas_level;
      is_global_alert_active = alarm_on_state;
      local_fire_snapshot = sensor_data.combined_local_fire;

      temp_fire_snapshot = g_temp_fire_state;
      gas_fire_snapshot = g_gas_fire_state;
      rf_fire_snapshot = g_rf_triggered_fire_state;
      manual_fire_snapshot = g_manual_triggered_fire_state;
      web_fire_snapshot = g_web_triggered_fire_state;
      remote_fire_snapshot = sensor_data.remote_fire;
      local_test_snapshot = g_local_alarm_test_state;
      sensor_latched_snapshot = g_sensor_latched_fire;
      xSemaphoreGive(data_mutex);
    }

    if (!is_mqtt_connected) {
      ESP_LOGW(TAG, "MQTT disconnected; suspending data publish task.");
      vTaskSuspend(NULL);
      continue;
    }

    is_gas_high = (current_gas >= GAS_FIRE_ON_THRESHOLD);

    ESP_LOGI(STATUS_TAG,
             "Temp: %.1f | Gas: %d | Src[T:%d G:%d RF:%d M:%d W:%d LT:%d R:%d "
             "L:%d] | ==> ALARM: %s",
             current_temp, current_gas, temp_fire_snapshot, gas_fire_snapshot,
             rf_fire_snapshot, manual_fire_snapshot, web_fire_snapshot,
             local_test_snapshot, remote_fire_snapshot, sensor_latched_snapshot,
             is_global_alert_active ? "YES" : "NO");

    // --- Publish detailed data to MQTT ---
    if (is_mqtt_connected && MQTT_TOPIC_DATA[0] != '\0') {
      // Lưu ý: led_status giờ đây phản ánh trạng thái kích hoạt từ web (hoặc
      // báo cháy)
      int len = asprintf(&msg,
                         "{\"id_thiet_bi\":\"%s\",\"nhiet_do\":%.2f,\"khi_ga\":"
                         "\"%s\",\"lua\":%s,\"led_status\":%s}",
                         DEVICE_ID, current_temp, is_gas_high ? "cao" : "thap",
                         local_fire_snapshot ? "true" : "false",
                         (web_fire_snapshot || local_test_snapshot) ? "true"
                                                                    : "false");

      if (len > 0) {
        esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_DATA, msg, 0, 1, 0);
        free(msg);
        msg = NULL;
      }
    }

    if (is_mqtt_connected && MQTT_TOPIC_DEVICE_TELEMETRY[0] != '\0') {
      // [UPDATE] LED/Buzzer phản ánh alarm toàn cục, không phân biệt
      // local/remote
      const char *led_mode = (is_global_alert_active || local_test_snapshot)
                                 ? "alarm_flash"
                                 : "off";
      bool buzzer_active = (is_global_alert_active || local_test_snapshot);

      int len = asprintf(
          &msg,
          "{\"device_id\":\"%s\",\"temperature\":%.2f,\"gas_level\":%d,"
          "\"gas_status\":\"%s\",\"alarm_active\":%s,"
          "\"sources\":{\"temp\":%s,\"gas\":%s,\"rf\":%s,"
          "\"manual\":%s,\"web\":%s,\"remote\":%s,\"sensor_latched\":%s},"
          "\"local_test\":%s,\"buzzer\":%s,\"led_mode\":\"%s\"}",
          DEVICE_ID, current_temp, current_gas, is_gas_high ? "high" : "normal",
          bool_json(local_fire_snapshot), bool_json(temp_fire_snapshot),
          bool_json(gas_fire_snapshot), bool_json(rf_fire_snapshot),
          bool_json(manual_fire_snapshot), bool_json(web_fire_snapshot),
          bool_json(remote_fire_snapshot), bool_json(sensor_latched_snapshot),
          bool_json(local_test_snapshot), bool_json(buzzer_active), led_mode);

      if (len > 0) {
        esp_mqtt_client_publish(mqtt_client, MQTT_TOPIC_DEVICE_TELEMETRY, msg,
                                0, 0, 0);
        free(msg);
        msg = NULL;
      }
    }
  }
}

static void create_task_checked(TaskFunction_t task_func, const char *name,
                                uint32_t stack_depth, void *params,
                                UBaseType_t priority,
                                TaskHandle_t *task_handle) {
  if (xTaskCreate(task_func, name, stack_depth, params, priority,
                  task_handle) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create task: %s", name);
    abort();
  }
}

// ============================
// --- APP MAIN ---
// ============================
void app_main(void) {
  // --- Initialize Core System Services ---
  init_nvs();
  data_mutex = xSemaphoreCreateMutex();
  publish_timer_sem = xSemaphoreCreateBinary();
  alarm_event_queue =
      xQueueCreate(ALARM_EVENT_QUEUE_LENGTH, sizeof(alarm_event_t));
  gpio_interrupt_queue =
      xQueueCreate(GPIO_INTERRUPT_QUEUE_LENGTH, sizeof(gpio_num_t));
  data_publish_timer =
      xTimerCreate("publish_timer", pdMS_TO_TICKS(DATA_PUBLISH_INTERVAL_MS),
                   pdTRUE, NULL, data_publish_timer_cb);

  if (data_mutex == NULL || publish_timer_sem == NULL ||
      alarm_event_queue == NULL || gpio_interrupt_queue == NULL ||
      data_publish_timer == NULL) {
    ESP_LOGE(TAG, "Failed to create required RTOS objects.");
    abort();
  }

  // --- Initialize Peripherals ---
  gpio_reset_pin(BUZZ_PIN);
  gpio_set_direction(BUZZ_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(BUZZ_PIN, 0);
  gpio_reset_pin(LED_PIN);
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_PIN, 0);

  esp_err_t gpio_isr_ret = gpio_install_isr_service(0);
  if (gpio_isr_ret != ESP_OK && gpio_isr_ret != ESP_ERR_INVALID_STATE) {
    ESP_ERROR_CHECK(gpio_isr_ret);
  }

  init_rf_control_pins();
  init_manual_control_pins();

  // --- Initialize Network and Protocols ---
  ESP_ERROR_CHECK(mqtt_app_init());
  ESP_ERROR_CHECK(esp_read_mac(s_local_mac, ESP_MAC_WIFI_STA));
  wifi_init_sta();
  ESP_ERROR_CHECK(espnow_init_and_setup());

  // --- Initialize Sensors ---
  initSwich(&rf_receiver);
  ESP_ERROR_CHECK(enableReceive(&rf_receiver, RF_RECEIVER_PIN));
  ds18b20_init();
  mq2_init();
  ESP_LOGW(TAG, "--- Calibrating MQ2 Sensor... ---");
  mq2_calibrate();
  ESP_LOGI(TAG, "--- MQ2 Calibration complete.");

  // --- Create Application Tasks ---
  create_task_checked(temp_gas_sensor_task, "temp_gas_task", 4096, NULL, 5,
                      NULL);
  create_task_checked(rf_control_task, "rf_control_task", 4096, NULL, 6, NULL);
  create_task_checked(manual_control_task, "manual_control_task", 4096, NULL, 7,
                      NULL);
  create_task_checked(alarm_control_task, "alarm_control_task", 4096, NULL, 4,
                      NULL);
  create_task_checked(espnow_heartbeat_task, "espnow_heartbeat", 2048, NULL, 3,
                      NULL);
  create_task_checked(alarm_event_monitor_task, "alarm_event_task", 2048, NULL,
                      2, NULL);
  create_task_checked(data_publish_task, "data_publish_task", 4096, NULL, 3,
                      &data_publish_task_handle);
  bool is_mqtt_connected = false;
  if (xSemaphoreTake(data_mutex, portMAX_DELAY) == pdTRUE) {
    is_mqtt_connected = mqtt_connected;
    xSemaphoreGive(data_mutex);
  }
  if (!is_mqtt_connected) {
    vTaskSuspend(data_publish_task_handle);
  }
  if (xTimerStart(data_publish_timer, 0) != pdPASS) {
    ESP_LOGE(TAG, "Failed to start data publish timer.");
    abort();
  }

  ESP_LOGI(TAG, "System initialization complete. Web trigger mode active.");
}
