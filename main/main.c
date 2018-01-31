#include "string.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/portmacro.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "sdkconfig.h"
#include "xtensa/xos_types.h"
#include "driver/i2c.h"
#include "driver/timer.h"
#include "aws_iot_config.h"
#include "aws_iot_log.h"
#include "aws_iot_version.h"
#include "aws_iot_mqtt_client_interface.h"
#include "soc/soc.h"
#include "bme280.h"

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1

#define BME280_FLOAT_ENABLE 1

#define ENV_DATA_QUEUE_SIZE 10

#define BME_TIMER_QUEUE_SIZE 4
#define TIMER_DIVIDER 16 // Hardware timer clock divider
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) //convert counter value to seconds

static const char *TAG = "envmon";

extern const uint8_t root_ca_pem_start[] asm("_binary_root_ca_pem_start");
extern const uint8_t root_ca_pem_end[] asm("_binary_root_ca_pem_end");
extern const uint8_t certificate_pem_crt_start[] asm("_binary_certificate_pem_crt_start");
extern const uint8_t certificate_pem_crt_end[] asm("_binary_certificate_pem_crt_end");
extern const uint8_t private_pem_key_start[] asm("_binary_private_pem_key_start");
extern const uint8_t private_pem_key_end[] asm("_binary_private_pem_key_end");

static EventGroupHandle_t wifi_event_group;

static xQueueHandle bme_read_timer_queue;
static xQueueHandle env_data_queue;

const int CONNECTED_BIT = BIT0;

typedef struct {
	double temperature;
	double pressure;
	double humidity;
} env_data_t;

int8_t bme280_user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Stop       | -                   |
     * | Start      | -                   |
     * | Read       | (reg_data[0])       |
     * | Read       | (....)              |
     * | Read       | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */
    int8_t rslt = 0;
	esp_err_t rc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, reg_addr, true);

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_READ, true);

	if (len > 1) {
		i2c_master_read(cmd, reg_data, len-1, I2C_MASTER_ACK);
	}
	i2c_master_read_byte(cmd, reg_data+len-1, I2C_MASTER_NACK);
	i2c_master_stop(cmd);

	rc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
	if (rc == ESP_OK)
	{
		rslt = 0;
	}
	else
	{
		rslt = FAIL;
	}

	i2c_cmd_link_delete(cmd);

	return rslt;
}

int8_t bme280_user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{
    /*
     * The parameter dev_id can be used as a variable to store the I2C address of the device
     */

    /*
     * Data on the bus should be like
     * |------------+---------------------|
     * | I2C action | Data                |
     * |------------+---------------------|
     * | Start      | -                   |
     * | Write      | (reg_addr)          |
     * | Write      | (reg_data[0])       |
     * | Write      | (....)              |
     * | Write      | (reg_data[len - 1]) |
     * | Stop       | -                   |
     * |------------+---------------------|
     */
    int8_t rslt = 0;
	esp_err_t rc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_id << 1) | I2C_MASTER_WRITE, true);

	i2c_master_write_byte(cmd, reg_addr, true);
	i2c_master_write(cmd, reg_data, len, true);
	i2c_master_stop(cmd);

	rc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
	if (rc == ESP_OK)
	{
		rslt = 0;
	}
	else
	{
		rslt = FAIL;
	}

	i2c_cmd_link_delete(cmd);

    return rslt;
}

void bme280_user_delay_ms(uint32_t period)
{
    /*
     * Return control or wait,
     * for a period amount of milliseconds
     */
	vTaskDelay(period / portTICK_PERIOD_MS);
}

int8_t bme280_read_sensor_data_normal_mode(struct bme280_dev *dev, env_data_t *env_data)
{
	int8_t rslt;
	uint8_t settings_sel;
	struct bme280_data comp_data;

	/* Recommended mode of operation: Indoor navigation */
	dev->settings.osr_h = BME280_OVERSAMPLING_1X;
	dev->settings.osr_p = BME280_OVERSAMPLING_16X;
	dev->settings.osr_t = BME280_OVERSAMPLING_2X;
	dev->settings.filter = BME280_FILTER_COEFF_16;
	dev->settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

	settings_sel = BME280_OSR_PRESS_SEL;
	settings_sel |= BME280_OSR_TEMP_SEL;
	settings_sel |= BME280_OSR_HUM_SEL;
	settings_sel |= BME280_STANDBY_SEL;
	settings_sel |= BME280_FILTER_SEL;
	rslt = bme280_set_sensor_settings(settings_sel, dev);
	if (rslt != BME280_OK)
	{
		ESP_LOGE(TAG, "Unable to set BME280 sensor settings.")
	}
	rslt = bme280_set_sensor_mode(BME280_NORMAL_MODE, dev);
	if (rslt != BME280_OK)
	{
		ESP_LOGE(TAG, "Unable to set BME280 sensor mode.")
	}

	/* Delay while the sensor completes a measurement */
	dev->delay_ms(70);

	rslt = bme280_get_sensor_data(BME280_ALL, &comp_data, dev);

	env_data->temperature = comp_data.temperature / 100.0;
	env_data->pressure = comp_data.pressure / 10000.0;
	env_data->humidity = comp_data.humidity / 1000.0;

	return rslt;
}

void i2c_master_init()
{
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = CONFIG_BME280_I2C_SDA_PIN,
		.scl_io_num = CONFIG_BME280_I2C_SCL_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 1000000
	};
	i2c_param_config(I2C_NUM_0, &i2c_config);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

void IRAM_ATTR bme_read_timer_isr(void *param)
{
    timer_idx_t timer_idx = (timer_idx_t) param;

    // The timer interrupt must be explicitly cleared in the
    // interrupt service routine (ISR) or a core dump will be
    // triggered.
    TIMERG0.int_clr_timers.t0 = 1;
    TIMERG0.hw_timer[timer_idx].config.alarm_en = TIMER_ALARM_EN;

    // I could use direct task notify (vTaskNotifyGiveFromISR())
    // instead of queue to trigger sensor reading, but task
    // notify functions are not explicitly supported in
    // ESP-IDF's configuration of FreeRTOS as of January 2018.
    // There is no instance of direct task notify being used
    // in ESP-IDF code base currently.
    xQueueSendFromISR(bme_read_timer_queue, &param, NULL);
}

void task_bme280_read(void *ignore)
{
	struct bme280_dev dev;
	int8_t rslt;

	i2c_master_init();

	dev.dev_id = CONFIG_BME280_I2C_ADDR;
	dev.intf = BME280_I2C_INTF;
	dev.read = bme280_user_i2c_read;
	dev.write = bme280_user_i2c_write;
	dev.delay_ms = bme280_user_delay_ms;

	rslt = bme280_init(&dev);
	if (rslt != BME280_OK)
	{
		ESP_LOGE(TAG, "Unable to initialize BME280.");
	}

	for ( ;; )
	{
		timer_idx_t timer_idx;
		env_data_t env_data;
		xQueueReceive(bme_read_timer_queue, &timer_idx, portMAX_DELAY);
		rslt = bme280_read_sensor_data_normal_mode(&dev, &env_data);
		if (rslt != SUCCESS)
		{
			ESP_LOGE(TAG, "task_bme280_read(): sensor read failed.");
			continue;
		}

		xQueueSend(env_data_queue, &env_data, portMAX_DELAY);

		ESP_LOGI(TAG, "Temp %0.2f C, Pres %0.2f, Hum %0.2f%%",
				env_data.temperature, env_data.pressure, env_data.humidity);
	}
}

void disconnectCallbackHandler(AWS_IoT_Client *pClient, void *data)
{
	ESP_LOGW(TAG, "MQTT Disconnect");
	IoT_Error_t rc = FAILURE;

	if (NULL == pClient)
	{
		return;
	}

	if (aws_iot_is_autoreconnect_enabled(pClient))
	{
		ESP_LOGI(TAG, "Auto Reconnect is enabled, reconnecting attempt will start now.");
	}
	else
	{
		ESP_LOGW(TAG, "Auto Reconnect is not enabled. Starting manual reconnect...");
		if (NETWORK_RECONNECTED == rc)
		{
			ESP_LOGW(TAG, "Manual Reconnect successful.");
		}
		else
		{
			ESP_LOGW(TAG, "Manual Reconnect failed - %d", rc);
		}
	}
}

void task_mqtt_report(void *params)
{
	char cPayload[200];
    IoT_Error_t rc;

	AWS_IoT_Client client;
	IoT_Client_Init_Params mqttInitParams = iotClientInitParamsDefault;
	IoT_Client_Connect_Params connectParams = iotClientConnectParamsDefault;

	IoT_Publish_Message_Params paramsQOS0;

	mqttInitParams.enableAutoReconnect = true;
	mqttInitParams.pHostURL = CONFIG_MQTT_SERVER_HOST;
	mqttInitParams.port = CONFIG_MQTT_SERVER_PORT;

	mqttInitParams.pRootCALocation = (const char *) root_ca_pem_start;
	mqttInitParams.pDeviceCertLocation = (const char *) certificate_pem_crt_start;
	mqttInitParams.pDevicePrivateKeyLocation = (const char *) private_pem_key_start;

	mqttInitParams.mqttCommandTimeout_ms = 20000;
	mqttInitParams.tlsHandshakeTimeout_ms = 5000;
	mqttInitParams.isSSLHostnameVerify = true;
	mqttInitParams.disconnectHandler = disconnectCallbackHandler;
	mqttInitParams.disconnectHandlerData = NULL;

	rc = aws_iot_mqtt_init(&client, &mqttInitParams);
	if (SUCCESS != rc)
	{
		ESP_LOGE(TAG, "aws_iot_mqtt_init returned error: %d ", rc);
		abort();
	}

	xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY);

	connectParams.keepAliveIntervalInSec = 10;
	connectParams.isCleanSession = true;
	connectParams.MQTTVersion = MQTT_3_1_1;
	connectParams.pClientID = CONFIG_MQTT_CLIENT_ID;
	connectParams.clientIDLen = (uint16_t) strlen(CONFIG_MQTT_CLIENT_ID);
	connectParams.isWillMsgPresent = false;

	ESP_LOGI(TAG, "Connecting to the MQTT server");
	do {
		rc = aws_iot_mqtt_connect(&client, &connectParams);
		if (SUCCESS != rc)
		{
			ESP_LOGE(TAG, "Error(%d) connecting to %s:%d",
					rc, mqttInitParams.pHostURL, mqttInitParams.port);
			vTaskDelay(1000 / portTICK_RATE_MS);
		}
	} while (SUCCESS != rc);

	rc = aws_iot_mqtt_autoreconnect_set_status(&client, true);
	if (SUCCESS != rc)
	{
		ESP_LOGE(TAG, "Unable to set Auto Reconnect to true - %d", rc);
		abort();
	}

	const int TOPIC_LEN = strlen(CONFIG_MQTT_TOPIC);

	paramsQOS0.qos = QOS0;
	paramsQOS0.payload = (void *) cPayload;
	paramsQOS0.isRetained = 0;

	while ((NETWORK_ATTEMPTING_RECONNECT == rc ||
			NETWORK_RECONNECTED == rc ||
			SUCCESS == rc))
	{
		env_data_t env_data;
		BaseType_t queue_rcv_rslt;

		rc = aws_iot_mqtt_yield(&client, 100);
		if (NETWORK_ATTEMPTING_RECONNECT == rc)
		{
			continue;
		}

		queue_rcv_rslt = xQueueReceive(env_data_queue, &env_data, portMAX_DELAY);
		if (queue_rcv_rslt != pdPASS)
		{
			ESP_LOGE(TAG, "Receiving from env_data_queue failed.");
		}

		sprintf(cPayload,
				"{\"temp_c\": %0.2f, \"pressure_hpa\": %0.2f, \"humidity_pct\": %0.2f}",
				env_data.temperature, env_data.pressure, env_data.humidity);
		paramsQOS0.payloadLen = strlen(cPayload);
		rc = aws_iot_mqtt_publish(&client, CONFIG_MQTT_TOPIC, TOPIC_LEN, &paramsQOS0);
	}

	ESP_LOGE(TAG, "An error occurred in the main loop.");
	abort();
}

static esp_err_t event_handler(void *ctx, system_event_t *event)
{
	switch(event->event_id)
	{
	case SYSTEM_EVENT_STA_START:
		tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, "envmon");
		esp_wifi_connect();
		break;
	case SYSTEM_EVENT_STA_GOT_IP:
		xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
		break;
	case SYSTEM_EVENT_STA_DISCONNECTED:
		esp_wifi_connect();
		xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
		break;
	default:
		break;
	}
	return ESP_OK;
}

static void wifi_init(void)
{
	tcpip_adapter_init();
	wifi_event_group = xEventGroupCreate();
	ESP_ERROR_CHECK( esp_event_loop_init(event_handler, NULL) );
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK( esp_wifi_init(&cfg) );
    ESP_ERROR_CHECK( esp_wifi_set_storage(WIFI_STORAGE_RAM) );
    ESP_ERROR_CHECK( esp_wifi_set_mode(WIFI_MODE_STA) );
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_WIFI_SSID,
            .password = CONFIG_WIFI_PASSWORD,
            .bssid_set = false
        }
    };
    ESP_LOGI(TAG, "Setting WiFi configuration SSID %s...", wifi_config.sta.ssid);
    ESP_ERROR_CHECK( esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK( esp_wifi_start() );

}

static void bme_read_timer_init(timer_idx_t timer_idx, double timer_interval_sec)
{
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_EN;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = 1;

    timer_init(TIMER_GROUP_0, timer_idx, &config);

    timer_set_counter_value(TIMER_GROUP_0, timer_idx, 0x00000000ULL);

    timer_set_alarm_value(TIMER_GROUP_0, timer_idx, timer_interval_sec * TIMER_SCALE);
    timer_enable_intr(TIMER_GROUP_0, timer_idx);
    timer_isr_register(TIMER_GROUP_0, timer_idx, bme_read_timer_isr,
        (void *) timer_idx, ESP_INTR_FLAG_IRAM, NULL);

    ESP_ERROR_CHECK( timer_start(TIMER_GROUP_0, timer_idx) );
}

void app_main(void)
{
    nvs_flash_init();
    wifi_init();

    env_data_queue = xQueueCreate(ENV_DATA_QUEUE_SIZE, sizeof(env_data_t));
    if (env_data_queue == NULL)
    {
    	ESP_LOGE(TAG, "Failed to create env_data_queue.");
    }

    bme_read_timer_queue = xQueueCreate(BME_TIMER_QUEUE_SIZE, sizeof(timer_idx_t));
    if (bme_read_timer_queue == NULL)
    {
    	ESP_LOGE(TAG, "Failed to create bme_read_timer_queue.");
    }

    xTaskCreate(task_bme280_read, "task_bme280_read", 1024 * 8, NULL, 10, NULL);

    bme_read_timer_init(TIMER_0, CONFIG_BME280_READ_INTERVAL_SEC);

    xTaskCreate(&task_mqtt_report, "task_mqtt_report", 1024 * 36, NULL, 5, NULL);
}

