/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "tcpip_adapter.h"
#include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

//mpu6050
#include "freertos/queue.h"
#include "freertos/FreeRTOSConfig.h"
#include "driver/gpio.h"

#include <driver/i2c.h>
#include <math.h>
#include "sdkconfig.h"

#include <cJSON.h>

#define PIN_SDA 21
#define PIN_CLK 22
#define I2C_ADDRESS 0x68 // I2C address of MPU6050

#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_DLPF 0x1A
#define QUEUE_LENGTH 100

#define CUTOFF_260_HZ 0x00
#define CUTOFF_21_HZ 0x04
#define CUTOFF_10_HZ 0x05

#define ACCELL_2G 0x1C
#define ACCEL_CONFIG 0x08

#define GYRO_COFNIG 0x1B
#define GYRO_540 0x10

#define PORT 3333

static const char *TAG = "example";

//queue for sensor data
QueueHandle_t dataQueue = NULL;

//struct for queue and sensor data
typedef struct {
	short accel_x;
	short accel_y;
	short accel_z;
	short temp;
	short gyro_x;
	short gyro_y;
	short gyro_z;
	int8_t rssi;
} data_struct;


static void tcp_server_task(void *pvParameters)
{
	data_struct data_rec;
	int queueEmpty;
    char rx_buffer[128];
    sprintf(rx_buffer, "hello");
    char addr_str[128];
    int addr_family;
    int ip_protocol;

    while (1) {

#ifdef CONFIG_EXAMPLE_IPV4
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
        inet_ntoa_r(dest_addr.sin_addr, addr_str, sizeof(addr_str) - 1);
#else
        struct sockaddr_in6 dest_addr;
        bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
        inet6_ntoa_r(dest_addr.sin6_addr, addr_str, sizeof(addr_str) - 1);
#endif

        int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (listen_sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket created");

        int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        err = listen(listen_sock, 1);
        if (err != 0) {
            ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_in6 source_addr; // Large enough for both IPv4 or IPv6
        uint addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }
        ESP_LOGI(TAG, "Socket accepted");
        uint32_t count = 0;

        while (1) {

        	queueEmpty = xQueueReceive(dataQueue, &data_rec, 0);
        	if (queueEmpty == pdTRUE) {
        		sprintf(rx_buffer,"%d,%d,%d,%d,%d,%d,%d,%d,%d\n",data_rec.accel_x,data_rec.accel_y,data_rec.accel_z,data_rec.temp,data_rec.gyro_x,data_rec.gyro_y,data_rec.gyro_z,data_rec.rssi,count);
				count++;
        		int err = send(sock, rx_buffer, 55, 0);
				if (err < 0) {
					ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
					break;
				}
				memset(rx_buffer, '\0', sizeof(rx_buffer));
        	}
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
            break;
            //we need to somehow return to the top of the code -maybe a while loop here and try to make hte socket available again
        }

    }
    vTaskDelete(NULL);
}

int32_t twos_complement_to_integer(uint8_t data_1, uint8_t data_2) {
	int32_t raw = ((data_1 << 8) | data_2);
	if (raw >= (1<<15)) {
		raw -= (1<<16);
	}
	return raw;
}

//sensor reading stuff
#undef ESP_ERROR_CHECK
#define ESP_ERROR_CHECK(x)   do { esp_err_t rc = (x); if (rc != ESP_OK) { ESP_LOGE("err", "esp_err_t = %d", rc); assert(0 && #x);} } while(0);

void task_mpu6050(void *ignore) {
	ESP_LOGD(TAG, ">> mpu6050");
	i2c_config_t conf;
	conf.mode = I2C_MODE_MASTER;
	conf.sda_io_num = PIN_SDA;
	conf.scl_io_num = PIN_CLK;
	conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
	conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
	conf.master.clk_speed = 100000;
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
	ESP_ERROR_CHECK(i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0));

	i2c_cmd_handle_t cmd;
	vTaskDelay(200/portTICK_PERIOD_MS);

	//I should really make this into a functinon
	//will do soon
	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd, MPU6050_PWR_MGMT_1, 1);
	i2c_master_write_byte(cmd, 0, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd, MPU6050_DLPF, 1);
	i2c_master_write_byte(cmd, CUTOFF_10_HZ, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd, ACCEL_CONFIG, 1);
	i2c_master_write_byte(cmd, ACCELL_2G, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	//config gyro
	cmd = i2c_cmd_link_create();
	ESP_ERROR_CHECK(i2c_master_start(cmd));
	ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
	i2c_master_write_byte(cmd, GYRO_COFNIG, 1);
	i2c_master_write_byte(cmd, GYRO_540, 1);
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);


	uint8_t data[14];

	data_struct sensor_data;
	data_struct received_data;
	wifi_ap_record_t ap;


	int current_time = esp_timer_get_time();
	int previous_time = current_time;


	while(1) {
		// Tell the MPU6050 to position the internal register pointer to register
		// MPU6050_ACCEL_XOUT_H.
		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_WRITE, 1));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, MPU6050_ACCEL_XOUT_H, 1));
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
		i2c_cmd_link_delete(cmd);

		cmd = i2c_cmd_link_create();
		ESP_ERROR_CHECK(i2c_master_start(cmd));
		ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_MASTER_READ, 1));

		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data,   0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+1, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+2, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+3, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+4, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+5, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+6, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+7, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+8, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+9, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+10, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+11, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+12, 0));
		ESP_ERROR_CHECK(i2c_master_read_byte(cmd, data+13, 1));

		//i2c_master_read(cmd, data, sizeof(data), 1);
		ESP_ERROR_CHECK(i2c_master_stop(cmd));
		ESP_ERROR_CHECK(i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000/portTICK_PERIOD_MS));
		i2c_cmd_link_delete(cmd);

		sensor_data.accel_x = twos_complement_to_integer(data[0], data[1]);
		sensor_data.accel_y = twos_complement_to_integer(data[2],data[3]);
		sensor_data.accel_z = twos_complement_to_integer(data[4],data[5]);
		sensor_data.temp = twos_complement_to_integer(data[6],data[7]);
		sensor_data.gyro_x = twos_complement_to_integer(data[8],data[9]);
		sensor_data.gyro_y = twos_complement_to_integer(data[10],data[11]);
		sensor_data.gyro_z = twos_complement_to_integer(data[12],data[13]);

		esp_wifi_sta_get_ap_info(&ap);
		sensor_data.rssi = ap.rssi;
		//printf("%d, %d, %d, %d, %d, %d, %d \n", sensor_data.accel_x, sensor_data.accel_y, sensor_data.accel_z, sensor_data.temp, sensor_data.gyro_x, sensor_data.gyro_y, sensor_data.gyro_z);
		//add to queue and if the queue is full

		if (xQueueSend(dataQueue, &sensor_data, 0) == errQUEUE_FULL) {
			xQueueReceive(dataQueue, &received_data, 0); //remove an item from the queue
			xQueueSend(dataQueue, &sensor_data, 0); //try again if this fails lol fuck
			//printf("%d, %d, %d, %d, %d, %d, %d \n", received_data.accel_x, received_data.accel_y, received_data.accel_z, received_data.temp, received_data.gyro_x, received_data.gyro_y, received_data.gyro_z);
			//printf("queue overflow");
		}
		vTaskDelay(10/portTICK_RATE_MS);


		//measure here
		current_time = esp_timer_get_time();
		//printf("%d\n", (current_time-previous_time));
		previous_time = current_time;

	}

	vTaskDelete(NULL);
}


void app_main()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
    dataQueue = xQueueCreate(QUEUE_LENGTH, sizeof(data_struct));
	xTaskCreate(task_mpu6050, "task_mpu6050", 4096, NULL,5,NULL);
}
