// main.c
#include <stdio.h>
#include <string.h>
#include <stdlib.h>  // For malloc/free
#include <inttypes.h>  // For PRId16
#include <math.h>  // For roundf (optional)
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_mac.h>
#include "driver/gpio.h"  // For RGB LED
#include "driver/spi_master.h"  // For SPI


// Custom protocol handler
#include "protocol_handler.h"


// SPI bridge
#include "spi_bridge.h"


// Forward declarations
static esp_err_t websocket_handler(httpd_req_t *req);
static httpd_handle_t start_webserver(void);
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
void wifi_init_softap(void);


// --- Configuration ---
// Wi-Fi Access Point configuration
#define ESP_WIFI_SSID      "CL_AWG_NET" // The name of the Wi-Fi network
#define ESP_WIFI_PASS      "CL_AWG_NET_A"           // The password for the Wi-Fi network
#define ESP_WIFI_CHANNEL   1                       // Wi-Fi channel
#define MAX_STA_CONN       4                       // Maximum number of connected clients


// SPI Pins
//#define SPI_MOSI 23  // legacy pins
//#define SPI_MISO 19
#define SPI_SCLK 14
#define SPI_CS 15
#define SPI_DQ0 13
#define SPI_DQ1 10
#define SPI_DQ2 12
#define SPI_DQ3 11
#define SPI_FREQ_HZ 5000000  // 5 MHz
#define SPI_MODE 0


// RGB LED Pins
#define LED_R_GPIO 2
#define LED_G_GPIO 4
#define LED_B_GPIO 16

// GPIO Pins for enable lines for each DAC
#define DAC_ENABLE_0 47
#define DAC_ENABLE_1 48


// Logging tag for console output
static const char *TAG = "WEBSOCKET_SERVER";


// SPI device handle
spi_device_handle_t fpga_spi;


// Function to set RGB LED color (1=on, 0=off for each channel)
void set_led_color(uint8_t r, uint8_t g, uint8_t b) {
    gpio_set_level(LED_R_GPIO, r);
    gpio_set_level(LED_G_GPIO, g);
    gpio_set_level(LED_B_GPIO, b);
}


// Flash LED for a duration (ms)
void flash_led(uint8_t r, uint8_t g, uint8_t b, uint32_t duration_ms) {
    set_led_color(r, g, b);
    vTaskDelay(pdMS_TO_TICKS(duration_ms));
    set_led_color(0, 0, 0);  // Off
}


/**
 * @brief Plots a horizontal ASCII representation of the waveform to the console for better readability.
 *
 * @param data Pointer to the array of 16-bit signed waveform points.
 * @param num_points Number of points in the waveform.
 */
void plot_waveform(const int16_t *data, uint32_t num_points) {
    if (num_points == 0) {
        ESP_LOGI(TAG, "No waveform points to plot.");
        return;
    }


    const int height = 10;
    const uint32_t max_cols = 80;
    uint32_t step = (num_points > max_cols) ? (num_points / max_cols) : 1;
    uint32_t plot_points = num_points / step;


    // Find min and max values
    int16_t min_val = INT16_MAX;
    int16_t max_val = INT16_MIN;
    for (uint32_t i = 0; i < num_points; i++) {
        if (data[i] < min_val) min_val = data[i];
        if (data[i] > max_val) max_val = data[i];
    }


    int32_t range = (int32_t)max_val - (int32_t)min_val;
    if (range == 0) {
        ESP_LOGI(TAG, "Constant waveform: %" PRId16, min_val);
        return;
    }


    // Calculate zero level (midpoint for bipolar waves)
    int zero_level = height / 2;
    if (min_val >= 0) zero_level = height;
    else if (max_val <= 0) zero_level = 0;


    // Buffer for each row (extra space for axes)
    char lines[11][100];  // height+1 rows, up to 80 cols + axes + null
    for (int row = 0; row <= height; row++) {
        memset(lines[row], ' ', max_cols + 2);
        lines[row][max_cols + 2] = '\0';  // Null-terminate
    }


    // Plot points and connect with lines
    int prev_y = -1;
    for (uint32_t col = 0; col < plot_points; col++) {
        int16_t val = data[col * step];
        float norm = ((float)val - min_val) / range;
        int y = height - (int)(norm * height + 0.5f);  // Invert: top = max


        // Plot point
        lines[y][col + 1] = '*';  // +1 for y-axis space


        // Connect to previous
        if (prev_y != -1) {
            int start = (prev_y < y) ? prev_y : y;
            int end = (prev_y > y) ? prev_y : y;
            char connect_char = (prev_y > y) ? '\\' : (prev_y < y) ? '/' : '-';
            for (int ly = start + 1; ly < end; ly++) {
                lines[ly][col] = '|';  // Vertical fill between
            }
            lines[start][col] = connect_char;  // Diagonal or horizontal
        }
        prev_y = y;
    }


    // Add axes
    for (int row = 0; row <= height; row++) {
        if (row == 0) lines[row][0] = '^';  // Top
        else if (row == height) lines[row][0] = 'v';  // Bottom
        else lines[row][0] = ' ';  // Spacer
        if (row == zero_level) {
            for (uint32_t col = 1; col <= max_cols; col++) {
                if (lines[row][col] == ' ') lines[row][col] = '-';
            }
        }
    }


    // Log the plot
    ESP_LOGI(TAG, "Waveform plot (min: %" PRId16 ", max: %" PRId16 ", points: %u):", min_val, max_val, num_points);
    for (int row = 0; row <= height; row++) {
        ESP_LOGI(TAG, "%s", lines[row]);
    }
}


/**
 * @brief The main handler for WebSocket traffic.
 *
 * This function is registered with the HTTP server for the "/ws" endpoint.
 * It handles new client connections and incoming data frames.
 */
static esp_err_t websocket_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "Handshake successful, new client connected");
        flash_led(0, 1, 0, 500);  // Flash green on connect
        return ESP_OK;
    }


    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_BINARY; // We expect binary data per the protocol


    // First, receive the frame metadata to get the length
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
        return ret;
    }


    // If there's data, allocate a buffer and receive the actual payload
    if (ws_pkt.len > 0) {
        buf = calloc(1, ws_pkt.len);
        if (buf == NULL) {
            ESP_LOGE(TAG, "Failed to allocate memory for websocket frame");
            return ESP_ERR_NO_MEM;
        }
        ws_pkt.payload = buf;
        // Receive the full data frame
        ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "httpd_ws_recv_frame failed with %d", ret);
            free(buf);
            return ret;
        }


        // --- Protocol Parsing ---
        ParsedPacket packet;
        ProtocolError err = parse_websocket_frame(ws_pkt.payload, ws_pkt.len, &packet);


        if (err != PROTOCOL_SUCCESS) {
            ESP_LOGE(TAG, "Protocol parse error: %s", protocol_error_to_string(err));
            flash_led(1, 0, 0, 1000);  // Flash red on error
        } else {
            switch (packet.type) {
                case TRANSMISSION_TYPE_JSON:
                    {
                        char *json = packet.payload.json_payload.json_string;
                        ESP_LOGI(TAG, "Received JSON: %s", json);
                        if (strstr(json, "frequency") != NULL) {
                            uint16_t channel;
                            double freq;
                            // Assume simple JSON without spaces: {"channel":0,"frequency":1000.5}
                            if (sscanf(json, "{\"channel\":%hu,\"frequency\":%lf}", &channel, &freq) == 2) {
                                if (channel > 1) {
                                    ESP_LOGE(TAG, "Invalid channel in JSON: %hu", channel);
                                    flash_led(1, 0, 0, 1000);
                                } else {
                                    spi_channel_t ch = (channel == 0) ? SPI_CH_A : SPI_CH_B;
                                    spi_bridge_set_frequency(ch, freq);
                                    spi_bridge_process_frequency(ch);
                                    const char *err_str = spi_bridge_get_last_error();
                                    if (err_str[0] != '\0') {
                                        ESP_LOGE(TAG, "SPI bridge error: %s", err_str);
                                        flash_led(1, 0, 0, 1000);
                                        spi_bridge_clear_error();
                                    } else {
                                        flash_led(0, 0, 1, 500);  // Blue on success
                                    }
                                }
                            } else {
                                ESP_LOGE(TAG, "Failed to parse frequency JSON: %s", json);
                                flash_led(1, 0, 0, 1000);
                            }
                        }
                    }
                    break;
                case TRANSMISSION_TYPE_BYTES:
                    ESP_LOGI(TAG, "Received bytes (%zu bytes)", packet.payload.bytes_payload.length);
                    ESP_LOG_BUFFER_HEX(TAG, packet.payload.bytes_payload.data, packet.payload.bytes_payload.length);
                    flash_led(0, 1, 0, 500);  // Green flash
                    break;
                case TRANSMISSION_TYPE_WAVEFORM:
                    {
                        WaveformPayload *w = &packet.payload.waveform_payload;
                        plot_waveform(w->data_points, w->num_points);
                        if (w->num_points > 256) {
                            ESP_LOGE(TAG, "Invalid waveform points: %u (max 256)", w->num_points);
                            flash_led(1, 0, 0, 1000);  // Red flash
                        } else {
                            uint16_t channel = packet.channel;
                            if (channel > 1) {
                                ESP_LOGE(TAG, "Invalid channel: %u", channel);
                                flash_led(1, 0, 0, 1000);  // Red flash
                            } else {
                                uint16_t *samples = (uint16_t*)malloc(w->num_points * sizeof(uint16_t));
                                if (samples == NULL) {
                                    ESP_LOGE(TAG, "Failed to allocate samples buffer");
                                    flash_led(1, 0, 0, 1000);
                                } else {
                                    for (uint16_t i = 0; i < w->num_points; i++) {
                                        int16_t val = w->data_points[i];
                                        samples[i] = (uint16_t)(val + 32768);  // Scale -32768..32767 to 0..65535
                                    }
                                    spi_channel_t ch = (channel == 0) ? SPI_CH_A : SPI_CH_B;
                                    spi_bridge_set_waveform(ch, samples, w->num_points);
                                    spi_bridge_process();
                                    const char *err_str = spi_bridge_get_last_error();
                                    if (err_str[0] != '\0') {
                                        ESP_LOGE(TAG, "SPI bridge error: %s", err_str);
                                        flash_led(1, 0, 0, 1000);
                                        spi_bridge_clear_error();
                                    } else {
                                        flash_led(0, 0, 1, 500);  // Blue on success
                                    }
                                    free(samples);
                                }
                            }
                        }
                    }
                    break;
                case TRANSMISSION_TYPE_TOGGLE:
                    {
                        uint8_t value = packet.payload.toggle_payload.value;
                        ESP_LOGI(TAG, "Received toggle value: %u", value);
                        if (value != 0 && value != 255) {
                            ESP_LOGE(TAG, "Invalid toggle value: %u (must be 0 or 255)", value);
                            flash_led(1, 0, 0, 1000);  // Red flash
                        } else {
                            if (packet.channel == 0) {
                                // Toggle DAC 0
                                gpio_set_level(DAC_ENABLE_0, (value == 0) ? 1 : 0);  // Active low
                                ESP_LOGI(TAG, "DAC 0 %s", (value == 0) ? "disabled" : "enabled");
                            } else if (packet.channel == 1) {
                                // Toggle DAC 1
                                gpio_set_level(DAC_ENABLE_1, (value == 0) ? 1 : 0);  // Active low
                                ESP_LOGI(TAG, "DAC 1 %s", (value == 0) ? "disabled" : "enabled");
                            } else {
                                ESP_LOGE(TAG, "Invalid DAC channel for toggle: %u", packet.channel);
                                flash_led(1, 0, 0, 1000);  // Red flash
                                break;
                            }
                            // Handle the toggle action here
                            // For example, enable/disable a feature based on the value
                            ESP_LOGI(TAG, "Toggle action executed for value: %u", value);
                            flash_led(0, 0, 1, 500);  // Blue on success
                        }
                    }
                    break;
                default:
                    ESP_LOGW(TAG, "Unknown transmission type: 0x%04X", packet.type);
                    break;
            }
        }
        // Always free the parsed packet
        free_parsed_packet(&packet);
    }
   
    free(buf); // Free the buffer that held the raw websocket data
    return ESP_OK;
}


/**
 * @brief Starts the HTTP server which hosts our WebSocket endpoint.
 */
static httpd_handle_t start_webserver(void) {
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 591; // Use port 591 for the server


    ESP_LOGI(TAG, "Starting HTTP server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Registering URI handler for /ws");
       
        // Define the WebSocket endpoint
        static const httpd_uri_t ws_uri = {
            .uri        = "/ws",
            .method     = HTTP_GET,
            .handler    = websocket_handler,
            .user_ctx   = NULL,
            .is_websocket = true // This enables WebSocket upgrade
        };
        httpd_register_uri_handler(server, &ws_uri);
        return server;
    }


    ESP_LOGE(TAG, "Error starting server!");
    return NULL;
}


/**
 * @brief Event handler for Wi-Fi events (client connect/disconnect).
 */
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "Client " MACSTR " connected, AID=%d", MAC2STR(event->mac), event->aid);
        flash_led(0, 1, 0, 500);  // Green flash on client connect
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "Client " MACSTR " disconnected, AID=%d", MAC2STR(event->mac), event->aid);
    }
}


/**
 * @brief Initializes Wi-Fi in Soft Access Point (AP) mode.
 */
void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();


    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));


    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));


    wifi_config_t wifi_config = {
        .ap = {
            .ssid = ESP_WIFI_SSID,
            .ssid_len = strlen(ESP_WIFI_SSID),
            .channel = ESP_WIFI_CHANNEL,
            .password = ESP_WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK
        },
    };


    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());


    ESP_LOGI(TAG, "Wi-Fi SoftAP started. SSID: '%s' Password: '%s'",
             ESP_WIFI_SSID, ESP_WIFI_PASS);
}


/**
 * @brief Main application entry point for ESP-IDF.
 */
void app_main(void) {
    // Initialize NVS (Non-Volatile Storage) - required for Wi-Fi
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);


    // Initialize RGB LED GPIOs
    gpio_set_direction(LED_R_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_G_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED_B_GPIO, GPIO_MODE_OUTPUT);

    // Initialize DAC enable GPIOs
    gpio_set_direction(DAC_ENABLE_0, GPIO_MODE_OUTPUT);
    gpio_set_direction(DAC_ENABLE_1, GPIO_MODE_OUTPUT);
    gpio_set_level(DAC_ENABLE_0, 1);  // Start with DACs disabled
    gpio_set_level(DAC_ENABLE_1, 1);

    set_led_color(0, 0, 0);  // Start off


    // Start the Wi-Fi Access Point
    wifi_init_softap();
   
    // Start the web server
    start_webserver();


    // Initialize SPI master for communication with FPGA
    spi_bus_config_t buscfg = {
        .mosi_io_num = -1,
        .miso_io_num = -1,
        .sclk_io_num = SPI_SCLK,
        .data0_io_num = SPI_DQ0,
        .data1_io_num = SPI_DQ1,
        .data2_io_num = SPI_DQ2,
        .data3_io_num = SPI_DQ3,
        .max_transfer_sz = 4096,
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_QUAD | SPICOMMON_BUSFLAG_IOMUX_PINS,
        .intr_flags = 0
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));  // Enable DMA for larger transfers


    spi_device_interface_config_t devcfg = {
        .command_bits = 0,
        .address_bits = 0,
        .dummy_bits = 0,
        .clock_speed_hz = SPI_FREQ_HZ,
        .duty_cycle_pos = 0,
        .mode = SPI_MODE,
        .spics_io_num = SPI_CS,
        .cs_ena_pretrans = 0,
        .cs_ena_posttrans = 0,
        .queue_size = 10,  // Use a reasonable queue size for SPI transactions
        .flags = 0,
        .pre_cb = NULL,
        .post_cb = NULL
    };
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &devcfg, &fpga_spi));


    // Initialize SPI bridge state
    spi_bridge_init();
}


