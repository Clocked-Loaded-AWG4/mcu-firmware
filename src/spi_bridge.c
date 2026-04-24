// spi_bridge.c
#include "spi_bridge.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>
#include <stdlib.h>  // For malloc/free
#include "esp_log.h"
#include "driver/spi_master.h"


static const char *TAG = "SPI_BRIDGE";


/* =========================
   CONFIG / LIMITS
   ========================= */
#define MAX_SAMPLES     256   // FPGA uses 8-bit address, so max 256 samples
#define MAX_FRAME_SIZE  (5 + 2 + (2 * MAX_SAMPLES) + 4)  // Wave header + count + samples + frequency
#define FREQ_FRAME_SIZE (5 + 4)  // SOF + CMD + CH + LEN(2) + 4-byte freq


/* =========================
   PROTOCOL CONSTANTS (MUST match buffer_control.v)
   ========================= */
#define SOF_MARKER    0xA5
#define CMD_SET_FREQ  0x10   // From buffer_control.v [file:1]
#define CMD_LOAD_WAVE 0x20   // From buffer_control.v [file:1]
#define CMD_FULL_FRAME 0x30 


/* =========================
   PER-CHANNEL STATE
   ========================= */
typedef struct {
    bool     waveform_valid;
    bool     dirty;
    uint16_t samples[MAX_SAMPLES];
    uint16_t sample_count;

    double   frequency_hz;
    uint32_t sample_rate_hz;
} channel_state_t;


static channel_state_t chanA;
static channel_state_t chanB;
static char last_error[256] = "";


// External SPI device handle (provided by main.c)
extern spi_device_handle_t fpga_spi;


/* =========================
   ERROR LOGGING
   ========================= */
static void set_error(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vsnprintf(last_error, sizeof(last_error), fmt, args);
    va_end(args);
    ESP_LOGE(TAG, "%s", last_error);
}


const char* spi_bridge_get_last_error(void)
{
    return last_error;
}


void spi_bridge_clear_error(void)
{
    last_error[0] = '\0';
}


/* =========================
   INITIALIZATION
   ========================= */
void spi_bridge_init(void)
{
    memset(&chanA, 0, sizeof(chanA));
    memset(&chanB, 0, sizeof(chanB));
    chanA.frequency_hz = 1000.0;
    chanB.frequency_hz = 1000.0;
    spi_bridge_clear_error();
    ESP_LOGI(TAG, "SPI Bridge initialized");
}


void spi_bridge_init_channel(
    spi_channel_t ch,
    double initial_frequency_hz
)
{
    if (initial_frequency_hz < 0.0) {
        set_error("Invalid initial frequency: %.3f", initial_frequency_hz);
        return;
    }

    channel_state_t *c = (ch == SPI_CH_A) ? &chanA : &chanB;
    c->frequency_hz = initial_frequency_hz;
    c->sample_rate_hz = (uint32_t)lround(c->frequency_hz * (double)200.0);
    c->dirty = false;

    ESP_LOGI(
        TAG,
        "Channel %s initialized: frequency=%.3f Hz, sample_rate=%lu Hz",
        (ch == SPI_CH_A) ? "A" : "B",
        c->frequency_hz,
        (unsigned long)c->sample_rate_hz
    );
}


/* =========================
   INPUT VALIDATION
   ========================= */
static bool validate_waveform(const uint16_t *samples, uint16_t sample_count)
{
    if (samples == NULL && sample_count > 0) {
        set_error("NULL pointer to samples");
        return false;
    }


    if (sample_count == 0 || sample_count > MAX_SAMPLES) {
        set_error("Invalid sample count: %u (must be 1-%u)", sample_count, MAX_SAMPLES);
        return false;
    }


    return true;
}


/* =========================
   DATA SETTERS
   ========================= */


void spi_bridge_set_waveform(
    spi_channel_t ch,
    const uint16_t *samples,
    uint16_t sample_count
)
{
    if (!validate_waveform(samples, sample_count)) {
        return;
    }


    channel_state_t *c = (ch == SPI_CH_A) ? &chanA : &chanB;


    memcpy(c->samples, samples, sample_count * sizeof(uint16_t));
    c->sample_count   = sample_count;
    c->waveform_valid = true;
    c->dirty = true;

    double sample_rate = c->frequency_hz * (double)c->sample_count;
    if (sample_rate > 4294967295.0) {
        sample_rate = 4294967295.0;
    }
    c->sample_rate_hz = (uint32_t)lround(sample_rate);


    ESP_LOGI(TAG, "Channel %s: stored %u samples, sample_rate=%lu Hz",
             (ch == SPI_CH_A) ? "A" : "B", sample_count, (unsigned long)c->sample_rate_hz);

    //spi_bridge_set_frequency(ch, c->freq_hz);
}


void spi_bridge_set_frequency(
    spi_channel_t ch,
    double frequency_hz
)
{
    if (frequency_hz < 0.0) {
        set_error("Negative frequency not allowed: %.3f", frequency_hz);
        return;
    }

    channel_state_t *c = (ch == SPI_CH_A) ? &chanA : &chanB;

    // if (c->sample_count == 0) {
    //     set_error("Frequency set before waveform (sample_count=0)");
    //     return;
    // }

    double sample_rate = frequency_hz * (double)c->sample_count;

    /* Clamp, not really needed since the sample rate at max frequency (100,000 Hz) and max samples (200) is well within uint32_t range */
    if (sample_rate > 1000000.0) { // 1 MHz is max update of DAC
        sample_rate = 1000000.0;
    }

    c->frequency_hz = frequency_hz;
    c->sample_rate_hz = (uint32_t)lround(sample_rate);
    c->dirty = true;

    ESP_LOGI(TAG,
        "Channel %s: waveform=%.3f Hz, samples=%u → sample_rate=%lu Hz",
        (ch == SPI_CH_A) ? "A" : "B", frequency_hz, c->sample_count, (unsigned long)c->sample_rate_hz);
}

/* =========================
   INTERNAL: BUILD FRAMES
   ========================= */


// static int build_wave_spi_frame(spi_channel_t ch, channel_state_t *c, uint8_t *frame_buffer)
// {
//     uint16_t payload_len = 2 + (2 * c->sample_count);
//     int idx = 0;


//     // Header
//     frame_buffer[idx++] = SOF_MARKER;
//     frame_buffer[idx++] = CMD_LOAD_WAVE;
//     frame_buffer[idx++] = (ch == SPI_CH_A) ? 0x00 : 0x01;
//     frame_buffer[idx++] = (payload_len >> 8) & 0xFF;
//     frame_buffer[idx++] = (payload_len >> 0) & 0xFF;


//     // Sample count
//     frame_buffer[idx++] = (c->sample_count >> 8) & 0xFF;
//     frame_buffer[idx++] = (c->sample_count >> 0) & 0xFF;


//     // Samples
//     for (uint16_t i = 0; i < c->sample_count; i++) {
//         uint16_t sample = c->samples[i];
//         frame_buffer[idx++] = (sample >> 8) & 0xFF;
//         frame_buffer[idx++] = (sample >> 0) & 0xFF;
//     }


//     return idx;
// }


// static int build_freq_spi_frame(spi_channel_t ch, channel_state_t *c, uint8_t *frame_buffer)
// {
//     // CMD_SET_FREQ payload is 4 bytes: uint32 freq_hz, big-endian [file:1]
//     uint16_t payload_len = 4;
//     int idx = 0;


//     frame_buffer[idx++] = SOF_MARKER;        // 0xA5
//     frame_buffer[idx++] = CMD_SET_FREQ;      // 0x10
//     frame_buffer[idx++] = (ch == SPI_CH_A) ? 0x00 : 0x01;  // CHAN
//     frame_buffer[idx++] = (payload_len >> 8) & 0xFF;       // LEN_HI
//     frame_buffer[idx++] = (payload_len >> 0) & 0xFF;       // LEN_LO


//     // Big-endian uint32 freq_hz
//     uint32_t f = c->freq_hz;
//     frame_buffer[idx++] = (f >> 24) & 0xFF;
//     frame_buffer[idx++] = (f >> 16) & 0xFF;
//     frame_buffer[idx++] = (f >>  8) & 0xFF;
//     frame_buffer[idx++] = (f >>  0) & 0xFF;


//     return idx;
// }

static int build_complete_spi_frame(spi_channel_t ch, channel_state_t *c, uint8_t *frame_buffer)
{
    uint16_t payload_len = 2 + (2 * c->sample_count) + 4;
    int idx = 0;


    // Header
    frame_buffer[idx++] = SOF_MARKER;
    frame_buffer[idx++] = CMD_FULL_FRAME;
    frame_buffer[idx++] = (ch == SPI_CH_A) ? 0x00 : 0x01;
    frame_buffer[idx++] = (payload_len >> 8) & 0xFF;
    frame_buffer[idx++] = (payload_len >> 0) & 0xFF;

    // Big-endian uint32 freq_hz
    uint32_t f = c->sample_rate_hz;
    frame_buffer[idx++] = (f >> 24) & 0xFF;
    frame_buffer[idx++] = (f >> 16) & 0xFF;
    frame_buffer[idx++] = (f >>  8) & 0xFF;
    frame_buffer[idx++] = (f >>  0) & 0xFF;

    // Sample count
    frame_buffer[idx++] = (c->sample_count >> 8) & 0xFF;
    frame_buffer[idx++] = (c->sample_count >> 0) & 0xFF;


    // Samples
    for (uint16_t i = 0; i < c->sample_count; i++) {
        uint16_t sample = c->samples[i];
        frame_buffer[idx++] = (sample >> 8) & 0xFF;
        frame_buffer[idx++] = (sample >> 0) & 0xFF;
    }


    return idx;
}


/* =========================
   INTERNAL: SEND FRAMES
   ========================= */


static void send_spi_frame_bytes(const uint8_t *frame_buffer, int frame_len)
{
    // No dummy_rx needed anymore -- we're doing half-duplex transmit only

    spi_transaction_t trans = {
        .flags     = SPI_TRANS_MODE_QIO,  // Quad mode for full functionality
        .tx_buffer = frame_buffer,
        .rx_buffer = NULL,                // Half-duplex: no RX
        .length    = frame_len * 8,       // Length in bits
        .rxlength  = 0                    // No RX length
    };

    esp_err_t ret = spi_device_transmit(fpga_spi, &trans);
    if (ret != ESP_OK) {
        set_error("SPI transmission failed: %d", ret);
        ESP_LOGE(TAG, "SPI transmission details: frame_len=%d, flags=0x%x", frame_len, trans.flags);  // Added for debugging
    } else {
        ESP_LOGI(TAG, "Frame sent successfully (%d bytes)", frame_len);
    }

    // No free(dummy_rx) needed
}


// static void send_wave_spi_frame(spi_channel_t ch, channel_state_t *c)
// {
//     if (!c->waveform_valid) {
//         set_error("Channel %s: no valid waveform", (ch == SPI_CH_A) ? "A" : "B");
//         return;
//     }


//     uint8_t frame_buffer[MAX_FRAME_SIZE];
//     int frame_len = build_wave_spi_frame(ch, c, frame_buffer);


//     ESP_LOGI(TAG, "Sending waveform frame (%d bytes) for channel %s",
//              frame_len, (ch == SPI_CH_A) ? "A" : "B");
//     ESP_LOG_BUFFER_HEX(TAG, frame_buffer, (frame_len < 32) ? frame_len : 32);


//     send_spi_frame_bytes(frame_buffer, frame_len);


//     // Clear flag after a successful attempt
//     //c->waveform_valid = false;
// }


// static void send_freq_spi_frame(spi_channel_t ch, channel_state_t *c)
// {
//     // if (!c->freq_valid) {
//     //     set_error("Channel %s: no valid frequency", (ch == SPI_CH_A) ? "A" : "B");
//     //     return;
//     // }


//     uint8_t frame_buffer[FREQ_FRAME_SIZE];
//     int frame_len = build_freq_spi_frame(ch, c, frame_buffer);


//     ESP_LOGI(TAG, "Sending frequency frame (%d bytes) for channel %s",
//              frame_len, (ch == SPI_CH_A) ? "A" : "B");
//     ESP_LOG_BUFFER_HEX(TAG, frame_buffer, frame_len);


//     send_spi_frame_bytes(frame_buffer, frame_len);


//     // Clear flag after sending
//     //c->freq_valid = false;
// }

static bool send_complete_spi_frame(spi_channel_t ch, channel_state_t *c)
{
    if (!c->waveform_valid) { // || !c->freq_valid
        set_error("Channel %s: no valid waveform or frequency to send", (ch == SPI_CH_A) ? "A" : "B");
        return false;
    }

    uint8_t frame_buffer[MAX_FRAME_SIZE];
    int frame_len = build_complete_spi_frame(ch, c, frame_buffer);

    ESP_LOGI(TAG, "Sending complete frame (%d bytes) for channel %s",
             frame_len, (ch == SPI_CH_A) ? "A" : "B");
    ESP_LOG_BUFFER_HEX(TAG, frame_buffer, frame_len);

    send_spi_frame_bytes(frame_buffer, frame_len);

    return true;
}


/* =========================
   PUBLIC PROCESS FUNCTIONS
   ========================= */


void spi_bridge_process(void)
{
    if (chanA.dirty) {
        if (send_complete_spi_frame(SPI_CH_A, &chanA)) {
            chanA.dirty = false;
        }
    }


    if (chanB.dirty) {
        if (send_complete_spi_frame(SPI_CH_B, &chanB)) {
            chanB.dirty = false;
        }
    }
}


// void spi_bridge_process_frequency(spi_channel_t ch)
// {
//     channel_state_t *c = (ch == SPI_CH_A) ? &chanA : &chanB;
//     // if (!c->freq_valid) {
//     //     set_error("Channel %s: no pending frequency to send",
//     //               (ch == SPI_CH_A) ? "A" : "B");
//     //     return;
//     // }


//     send_freq_spi_frame(ch, c);
// }
