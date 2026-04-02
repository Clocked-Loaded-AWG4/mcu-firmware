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
    uint16_t samples[MAX_SAMPLES];
    uint16_t sample_count;


    //bool     freq_valid;
    uint32_t freq_hz;        // Quantized frequency in Hz for FPGA
} channel_state_t;


static channel_state_t chanA = { .freq_hz = 1000 }; // startup frequency is 1000
static channel_state_t chanB = { .freq_hz = 1000 };
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
    spi_bridge_clear_error();
    ESP_LOGI(TAG, "SPI Bridge initialized");
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


    ESP_LOGI(TAG, "Channel %s: stored %u samples",
             (ch == SPI_CH_A) ? "A" : "B", sample_count);

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
    if (sample_rate > 4294967295.0) { //4294967295 is max uint32_t
        sample_rate = 4294967295.0;
    }

    c->freq_hz = (uint32_t)lround(sample_rate); //this is crazy
    //c->freq_valid = true;

    ESP_LOGI(TAG,
        "Channel %s: waveform=%.3f Hz, samples=%u → sample_rate=%lu Hz",
        (ch == SPI_CH_A) ? "A" : "B", frequency_hz, c->sample_count, (unsigned long)c->freq_hz);
}


void spi_bridge_set_frequency_points(
    spi_channel_t ch,
    int numsamples
)
{

    channel_state_t *c = (ch == SPI_CH_A) ? &chanA : &chanB;

    // if (c->sample_count == 0) {
    //     set_errdoor("Frequency set before waveform (sample_count=0)");
    //     return;
    // }

    double freq = c->freq_hz / c->sample_count;

    double sample_rate = numsamples * freq;

    /* Clamp, not really needed since the sample rate at max frequency (100,000 Hz) and max samples (200) is well within uint32_t range */
    if (sample_rate > 4294967295.0) { //4294967295 is max uint32_t
        sample_rate = 4294967295.0;
    }

    c->freq_hz = (uint32_t)lround(sample_rate); //this is crazy
    //c->freq_valid = true;

    ESP_LOGI(TAG,
        "Channel %s: waveform=%.3f Hz, samples=%u → sample_rate=%lu Hz",
        (ch == SPI_CH_A) ? "A" : "B", freq, c->sample_count, (unsigned long)c->freq_hz);
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
    uint32_t f = c->freq_hz;
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

static void send_complete_spi_frame(spi_channel_t ch, channel_state_t *c)
{
    if (!c->waveform_valid) { // || !c->freq_valid
        set_error("Channel %s: no valid waveform or frequency to send", (ch == SPI_CH_A) ? "A" : "B");
        return;
    }

    uint8_t frame_buffer[MAX_FRAME_SIZE];
    int frame_len = build_complete_spi_frame(ch, c, frame_buffer);

    ESP_LOGI(TAG, "Sending complete frame (%d bytes) for channel %s",
             frame_len, (ch == SPI_CH_A) ? "A" : "B");
    ESP_LOG_BUFFER_HEX(TAG, frame_buffer, frame_len);

    send_spi_frame_bytes(frame_buffer, frame_len);

    // Clear flags after sending
    //c->waveform_valid = false;
    //c->freq_valid = false;
}


/* =========================
   PUBLIC PROCESS FUNCTIONS
   ========================= */


void spi_bridge_process(void)
{
    // Send frequency first, then waveform if both pending
    // if (chanA.freq_valid) {
    //     send_freq_spi_frame(SPI_CH_A, &chanA);
    // }
    if (chanA.waveform_valid) {
        send_complete_spi_frame(SPI_CH_A, &chanA);
    }


    // if (chanB.freq_valid) {
    //     send_freq_spi_frame(SPI_CH_B, &chanB);
    // }
    if (chanB.waveform_valid) {
        send_complete_spi_frame(SPI_CH_B, &chanB);
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
