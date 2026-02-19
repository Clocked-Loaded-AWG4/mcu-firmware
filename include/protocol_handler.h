// protocol_handler.h
#ifndef PROTOCOL_HANDLER_H
#define PROTOCOL_HANDLER_H

#include <stdint.h> // For fixed-width integer types like uint16_t
#include <stddef.h> // For size_t

/**
 * @brief Defines the possible types for a transmission payload.
 */
typedef enum {
    TRANSMISSION_TYPE_UNKNOWN = 0,
    TRANSMISSION_TYPE_JSON    = 0x01,
    TRANSMISSION_TYPE_BYTES   = 0x02,
    TRANSMISSION_TYPE_WAVEFORM = 0x03,
    TRANSMISSION_TYPE_TOGGLE  = 0x04
} TransmissionType;

/**
 * @brief Defines error codes for the parsing functions.
 */
typedef enum {
    PROTOCOL_SUCCESS = 0,
    ERROR_NULL_POINTER = -1,
    ERROR_INSUFFICIENT_LENGTH = -2,
    ERROR_UNKNOWN_TRANSMISSION_TYPE = -4,
    ERROR_INVALID_PAYLOAD_FORMAT = -5,
    ERROR_MEMORY_ALLOCATION_FAILED = -6
} ProtocolError;

/**
 * @brief Represents a JSON payload, which is a null-terminated string.
 */
typedef struct {
    char* json_string;
} JsonPayload;

/**
 * @brief Represents a raw bytes payload.
 */
typedef struct {
    uint8_t* data;
    size_t   length;
} BytesPayload;

/**
 * @brief Represents waveform data payload.
 */
typedef struct {
    uint16_t num_points;
    int16_t* data_points; // Array of data points
} WaveformPayload;

/**
 * @brief Represents a toggle payload (single byte value of 0's or 1's (255)).
 */
typedef struct {
    uint8_t value; // 0 or 1
} TogglePayload;

/**
 * @brief A structure to hold the fully parsed packet data.
 * The payload is a union as it can be one of several types.
 */
typedef struct {
    TransmissionType type;
    uint16_t channel; // Reserved field from header
    union {
        JsonPayload     json_payload;
        BytesPayload    bytes_payload;
        WaveformPayload waveform_payload;
        TogglePayload   toggle_payload;
    } payload;
} ParsedPacket;

/**
 * @brief Parses a raw data buffer from a websocket frame.
 *
 * This function acts as the main entry point. It parses the packet header,
 * determines the transmission type, and dispatches to the appropriate
 * payload parser.
 *
 * NOTE: If this function returns PROTOCOL_SUCCESS, the caller is responsible
 * for freeing the parsed packet's memory using `free_parsed_packet`.
 *
 * @param buffer The raw byte buffer received from the websocket.
 * @param buffer_size The total size of the buffer.
 * @param out_packet A pointer to a ParsedPacket structure to be filled with parsed data.
 * @return A ProtocolError code indicating success or the type of failure.
 */
ProtocolError parse_websocket_frame(const uint8_t* buffer, size_t buffer_size, ParsedPacket* out_packet);

/**
 * @brief Frees any dynamically allocated memory within a ParsedPacket structure.
 *
 * This must be called after you are finished with a successfully parsed packet
 * to prevent memory leaks.
 *
 * @param packet A pointer to the ParsedPacket to clean up.
 */
void free_parsed_packet(ParsedPacket* packet);

/**
 * @brief Converts a ProtocolError enum to a human-readable string.
 *
 * @param error The error code.
 * @return A constant string describing the error.
 */
const char* protocol_error_to_string(ProtocolError error);


#endif // PROTOCOL_HANDLER_H