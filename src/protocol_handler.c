// protocol_handler.c
#include "protocol_handler.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// For converting network byte order (big-endian) to host byte order.
// On POSIX systems, this is available in <arpa/inet.h> or <netinet/in.h>.
// On Windows, it's in <winsock2.h>.
#if defined(_WIN32) || defined(_WIN64)
#include <winsock2.h>
#else
#include <arpa/inet.h>
#endif

// Define header size for clarity
#define PACKET_HEADER_SIZE 4

/**
 * @brief Parses a waveform payload from the raw data buffer. IMPORTANT: Allocated memory for data points must be freed by the caller.
 * @param payload Pointer to the start of the payload data.
 * @param payload_size The size of the payload.
 * @param out_waveform Pointer to the WaveformPayload struct to populate.
 * @return A ProtocolError code.
 */
static ProtocolError handle_waveform_payload(const uint8_t* payload, size_t payload_size, WaveformPayload* out_waveform) {
    if (payload_size < 2) {
        return ERROR_INSUFFICIENT_LENGTH;
    }

    // First 2 bytes of payload are the number of data points
    uint16_t num_points;
    memcpy(&num_points, payload, sizeof(uint16_t));
    num_points = ntohs(num_points); // Convert from big-endian

    // Validate that the payload size matches the expected size
    size_t expected_size = sizeof(uint16_t) + (size_t)num_points * sizeof(uint16_t);
    if (payload_size != expected_size) {
        return ERROR_INVALID_PAYLOAD_FORMAT;
    }

    out_waveform->num_points = num_points;

    if (num_points == 0) {
        out_waveform->data_points = NULL;
        return PROTOCOL_SUCCESS;
    }

    // Allocate memory for the data points
    out_waveform->data_points = (uint16_t*)malloc(num_points * sizeof(uint16_t));
    if (!out_waveform->data_points) {
        return ERROR_MEMORY_ALLOCATION_FAILED;
    }

    const uint8_t* data_stream = payload + sizeof(uint16_t);
    for (uint16_t i = 0; i < num_points; ++i) {
        uint16_t point;
        memcpy(&point, data_stream + (i * sizeof(uint16_t)), sizeof(uint16_t));
        out_waveform->data_points[i] = ntohs(point); // Convert each point
    }

    return PROTOCOL_SUCCESS;
}


ProtocolError parse_websocket_frame(const uint8_t* buffer, size_t buffer_size, ParsedPacket* out_packet) {
    if (!buffer || !out_packet) {
        return ERROR_NULL_POINTER;
    }
    
    // A valid packet must have at least a 4-byte header.
    if (buffer_size < PACKET_HEADER_SIZE) {
        return ERROR_INSUFFICIENT_LENGTH;
    }

    // Initialize the output packet
    memset(out_packet, 0, sizeof(ParsedPacket));

    // --- Parse Header ---
    uint16_t type_identifier;
    uint16_t reserved_bytes;

    memcpy(&type_identifier, buffer, sizeof(uint16_t));
    memcpy(&reserved_bytes, buffer + sizeof(uint16_t), sizeof(uint16_t));
    
    // Convert from network byte order (big-endian) to host byte order
    type_identifier = ntohs(type_identifier);
    reserved_bytes = ntohs(reserved_bytes);
    
    // No check for reserved_bytes == 0; use as parameter (e.g., channel)
    
    out_packet->type = (TransmissionType)type_identifier;
    out_packet->channel = reserved_bytes;
    
    // --- Parse Payload ---
    const uint8_t* payload = buffer + PACKET_HEADER_SIZE;
    size_t payload_size = buffer_size - PACKET_HEADER_SIZE;

    switch (out_packet->type) {
        case TRANSMISSION_TYPE_JSON:
            // Payload is a string, so we allocate and copy it.
            // Add 1 for the null terminator.
            out_packet->payload.json_payload.json_string = (char*)malloc(payload_size + 1);
            if (!out_packet->payload.json_payload.json_string) {
                return ERROR_MEMORY_ALLOCATION_FAILED;
            }
            memcpy(out_packet->payload.json_payload.json_string, payload, payload_size);
            out_packet->payload.json_payload.json_string[payload_size] = '\0'; // Ensure null termination
            break;

        case TRANSMISSION_TYPE_BYTES:
            out_packet->payload.bytes_payload.length = payload_size;
            if (payload_size > 0) {
                 out_packet->payload.bytes_payload.data = (uint8_t*)malloc(payload_size);
                if (!out_packet->payload.bytes_payload.data) {
                    return ERROR_MEMORY_ALLOCATION_FAILED;
                }
                memcpy(out_packet->payload.bytes_payload.data, payload, payload_size);
            } else {
                 out_packet->payload.bytes_payload.data = NULL;
            }
            break;

        case TRANSMISSION_TYPE_WAVEFORM:
            // Delegate to the specific waveform handler function
            return handle_waveform_payload(payload, payload_size, &out_packet->payload.waveform_payload);

        case TRANSMISSION_TYPE_TOGGLE:
            // Payload: one byte of either 0's or 1's
            if (payload_size != 1) {
                return ERROR_INVALID_PAYLOAD_FORMAT;
            }
            out_packet->payload.toggle_payload.value = payload[0];
            break;
        default:
            out_packet->type = TRANSMISSION_TYPE_UNKNOWN;
            return ERROR_UNKNOWN_TRANSMISSION_TYPE;
    }

    return PROTOCOL_SUCCESS;
}

void free_parsed_packet(ParsedPacket* packet) {
    if (!packet) {
        return;
    }

    switch (packet->type) {
        case TRANSMISSION_TYPE_JSON:
            free(packet->payload.json_payload.json_string);
            packet->payload.json_payload.json_string = NULL;
            break;
        case TRANSMISSION_TYPE_BYTES:
            free(packet->payload.bytes_payload.data);
            packet->payload.bytes_payload.data = NULL;
            break;
        case TRANSMISSION_TYPE_WAVEFORM:
            free(packet->payload.waveform_payload.data_points);
            packet->payload.waveform_payload.data_points = NULL;
            break;
        case TRANSMISSION_TYPE_TOGGLE:
            // No dynamic memory to free for toggle payload
            break;
        default:
            // No memory was allocated for unknown types
            break;
    }
}


const char* protocol_error_to_string(ProtocolError error) {
    switch(error) {
        case PROTOCOL_SUCCESS:                return "PROTOCOL_SUCCESS";
        case ERROR_NULL_POINTER:              return "ERROR_NULL_POINTER";
        case ERROR_INSUFFICIENT_LENGTH:       return "ERROR_INSUFFICIENT_LENGTH";
        case ERROR_UNKNOWN_TRANSMISSION_TYPE: return "ERROR_UNKNOWN_TRANSMISSION_TYPE";
        case ERROR_INVALID_PAYLOAD_FORMAT:    return "ERROR_INVALID_PAYLOAD_FORMAT";
        case ERROR_MEMORY_ALLOCATION_FAILED:  return "ERROR_MEMORY_ALLOCATION_FAILED";
        default:                              return "UNKNOWN_ERROR";
    }
}
