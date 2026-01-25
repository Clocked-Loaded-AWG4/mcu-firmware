#include "unity.h"
#include "protocol_handler.h"  // Assumes this defines ParsedPacket, parse_websocket_frame, free_parsed_packet, ProtocolError, TRANSMISSION_TYPE_*, etc.

// Test valid JSON packet parsing
void test_parse_json(void) {
    // Buffer: type 0x0001, zeros 0x0000, payload "{\"key\":\"val\"}"
    uint8_t buf[] = {0x00, 0x01, 0x00, 0x00, '{', '"', 'k', 'e', 'y', '"', ':', '"', 'v', 'a', 'l', '"', '}'};
    ParsedPacket packet;
    ProtocolError err = parse_websocket_frame(buf, sizeof(buf), &packet);
    TEST_ASSERT_EQUAL(PROTOCOL_SUCCESS, err);
    TEST_ASSERT_EQUAL(TRANSMISSION_TYPE_JSON, packet.type);
    TEST_ASSERT_EQUAL_STRING("{\"key\":\"val\"}", packet.payload.json_payload.json_string);
    free_parsed_packet(&packet);
}

// Test valid waveform packet parsing (big-endian, 2 points)
void test_parse_waveform(void) {
    // Buffer: type 0x0003, zeros 0x0000, num_points 0x0002, points +32767 (0x7FFF), -32768 (0x8000)
    uint8_t buf[] = {0x00, 0x03, 0x00, 0x00, 0x00, 0x02, 0x7F, 0xFF, 0x80, 0x00};
    ParsedPacket packet;
    ProtocolError err = parse_websocket_frame(buf, sizeof(buf), &packet);
    TEST_ASSERT_EQUAL(PROTOCOL_SUCCESS, err);
    TEST_ASSERT_EQUAL(TRANSMISSION_TYPE_WAVEFORM, packet.type);
    TEST_ASSERT_EQUAL(2, packet.payload.waveform_payload.num_points);
    TEST_ASSERT_EQUAL_INT16(32767, packet.payload.waveform_payload.data_points[0]);
    TEST_ASSERT_EQUAL_INT16(-32768, packet.payload.waveform_payload.data_points[1]);
    free_parsed_packet(&packet);
}

// Test invalid packet (e.g., wrong type)
void test_parse_invalid_type(void) {
    uint8_t buf[] = {0x00, 0xFF, 0x00, 0x00};  // Invalid type 0x00FF
    ParsedPacket packet;
    ProtocolError err = parse_websocket_frame(buf, sizeof(buf), &packet);
    TEST_ASSERT_NOT_EQUAL(PROTOCOL_SUCCESS, err);
}

// Setup/teardown (empty for now)
void setUp(void) {}
void tearDown(void) {}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_parse_json);
    RUN_TEST(test_parse_waveform);
    RUN_TEST(test_parse_invalid_type);
    return UNITY_END();
}