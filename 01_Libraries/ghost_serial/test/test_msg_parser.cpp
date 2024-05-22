#include "ghost_serial/cobs/cobs.hpp"
#include "ghost_serial/msg_parser/msg_parser.hpp"

#include "gtest/gtest.h"

class TestMsgParser : public ::testing::Test {
protected:

	void SetUp() override {
	}
};

TEST_F(TestMsgParser, testMsgBasic){
	// Msg Configuration
	std::string start_seq = "start";
	int max_msg_len  = 3;
	int true_msg_len = 3;
	int cobs_packet_length = start_seq.length() + true_msg_len + 2;

	// Msg Data
	unsigned char input_buffer[8] = {'s', 't', 'a', 'r', 't', 'm', 's', 'g'};

	// Msg Parser
	auto msg_parser = ghost_serial::MsgParser(max_msg_len, start_seq);

	// Apply COBS Encoding to incoming Msg
	unsigned char encoded_input_buffer[cobs_packet_length] = {0,};
	COBS::cobsEncode(input_buffer, start_seq.length() + true_msg_len, encoded_input_buffer);

	// Parse the stream
	int parsed_msg_len;
	unsigned char output_buffer[max_msg_len] = {0,};
	ASSERT_TRUE(msg_parser.parseByteStream(
					encoded_input_buffer,
					sizeof(encoded_input_buffer) / sizeof(encoded_input_buffer[0]),
					output_buffer,
					parsed_msg_len)
	            );

	// Check that msg is correctly parsed
	unsigned char expected[] = {'m', 's', 'g'};
	for(int i = 0; i < true_msg_len; i++){
		ASSERT_EQ(expected[i], output_buffer[i]);
	}
}

TEST_F(TestMsgParser, testMsgWNoise){
	// Msg Configuration
	std::string start_seq = "start";
	int max_msg_len  = 3;
	int cobs_packet_length = start_seq.length() + max_msg_len + 2;

	// Msg Data
	unsigned char encoded_input_buffer[] = {'f', 'a', 'k', 'e', 0x09, 's', 't', 'a', 'r', 't', 'm', 's', 'g', 0x00};

	// Msg Parser
	auto msg_parser = ghost_serial::MsgParser(max_msg_len, start_seq);

	// Parse the stream
	int parsed_msg_len;
	unsigned char output_buffer[max_msg_len] = {0,};
	msg_parser.parseByteStream(
		encoded_input_buffer,
		sizeof(encoded_input_buffer) / sizeof(encoded_input_buffer[0]),
		output_buffer,
		parsed_msg_len);

	// Check that msg is correctly parsed
	unsigned char expected[] = {'m', 's', 'g'};
	for(int i = 0; i < max_msg_len; i++){
		ASSERT_EQ(expected[i], output_buffer[i]);
	}
}

TEST_F(TestMsgParser, testMsgSplit){
	// Msg Configuration
	std::string start_seq = "st";
	int max_msg_len  = 5;
	int cobs_packet_length = start_seq.length() + max_msg_len + 2;

	// Msg Data
	unsigned char encoded_input_buffer1[] = {0x08, 's', 't', 's', 'p', 'l'};
	unsigned char encoded_input_buffer2[] = {'i', 't', 0x00};

	// Msg Parser
	auto msg_parser = ghost_serial::MsgParser(max_msg_len, start_seq);

	// Parse the stream
	int parsed_msg_len;
	unsigned char output_buffer[max_msg_len] = {0,};
	bool result = msg_parser.parseByteStream(
		encoded_input_buffer1,
		sizeof(encoded_input_buffer1) / sizeof(encoded_input_buffer1[0]),
		output_buffer,
		parsed_msg_len);

	ASSERT_FALSE(result);

	result = msg_parser.parseByteStream(
		encoded_input_buffer2,
		sizeof(encoded_input_buffer2) / sizeof(encoded_input_buffer2[0]),
		output_buffer,
		parsed_msg_len);

	ASSERT_TRUE(result);

	// Check that msg is correctly parsed
	unsigned char expected[] = {'s', 'p', 'l', 'i', 't'};
	for(int i = 0; i < max_msg_len; i++){
		ASSERT_EQ(expected[i], output_buffer[i]);
	}
}

TEST_F(TestMsgParser, testMsgLengthExceeded){
	// Msg Configuration
	std::string start_seq = "start";
	int max_msg_len  = 3;
	int cobs_packet_length = start_seq.length() + max_msg_len + 2;

	// Msg Data
	unsigned char input_buffer1[11] = {0x10, 's', 't', 'a', 'r', 't', 'm', 's', 'g', 'p', 0x00};
	unsigned char input_buffer2[10] = {0x09, 's', 't', 'a', 'r', 't', 'm', 's', 'g', 0x00};

	// Msg Parser
	auto msg_parser = ghost_serial::MsgParser(max_msg_len, start_seq);

	// Parse the stream
	int parsed_msg_len;
	unsigned char output_buffer[max_msg_len] = {0,};
	ASSERT_FALSE(msg_parser.parseByteStream(
					 input_buffer1,
					 sizeof(input_buffer1) / sizeof(input_buffer1[0]),
					 output_buffer,
					 parsed_msg_len)
	             );
}

TEST_F(TestMsgParser, testMsgShortMsg){
	// Msg Configuration
	std::string start_seq = "start";
	int max_msg_len  = 10;
	int msg_len  = 3;
	int cobs_packet_length = start_seq.length() + msg_len + 2;

	// Msg Data
	unsigned char input_buffer[11] = {0x10, 's', 't', 'a', 'r', 't', 'm', 's', 'g', 0x00};

	// Msg Parser
	auto msg_parser = ghost_serial::MsgParser(max_msg_len, start_seq);

	// Parse the stream
	int parsed_msg_len;
	unsigned char output_buffer[max_msg_len] = {0,};
	ASSERT_TRUE(msg_parser.parseByteStream(
					input_buffer,
					sizeof(input_buffer) / sizeof(input_buffer[0]),
					output_buffer,
					parsed_msg_len));

	ASSERT_EQ(parsed_msg_len, msg_len);

	// Check that msg is correctly parsed
	unsigned char expected[] = {'m', 's', 'g'};
	for(int i = 0; i < msg_len; i++){
		ASSERT_EQ(expected[i], output_buffer[i]);
	}
}

TEST_F(TestMsgParser, testChecksumValid){
	// Msg Configuration
	std::string start_seq = "start";
	int msg_len  = 3;
	int cobs_packet_length = start_seq.length() + msg_len + 2;

	// Msg Data
	uint16_t checksum_byte = (uint16_t) 'm' + (uint16_t) 's' + (uint16_t) 'g';
	checksum_byte &= 0x00FF; // Truncate to one byte
	unsigned char input_buffer[11] = {10, 's', 't', 'a', 'r', 't', 'm', 's', 'g', (uint8_t) checksum_byte, 0x00};

	// Msg Parser
	auto msg_parser = ghost_serial::MsgParser(msg_len, start_seq, true);

	// Parse the stream
	int parsed_msg_len;
	unsigned char output_buffer[msg_len] = {0,};
	ASSERT_TRUE(msg_parser.parseByteStream(
					input_buffer,
					sizeof(input_buffer) / sizeof(input_buffer[0]),
					output_buffer,
					parsed_msg_len)
	            );

	// Check that msg is correctly parsed
	unsigned char expected[] = {'m', 's', 'g'};
	for(int i = 0; i < msg_len; i++){
		ASSERT_EQ(expected[i], output_buffer[i]);
	}
}

TEST_F(TestMsgParser, testChecksumInvalid){
	// Msg Configuration
	std::string start_seq = "start";
	int msg_len  = 3;
	int cobs_packet_length = start_seq.length() + msg_len + 2;

	// Msg Data
	uint16_t checksum_byte = (uint16_t) 'm' + (uint16_t) 's' + (uint16_t) 'g' + 1; // Bit flipped in transit
	checksum_byte &= 0x00FF;
	unsigned char input_buffer[11] = {0x10, 's', 't', 'a', 'r', 't', 'm', 's', 'g', (uint8_t) checksum_byte, 0x00};

	// Msg Parser
	auto msg_parser = ghost_serial::MsgParser(msg_len, start_seq, true);

	// Parse the stream
	int parsed_msg_len;
	unsigned char output_buffer[msg_len] = {0,};
	ASSERT_FALSE(msg_parser.parseByteStream(
					 input_buffer,
					 sizeof(input_buffer) / sizeof(input_buffer[0]),
					 output_buffer,
					 parsed_msg_len)
	             );
}

int main(int argc, char **argv) {
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}