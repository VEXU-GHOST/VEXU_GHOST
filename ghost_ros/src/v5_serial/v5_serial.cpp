#include "v5_serial/v5_serial.hpp"

using namespace std::chrono_literals;

namespace v5_serial{

V5SerialInterface::V5SerialInterface(std::string config_file){
    loadConfiguration(config_file);

}

V5SerialInterface::~V5SerialInterface(){
    if(isConnected()){
        close(serial_port_);
    }
    reader_thread_.join();
    writer_thread_.join();
}

void V5SerialInterface::loadConfiguration(std::string filename){
    config_yaml_ = YAML::LoadFile(filename);
    port_name_ = config_yaml_["port_name"].as<std::string>();
}

void V5SerialInterface::openPort(){
    try{
        serial_port_ = open(port_name_.c_str(), O_RDWR);
        
        if (serial_port_ < 0) {
            std::string err_string = "[V5_Serial_Main] Error: Failed to open serial device on " + port_name_;
            perror(err_string.c_str());
            connected_ = false;
        }
        else {
            connected_ = true;
        }
    }
    catch(const std::exception& e){
        connected_ = false;
    }
}

void V5SerialInterface::startReaderThread(){
    
}

void V5SerialInterface::startWriterThread(){

}

void V5SerialInterface::readerThreadLoop(){
    while(globals::run){
        try{
            if(isConnected()){
    // int serial_port;
    // bool waiting_for_port = true;
    // while(waiting_for_port){
    //     try{
    //         serial_port = open(port_name.c_str(), O_RDONLY);
            
    //         if (serial_port < 0) {
    //             perror("Failed to open SERIAL_DEVICE");
    //         }
    //         else{
    //             waiting_for_port = false;
    //         }
    //     }
    //     catch(const std::exception& e){
    //         std::cout << e.what() << std::endl;
    //         return;
    //     }
    //     std::this_thread::sleep_for(10ms);
    // }

    // int bytes_available, retval;
    // retval = ioctl(serial_port, FIONREAD, &bytes_available);
    // std::cout << bytes_available << " Bytes in serial buffer" << std::endl;
    // std::cout << "Flushing Serial Device" << std::endl;

    // ioctl(serial_port, TCFLSH, 0); // flush receive

    // retval = ioctl(serial_port, FIONREAD, &bytes_available);
    // std::cout << bytes_available << " Bytes in serial buffer" << std::endl;

    // const int max_msg_len = 254;
    // const char start_seq[] = {'s', 'o', 'u', 't'};
    // const size_t start_seq_len = sizeof(start_seq)/sizeof(start_seq[0]);
    // int start_seq_count = 0;

    // unsigned char raw_msg_buffer[254] = {0,};
    
    // uint8_t msg_packet_encoded[max_msg_len] = {0,};
    // char msg_packet_decoded[max_msg_len] = {0,};

    // int msg_packet_index = 1;

    // while(globals::run){
    //     // Acquire serial buffer mutex

    //     // Check for bytes in serial buffer
    //     ioctl(serial_port, FIONREAD, &bytes_available);
    //     if(bytes_available !=0){
    //         // Read buffer length from serial port
    //         int num_bytes = read(serial_port, raw_msg_buffer, sizeof(raw_msg_buffer));
    //         std::cout << std::dec << std::endl;
    //         std::cout << "-----------" << std::endl;
    //         std::cout << "Read " << num_bytes << " bytes" << std::endl;
    //         std::cout << "Raw Serial Buffer: " << std::endl;
    //         std::cout << raw_msg_buffer << std::endl;
    //         // for(int k = 0; k < num_bytes; k++){
    //         //     std::cout << std::hex << (int) raw_msg_buffer[k] << " ";
    //         // }
    //         // std::cout << std::endl << "Iterate through each byte and decode" << std::endl;

    //         // Iterate through serial data
    //         for(int i = 0; i < num_bytes; i++){
    //             // std::cout << "Byte: " << std::hex << (int) raw_msg_buffer[i] << std::endl;

    //             if(start_seq_count == start_seq_len){
    //                 // Collect packet from msg buffer 2 2 1 1 0 
    //                 if(raw_msg_buffer[i] == 0x00){
    //                     // Process Msg
    //                     // std::cout << "Found delimiter" << std::endl;
    //                     COBS::cobsDecode(msg_packet_encoded, sizeof(msg_packet_encoded), msg_packet_decoded);
    //                     std::cout << "Processed Msg:" << std::endl;
    //                     std::cout << msg_packet_decoded << std::endl;

    //                     // Reset state
    //                     start_seq_count = 0;
    //                     msg_packet_index = 1;
    //                 }
    //                 else if(msg_packet_index < max_msg_len){
    //                     // Accumulate
    //                     // std::cout << "Adding " << raw_msg_buffer[i] << " to msg buffer" << std::endl;
    //                     msg_packet_encoded[msg_packet_index] = raw_msg_buffer[i];
    //                     msg_packet_index++;
    //                 }
    //                 else{
    //                     std::cout << "ERROR: Msg exceeds maximum length." << std::endl;
    //                     start_seq_count = 0;
    //                     msg_packet_index = 1;
    //                 }
    //             }
    //             else if(start_seq_count >= 0 && start_seq_count < start_seq_len){
    //                 // Searching for start sequence
    //                 if(raw_msg_buffer[i] == start_seq[start_seq_count]){
    //                     // std::cout << "Found an " << start_seq[start_seq_count] << "!" << std::endl;
    //                     // Input looks like start sequence component
    //                     start_seq_count++;
    //                 }
    //                 else{
    //                     // Remove PROS Header from msg
    //                     msg_packet_encoded[0] = raw_msg_buffer[i] - start_seq_len;
                        
    //                     // No start sequence, reset
    //                     start_seq_count = 0;
    //                     msg_packet_index = 1;

    //                     // std::cout << "looking for start" << std::endl;
    //                 }
    //             }
    //             else{
    //                 // ERROR
    //                 std::cout << "ERROR in V5 Serial Consumer. start_seq_count = " << start_seq_count << std::endl;
    //                 start_seq_count = 0;
    //                 msg_packet_index = 1; // ?
    //             }
    //         }
    //     }
    //     std::this_thread::sleep_for(5ms);
    // }
            }
        }
        catch(const std::exception& e){
            std::cout << "[Serial Reader Error]: " << e.what() << std::endl;
        }
        std::this_thread::sleep_for(5ms);
    }
}

void V5SerialInterface::writerThreadLoop(){
    while(globals::run){
        if(isConnected()){

        }
        std::this_thread::sleep_for(5ms);
    }
}

void v5_serial_main(std::string config_file){

    V5SerialInterface ghost_serial(config_file);
    ghost_serial.startReaderThread();
    ghost_serial.startWriterThread();

    while(globals::run){
        if(!ghost_serial.isConnected()){
            ghost_serial.openPort();
        }
        std::this_thread::sleep_for(500ms);
    }
}

} // namespace v5_serial
