
#include <chrono>

namespace stamped_queue{

template <typename data_type>
class StampedQueue {
public:
    StampedQueue(){
    }

    StampedQueue(std::chrono::milliseconds max_history_duration){

    }
    void AddValue(std::chrono::time_point<std::chrono::system_clock> timestamp, data_type data){

    }

    std::vector<std::pair<std::chrono::time_point<std::chrono::system_clock>, data_type>> GetHistory(){

    }

private:
    std::chrono::milliseconds  max_history_duration_;
    std::vector<std::pair<std::chrono::time_point<std::chrono::system_clock>, data_type>> data_history_;
};

} // namespace sensor_queue
