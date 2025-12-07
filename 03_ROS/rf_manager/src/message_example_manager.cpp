#include <iostream>
#include <thread>
#include <chrono>
#include <mutex>
#include <string>

// ---------------------------------------------------------------------------
//  Simulated Radio Link (replace with serial/UDP/TCP later)
// ---------------------------------------------------------------------------
class RadioLink {
public:
    RadioLink(const std::string& name)
        : link_name(name), linked(false)
    {
        // Simulate link connection after some time
        std::thread([this]() {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            linked = true;
        }).detach();
    }

    bool isLinked() const {
        return linked;
    }

    // Send a message with no arguments
    void send(const std::string& msg) {
        std::lock_guard<std::mutex> guard(mu);
        std::cout << "[SEND] " << msg << std::endl;
    }

    // Send a message with an integer parameter
    void send(const std::string& msg, int value) {
        std::lock_guard<std::mutex> guard(mu);
        std::cout << "[SEND] " << msg << " : " << value << std::endl;
    }

    // Send a message with float parameter
    void send(const std::string& msg, double value) {
        std::lock_guard<std::mutex> guard(mu);
        std::cout << "[SEND] " << msg << " : " << value << std::endl;
    }

private:
    std::string link_name;
    bool linked;
    mutable std::mutex mu;
};

// ---------------------------------------------------------------------------
//  Background sending loop (replaces VEX sendTask())
// ---------------------------------------------------------------------------
int sendTask(RadioLink& link) 
{
    // Wait for link to come online
    while (!link.isLinked()) {
        std::cout << "Waiting for link...\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    std::cout << "Link established.\n";

    // Loop like original VEX code
    while (true) 
    {
        link.send("drive");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        link.send("go_forward", 100);
        std::this_thread::sleep_for(std::chrono::milliseconds(500));

        link.send("start_motor", 50.0);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

// ---------------------------------------------------------------------------
//  Main
// ---------------------------------------------------------------------------
int main() 
{
    RadioLink link("ros_radio_link");

    std::thread sender(sendTask, std::ref(link));

    // Main loop showing link status
    while (true) 
    {
        std::cout << "Link: " << (link.isLinked() ? "OK" : "DOWN") << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    sender.join();
    return 0;
}