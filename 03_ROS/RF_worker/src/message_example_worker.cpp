#include <iostream>
#include <string>
#include <unordered_map>
#include <functional>
#include <thread>
#include <chrono>
#include <mutex>

// ============================================================================
//   Simulated Radio Worker Link
//   (Replace with real serial, TCP, or UDP receiving later)
// ============================================================================

class RadioLinkWorker {
public:
    using CallbackFunc = std::function<void(const std::string&,
                                            const std::string&,
                                            int32_t,
                                            double)>;

    // Constructor
    RadioLinkWorker(const std::string& link_name)
        : name(link_name), linked(false)
    {
        // Simulate the link coming online after 2 seconds
        std::thread([this]() {
            std::this_thread::sleep_for(std::chrono::seconds(2));
            linked = true;
            std::cout << "[Worker] Link established.\n";

            // Fake message reception loop
            simulateIncomingMessages();
        }).detach();
    }

    bool isLinked() const {
        return linked;
    }

    // Register a specific message callback
    void received(const std::string& msgName,
                  std::function<void(const std::string&, const std::string&, double)> cb)
    {
        messageCallbacks[msgName] =
            [cb](const std::string& m, const std::string& l, int32_t, double v) {
                cb(m, l, v);
            };
    }

    // Register a message+index callback (like start_motor)
    void received(const std::string& msgName,
                  std::function<void(const std::string&, const std::string&, int32_t, double)> cb)
    {
        messageCallbacks[msgName] = cb;
    }

    // Register generic fallback callback
    void received(std::function<void(const std::string&, const std::string&, int32_t, double)> cb)
    {
        fallbackCallback = cb;
    }

private:
    std::string name;
    bool linked;
    std::mutex mu;

    std::unordered_map<std::string, CallbackFunc> messageCallbacks;
    CallbackFunc fallbackCallback;

    // ------------------------------------------------------------------------
    // Simulate receiving messages (this replaces LinkA.receive events)
    // ------------------------------------------------------------------------
    void simulateIncomingMessages() 
    {
        std::thread([this]() {
            while (true) {
                std::this_thread::sleep_for(std::chrono::milliseconds(700));

                // Fake messages (mimicking sender in your other file)
                receiveMessage("drive", -1, 0.0);
                std::this_thread::sleep_for(std::chrono::milliseconds(700));

                receiveMessage("go_forward", -1, 100.0);
                std::this_thread::sleep_for(std::chrono::milliseconds(700));

                receiveMessage("start_motor", 3, 50.0);
            }
        }).detach();
    }

    // Core message dispatch system
    void receiveMessage(const std::string& msgName, int32_t index, double value)
    {
        std::lock_guard<std::mutex> guard(mu);

        if (messageCallbacks.count(msgName)) {
            messageCallbacks[msgName](msgName, name, index, value);
        } else if (fallbackCallback) {
            fallbackCallback(msgName, name, index, value);
        } else {
            std::cout << "[Worker] Unhandled message: " << msgName << std::endl;
        }
    }
};

// ============================================================================
//   Callback Implementations  (matches your VEX worker code)
// ============================================================================

void drive_received(const std::string& msg,
                    const std::string& linkname,
                    double value)
{
    printf("%s: was received on '%s' link\n", msg.c_str(), linkname.c_str());
}

void go_forward(const std::string& msg,
                const std::string& linkname,
                double value)
{
    printf("%s: was received on '%s' link with value %.2f\n",
           msg.c_str(), linkname.c_str(), value);
}

void start_motor(const std::string& msg,
                 const std::string& linkname,
                 int32_t index,
                 double value)
{
    printf("%s: was received on '%s' link with index %d and value %.2f\n",
           msg.c_str(), linkname.c_str(), index, value);
}

void receive_message(const std::string& msg,
                     const std::string& linkname,
                     int32_t index,
                     double value)
{
    printf("receive_message: %s was received on %s\n",
           msg.c_str(), linkname.c_str());
}

// ============================================================================
//   Main
// ============================================================================
int main()
{
    RadioLinkWorker link("ros_radio_link_worker");

    // Register callbacks exactly like VEX API
    link.received("drive", drive_received);
    link.received("go_forward", go_forward);
    link.received("start_motor", start_motor);

    // Generic fallback callback
    link.received(receive_message);

    // Display link status
    while (true)
    {
        std::cout << "Link: " << (link.isLinked() ? "OK" : "--") << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    return 0;
}