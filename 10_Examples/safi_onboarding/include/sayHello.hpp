#ifndef safi_onboarding_say_hello_hpp
#define safi_onboarding_say_hello_hpp

#include <string>

class sayHello {
public:
  sayHello() = default;

  std::string greet() const {
    return "Hello from sayHello!";
  }

  int add(int a, int b) const {
    return a + b;
  }
};

#endif  // safi_onboarding_say_hello_hpp
