#include <iostream>
#include "firstCPPClass.h"

using namespace std;

class runnerClass {

public:
    void callOtherClass() {
        //firstCPPClass helloUser;
        firstCPPClass helloUser("name");
        cout << helloUser.getName() << endl;
    }
};

int main() {
    runnerClass testing;
    testing.callOtherClass();

    return 0;
}