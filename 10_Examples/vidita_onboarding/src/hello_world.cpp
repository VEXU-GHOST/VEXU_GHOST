#include <iostream>
#include <vector>
#include "hello_world.h"
using namespace std;

void hello_world::myMethod(int startInd) {
    vector<int> myVector = {5, 6, 7, 8, 9, 1, 2, 3, 4};
    for (int i = startInd; i < myVector.size(); i++) {
        cout << myVector.at(i) << endl;
    }
    cout << "Goodbye now." << endl;
}



