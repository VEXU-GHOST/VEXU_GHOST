#include <iostream>
#include <string>

class trivial_methods{
    public:
        int number = 616;
        void printName(){
            std::cout << "hihi" << std::endl;
        }

        bool isEven(int num) {
            if(num % 2 == 0) {
                return true;
            }
            return false;
        }

        bool testHowSlay(std::string idk) {
            if(idk == "very slay"){
                std::cout << idk << std::endl;
                return true;
            }
            else if (idk == "slay"){
                std::cout << idk << std::endl;
                return true;
            }
            else {
                std::cout << "not slay :(" << std::endl;
            }
            return false;
        }
};
