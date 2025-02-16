#ifndef MYCLASS_H
#define MYCLASS_H

class MyClass {
private:
    int value;  
public:
    MyClass();

    int getValue() const;

    void setValue(int newValue);
};

#endif