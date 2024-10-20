#include <iostream>
#include <string>

using namespace std;

class BankAccount
{

public:

    string name;    //Variable

    int balance;    //Variable

    void withdraw(int amount);   //my method

    void print();    //my method

    BankAccount(string AccountName, int AccountBalance); // declare constructor
   
};


