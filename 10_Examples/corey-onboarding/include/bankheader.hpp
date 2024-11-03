#include <iostream>
#include <string>

using namespace std;

class BankAccount
{

private:

    string name;    //Variable

    int balance;    //Variable

public:

    void setName(const string& newName);    //setter for name member variable

    string getName() const;     //getter for name member variable

    void setBalance(int newBalance);    //setter for balance

    int getBalance() const;     //getter for Balance


    void withdraw(int amount);   //my method

    void print();    //my method

    BankAccount(string AccountName, int AccountBalance); // declare constructor


   
};


