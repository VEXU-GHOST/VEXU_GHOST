#include <iostream>
#include <string>
#include "bankheader.hpp"

using namespace std;

BankAccount::BankAccount(string AccountName, int AccountBalance)
{

    setName(AccountName);
    setBalance(AccountBalance);

}

string BankAccount::getName() const
{
    return name;
}

void BankAccount::setName(const string& newName)
{
    name = newName;
}

int BankAccount::getBalance() const
{
    return balance;
}

void BankAccount::setBalance(int newBalance)
{
    balance = newBalance;
}


void BankAccount::withdraw(int amount)
{
    setBalance(getBalance() - amount);
}

void BankAccount::print()
{
    cout << name << " has " << "a balance of " << getBalance() << endl;
}


int main ()
{
    BankAccount account1("kelly", 50000);
    account1.print();
    account1.withdraw(2356);
    account1.print();  

    return 0;
}
//^ commented out to run the gtest .