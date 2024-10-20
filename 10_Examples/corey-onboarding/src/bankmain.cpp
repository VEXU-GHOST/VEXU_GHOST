#include <iostream>
#include <string>
#include "bankheader.h"

using namespace std;

BankAccount::BankAccount(string AccountName, int AccountBalance)
{

    name = AccountName;
    balance = AccountBalance;

}

void BankAccount::withdraw(int amount)
{
    balance = balance - amount;
}

void BankAccount::print()
{
    cout << name << " has " << "a balance of " << balance << endl;
}


// int main ()
// {
//     BankAccount account1("kelly", 50000);
//     account1.print();
//     account1.withdraw(2356);
//     account1.print();

//     return 0;
// }
//^ commented out to run the gtest