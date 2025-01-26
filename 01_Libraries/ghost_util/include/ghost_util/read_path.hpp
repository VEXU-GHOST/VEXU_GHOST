#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
namespace ghost_util
{
int readPathFromFile(std::string filename, std::vector<double> &x_values, std::vector<double> &y_values, std::vector<double> &angle_values) {
    std::ifstream file(filename);

    if (!file.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return 1;
    }
    // Open the CSV file
    
   

    // Vectors to store the data
    //td::vector<double> x_values;
    //std::vector<double> y_values;
   // std::vector<double> angle_values;

   //this method transformed to fit JerryPathIO (treats center as 0) 
   //to Ghost convention (bottow right corner as 0). At bottom 0 corner, 
   //we want both axis to be positive (left = positive, top= positive)

    std::string line;
    // Read the file line by line
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string value;
        
        // Read X value
        if (std::getline(ss, value, ',')) {
            //std::cout << value << " ";
            y_values.push_back((std::stod(value) / -100.0)+ 1.83);
        }
        
        // Read Y value
        if (std::getline(ss, value, ',')) {
           x_values.push_back((std::stod(value) / 100.0)+ 1.83);
        }
        
        // Read Angle value
        if (std::getline(ss, value, ',')) {
            angle_values.push_back(std::stod(value));
        }
    }

    // Close the file
    file.close();
/*
    // Print the data to verify
    std::cout << "X values: ";
    for (const auto& x : x_values) {
        std::cout << x << " ";
    }
    std::cout << std::endl;

    std::cout << "Y values: ";
    for (const auto& y : y_values) {
        std::cout << y << " ";
    }
    std::cout << std::endl;

    std::cout << "Angle values: ";
    for (const auto& angle : angle_values) {
        std::cout << angle << " ";
    }
    std::cout << std::endl;
*/

    return 0;
}
}  // namespace ghost_util