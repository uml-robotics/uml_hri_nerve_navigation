#include <iostream>
#include <nlohmann/json.hpp>
#include <fstream>

using namespace nlohmann;

int main (int argc, char** argv){
    std::fstream file("../../../src/uml_hri_nerve_navigation/test_defs/sample.json", std::ios::in);
    if (file.is_open()){
        std::cout << "FILE IS OPEN" << std::endl;
    }
    else{
        std::cout << "IT'S NOT OUT OPEN";
    }
    json json_file = json::parse(file);
    std::string test_string = json_file.at("test");
    std::cout << test_string << std::endl;
    return 0;
}