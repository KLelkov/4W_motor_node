#include <iostream>
#include <stdio.h>
#include "CppSerial.cpp"

int main()
{
    char portname[] = "/dev/ttyUSB0";
    CppSerial ser;
    ser.Open(portname, 115200);
    unsigned char output_buffer[256] = {};
    int len = 0;
    len = ser.ReadLine(output_buffer);
    std::cout << std::endl;
    std::cout << "Back in main()" << std::endl;
    std::cout << len << std::endl;
    std::cout << output_buffer << std::endl;
    //for (auto&& c : output_buffer){
    //  std::cout << c;
    //}


}
