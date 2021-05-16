//
// Created by burban on 01.02.21.
//

#include <chrono>
#include <iostream>
#include <fstream>
#include "Benchmark.h"


void Benchmark::startTimeMeasure(const std::string &function_name) {
    current_runtime.push(std::chrono::high_resolution_clock::now());

}

void Benchmark::endTimeMeasure(const std::string &function_name) {
    auto duration = std::chrono::high_resolution_clock::now() - current_runtime.top();
    current_runtime.pop();
    complete_runtimes[function_name] =
            complete_runtimes[function_name] + std::chrono::duration_cast<std::chrono::microseconds>(duration);
    invokation_counter[function_name]++;
    if (log_to_console) {
        std::cout << function_name << " executed in: "
                  << std::chrono::duration_cast<std::chrono::milliseconds>(duration).count() << " ms" << std::endl;
    }
}

bool Benchmark::isLogToConsole() const {
    return log_to_console;
}

void Benchmark::setLogToConsole(bool logToConsole) {
    log_to_console = logToConsole;
}

void Benchmark::printAverageRuntimesToFile(const std::string &filename) {
    std::ofstream myfile;
    myfile.open(filename);
    for (auto &elem : metadata) {
        myfile << elem.first;
        myfile << ": ";
        myfile << elem.second;
        myfile << std::endl;
    }
    myfile << "---------------------------------------------";
    myfile << std::endl;
    myfile << std::endl;

    for (auto &elem : complete_runtimes) {
        auto average_runtime = elem.second / invokation_counter[elem.first];
        myfile << elem.first << ": ";
        if (average_runtime.count() > 1000) {
            myfile << std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(average_runtime).count());
            myfile << "ms";
            myfile << std::endl;
        } else {
            myfile << std::to_string(average_runtime.count());
            myfile << " \u03BCs";
            myfile << std::endl;
        }
    }
    myfile.close();

}

void Benchmark::addMetadata(std::string key, std::string value) {
    metadata.insert_or_assign(key, value);
}
