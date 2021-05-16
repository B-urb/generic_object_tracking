//
// Created by burban on 01.02.21.
//

#ifndef MASTER_CPP_BENCHMARK_H
#define MASTER_CPP_BENCHMARK_H

#include <stack>
#include "unordered_map"
//https://stackoverflow.com/questions/1008019/c-singleton-design-pattern

class Benchmark {
    public:
        static Benchmark& getInstance()
        {
            static Benchmark    instance; // Guaranteed to be destroyed.
            // Instantiated on first use.
            return instance;
        }
    private:
        Benchmark() {}      // Constructor? (the {} brackets) are needed here.
        std::unordered_map<std::string,  std::chrono::microseconds> complete_runtimes;
        std::stack<std::chrono::time_point<std::chrono::high_resolution_clock>> current_runtime;
        std::unordered_map<std::string, int> invokation_counter;
        bool log_to_console = false;
        std::unordered_map<std::string, std::string> metadata;

public:
    bool isLogToConsole() const;

    void setLogToConsole(bool logToConsole);
    void addMetadata(std::string key, std::string value);

public:
        Benchmark(Benchmark const&)               = delete;
        void operator=(Benchmark const&)  = delete;
        void startTimeMeasure(const std::string& function_name);
        void endTimeMeasure(const std::string& function_name);
        void printAverageRuntimesToFile(const std::string& filename);


        // Note: Scott Meyers mentions in his Effective Modern
        //       C++ book, that deleted functions should generally
        //       be public as it results in better error messages
        //       due to the compilers behavior to check accessibility
        //       before deleted status

};



#endif //MASTER_CPP_BENCHMARK_H
