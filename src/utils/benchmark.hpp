#ifndef BENCHMARK_HPP
#define BENCHMARK_HPP

#include <iostream>
#include <chrono>

//depois faço as funções necessárias para rodar benchmarks de tempo e memória
//exemplo de função de benchmark de tempo:
template<typename Func, typename... Args>

double benchmarkTime(const std::string& testName, Func func, Args&&... args) {
    auto start = std::chrono::high_resolution_clock::now();
    func(std::forward<Args>(args)...);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << testName << " - Tempo gasto: " << elapsed.count() << " segundos" << std::endl;
    return elapsed.count();
}

#endif