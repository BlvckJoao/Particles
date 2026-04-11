#ifndef BENCHMARK_HPP
#define BENCHMARK_HPP

#include <iostream>
#include <fstream>
#include <string>
#include <chrono>
#include <thread>

// ── Helpers internos ─────────────────────────────────────────────────────────

struct CpuStats {
    long long user, nice, system, idle, iowait, irq, softirq;
};

inline CpuStats readCpuStats() {
    std::ifstream file("/proc/stat");
    std::string cpu;
    CpuStats s;
    file >> cpu >> s.user >> s.nice >> s.system >> s.idle
         >> s.iowait >> s.irq >> s.softirq;
    return s;
}

// ── Função livre: benchmark de tempo ─────────────────────────────────────────

template<typename Func, typename... Args>
double benchmarkTime(const std::string& testName, Func func, Args&&... args) {
    auto start = std::chrono::high_resolution_clock::now();
    func(std::forward<Args>(args)...);
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> elapsed = end - start;
    std::cout << testName << " - Tempo gasto: " << elapsed.count() << " segundos\n";
    return elapsed.count();
}

// ── Classe principal ──────────────────────────────────────────────────────────

class Benchmark {
private:
    CpuStats cpuStart_;
    std::chrono::steady_clock::time_point t0_;
    int    frameCount_;
    std::chrono::steady_clock::time_point lastFpsTime_;
    double fps_;
    
public:
    Benchmark()
        : cpuStart_(readCpuStats()),
          t0_(std::chrono::steady_clock::now()),
          frameCount_(0),
          lastFpsTime_(t0_),
          fps_(0.0)
    {}

    // Chame no início de cada frame/iteração
    void tick() {
        ++frameCount_;

        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - lastFpsTime_).count();

        if (elapsed >= 1.0) {           // atualiza FPS a cada 1 segundo
            fps_ = frameCount_ / elapsed;
            frameCount_ = 0;
            lastFpsTime_ = now;
        }
    }

    // Imprime CPU%, tempo decorrido e FPS na mesma linha (bom para loops)
    void report(const std::string& label) {
        auto s2  = readCpuStats();
        auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - t0_).count();

        long long idle1  = cpuStart_.idle  + cpuStart_.iowait;
        long long idle2  = s2.idle         + s2.iowait;
        long long total1 = cpuStart_.user  + cpuStart_.nice + cpuStart_.system + idle1;
        long long total2 = s2.user         + s2.nice        + s2.system        + idle2;

        double cpuUsage = 100.0 * (1.0 - static_cast<double>(idle2 - idle1)
                                             / (total2 - total1));

        std::cout << "\r[" << label << "]"
                  << "  CPU: "   << cpuUsage << "%"
                  << "  FPS: "   << fps_
                  << "  Tempo: " << ms << "ms"
                  << "     " << std::flush;
    }

    // Reseta todos os contadores
    void reset() {
        cpuStart_    = readCpuStats();
        t0_          = std::chrono::steady_clock::now();
        lastFpsTime_ = t0_;
        frameCount_  = 0;
        fps_         = 0.0;
    }

    // Getters individuais, caso precise dos valores separados
    double getFps()     const { return fps_; }
    double getCpuUsage() const {
        auto s2     = readCpuStats();
        long long i1 = cpuStart_.idle + cpuStart_.iowait;
        long long i2 = s2.idle        + s2.iowait;
        long long t1 = cpuStart_.user + cpuStart_.nice + cpuStart_.system + i1;
        long long t2 = s2.user        + s2.nice        + s2.system        + i2;
        return 100.0 * (1.0 - static_cast<double>(i2 - i1) / (t2 - t1));
    }
};

#endif