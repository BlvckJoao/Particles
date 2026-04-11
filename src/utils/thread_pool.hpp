#ifndef THREAD_POOL_HPP
#define THREAD_POOL_HPP

#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <future>

//para uso futuro, caso queira implementar uma versão paralela do solver de colisões usando threads

// =============================================================================
// ThreadPool
//
// Cria N threads que ficam dormindo esperando tarefas.
// submit() enfileira uma tarefa e retorna um std::future pra sincronizar.
// As threads são destruídas no destrutor.
// =============================================================================

class ThreadPool {
public:
        explicit ThreadPool(size_t numThreads) : stop(false) {
                for (size_t i = 0; i < numThreads; ++i) {
                        workers.emplace_back([this] {
                                while (true) {
                                        std::function<void()> task;
                                        {
                                                std::unique_lock<std::mutex> lock(queueMutex);
                                                // Dorme até ter tarefa ou receber sinal de stop
                                                condition.wait(lock, [this] {
                                                        return stop || !tasks.empty();
                                                });
                                                if (stop && tasks.empty()) return;
                                                task = std::move(tasks.front());
                                                tasks.pop();
                                        }
                                        task();
                                }
                        });
                }
        }

        // Enfileira uma tarefa e retorna future para sincronizar
        template<typename F>
        std::future<void> submit(F&& f) {
                auto task = std::make_shared<std::packaged_task<void()>>(std::forward<F>(f));
                std::future<void> result = task->get_future();
                {
                        std::unique_lock<std::mutex> lock(queueMutex);
                        tasks.emplace([task] { (*task)(); });
                }
                condition.notify_one();
                return result;
        }

        ~ThreadPool() {
                {
                        std::unique_lock<std::mutex> lock(queueMutex);
                        stop = true;
                }
                condition.notify_all();
                for (auto& w : workers) w.join();
        }

private:
        std::vector<std::thread>          workers;
        std::queue<std::function<void()>> tasks;
        std::mutex                        queueMutex;
        std::condition_variable           condition;
        bool                              stop;
};

#endif