#include <condition_variable>
#include <mutex>
#include <queue>
#include <thread>

template <class T> class SafeQueue {
public:
  SafeQueue(void) : q(), m(), c_full(),c_empty() {}

  ~SafeQueue(void) {}

  void enqueue(T t) {
    std::lock_guard<std::mutex> lock(m);
    while (q.size() == std::thread::hardware_concurrency()) {
        c_full.wait(lock);
    }
    q.push(t);
    c_empty.notify_one();
  }

  T dequeue(void) {
    std::unique_lock<std::mutex> lock(m);
    while (q.empty())
      c_empty.wait(lock);
    T val = q.front();
    q.pop();
    c_full.notify_one();
    return val;
  }

private:
  std::queue<T> q;
  mutable std::mutex m;
  std::condition_variable c_full;
  std::condition_variable c_empty;
};
