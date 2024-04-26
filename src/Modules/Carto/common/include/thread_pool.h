

#ifndef COMMON_THREAD_POOL_H_
#define COMMON_THREAD_POOL_H_



#include "mytime.h"

#include <deque>
#include <functional>
#include <memory>
#include <thread>
#include <unordered_map>
#include <vector>
#include <map>

#include "mutex.h"
#include "task.h"


namespace common {

class Task;

class ThreadPoolInterface {
 public:
  ThreadPoolInterface() {}
  virtual ~ThreadPoolInterface() {}
  virtual std::weak_ptr<Task> Schedule(std::unique_ptr<Task> task) = 0;

  virtual void ClearTask() = 0;
 virtual  void Reset() = 0;
 protected:
  void Execute(Task* task);
  void SetThreadPool(Task* task);

 private:
  friend class Task;

  virtual void NotifyDependenciesCompleted(Task* task) = 0;
};

// A fixed number of threads working on tasks. Adding a task does not block.
// Tasks may be added whether or not their dependencies are completed.
// When all dependencies of a task are completed, it is queued up for execution
// in a background thread. The queue must be empty before calling the
// destructor. The thread pool will then wait for the currently executing work
// items to finish and then destroy the threads.
class ThreadPool : public ThreadPoolInterface {
 public:
  explicit ThreadPool(int num_threads);
  ThreadPool();
  ~ThreadPool();

  ThreadPool(const ThreadPool&) = delete;
  ThreadPool& operator=(const ThreadPool&) = delete;

  // When the returned weak pointer is expired, 'task' has certainly completed,
  // so dependants no longer need to add it as a dependency.
  std::weak_ptr<Task> Schedule(std::unique_ptr<Task> task)  ;



  // add by lishen
  void ClearTask();

  // add by lishen
  void Reset();

 private:
  void DoWork();

  void NotifyDependenciesCompleted(Task* task)  ;

  Mutex mutex_;

  Mutex reset_mutex_;

  bool is_reset ;

  bool running_  ;
  std::vector<std::thread> pool_;
  std::deque<std::shared_ptr<Task>> task_queue_ ;

  std::unordered_map<Task*, std::shared_ptr<Task>> tasks_not_ready_;
};

}  // namespace common


#endif  // COMMON_THREAD_POOL_H_
