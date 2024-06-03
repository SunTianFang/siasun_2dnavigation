

#ifndef COMMON_TASK_H_
#define COMMON_TASK_H_

#include <set>

#include "mutex.h"
#include "mylogging.h"
#include "thread_pool.h"


using namespace myLogging;

namespace common {

class ThreadPoolInterface;

class Task {
 public:
  friend class ThreadPoolInterface;

  typedef  std::function<void()> WorkItem ;
  enum State { NEW, DISPATCHED, DEPENDENCIES_COMPLETED, RUNNING, COMPLETED };

  Task();
  ~Task();

  State GetState() ;

  // State must be 'NEW'.
  void SetWorkItem(const WorkItem& work_item) ;

  // State must be 'NEW'. 'dependency' may be nullptr, in which case it is
  // assumed completed.
  void AddDependency(std::weak_ptr<Task> dependency) ;


  // add by lishen
  void Reset(void);


 private:
  // Allowed in all states.
  void AddDependentTask(Task* dependent_task);

  // State must be 'DEPENDENCIES_COMPLETED' and becomes 'COMPLETED'.
  void Execute() ;

  // State must be 'NEW' and becomes 'DISPATCHED' or 'DEPENDENCIES_COMPLETED'.
  void SetThreadPool(ThreadPoolInterface* thread_pool) ;

  // State must be 'NEW' or 'DISPATCHED'. If 'DISPATCHED', may become
  // 'DEPENDENCIES_COMPLETED'.
  void OnDependenyCompleted();

  WorkItem work_item_ ;
  ThreadPoolInterface* thread_pool_to_notify_  ;
  State state_  ;
  unsigned int uncompleted_dependencies_  ;
  std::set<Task*> dependent_tasks_ ;

  Mutex mutex_;
};

}  // namespace common


#endif  // COMMON_TASK_H_
