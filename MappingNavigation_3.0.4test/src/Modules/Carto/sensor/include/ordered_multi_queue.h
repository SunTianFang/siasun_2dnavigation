
#ifndef SENSOR_INTERNAL_ORDERED_MULTI_QUEUE_H_
#define SENSOR_INTERNAL_ORDERED_MULTI_QUEUE_H_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <tuple>

#include "blocking_queue.h"
#include "port.h"
#include "mytime.h"
#include "dispatchable.h"


namespace sensor {

struct QueueKey {
  int trajectory_id;
  std::string sensor_id;

  bool operator<(const QueueKey& other) const {
    return std::forward_as_tuple(trajectory_id, sensor_id) <
           std::forward_as_tuple(other.trajectory_id, other.sensor_id);
  }
};

// Maintains multiple queues of sorted sensor data and dispatches it in merge
// sorted order. It will wait to see at least one value for each unfinished
// queue before dispatching the next time ordered value across all queues.
//
// This class is thread-compatible.
class OrderedMultiQueue {
 public:

//EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 // typedef std::function<void(std::unique_ptr<Data>)> Callback;



 typedef std::function<void(const std::string&, std::unique_ptr<Data>)> Callback;

//  typedef void (*Callback)(const std::string&, std::unique_ptr<Data>);

  OrderedMultiQueue();
  ~OrderedMultiQueue();

  // Adds a new queue with key 'queue_key' which must not already exist.
  // 'callback' will be called whenever data from this queue can be dispatched.
  void AddQueue(const QueueKey& queue_key,  Callback callback);

  // Marks a queue as finished, i.e. no further data can be added. The queue
  // will be removed once the last piece of data from it has been dispatched.
  void MarkQueueAsFinished(const QueueKey& queue_key);

  // Adds 'data' to a queue with the given 'queue_key'. Data must be added
  // sorted per queue.
  void Add(const QueueKey& queue_key, std::unique_ptr<Data> data);

  // Dispatches all remaining values in sorted order and removes the underlying
  // queues.
  void Flush();

  // Must only be called if at least one unfinished queue exists. Returns the
  // key of a queue that needs more data before the OrderedMultiQueue can
  // dispatch data.
  QueueKey GetBlocker() const;

  void Clear();

 private:
  struct Queue {
    std::unique_ptr<common::BlockingQueue<std::unique_ptr<Data>>> queue;
    Callback callback;
    bool finished ;  //Lishen

  };

  void Dispatch();
  void CannotMakeProgress(const QueueKey& queue_key);
  common::Time GetCommonStartTime(int trajectory_id);

  // Used to verify that values are dispatched in sorted order.
  common::Time last_dispatched_time_  ;   //  = common::Time::min();

  std::map<int, common::Time> common_start_time_per_trajectory_;
  std::map<QueueKey, Queue> queues_;

  std::map<int, Queue> intqueues_;


  QueueKey blocker_;
};

}  // namespace sensor


#endif  // SENSOR_INTERNAL_ORDERED_MULTI_QUEUE_H_
