
#ifndef SENSOR_MAP_BY_TIME_H_
#define SENSOR_MAP_BY_TIME_H_

#include <algorithm>
#include <iterator>
#include <map>
#include <memory>
#include <vector>

#include "port.h"
#include "mytime.h"
#include "id.h"
#include "mylogging.h"



using namespace myLogging;

namespace sensor {

// 'DataType' must contain a 'time' member of type common::Time.
template <typename DataType>
class MapByTime {
 public:

 //EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Appends data to a 'trajectory_id', creating trajectories as needed.
  void Append(const int trajectory_id, const DataType& data) {
    CHECK_GE(trajectory_id, 0);
    auto& trajectory = data_[trajectory_id];
    if (!trajectory.empty()) {


	  
	 // printf(" data sec = %ld    \n", common::ToUniversal(data.time));

	 //  printf(" prev sec = %ld\n", common::ToUniversal(std::prev(trajectory.end())->first));

    //  CHECK_GT(data.time, std::prev(trajectory.end())->first);

		


    }
    trajectory.insert(std::pair<common::Time, DataType>{data.time, data});
  }



  // Removes data no longer needed once 'node_id' gets removed from 'nodes'.
  // 'NodeType' must contain a 'time' member of type common::Time.
  template <typename NodeType>
  void Trim(const mapping::MapById<mapping::NodeId, NodeType>& nodes,
            const mapping::NodeId& node_id) {
    const int trajectory_id = node_id.trajectory_id;
    CHECK_GE(trajectory_id, 0);
    if (data_.count(trajectory_id) == 0) {
      return;
    }

    // Data only important between 'gap_start' and 'gap_end' is no longer
    // needed. We retain the first and last data of the gap so that
    // interpolation with the adjacent data outside the gap is still possible.
    const auto node_it = nodes.find(node_id);
    CHECK(node_it != nodes.end());
    const common::Time gap_start =
        node_it != nodes.BeginOfTrajectory(trajectory_id)
            ? std::prev(node_it)->data.time
            : common::Time::min();
    const auto next_it = std::next(node_it);
    const common::Time gap_end = next_it != nodes.EndOfTrajectory(trajectory_id)
                                     ? next_it->data.time
                                     : common::Time::max();
    CHECK_LT(gap_start, gap_end);

    auto& trajectory = data_.at(trajectory_id);
    auto data_it = trajectory.lower_bound(gap_start);
    auto data_end = trajectory.upper_bound(gap_end);
    if (data_it == data_end) {
      return;
    }
    if (gap_end != common::Time::max()) {
      // Retain the last data inside the gap.
      data_end = std::prev(data_end);
      if (data_it == data_end) {
        return;
      }
    }
    if (gap_start != common::Time::min()) {
      // Retain the first data inside the gap.
      data_it = std::next(data_it);
    }
    while (data_it != data_end) {
      data_it = trajectory.erase(data_it);
    }
    if (trajectory.empty()) {
      data_.erase(trajectory_id);
    }
  }

  bool HasTrajectory(const int trajectory_id) const {
    return data_.count(trajectory_id) != 0;
  }

  class ConstIterator {
   public:
    typedef std::bidirectional_iterator_tag iterator_category  ;
    typedef DataType value_type  ;
    typedef int64 difference_type ;
    typedef const DataType* pointer  ;
    typedef const DataType& reference  ;

    explicit ConstIterator(
        typename std::map<common::Time, DataType>::const_iterator iterator)
        : iterator_(iterator) {}

    const DataType& operator*() const { return iterator_->second; }

    const DataType* operator->() const { return &iterator_->second; }

    ConstIterator& operator++() {
      ++iterator_;
      return *this;
    }

    ConstIterator& operator--() {
      --iterator_;
      return *this;
    }

    bool operator==(const ConstIterator& it) const {
      return iterator_ == it.iterator_;
    }

    bool operator!=(const ConstIterator& it) const { return !operator==(it); }

   private:
    typename std::map<common::Time, DataType>::const_iterator iterator_;
  };

  class ConstTrajectoryIterator {
   public:
    typedef std::bidirectional_iterator_tag iterator_category ;
    typedef int value_type ;
    typedef int64 difference_type  ;
    typedef const int* pointer ;
    typedef const int& reference ;

    explicit ConstTrajectoryIterator(
        typename std::map<int, std::map<common::Time, DataType>>::const_iterator
            current_trajectory)
        : current_trajectory_(current_trajectory) {}

    int operator*() const { return current_trajectory_->first; }

    ConstTrajectoryIterator& operator++() {
      ++current_trajectory_;
      return *this;
    }

    ConstTrajectoryIterator& operator--() {
      --current_trajectory_;
      return *this;
    }

    bool operator==(const ConstTrajectoryIterator& it) const {
      return current_trajectory_ == it.current_trajectory_;
    }

    bool operator!=(const ConstTrajectoryIterator& it) const {
      return !operator==(it);
    }

   private:
    typename std::map<int, std::map<common::Time, DataType>>::const_iterator
        current_trajectory_;
  };

  ConstIterator BeginOfTrajectory(const int trajectory_id) const {
    return ConstIterator(data_.at(trajectory_id).begin());
  }

  ConstIterator EndOfTrajectory(const int trajectory_id) const {
    return ConstIterator(data_.at(trajectory_id).end());
  }

  // Returns Range object for range-based loops over the trajectory IDs.
  mapping::Range<ConstTrajectoryIterator> trajectory_ids() const {
    return mapping::Range<ConstTrajectoryIterator>(
        ConstTrajectoryIterator(data_.begin()),
        ConstTrajectoryIterator(data_.end()));
  }

  mapping::Range<ConstIterator> trajectory(const int trajectory_id) const {
    return mapping::Range<ConstIterator>(BeginOfTrajectory(trajectory_id),
                                         EndOfTrajectory(trajectory_id));
  }

  // Returns an iterator to the first element in the container belonging to
  // trajectory 'trajectory_id' whose time is not considered to go before
  // 'time', or EndOfTrajectory(trajectory_id) if all keys are considered to go
  // before 'time'. 'trajectory_id' must refer to an existing trajectory.
  ConstIterator lower_bound(const int trajectory_id,
                            const common::Time time) const {
    return ConstIterator(data_.at(trajectory_id).lower_bound(time));
  }
  void clear(){

		data_.clear();
	}


 private:
  std::map<int, std::map<common::Time, DataType>> data_;
};

}  // namespace sensor


#endif  // SENSOR_MAP_BY_TIME_H_
