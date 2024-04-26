
#ifndef COMMON_RATE_TIMER_H_
#define COMMON_RATE_TIMER_H_

#include <chrono>
#include <deque>
#include <iomanip>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include "mymath.h"
#include "port.h"
#include "mytime.h"
#include "mylogging.h"


using namespace myLogging;

namespace common {

// Computes the rate at which pulses come in.

class RateTimer {
 public:
  // Computes the rate at which pulses come in over 'window_duration' in wall
  // time.
  explicit RateTimer(double window_duration)
      : window_duration_(window_duration) {}
  ~RateTimer() {}

  RateTimer(const RateTimer& tmp) 
  {	
  	window_duration_ = tmp.window_duration_;

	for(int i = 0; i < tmp.events_.size(); i++)
	{
		events_.push_back(tmp.events_.at(i));

	}
  }


  RateTimer& operator=(const RateTimer&) = delete;

  // Returns the pulse rate in Hz.
  double ComputeRate() const {
    if (events_.empty()) {
      return 0.;
    }
    return static_cast<double>(events_.size() - 1) /
           ((events_.back().time - events_.front().time));
  }

  // Returns the ratio of the pulse rate (with supplied times) to the wall time
  // rate. For example, if a sensor produces pulses at 10 Hz, but we call Pulse
  // at 20 Hz wall time, this will return 2.
  double ComputeWallTimeRateRatio() const {
    if (events_.empty()) {
      return 0.;
    }
   // return common::ToSeconds((events_.back().time - events_.front().time)) /
   ////        std::chrono::duration_cast<std::chrono::duration<double>>(
    //           events_.back().wall_time - events_.front().wall_time)
   //            .count();


    return (events_.back().time - events_.front().time) / (events_.back().wall_time - events_.front().wall_time);
               

  }

  // Records an event that will contribute to the computed rate.
  void Pulse(common::Time time) {
    events_.push_back(Event{time, common::Time::now()});

   
    while (events_.size() > 2 &&
           (events_.back().wall_time - events_.front().wall_time) >
               window_duration_) {
      events_.pop_front();
    }
  }

  // Returns a debug string representation.
  std::string DebugString() const {
    if (events_.size() < 2) {
      return "unknown";
    }
    std::ostringstream out;
    out << std::fixed << std::setprecision(2) << ComputeRate() << " Hz "
        << DeltasDebugString() << " (pulsed at "
        << ComputeWallTimeRateRatio() * 100. << "% real time)";
    return out.str();
  }

 private:
  struct Event {
    common::Time time;
    common::Time wall_time;
  };

  // Computes all differences in seconds between consecutive pulses.
  std::vector<double> ComputeDeltasInSeconds() const {
    CHECK_GT(events_.size(), 1);
    const size_t count = events_.size() - 1;
    std::vector<double> result;
    result.reserve(count);
    for (size_t i = 0; i != count; ++i) {
      result.push_back(
         (events_[i + 1].time - events_[i].time));
    }
    return result;
  }

  // Returns the average and standard deviation of the deltas.
  std::string DeltasDebugString() const {
    const auto deltas = ComputeDeltasInSeconds();
    const double sum = std::accumulate(deltas.begin(), deltas.end(), 0.);
    const double mean = sum / deltas.size();

    double squared_sum = 0.;
    for (const double x : deltas) {
      squared_sum += common::Pow2(x - mean);
    }
    const double sigma = std::sqrt(squared_sum / (deltas.size() - 1));

    std::ostringstream out;
    out << std::scientific << std::setprecision(2) << mean << " s +/- " << sigma
        << " s";
    return out.str();
  }

  std::deque<Event> events_;
  double window_duration_;
};

}  // namespace common


#endif  //COMMON_RATE_TIMER_H_
