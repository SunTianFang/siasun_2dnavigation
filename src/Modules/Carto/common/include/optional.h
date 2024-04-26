
#ifndef COMMON_OPTIONAL_H_
#define COMMON_OPTIONAL_H_

#include <memory>

#include "make_unique.h"
#include "mylogging.h"

using namespace myLogging;

namespace common {

template <class T>
class optional {
 public:
  optional() {}

  optional(const optional& other) {
    if (other.has_value()) {
      value_ = common::make_unique<T>(other.value());
    }
  }

  explicit optional(const T& value) { value_ = common::make_unique<T>(value); }

  bool has_value() const { return value_ != nullptr; }

  const T& value() const {
    CHECK(value_ != nullptr);
    return *value_;
  }

  optional<T>& operator=(const T& other_value) {
    this->value_ = common::make_unique<T>(other_value);
    return *this;
  }

  optional<T>& operator=(const optional<T>& other) {
    if (!other.has_value()) {
      this->value_ = nullptr;
    } else {
      this->value_ = common::make_unique<T>(other.value());
    }
    return *this;
  }

 private:
  std::unique_ptr<T> value_;
};

}  // namespace common


#endif  // COMMON_OPTIONAL_H_
