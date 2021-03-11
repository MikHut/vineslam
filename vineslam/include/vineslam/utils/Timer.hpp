#pragma once

#include <iostream>
#include <string>
#include <chrono>

namespace vineslam
{
class Timer
{
public:
  Timer()
  {
    log_ = "";
    prefix_ = "";
  }

  void tick(const std::string& prefix)
  {
    start_time_ = std::chrono::high_resolution_clock::now();
    prefix_ = prefix;
  }

  void tock()
  {
    std::chrono::high_resolution_clock::time_point end_time = std::chrono::high_resolution_clock::now();
    float duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time_).count();

    log_ += prefix_ + " took " + std::to_string(duration) + " ms.\n";
  }

  void getLog()
  {
    std::cout << "\n************************************************\n";
    std::cout << log_;
    std::cout << "\n------------------------------------------------\n\n";
  }

  void clearLog()
  {
    log_.clear();
  }

private:
  std::string log_;
  std::string prefix_;
  std::chrono::high_resolution_clock::time_point start_time_;
};
}  // namespace vineslam