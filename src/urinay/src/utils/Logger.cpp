/**
 * @file Logger.cpp
 * @author Oriol Gorriz (origovi2000@gmail.com)
 * @brief Contains the Logger class member functions implementation
 * @version 1.0
 * @date 2025-04-12
 *
 * @copyright Copyright (c) 2025 TUfast e.V.
 */

#include "utils/Logger.hpp"

#include <iostream>

std::map<std::string, Logger::Task> Logger::tasks_;
std::queue<std::string> Logger::info_msgs_, Logger::warn_msgs_;
bool Logger::print_immediately;

void Logger::tick(const std::string &clockName) {
  Task::Clock::time_point now = Task::Clock::now();
  std::map<std::string, Task>::iterator it = tasks_.find(clockName);
  if (it != tasks_.end()) {
    if (it->second.active) {
      std::cerr << "[urinay] WARN: Called tick() two times with same clockName before calling tock()" << std::endl;
    }
    else {
      it->second.activate(now);
    }
  } else {
    tasks_.emplace(clockName, Task(now));
  }
}

double Logger::tock(const std::string &clockName) {
  Task::Clock::time_point now = Task::Clock::now();
  std::map<std::string, Task>::iterator it = tasks_.find(clockName);
  if (it == tasks_.end()) {
    std::cerr << "[urinay] ERROR: Called tock() before calling tick()" << std::endl;
    return 0.0;
  } else {
    Task::Duration duration = it->second.stop(now);
    if (print_immediately) std::cout << "[urinay] " << clockName << " has taken: " << duration.count() * 1e3 << "ms" << std::endl;
    return duration.count();
  }
}

void Logger::loginfo(const std::string &msg) {
  if (print_immediately)
    std::cout << "[urinay] " << msg << std::endl;
  else
    info_msgs_.push(msg);
}

void Logger::logwarn(const std::string &msg) {
  if (print_immediately)
    std::cout << "[urinay] WARN: " << msg << std::endl;
  else
    warn_msgs_.push(msg);
}

void Logger::print_report() {
  std::stringstream to_print;
  // Table header
  to_print << "+-------------------------------------------------------+\n";
  to_print << "|                      URINAY LOG                       |\n";
  to_print << "+-------------------------------------------------------+\n";
  to_print << std::fixed << std::setprecision(2);
  to_print << "| " << std::setw(21) << std::left << "Task Name"
  << std::setw(11) << "Last (ms)"
  << std::setw(11) << "Max (ms)"
  << std::setw(11) << "Avg (ms)" << "|\n";
  to_print << "+-------------------------------------------------------+\n";

  // Print each task time stats
  for (const auto& pair : tasks_) {
    const Task& chrono = pair.second;
    to_print << "| " << std::setw(21) << std::left << pair.first
             << std::setw(11) << chrono.last_duration.count() * 1e3
             << std::setw(11) << chrono.max_duration.count() * 1e3
             << std::setw(11) << (chrono.total_time_active.count() * 1e3) / chrono.count << "|\n";
  }

  to_print << "+-------------------------------------------------------+\n";

  // Print log messages
  if (!info_msgs_.empty() or !warn_msgs_.empty()) {
    while (!warn_msgs_.empty()) {
      to_print << "| " << std::setw(54) << std::left << "[WARN] " + warn_msgs_.front() << "|\n";
      warn_msgs_.pop();
    }
    while (!info_msgs_.empty()) {
      to_print << "| " << std::setw(54) << std::left << "[INFO] " + info_msgs_.front() << "|\n";
      info_msgs_.pop();
    }
  } else {
    to_print << "| " << std::setw(54) << std::left << "No logs to show..." << "|\n";
  }

  to_print << "+-------------------------------------------------------+\n";

  std::cout << '\n' << to_print.str() << std::endl;
}
