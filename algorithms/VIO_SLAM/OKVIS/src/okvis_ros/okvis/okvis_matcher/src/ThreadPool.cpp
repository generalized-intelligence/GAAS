// Adapted from https://github.com/progschj/ThreadPool on September 3, 2014
// Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich

// Original copyright:
// Copyright (c) 2012 Jakob Progsch
//
// This software is provided 'as-is', without any express or implied
// warranty. In no event will the authors be held liable for any damages
// arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
//
//   1. The origin of this software must not be misrepresented; you must not
//   claim that you wrote the original software. If you use this software
//   in a product, an acknowledgment in the product documentation would be
//   appreciated but is not required.
//
//   2. Altered source versions must be plainly marked as such, and must not be
//   misrepresented as being the original software.
//
//   3. This notice may not be removed or altered from any source
//   distribution.

/*
 *    Modified: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 */

/**
 * @file ThreadPool.cpp
 * @brief Source file for the ThreadPool class.
 * @author Jakob Progsch
 * @author Stefan Leutenegger
 */


#include <okvis/ThreadPool.hpp>

/// \brief okvis Main namespace of this package.
namespace okvis {

// The constructor just launches some amount of workers.
ThreadPool::ThreadPool(size_t threads)
    : active_threads_(0),
      stop_(false)
{
  for (size_t i = 0; i < threads; ++i)
    workers_.emplace_back(std::bind(&ThreadPool::run, this));
}

// Destructor. This joins all threads.
ThreadPool::~ThreadPool()
{
  {
    std::unique_lock<std::mutex> lock(tasks_mutex_);
    stop_ = true;
  }
  tasks_condition_.notify_all();
  for (size_t i = 0; i < workers_.size(); ++i) {
    workers_[i].join();
  }
}

// Run a single thread.
void ThreadPool::run()
{
  while (true) {
    std::unique_lock<std::mutex> lock(this->tasks_mutex_);
    while (!this->stop_ && this->tasks_.empty()) {
      this->tasks_condition_.wait(lock);
    }
    if (this->stop_ && this->tasks_.empty()) {
      return;
    }
    std::function<void()> task(this->tasks_.front());
    this->tasks_.pop();
    ++active_threads_;
    // Unlock the queue while we execute the task.
    lock.unlock();
    task();
    lock.lock();
    --active_threads_;
    // This is the secret to making the waitForEmptyQueue() function work.
    // After finishing a task, notify that this work is done.
    wait_condition_.notify_all();
  }
}

// This method blocks until the queue is empty.
void ThreadPool::waitForEmptyQueue() const
{
  std::unique_lock<std::mutex> lock(this->tasks_mutex_);
  // Only exit if all tasks are complete by tracking the number of
  // active threads.
  while (active_threads_ || !tasks_.empty()) {
    this->wait_condition_.wait(lock);
  }
}
}  // namespace okvis
