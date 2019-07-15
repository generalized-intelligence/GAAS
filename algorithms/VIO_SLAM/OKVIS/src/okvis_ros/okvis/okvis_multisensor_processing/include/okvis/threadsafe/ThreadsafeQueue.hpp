/*********************************************************************************
 *  OKVIS - Open Keyframe-based Visual-Inertial SLAM
 *  Copyright (c) 2015, Autonomous Systems Lab / ETH Zurich
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 * 
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *   * Neither the name of Autonomous Systems Lab / ETH Zurich nor the names of
 *     its contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 *  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 *  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 *  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  Created on: 2013
 *      Author: Simon Lynen
 *    Modified: Stefan Leutenegger (s.leutenegger@imperial.ac.uk)
 *********************************************************************************/

/**
 * @file ThreadsafeQueue.hpp
 * @brief Header file for the ThreadsafeQueue class.
 * @author Simon Lynen
 * @author Stefan Leutenegger
 */

#ifndef INCLUDE_OKVIS_THREADSAFE_THREADSAFEQUEUE_HPP_
#define INCLUDE_OKVIS_THREADSAFE_THREADSAFEQUEUE_HPP_

#include <atomic>
#include <pthread.h>
#include <queue>
#include <string>
#include <sys/time.h>

#include <glog/logging.h>

/// \brief okvis Main namespace of this package.
namespace okvis {

/// \brief Namespace for helper classes for threadsafe operation.
namespace threadsafe {

class ThreadSafeQueueBase {
 public:
  ThreadSafeQueueBase() = default;
  virtual ~ThreadSafeQueueBase() {}
  virtual void NotifyAll() const = 0;
  virtual void Shutdown() = 0;
  virtual void Resume() = 0;
  virtual size_t Size() const = 0;
  virtual bool Empty() const = 0;
};

/**
 * @brief Class that implements a threadsafe FIFO queue.
 * @tparam QueueType Datatype that is safed in the queue.
 */
template<typename QueueType>
class ThreadSafeQueue {
  friend bool test_funcs(void* (*)(void*), void* (*)(void*),  // NOLINT
                         const std::string&, bool);

 public:

  /// \brief Notify all waiting threads. Only used in destructor and when shutting down.
  virtual void NotifyAll() const final {
    pthread_cond_broadcast(&condition_empty_);
    pthread_cond_broadcast(&condition_full_);
  }

  /// \brief Constructor.
  ThreadSafeQueue() {
    shutdown_ = false;
    pthread_mutex_init(&mutex_, NULL);
    pthread_cond_init(&condition_empty_, NULL);
    pthread_cond_init(&condition_full_, NULL);
  }

  /// \brief Destructor.
  virtual ~ThreadSafeQueue() {
    shutdown_ = true;
    NotifyAll();
    pthread_mutex_destroy(&mutex_);
    pthread_cond_destroy(&condition_empty_);
    pthread_cond_destroy(&condition_full_);
  }

  /// \brief Tell the queue shut down. This will notify all threads to wake up.
  virtual void Shutdown() final {
    shutdown_ = true;
    NotifyAll();
  }

  /// \brief Tell the queue to resume after a shutdown request.
  virtual void Resume() final {
    shutdown_ = false;
    NotifyAll();
  }

  /// \brief Push non-blocking to the queue.
  void Push(const QueueType& value) {
    PushNonBlocking(value);
  }

  /// \brief Push to the queue.
  void PushNonBlocking(const QueueType& value) {
    pthread_mutex_lock(&mutex_);
    queue_.push(value);
    pthread_cond_signal(&condition_empty_);  // Signal that data is available.
    pthread_mutex_unlock(&mutex_);
  }

  /// \brief Return the size of the queue.
  virtual size_t Size() const final {
    pthread_mutex_lock(&mutex_);
    size_t size = queue_.size();
    pthread_mutex_unlock(&mutex_);
    return size;
  }

  /// \brief Return true if the queue is empty.
  virtual bool Empty() const final {
    pthread_mutex_lock(&mutex_);
    bool empty = queue_.empty();
    pthread_mutex_unlock(&mutex_);
    return empty;
  }

  /// \brief Push to the queue if the size is less than max_queue_size, else block.
  /// \param[in] value New entry in queue.
  /// \param[in] max_queue_size Maximum queue size.
  /// \return False if shutdown is requested.
  bool PushBlockingIfFull(const QueueType& value, size_t max_queue_size) {
    while (!shutdown_) {
      pthread_mutex_lock(&mutex_);
      size_t size = queue_.size();
      if (size >= max_queue_size) {
        pthread_cond_wait(&condition_full_, &mutex_);
      }
      if (size >= max_queue_size) {
        pthread_mutex_unlock(&mutex_);
        continue;
      }
      queue_.push(value);
      pthread_cond_signal(&condition_empty_);  // Signal that data is available.
      pthread_mutex_unlock(&mutex_);
      return true;
    }
    return false;
  }

  /// \brief Push to the queue. If full, drop the oldest entry.
  /// \param[in] value New entry in queue.
  /// \param[in] max_queue_size Maximum queue size.
  /// \return True if oldest was dropped because queue was full.
  bool PushNonBlockingDroppingIfFull(const QueueType& value,
                                     size_t max_queue_size) {
    pthread_mutex_lock(&mutex_);
    bool result = false;
    if (queue_.size() >= max_queue_size) {
      queue_.pop();
      result = true;
    }
    queue_.push(value);
    pthread_cond_signal(&condition_empty_);  // Signal that data is available.
    pthread_mutex_unlock(&mutex_);
    return result;
  }

  /**
   * @brief Get the oldest entry still in the queue. Blocking if queue is empty.
   * @param[out] value Oldest entry in queue.
   * @return False if shutdown is requested.
   */
  bool Pop(QueueType* value) {
    return PopBlocking(value);
  }

  /**
   * @brief Get the oldest entry still in the queue. Blocking if queue is empty.
   * @param[out] value Oldest entry in queue.
   * @return False if shutdown is requested.
   */
  bool PopBlocking(QueueType* value) {
    CHECK_NOTNULL(value);
    while (!shutdown_) {
      pthread_mutex_lock(&mutex_);
      if (queue_.empty()) {
        pthread_cond_wait(&condition_empty_, &mutex_);
      }
      if (queue_.empty()) {
        pthread_mutex_unlock(&mutex_);
        continue;
      }
      QueueType _value = queue_.front();
      queue_.pop();
      pthread_cond_signal(&condition_full_);  // Notify that space is available.
      pthread_mutex_unlock(&mutex_);
      *value = _value;
      return true;
    }
    return false;
  }

  /**
   * @brief Get the oldest entry still in the queue. If queue is empty value is not altered.
   * @param[out] value Oldest entry in queue if queue was not empty.
   * @return True if queue was not empty.
   */
  bool PopNonBlocking(QueueType* value) {
    CHECK_NOTNULL(value);
    pthread_mutex_lock(&mutex_);
    if (queue_.empty()) {
      pthread_mutex_unlock(&mutex_);
      return false;
    }
    *value = queue_.front();
    queue_.pop();
    pthread_mutex_unlock(&mutex_);
    return true;
  }

  /**
   * @brief Get the oldest entry still in the queue. If the queue is empty wait for a given
   *        amount of time. If during this time an entry was pushed alter the value. If the
   *        queue is still empty, the value is not altered and it will return false
   * @param[out] value Oldest entry in queue if queue was not empty.
   * @param timeout_nanoseconds Maximum amount of time to wait for an entry if queue is empty.
   * @return True if value was updated. False if queue was empty and no new entry was pushed
   *         during the given timeout.
   */
  bool PopTimeout(QueueType* value, int64_t timeout_nanoseconds) {
    CHECK_NOTNULL(value);
    pthread_mutex_lock(&mutex_);
    if (queue_.empty()) {
      struct timeval tv;
      struct timespec ts;
      gettimeofday(&tv, NULL);
      ts.tv_sec = tv.tv_sec;
      ts.tv_nsec = tv.tv_usec * 1e3 + timeout_nanoseconds;
      pthread_cond_timedwait(&condition_empty_, &mutex_, &ts);
    }
    if (queue_.empty()) {
      pthread_mutex_unlock(&mutex_);
      return false;
    }
    QueueType _value = queue_.front();
    queue_.pop();
    pthread_cond_signal(&condition_full_);  // Notify that space is available.
    pthread_mutex_unlock(&mutex_);
    *value = _value;
    return true;
  }

  /**
   * @brief Get a copy of the front / oldest element in the queue. If queue is empty
   *        value is not altered. The queue itself is not changed, i.e. the returned
   *        element is still in the queue.
   * @param[out] value Oldest entry in queue if queue was not empty.
   * @return True if queue was not empty and value was updated.
   */
  bool getCopyOfFront(QueueType* value) {
    CHECK_NOTNULL(value);
    pthread_mutex_lock(&mutex_);
    if (queue_.empty()) {
      pthread_mutex_unlock(&mutex_);
      return false;
    }
    // COPY the value.
    *value = queue_.front();
    pthread_mutex_unlock(&mutex_);
    return true;
  }

  /**
   * @brief Get a copy of the front / oldest element in the queue. Blocking if queue is empty.
   *        The queue itself is not changed, i.e. the returned element is still in the queue.
   * @param value Oldest entry in the queue.
   * @return False if shutdown is requested.
   */
  bool getCopyOfFrontBlocking(QueueType* value) {
    CHECK_NOTNULL(value);
    while (!shutdown_) {
      pthread_mutex_lock(&mutex_);
      if (queue_.empty()) {
        pthread_cond_wait(&condition_empty_, &mutex_);
      }
      if (queue_.empty()) {
        pthread_mutex_unlock(&mutex_);
        continue;
      }
      *value = queue_.front();
      pthread_mutex_unlock(&mutex_);
      return true;
    }
    return false;
  }

  /**
   * @brief Get a copy of the back / newest element in the queue. If queue is empty
   *        value is not altered. The queue itself is not changed, i.e. the returned
   *        element is still in the queue.
   * @param[out] value Newest entry in queue if queue was not empty.
   * @return True if queue was not empty and value was updated.
   */
  bool getCopyOfBack(QueueType* value) {
    CHECK_NOTNULL(value);
    pthread_mutex_lock(&mutex_);
    if (queue_.empty()) {
      pthread_mutex_unlock(&mutex_);
      return false;
    }
    // COPY the value.
    *value = queue_.back();
    pthread_mutex_unlock(&mutex_);
    return true;
  }


  mutable pthread_mutex_t mutex_;           ///< The queue mutex.
  mutable pthread_cond_t condition_empty_;  ///< Condition variable to wait and signal that queue is not empty.
  mutable pthread_cond_t condition_full_;   ///< Condition variable to wait and signal when an element is popped.
  std::queue<QueueType> queue_;             ///< Actual queue.
  std::atomic_bool shutdown_;               ///< Flag if shutdown is requested.

};

}  // namespace threadsafe

}  // namespace okvis

#endif  // INCLUDE_OKVIS_THREADSAFE_THREADSAFEQUEUE_HPP_
