/*
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/


#ifndef GOAL_WORKER_HPP
#define GOAL_WORKER_HPP

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

namespace naoqi
{
namespace action
{

/**
 * @brief Runs action goals on threads that are joined, never detached. The driver
 * stops and joins them before the action server, node and session go away, so no
 * goal thread can outlive the objects it uses.
 */
class GoalWorker
{
public:
  GoalWorker() = default;
  GoalWorker(const GoalWorker &) = delete;
  GoalWorker & operator=(const GoalWorker &) = delete;
  ~GoalWorker() {shutdown();}

  /** Runs work on a new thread; threads of finished goals are joined first. */
  void start(std::function<void()> work)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    joinFinishedLocked();
    auto done = std::make_shared<std::atomic<bool>>(false);
    threads_.push_back(
      Entry{std::thread(
          [work = std::move(work), done] {
            work();
            *done = true;
          }), done});
  }

  /** Asks running goals to give up; goals that wait or poll check stopping(). */
  void requestStop() {stopping_ = true;}

  bool stopping() const {return stopping_;}

  /** Waits for every goal thread. */
  void join()
  {
    std::vector<Entry> threads;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      threads.swap(threads_);
    }
    for (auto & entry : threads) {
      if (entry.thread.joinable()) {
        entry.thread.join();
      }
    }
  }

  void shutdown()
  {
    requestStop();
    join();
  }

private:
  struct Entry
  {
    std::thread thread;
    std::shared_ptr<std::atomic<bool>> done;
  };

  void joinFinishedLocked()
  {
    for (auto it = threads_.begin(); it != threads_.end(); ) {
      if (*it->done) {
        it->thread.join();
        it = threads_.erase(it);
      } else {
        ++it;
      }
    }
  }

  std::mutex mutex_;
  std::vector<Entry> threads_;
  std::atomic<bool> stopping_{false};
};

/**
 * @brief Owns an action server (or a group of servers and services) and, when
 * destroyed, runs its shutdown hook first: stop the running goal in NAOqi, then join
 * the goal threads. Only then are the servers released.
 */
class ActionHandle
{
public:
  ActionHandle(std::shared_ptr<void> servers, std::function<void()> shutdown)
  : servers_(std::move(servers)), shutdown_(std::move(shutdown)) {}
  ~ActionHandle()
  {
    if (shutdown_) {
      shutdown_();
    }
  }
  ActionHandle(const ActionHandle &) = delete;
  ActionHandle & operator=(const ActionHandle &) = delete;

private:
  std::shared_ptr<void> servers_;
  std::function<void()> shutdown_;
};

}  // namespace action
}  // namespace naoqi

#endif
