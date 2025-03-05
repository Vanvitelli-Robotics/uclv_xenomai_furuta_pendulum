
#ifndef XENO_TOOLS__REALTIME_PUBLISHER_HPP_
#define XENO_TOOLS__REALTIME_PUBLISHER_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rclcpp/publisher.hpp"
#include <alchemy/task.h>
#include <alchemy/mutex.h>


namespace xeno_tools
{
template <class Msg>
class RealtimePublisher
{
private:
  using PublisherSharedPtr = typename rclcpp::Publisher<Msg>::SharedPtr;

public:
  /// The msg_ variable contains the data that will get published on the ROS topic.
  Msg msg_;

  /**  \brief Constructor for the realtime publisher
   *
   * \param publisher the publisher to wrap
   */
  explicit RealtimePublisher(PublisherSharedPtr publisher)
  : publisher_(publisher), is_running_(false), keep_running_(true), turn_(LOOP_NOT_STARTED)
  {
    thread_ = std::thread(&RealtimePublisher::publishingLoop, this);
    
      if (rt_mutex_create(&mutex,"my_mutex") != 0) {
        fprintf(stderr, "Errore nella creazione del mutex.\n");
    }
  }

  RealtimePublisher() : is_running_(false), keep_running_(false), turn_(LOOP_NOT_STARTED) {}

  /// Destructor
  ~RealtimePublisher()
  {
    stop();
    while (is_running()) {
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  /// Stop the realtime publisher from sending out more ROS messages
  void stop()
  {
    keep_running_ = false;
#ifdef NON_POLLING
    updated_cond_.notify_one();  // So the publishing loop can exit
#endif
  }

  /**  \brief Try to get the data lock from realtime
   *
   * To publish data from the realtime loop, you need to run trylock to
   * attempt to get unique access to the msg_ variable. Trylock returns
   * true if the lock was acquired, and false if it failed to get the lock.
   */
  bool trylock()
  {
    if (rt_mutex_acquire(&mutex, 1000000)==0) {
      if (turn_ == REALTIME) {
        return true;
      } else {
        rt_mutex_release(&mutex);
        return false;
      }
    } else {
      return false;
    }
  }

  /**  \brief Unlock the msg_ variable
   *
   * After a successful trylock and after the data is written to the mgs_
   * variable, the lock has to be released for the message to get
   * published on the specified topic.
   */
  void unlockAndPublish()
  {
    turn_ = NON_REALTIME;
    unlock();
  }

  /**  \brief Get the data lock form non-realtime
   *
   * To publish data from the realtime loop, you need to run trylock to
   * attempt to get unique access to the msg_ variable. Trylock returns
   * true if the lock was acquired, and false if it failed to get the lock.
   */
  void lock()
  {
#ifdef NON_POLLING
    rt_mutex_acquire(&mutex, 1000000);
#else
    // never actually block on the lock
    while (rt_mutex_acquire(&mutex, 1000000)!=0) {
      std::this_thread::sleep_for(std::chrono::microseconds(200));
    }
#endif
  }

  /**  \brief Unlocks the data without publishing anything
   *
   */
  void unlock()
  {
    rt_mutex_release(&mutex);
#ifdef NON_POLLING
    updated_cond_.notify_one();
#endif
  }

private:
  // non-copyable
  RealtimePublisher(const RealtimePublisher &) = delete;
  RealtimePublisher & operator=(const RealtimePublisher &) = delete;

  bool is_running() const { return is_running_; }

  void publishingLoop()
  {
    is_running_ = true;
    turn_ = REALTIME;

    while (keep_running_) {
      Msg outgoing;

      // Locks msg_ and copies it

#ifdef NON_POLLING
      std::unique_lock<std::mutex> lock_(msg_mutex_);
#else
      lock();
#endif

      while (turn_ != NON_REALTIME && keep_running_) {
#ifdef NON_POLLING
        updated_cond_.wait(lock_);
#else
        unlock();
        std::this_thread::sleep_for(std::chrono::microseconds(500));
        lock();
#endif
      }
      outgoing = msg_;
      turn_ = REALTIME;

      unlock();

      // Sends the outgoing message
      if (keep_running_) {
        publisher_->publish(outgoing);
      }
    }
    is_running_ = false;
  }

  PublisherSharedPtr publisher_;
  std::atomic<bool> is_running_;
  std::atomic<bool> keep_running_;

  std::thread thread_;
  
  //std::mutex msg_mutex_;  // Protects msg_
  RT_MUTEX mutex;
#ifdef NON_POLLING
  std::condition_variable updated_cond_;
#endif

  enum { REALTIME, NON_REALTIME, LOOP_NOT_STARTED };
  std::atomic<int> turn_;  // Who's turn is it to use msg_?
};

template <class Msg>
using RealtimePublisherSharedPtr = std::shared_ptr<RealtimePublisher<Msg>>;

}  // namespace xeno_tools
#endif  // XENO_TOOLS__REALTIME_PUBLISHER_HPP_
