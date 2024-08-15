#ifndef THREAD_PUBLISHER_H_
#define THREAD_PUBLISHER_H_

#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <queue>
#include <boost/bind.hpp>
#include <functional>

template <class T>
class ThreadPublisher{
public:
  ThreadPublisher(){ 
    shutdown_requested = false;
    stopped = false;
  }

  ~ThreadPublisher() {}

  void Register(std::function<void(const std::shared_ptr<const T>&)> cb){
    callbacks.push_back(cb);
  }

  void Start(){
    publish_thread = std::thread(boost::bind(&ThreadPublisher::Process, this));
  }

  void RequestShutdown(){
    std::unique_lock<std::mutex> locker(shutdown_mutex);
    shutdown_requested = true;
    locker.unlock(); 
  }

  bool ShutdownRequested(){
    bool is_requested;
    std::unique_lock<std::mutex> locker(shutdown_mutex);
    is_requested = shutdown_requested;
    locker.unlock(); 
    return is_requested;
  }

  void Publish(const std::shared_ptr<const T> msg){
    std::unique_lock<std::mutex> locker(msg_mutex);
    msgs.push(msg);
    locker.unlock();
    msg_cond.notify_one();
  }

  void Process(){
    while(!ShutdownRequested()){
      std::shared_ptr<const T> msg;
      std::unique_lock<std::mutex> locker(msg_mutex);
      while(msgs.empty()){
        if(ShutdownRequested()){
          locker.unlock();
          break;
        }else{
          msg_cond.wait(locker);
        }
      }

      if(ShutdownRequested()){
        break;
      }else{
        if(0){
          msg = msgs.back();
          while(!msgs.empty()){
            msgs.pop();
          }
        }else{
          msg = msgs.front();
          msgs.pop();
        }
      }
      locker.unlock();
      for(auto callback : callbacks){
        callback(msg);
      }
    }
    stopped = true;
  }

  void ShutDown(){
    // shutdown_requested = true;
    // msg_cond.notify_one();
    while(!stopped){
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      RequestShutdown();
      msg_cond.notify_one();
    }

    if(publish_thread.joinable()){
      publish_thread.join();
    }
  }

private:
  std::mutex msg_mutex;
  std::mutex shutdown_mutex;
  std::condition_variable msg_cond;
  std::queue<std::shared_ptr<const T> > msgs;

  std::thread publish_thread;
  std::vector<std::function<void(const std::shared_ptr<const T>&)>> callbacks;

  bool shutdown_requested;
  bool stopped;
};

#endif  // THREAD_PUBLISHER_H_