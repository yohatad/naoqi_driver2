// A NAOqi stand-in for tests: a standalone qi::Session with fake ALMemory, ALSystem,
// ALMotion, dialog and playback services that record every call they receive. Used
// in-process by the unit tests, and on its own by fake_naoqi_server.

#ifndef NAOQI_DRIVER_TEST_FAKE_NAOQI_HPP
#define NAOQI_DRIVER_TEST_FAKE_NAOQI_HPP

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <boost/make_shared.hpp>
#include <qi/anyobject.hpp>
#include <qi/session.hpp>

namespace naoqi_test
{

/** Thread-safe log of calls ("move(0.1,0,0)"), with a way to wait for one. */
class CallLog
{
public:
  void add(const std::string & call)
  {
    std::function<void(const std::string &)> observer;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      calls_.push_back(call);
      observer = observer_;
    }
    cv_.notify_all();
    if (observer) {
      observer(call);
    }
  }

  /** Also passes every call to observer (fake_naoqi_server prints them). */
  void setObserver(std::function<void(const std::string &)> observer)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    observer_ = std::move(observer);
  }

  std::vector<std::string> calls() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return calls_;
  }

  /** True once a call starting with prefix has been made (or timeout). */
  bool waitFor(
    const std::string & prefix,
    std::chrono::milliseconds timeout = std::chrono::seconds(2))
  {
    std::unique_lock<std::mutex> lock(mutex_);
    // A system_clock deadline: GCC 11's ThreadSanitizer misses the mutex release
    // in the steady_clock wait (pthread_cond_clockwait) and reports false races.
    return cv_.wait_until(
      lock, std::chrono::system_clock::now() + timeout,
      [&] {return countLocked(prefix) > 0;});
  }

  size_t count(const std::string & prefix) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return countLocked(prefix);
  }

  void clear()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    calls_.clear();
  }

private:
  size_t countLocked(const std::string & prefix) const
  {
    size_t n = 0;
    for (const auto & c : calls_) {
      if (c.rfind(prefix, 0) == 0) {
        ++n;
      }
    }
    return n;
  }

  mutable std::mutex mutex_;
  std::condition_variable cv_;
  std::vector<std::string> calls_;
  std::function<void(const std::string &)> observer_;
};

template<typename T>
std::string join(const std::vector<T> & values)
{
  std::ostringstream out;
  for (size_t i = 0; i < values.size(); ++i) {
    out << (i ? "," : "") << values[i];
  }
  return out.str();
}

/** What ALMemory.subscriber(key) returns: an object with a "signal" signal. */
class FakeMemorySubscriber
{
public:
  qi::Signal<qi::AnyValue> signal;
};

class FakeMemory
{
public:
  explicit FakeMemory(CallLog & log)
  : log_(log) {}

  std::string getData(const std::string & key)
  {
    if (key == "RobotConfig/Body/Type") {
      return "juliette";
    }
    if (key == "RobotConfig/Body/BaseVersion") {
      return "1.8";
    }
    return "";
  }

  qi::AnyObject subscriber(const std::string & key)
  {
    log_.add("subscriber(" + key + ")");
    auto sub = boost::make_shared<FakeMemorySubscriber>();
    std::lock_guard<std::mutex> lock(mutex_);
    subscribers_.emplace_back(key, sub);
    return qi::AnyObject(sub);
  }

  void insertData(const std::string & key, const qi::AnyValue & value)
  {
    if (fail_inserts) {
      throw std::runtime_error("ALMemory is unreachable");
    }
    {
      std::lock_guard<std::mutex> lock(mutex_);
      inserted_[key] = value;
    }
    log_.add("insertData(" + key + ")");
  }

  /** Makes insertData throw, as calls do when NAOqi is unreachable. Not a NAOqi member. */
  std::atomic<bool> fail_inserts{false};

  /** The last value inserted under key, or an invalid value. Not a NAOqi method. */
  qi::AnyValue inserted(const std::string & key)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = inserted_.find(key);
    return it == inserted_.end() ? qi::AnyValue() : it->second;
  }

  /** Fires the signal of every subscriber whose key starts with prefix. */
  void emit(const std::string & prefix, const std::string & value)
  {
    std::vector<boost::shared_ptr<FakeMemorySubscriber>> targets;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto & [key, sub] : subscribers_) {
        if (key.rfind(prefix, 0) == 0) {
          targets.push_back(sub);
        }
      }
    }
    for (const auto & sub : targets) {
      sub->signal(qi::AnyValue::from(value));
    }
  }

private:
  CallLog & log_;
  std::mutex mutex_;
  std::vector<std::pair<std::string, boost::shared_ptr<FakeMemorySubscriber>>> subscribers_;
  std::map<std::string, qi::AnyValue> inserted_;
};

class FakeDialog
{
public:
  explicit FakeDialog(CallLog & log)
  : log_(log) {}
  std::string getLanguage() {return "English";}
  void loadTopicContent(const std::string & content)
  {
    log_.add("loadTopicContent(" + content + ")");
  }
  void activateTopic(const std::string & name) {log_.add("activateTopic(" + name + ")");}
  void deactivateTopic(const std::string & name) {log_.add("deactivateTopic(" + name + ")");}
  void unloadTopic(const std::string & name) {log_.add("unloadTopic(" + name + ")");}
  void subscribe(const std::string & name) {log_.add("subscribe(" + name + ")");}
  void setFocus(const std::string & name) {log_.add("setFocus(" + name + ")");}

private:
  CallLog & log_;
};

class FakeSpeechRecognition
{
public:
  void _enableFreeSpeechToText() {}
};

class FakeSystem
{
public:
  std::string systemVersion() {return "2.9.5.172";}
};

class FakeMotion
{
public:
  explicit FakeMotion(CallLog & log)
  : log_(log) {}

  std::vector<std::vector<qi::AnyValue>> getRobotConfig()
  {
    return {
      {qi::AnyValue::from(std::string("Model Type"))},
      {qi::AnyValue::from(std::string("juliette_Y20"))}};
  }
  std::vector<std::string> getSensorNames() {return {};}

  void move(float x, float y, float theta)
  {
    std::ostringstream out;
    out << "move(" << x << "," << y << "," << theta << ")";
    log_.add(out.str());
  }
  void stopMove() {log_.add("stopMove()");}
  void moveTo(float x, float y, float theta)
  {
    std::ostringstream out;
    out << "moveTo(" << x << "," << y << "," << theta << ")";
    log_.add(out.str());
  }
  void setAngles(
    const std::vector<std::string> & names, const std::vector<float> & angles,
    float speed)
  {
    for (const auto & name : names) {
      if (name == "BadJoint") {
        throw std::runtime_error("Unknown joint: " + name);  // as NAOqi does
      }
    }
    std::ostringstream out;
    out << "setAngles(" << join(names) << ";" << join(angles) << ";" << speed << ")";
    log_.add(out.str());
  }
  void changeAngles(
    const std::vector<std::string> & names, const std::vector<float> & angles,
    float speed)
  {
    std::ostringstream out;
    out << "changeAngles(" << join(names) << ";" << join(angles) << ";" << speed << ")";
    log_.add(out.str());
  }
  void angleInterpolation(qi::AnyValue names, qi::AnyValue, qi::AnyValue, bool absolute)
  {
    log_.add(std::string("angleInterpolation(") + (absolute ? "absolute" : "relative") + ")");
    (void)names;
  }
  void angleInterpolationBezier(qi::AnyValue, qi::AnyValue, qi::AnyValue)
  {
    log_.add("angleInterpolationBezier()");
  }
  std::vector<float> getRobotPosition(bool) {return {0.f, 0.f, 0.f};}
  std::vector<float> getRobotVelocity() {return {0.f, 0.f, 0.f};}

private:
  CallLog & log_;
};

/** The robot's voice, shared by the speech fakes. */
class FakeVoice
{
public:
  /** Speaks for duration_ms, or until stop(). */
  void speak()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    stopped_ = false;
    cv_.wait_until(
      lock, std::chrono::system_clock::now() + std::chrono::milliseconds(duration_ms.load()),
      [this] {return stopped_;});
  }

  void stop()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopped_ = true;
    }
    cv_.notify_all();
  }

  std::atomic<int> duration_ms{200};

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  bool stopped_ = false;
};

/** ALTextToSpeech: its stopAll() also ends animated speech. */
class FakeTextToSpeech
{
public:
  FakeTextToSpeech(CallLog & log, std::shared_ptr<FakeVoice> voice)
  : log_(log), voice_(std::move(voice)) {}
  void say(const std::string & text) {log_.add("ALTextToSpeech.say(" + text + ")");}
  void stopAll()
  {
    log_.add("ALTextToSpeech.stopAll()");
    voice_->stop();
  }

private:
  CallLog & log_;
  std::shared_ptr<FakeVoice> voice_;
};

/** ALAnimatedSpeech as on NAOqi 2.5: say() returns once speech ends, and no stopAll(). */
class FakeAnimatedSpeech
{
public:
  FakeAnimatedSpeech(CallLog & log, std::shared_ptr<FakeVoice> voice)
  : log_(log), voice_(std::move(voice)) {}
  void say(const std::string & text, const std::map<std::string, std::string> &)
  {
    log_.add("ALAnimatedSpeech.say(" + text + ")");
    voice_->speak();
  }

private:
  CallLog & log_;
  std::shared_ptr<FakeVoice> voice_;
};

/** A service whose only method records "<name>.stopAll()". */
class FakeStoppable
{
public:
  FakeStoppable(CallLog & log, std::string name)
  : log_(log), name_(std::move(name)) {}
  void stopAll() {log_.add(name_ + ".stopAll()");}

private:
  CallLog & log_;
  std::string name_;
};

}  // namespace naoqi_test

QI_REGISTER_OBJECT(naoqi_test::FakeMemorySubscriber, signal);
QI_REGISTER_OBJECT(naoqi_test::FakeMemory, getData, subscriber, insertData);
QI_REGISTER_OBJECT(
  naoqi_test::FakeDialog, getLanguage, loadTopicContent, activateTopic, deactivateTopic,
  unloadTopic, subscribe, setFocus);
QI_REGISTER_OBJECT(naoqi_test::FakeSpeechRecognition, _enableFreeSpeechToText);
QI_REGISTER_OBJECT(naoqi_test::FakeSystem, systemVersion);
QI_REGISTER_OBJECT(
  naoqi_test::FakeMotion, getRobotConfig, getSensorNames, move, stopMove, moveTo, setAngles,
  changeAngles, angleInterpolation, angleInterpolationBezier, getRobotPosition,
  getRobotVelocity);
QI_REGISTER_OBJECT(naoqi_test::FakeTextToSpeech, say, stopAll);
QI_REGISTER_OBJECT(naoqi_test::FakeAnimatedSpeech, say);
QI_REGISTER_OBJECT(naoqi_test::FakeStoppable, stopAll);

namespace naoqi_test
{

/** A standalone session serving the fakes; everything it receives lands in log. */
class FakeNaoqi
{
public:
  FakeNaoqi()
  : session(qi::makeSession())
  {
    session->listenStandalone(qi::Url("tcp://127.0.0.1:0")).value();
    session->registerService("ALMemory", memory).value();
    session->registerService("ALDialog", boost::make_shared<FakeDialog>(log)).value();
    session->registerService(
      "ALSpeechRecognition", boost::make_shared<FakeSpeechRecognition>()).value();
    session->registerService("ALSystem", boost::make_shared<FakeSystem>()).value();
    session->registerService("ALMotion", boost::make_shared<FakeMotion>(log)).value();
    session->registerService(
      "ALTextToSpeech", boost::make_shared<FakeTextToSpeech>(log, voice)).value();
    session->registerService(
      "ALAnimatedSpeech", boost::make_shared<FakeAnimatedSpeech>(log, voice)).value();
    session->registerService(
      "ALAudioPlayer", boost::make_shared<FakeStoppable>(log, "ALAudioPlayer")).value();
  }

  ~FakeNaoqi()
  {
    session->close().wait();
    // libqi's event loop can still be finishing work for the session and its
    // objects after close(); freeing them now would race with it (ThreadSanitizer
    // sees it). Closed sessions own no threads, so keep them until the process ends.
    static std::mutex kept_mutex;
    static auto * kept = new std::vector<boost::shared_ptr<void>>();
    std::lock_guard<std::mutex> lock(kept_mutex);
    kept->push_back(session);
    kept->push_back(memory);
  }

  FakeNaoqi(const FakeNaoqi &) = delete;
  FakeNaoqi & operator=(const FakeNaoqi &) = delete;

  CallLog log;
  std::shared_ptr<FakeVoice> voice = std::make_shared<FakeVoice>();
  boost::shared_ptr<FakeMemory> memory = boost::make_shared<FakeMemory>(log);
  qi::SessionPtr session;
};

}  // namespace naoqi_test

#endif
