// An in-process NAOqi stand-in for unit tests: a standalone qi::Session with fake
// ALMemory, ALSystem and ALMotion services that record every call they receive.

#ifndef NAOQI_DRIVER_TEST_FAKE_NAOQI_HPP
#define NAOQI_DRIVER_TEST_FAKE_NAOQI_HPP

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
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
    {
      std::lock_guard<std::mutex> lock(mutex_);
      calls_.push_back(call);
    }
    cv_.notify_all();
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
    return cv_.wait_for(lock, timeout, [&] {return countLocked(prefix) > 0;});
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

class FakeMemory
{
public:
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

private:
  CallLog & log_;
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

QI_REGISTER_OBJECT(naoqi_test::FakeMemory, getData);
QI_REGISTER_OBJECT(naoqi_test::FakeSystem, systemVersion);
QI_REGISTER_OBJECT(
  naoqi_test::FakeMotion, getRobotConfig, getSensorNames, move, stopMove, moveTo, setAngles,
  changeAngles, angleInterpolation, angleInterpolationBezier, getRobotPosition);
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
    session->registerService("ALMemory", boost::make_shared<FakeMemory>()).value();
    session->registerService("ALSystem", boost::make_shared<FakeSystem>()).value();
    session->registerService("ALMotion", boost::make_shared<FakeMotion>(log)).value();
    for (const char * name : {"ALTextToSpeech", "ALAnimatedSpeech", "ALAudioPlayer"}) {
      session->registerService(name, boost::make_shared<FakeStoppable>(log, name)).value();
    }
  }

  ~FakeNaoqi() {session->close().wait();}

  CallLog log;
  qi::SessionPtr session;
};

}  // namespace naoqi_test

#endif
