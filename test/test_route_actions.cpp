// converter::routeActions() replaced the per-sensor std::bind calls in
// Driver::registerDefaultConverter. Check that each action reaches the right method,
// for one- and two-argument callbacks, and that the targets are kept alive.

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <naoqi_driver/converter/route_actions.hpp>

using naoqi::message_actions::MessageAction;
namespace actions = naoqi::message_actions;

namespace
{

/** Same registerCallback shape as the real converters. */
template<typename ... Args>
struct FakeConverter
{
  using Callback = std::function<void (Args...)>;
  void registerCallback(MessageAction action, Callback cb) {callbacks[action] = cb;}
  std::map<MessageAction, Callback> callbacks;
};

struct ImagePublisher
{
  void publish(const std::string & image, const double & info)
  {
    calls.push_back(image + "@" + std::to_string(info));
  }
  std::vector<std::string> calls;
};

struct ImageRecorder
{
  void write(const std::string & image, const double &) {written.push_back(image);}
  void bufferize(const std::string & image, const double &) {buffered.push_back(image);}
  std::vector<std::string> written, buffered;
};

struct ScanPublisher
{
  void publish(const int & scan) {published.push_back(scan);}
  std::vector<int> published;
};

struct ScanRecorder
{
  void write(const int & scan) {written.push_back(scan);}
  void bufferize(const int & scan) {buffered.push_back(scan);}
  std::vector<int> written, buffered;
};

}  // namespace

TEST(RouteActions, OneArgumentCallbacksReachTheirTargets)
{
  FakeConverter<int &> converter;
  auto publisher = std::make_shared<ScanPublisher>();
  auto recorder = std::make_shared<ScanRecorder>();
  naoqi::converter::routeActions(converter, publisher, recorder);

  int scan = 7;
  converter.callbacks.at(actions::PUBLISH)(scan);
  converter.callbacks.at(actions::RECORD)(scan);
  converter.callbacks.at(actions::LOG)(scan);
  scan = 8;
  converter.callbacks.at(actions::LOG)(scan);

  EXPECT_EQ(publisher->published, std::vector<int>({7}));
  EXPECT_EQ(recorder->written, std::vector<int>({7}));
  EXPECT_EQ(recorder->buffered, std::vector<int>({7, 8}));
}

TEST(RouteActions, TwoArgumentCallbacksForwardBothArguments)
{
  // Same shape as CameraConverter: (image, camera info) passed by value.
  FakeConverter<std::string, double> converter;
  auto publisher = std::make_shared<ImagePublisher>();
  auto recorder = std::make_shared<ImageRecorder>();
  naoqi::converter::routeActions(converter, publisher, recorder);

  converter.callbacks.at(actions::PUBLISH)("frame", 0.5);
  converter.callbacks.at(actions::RECORD)("frame", 0.5);
  converter.callbacks.at(actions::LOG)("old", 0.25);

  ASSERT_EQ(publisher->calls.size(), 1u);
  EXPECT_EQ(publisher->calls[0].substr(0, 9), "frame@0.5");
  EXPECT_EQ(recorder->written, std::vector<std::string>({"frame"}));
  EXPECT_EQ(recorder->buffered, std::vector<std::string>({"old"}));
}

TEST(RouteActions, CallbacksKeepPublisherAndRecorderAlive)
{
  FakeConverter<int &> converter;
  std::weak_ptr<ScanPublisher> publisher_alive;
  std::weak_ptr<ScanRecorder> recorder_alive;
  {
    auto publisher = std::make_shared<ScanPublisher>();
    auto recorder = std::make_shared<ScanRecorder>();
    publisher_alive = publisher;
    recorder_alive = recorder;
    naoqi::converter::routeActions(converter, publisher, recorder);
  }
  EXPECT_FALSE(publisher_alive.expired());
  EXPECT_FALSE(recorder_alive.expired());

  int scan = 1;
  converter.callbacks.at(actions::PUBLISH)(scan);
  EXPECT_EQ(publisher_alive.lock()->published.size(), 1u);

  converter.callbacks.clear();
  EXPECT_TRUE(publisher_alive.expired());
  EXPECT_TRUE(recorder_alive.expired());
}
