// The Converter / Publisher / Recorder / Subscriber / Service / Event wrappers now
// hold their model in a std::shared_ptr. These tests check that every call is still
// forwarded, that copies share one underlying object, and that the wrappers keep
// their target alive. Event is also exercised with a boost::shared_ptr target, as
// AudioEventRegister must stay boost-owned for libqi.

#include <memory>
#include <string>
#include <vector>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <gtest/gtest.h>

#include <naoqi_driver/converter/converter.hpp>
#include <naoqi_driver/event/event.hpp>
#include <naoqi_driver/publisher/publisher.hpp>
#include <naoqi_driver/recorder/recorder.hpp>
#include <naoqi_driver/service/service.hpp>
#include <naoqi_driver/subscriber/subscriber.hpp>

#include "test_utils.hpp"

using naoqi::message_actions::MessageAction;
using naoqi::recorder::GlobalRecorder;

namespace
{

struct MockConverter
{
  explicit MockConverter(std::string name, float frequency = 10.f) : name_(std::move(name)), frequency_(frequency) {}
  std::string name() const { return name_; }
  float frequency() const { return frequency_; }
  void reset() { ++resets; }
  void callAll(const std::vector<MessageAction>& actions) { calls.push_back(actions); }

  std::string name_;
  float frequency_;
  int resets = 0;
  std::vector<std::vector<MessageAction>> calls;
};

struct MockPublisher
{
  explicit MockPublisher(std::string topic) : topic_(std::move(topic)) {}
  std::string topic() const { return topic_; }
  bool isInitialized() const { return initialized; }
  bool isSubscribed() const { return subscribed; }
  void reset(rclcpp::Node* node) { last_node = node; initialized = true; }

  std::string topic_;
  bool initialized = false;
  bool subscribed = false;
  rclcpp::Node* last_node = nullptr;
};

struct MockRecorder
{
  explicit MockRecorder(std::string topic) : topic_(std::move(topic)) {}
  std::string topic() const { return topic_; }
  void reset(std::shared_ptr<GlobalRecorder> gr, float frequency) { global = gr; reset_frequency = frequency; initialized = true; }
  bool isInitialized() const { return initialized; }
  void subscribe(bool state) { subscribed = state; }
  bool isSubscribed() const { return subscribed; }
  void writeDump(const rclcpp::Time&) { ++dumps; }
  void setBufferDuration(float duration) { buffer_duration = duration; }

  std::string topic_;
  std::shared_ptr<GlobalRecorder> global;
  float reset_frequency = 0.f;
  float buffer_duration = 0.f;
  bool initialized = false;
  bool subscribed = false;
  int dumps = 0;
};

struct MockEndpoint  // shape shared by subscribers and services
{
  MockEndpoint(std::string name, std::string topic) : name_(std::move(name)), topic_(std::move(topic)) {}
  std::string name() const { return name_; }
  std::string topic() const { return topic_; }
  bool isInitialized() const { return initialized; }
  void reset(rclcpp::Node* node) { last_node = node; initialized = true; }

  std::string name_;
  std::string topic_;
  bool initialized = false;
  rclcpp::Node* last_node = nullptr;
};

struct MockEvent
{
  void resetPublisher(rclcpp::Node* node) { last_node = node; }
  void resetRecorder(std::shared_ptr<GlobalRecorder> gr) { global = gr; }
  void startProcess() { started = true; }
  void stopProcess() { started = false; }
  void writeDump(const rclcpp::Time&) { ++dumps; }
  void setBufferDuration(float duration) { buffer_duration = duration; }
  void isRecording(bool state) { recording = state; }
  void isPublishing(bool state) { publishing = state; }
  void isDumping(bool state) { dumping = state; }

  rclcpp::Node* last_node = nullptr;
  std::shared_ptr<GlobalRecorder> global;
  bool started = false;
  bool recording = false;
  bool publishing = false;
  bool dumping = false;
  float buffer_duration = 0.f;
  int dumps = 0;
};

} // namespace

class TypeErasureTest : public naoqi_test::RosTest {};

TEST_F(TypeErasureTest, ConverterForwardsAndSkipsEmptyActionLists)
{
  auto mock = std::make_shared<MockConverter>("imu", 50.f);
  naoqi::converter::Converter conv(mock);

  EXPECT_EQ(conv.name(), "imu");
  EXPECT_FLOAT_EQ(conv.frequency(), 50.f);

  conv.reset();
  EXPECT_EQ(mock->resets, 1);

  conv.callAll({});
  EXPECT_TRUE(mock->calls.empty());

  conv.callAll({naoqi::message_actions::PUBLISH, naoqi::message_actions::RECORD});
  ASSERT_EQ(mock->calls.size(), 1u);
  EXPECT_EQ(mock->calls[0].size(), 2u);
}

TEST_F(TypeErasureTest, ConverterCopiesShareOneTarget)
{
  auto mock = std::make_shared<MockConverter>("joint_states");
  naoqi::converter::Converter original(mock);
  std::vector<naoqi::converter::Converter> stored{original};  // as Driver::converters_ does

  stored[0].reset();
  original.reset();
  EXPECT_EQ(mock->resets, 2);
  EXPECT_TRUE(stored[0] == original);
  EXPECT_FALSE(original == naoqi::converter::Converter(std::make_shared<MockConverter>("other")));
}

TEST_F(TypeErasureTest, ConverterKeepsTargetAliveUntilLastCopyIsGone)
{
  std::weak_ptr<MockConverter> watch;
  {
    std::unique_ptr<naoqi::converter::Converter> conv;
    {
      auto mock = std::make_shared<MockConverter>("sonar");
      watch = mock;
      conv = std::make_unique<naoqi::converter::Converter>(mock);
    }
    EXPECT_FALSE(watch.expired());
    EXPECT_EQ(conv->name(), "sonar");
  }
  EXPECT_TRUE(watch.expired());
}

TEST_F(TypeErasureTest, ConverterAcceptsBoostPointersToo)
{
  auto mock = boost::make_shared<MockConverter>("legacy");
  naoqi::converter::Converter conv(mock);
  conv.callAll({naoqi::message_actions::LOG});
  EXPECT_EQ(conv.name(), "legacy");
  EXPECT_EQ(mock->calls.size(), 1u);
}

TEST_F(TypeErasureTest, PublisherForwardsAndComparesByTopic)
{
  auto mock = std::make_shared<MockPublisher>("/camera/front");
  naoqi::publisher::Publisher pub(mock);

  EXPECT_EQ(pub.topic(), "/camera/front");
  EXPECT_FALSE(pub.isInitialized());
  pub.reset(node_.get());
  EXPECT_TRUE(pub.isInitialized());
  EXPECT_EQ(mock->last_node, node_.get());

  mock->subscribed = true;
  EXPECT_TRUE(pub.isSubscribed());

  naoqi::publisher::Publisher same_topic(std::make_shared<MockPublisher>("/camera/front"));
  naoqi::publisher::Publisher other_topic(std::make_shared<MockPublisher>("/camera/bottom"));
  EXPECT_TRUE(pub == same_topic);
  EXPECT_FALSE(pub == other_topic);

  // The shared_ptr<Publisher> overload compares the pointees, not the addresses.
  auto lhs = std::make_shared<naoqi::publisher::Publisher>(pub);
  auto rhs = std::make_shared<naoqi::publisher::Publisher>(same_topic);
  EXPECT_TRUE(lhs == rhs);
}

TEST_F(TypeErasureTest, RecorderReceivesTheSameGlobalRecorder)
{
  auto mock = std::make_shared<MockRecorder>("/imu");
  naoqi::recorder::Recorder rec(mock);
  auto global = std::make_shared<GlobalRecorder>("naoqi_driver");

  rec.reset(global, 25.f);
  EXPECT_TRUE(rec.isInitialized());
  EXPECT_EQ(mock->global, global);
  EXPECT_FLOAT_EQ(mock->reset_frequency, 25.f);

  rec.subscribe(true);
  EXPECT_TRUE(rec.isSubscribed());
  rec.setBufferDuration(3.f);
  EXPECT_FLOAT_EQ(mock->buffer_duration, 3.f);
  rec.writeDump(node_->now());
  EXPECT_EQ(mock->dumps, 1);
  EXPECT_EQ(rec.topic(), "/imu");
}

TEST_F(TypeErasureTest, SubscriberAndServiceForward)
{
  auto sub_mock = std::make_shared<MockEndpoint>("teleop", "/cmd_vel");
  naoqi::subscriber::Subscriber sub(sub_mock);
  EXPECT_EQ(sub.name(), "teleop");
  EXPECT_EQ(sub.topic(), "/cmd_vel");
  sub.reset(node_.get());
  EXPECT_TRUE(sub.isInitialized());
  EXPECT_EQ(sub_mock->last_node, node_.get());

  auto srv_mock = std::make_shared<MockEndpoint>("get_language", "/naoqi_driver/get_language");
  naoqi::service::Service srv(srv_mock);
  EXPECT_EQ(srv.name(), "get_language");
  EXPECT_EQ(srv.topic(), "/naoqi_driver/get_language");
  srv.reset(node_.get());
  EXPECT_TRUE(srv_mock->initialized);
  EXPECT_EQ(srv_mock->last_node, node_.get());
}

template <typename Ptr>
void exerciseEvent(const Ptr& mock, rclcpp::Node* node)
{
  naoqi::event::Event event(mock);
  auto global = std::make_shared<GlobalRecorder>("naoqi_driver");

  event.resetPublisher(node);
  event.resetRecorder(global);
  event.startProcess();
  event.isRecording(true);
  event.isPublishing(true);
  event.isDumping(true);
  event.setBufferDuration(4.f);
  event.writeDump(node->now());

  EXPECT_EQ(mock->last_node, node);
  EXPECT_EQ(mock->global, global);
  EXPECT_TRUE(mock->started);
  EXPECT_TRUE(mock->recording);
  EXPECT_TRUE(mock->publishing);
  EXPECT_TRUE(mock->dumping);
  EXPECT_FLOAT_EQ(mock->buffer_duration, 4.f);
  EXPECT_EQ(mock->dumps, 1);

  event.stopProcess();
  EXPECT_FALSE(mock->started);
}

TEST_F(TypeErasureTest, EventForwardsWithStdTarget)
{
  exerciseEvent(std::make_shared<MockEvent>(), node_.get());
}

TEST_F(TypeErasureTest, EventForwardsWithBoostTargetLikeAudioEventRegister)
{
  exerciseEvent(boost::make_shared<MockEvent>(), node_.get());
}

TEST_F(TypeErasureTest, EventMapCopiesShareOneTarget)
{
  auto mock = boost::make_shared<MockEvent>();
  std::map<std::string, naoqi::event::Event> event_map;  // as Driver::event_map_ does
  event_map.insert(std::make_pair("audio", naoqi::event::Event(mock)));

  event_map.at("audio").startProcess();
  EXPECT_TRUE(mock->started);
  EXPECT_EQ(mock.use_count(), 2);  // test + the single model inside the map entry
}
