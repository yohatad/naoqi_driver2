/*
 * Copyright 2015 Aldebaran
 *
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

#include <memory>
#include <signal.h>

#include <atomic>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <qi/applicationsession.hpp>
#include <qi/session.hpp>
#include <qi/anymodule.hpp>

#if LIBQI_VERSION >= 29
#include "driver_authenticator.hpp"
#endif

#include <naoqi_driver/naoqi_driver.hpp>

#include "naoqi_driver/tools.hpp"
#include "naoqi_driver/ros_helpers.hpp"
#include "helpers/connection_helpers.hpp"

// A plain atomic pointer: unlike weak_ptr::lock(), loading it is async-signal-safe.
std::atomic<naoqi::Driver *> g_driver{nullptr};
static_assert(
  std::atomic<naoqi::Driver *>::is_always_lock_free,
  "the SIGINT handler needs a lock-free pointer");

// Set by SIGINT, also while still connecting to NAOqi.
std::atomic<bool> g_interrupted{false};
// Set when the NAOqi session drops while the driver runs.
std::atomic<bool> g_disconnected{false};
static_assert(std::atomic<bool>::is_always_lock_free, "the SIGINT handler needs a lock-free flag");

void sigintHandler(int sig)
{
  (void)sig;
  // Only async-signal-safe work here: just flag the run() loop to stop.
  // The actual cleanup (driver->stop()) happens afterwards in main(),
  // since it locks mutexes and isn't safe to call from a signal handler.
  g_interrupted = true;
  if (naoqi::Driver * driver = g_driver.load()) {
    driver->requestStop();
  }
}

/** Sleeps for delay, waking early on Ctrl+C. */
void sleepUnlessInterrupted(std::chrono::seconds delay)
{
  const auto end = std::chrono::steady_clock::now() + delay;
  while (!g_interrupted && std::chrono::steady_clock::now() < end) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

int main(int argc, char ** argv)
{
  const std::string no_password = "no_password";
  std::string protocol = "tcp://";

  std::string nao_ip;
  int nao_port;
  std::string user;
  std::string password;
  std::string network_interface;
  std::string listen_url;

  // Initialize ROS and create the driver Node
  rclcpp::init(argc, argv);
  auto bs = std::make_shared<naoqi::Driver>();

  // Setup the time helper
  naoqi::helpers::Node::setNode(bs);

  // Retrieve the parameters
  bs->declare_parameter<std::string>("nao_ip", "127.0.0.1");
  bs->declare_parameter<int>("nao_port", 9559);
  bs->declare_parameter<std::string>("user", "nao");
  bs->declare_parameter<std::string>("password", no_password);
  bs->declare_parameter<std::string>("network_interface", "eth0");
  bs->declare_parameter<std::string>("qi_listen_url", "tcp://0.0.0.0:0");

  bs->get_parameter("nao_ip", nao_ip);
  bs->get_parameter("nao_port", nao_port);
  bs->get_parameter("user", user);
  bs->get_parameter("password", password);
  bs->get_parameter("network_interface", network_interface);
  bs->get_parameter("qi_listen_url", listen_url);

  if (password.compare(no_password) != 0) {
#if LIBQI_VERSION >= 29
    protocol = "tcps://";
    nao_port = (nao_port == 9559) ? 9503 : nao_port;
#else
    RCLCPP_WARN_STREAM(bs->get_logger(), "No need to set a password, ignored.");
#endif
  }

  // Build the q::ApplicationSession, and connect the associated session
  const std::string address = protocol + nao_ip + ":" + std::to_string(nao_port);
  qi::Url url(address);
  qi::ApplicationSession app(argc, argv);

#if LIBQI_VERSION >= 29
  if (password.compare(no_password) != 0) {
    naoqi::DriverAuthenticatorFactory * factory = new naoqi::DriverAuthenticatorFactory;
    factory->user = user;
    factory->pass = password;

    app.session()->setClientAuthenticatorFactory(
      qi::ClientAuthenticatorFactoryPtr(factory));
  }
#endif

  // Ctrl+C must also work while retrying the connection.
  signal(SIGINT, sigintHandler);

  // Retry with growing delays: the robot may still be booting or out of reach.
  auto session = app.session();
  for (int attempt = 1; ; ++attempt) {
    qi::Future<void> connection = session->connect(url);
    if (!connection.hasError()) {
      break;
    }
    const auto delay = naoqi::helpers::reconnectDelay(attempt);
    RCLCPP_ERROR(
      bs->get_logger(), "Cannot connect to NAOqi at %s (%s), retrying in %ld s",
      address.c_str(), connection.error().c_str(), static_cast<long>(delay.count()));
    sleepUnlessInterrupted(delay);
    if (g_interrupted) {
      return EXIT_FAILURE;
    }
  }
  // The session needs to listen on the network to process audio callbacks.
  if (!listen_url.empty()) {
    session->listen(listen_url);
  }

  // Pass the create session to the driver node, and init the node
  bs->setQiSession(session);

  // If NAOqi goes away, stop and exit with an error; the launch file respawns the
  // driver, which then reconnects with the backoff above.
  rclcpp::Logger logger = bs->get_logger();
  session->disconnected.connect(
    [logger](const std::string & reason) {
      g_disconnected = true;
      RCLCPP_ERROR(logger, "Lost the connection to NAOqi (%s), stopping", reason.c_str());
      if (naoqi::Driver * driver = g_driver.load()) {
        driver->requestStop();
      }
    });

  // Run the driver node. Stop it on SIGINT.
  g_driver = bs.get();
  if (g_interrupted) {
    bs->requestStop();
  }
  bs->run();
  g_driver = nullptr;
  bs->stop();
  const int exit_code = g_disconnected ? EXIT_FAILURE : EXIT_SUCCESS;

  // Close the qi::Session and shutdown ROS.
  app.session()->close();
  rclcpp::shutdown();
  return exit_code;
}
