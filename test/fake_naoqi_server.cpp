// The fake NAOqi of fake_naoqi.hpp in its own process, for end-to-end tests of
// naoqi_driver_node: prints "listening on <url>", then one line per call it receives,
// until SIGINT or SIGTERM.

#include <atomic>
#include <chrono>
#include <csignal>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>

#include "fake_naoqi.hpp"

namespace
{

std::atomic<bool> g_stop{false};

void onSignal(int)
{
  g_stop = true;
}

}  // namespace

int main()
{
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);

  naoqi_test::FakeNaoqi naoqi;
  std::mutex print_mutex;
  naoqi.log.setObserver(
    [&print_mutex](const std::string & call) {
      std::lock_guard<std::mutex> lock(print_mutex);
      std::cout << call << std::endl;
    });
  {
    std::lock_guard<std::mutex> lock(print_mutex);
    std::cout << "listening on " << naoqi.session->endpoints().front().str() << std::endl;
  }
  while (!g_stop) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  naoqi.log.setObserver(nullptr);
  return 0;
}
