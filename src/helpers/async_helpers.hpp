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

#ifndef ASYNC_HELPERS_HPP
#define ASYNC_HELPERS_HPP

#include <functional>
#include <string>
#include <utility>

#include <qi/future.hpp>
#include <rclcpp/rclcpp.hpp>

namespace naoqi
{
namespace helpers
{

/** Calls handler(error) if the asynchronous NAOqi call behind future fails. */
template<typename T>
void onAsyncError(qi::Future<T> future, std::function<void(const std::string &)> handler)
{
  future.then(
    qi::FutureCallbackType_Sync,
    [handler](const qi::Future<T> & done) {
      if (done.hasError()) {
        handler(done.error());
      }
    });
}

/**
 * Logs a warning when a fire-and-forget NAOqi call fails. Without it the error stays
 * in a future nobody reads (for example a mistyped joint name in setAngles).
 */
template<typename T>
void warnOnAsyncError(qi::Future<T> future, const rclcpp::Logger & logger, const std::string & call)
{
  onAsyncError<T>(
    std::move(future), [logger, call](const std::string & error) {
      RCLCPP_WARN(logger, "%s failed: %s", call.c_str(), error.c_str());
    });
}

}  // namespace helpers
}  // namespace naoqi

#endif
