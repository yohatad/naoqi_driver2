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


#ifndef CONNECTION_HELPERS_HPP
#define CONNECTION_HELPERS_HPP

#include <algorithm>
#include <chrono>

namespace naoqi
{
namespace helpers
{

/** Delay before connection attempt number attempt + 1: 1, 2, 4, 8, 16, then 30 s. */
inline std::chrono::seconds reconnectDelay(int attempt)
{
  const int exponent = std::clamp(attempt - 1, 0, 5);
  return std::chrono::seconds(std::min(30, 1 << exponent));
}

}  // namespace helpers
}  // namespace naoqi

#endif
