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

#ifndef STRING_HELPERS_HPP
#define STRING_HELPERS_HPP

#include <string>
#include <vector>

namespace naoqi
{
namespace helpers
{
namespace string
{

/**
 * @brief Splits a string on every occurrence of a delimiter.
 * Same semantics as boost::split(out, str, boost::is_any_of(delim_char)):
 * empty tokens are kept and an empty input yields a single empty token.
 */
inline std::vector<std::string> split(const std::string & str, char delimiter)
{
  std::vector<std::string> tokens;
  std::string::size_type start = 0;
  std::string::size_type pos;
  while ((pos = str.find(delimiter, start)) != std::string::npos) {
    tokens.push_back(str.substr(start, pos - start));
    start = pos + 1;
  }
  tokens.push_back(str.substr(start));
  return tokens;
}

/**
 * @brief Replaces every occurrence of from by to, in place.
 * Same semantics as boost::algorithm::replace_all: occurrences are found
 * left to right without overlap, and the replacement text is not rescanned.
 */
inline void replaceAll(std::string & str, const std::string & from, const std::string & to)
{
  if (from.empty()) {
    return;
  }
  std::string::size_type pos = 0;
  while ((pos = str.find(from, pos)) != std::string::npos) {
    str.replace(pos, from.size(), to);
    pos += to.size();
  }
}

}  // namespace string
}  // namespace helpers
}  // namespace naoqi

#endif
