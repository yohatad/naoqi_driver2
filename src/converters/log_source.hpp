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

#ifndef LOG_SOURCE_HPP
#define LOG_SOURCE_HPP

#include <cstdlib>
#include <string>

namespace naoqi
{
namespace converter
{

struct LogSource
{
  std::string file;
  std::string function;
  int line = 0;
};

/**
 * @brief Parses a qi::LogMessage source of the form "file:function:line".
 * The function part may itself contain colons (e.g. "void ns::f()"), so the
 * file ends at the first colon and the line starts after the last one.
 * Sources with fewer fields are accepted; missing parts stay empty / zero.
 */
inline LogSource parseLogSource(const std::string & source)
{
  LogSource out;
  const auto first = source.find(':');
  if (first == std::string::npos) {
    out.file = source;
    return out;
  }
  out.file = source.substr(0, first);

  const auto last = source.rfind(':');
  if (last == first) {
    out.function = source.substr(first + 1);
    return out;
  }
  out.function = source.substr(first + 1, last - first - 1);
  out.line = std::atoi(source.c_str() + last + 1);
  return out;
}

}  // namespace converter
}  // namespace naoqi

#endif
