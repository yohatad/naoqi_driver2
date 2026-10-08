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


#ifndef LISTEN_VALIDATION_HPP
#define LISTEN_VALIDATION_HPP

#include <string>
#include <unordered_map>

#include <naoqi_bridge_msgs/action/listen.hpp>

namespace naoqi
{
namespace action
{

/** QiChat language name for an ISO 639-1 code, or "" when unsupported. */
inline std::string qichatLanguage(const std::string & iso_language)
{
  static const std::unordered_map<std::string, std::string> iso_to_qichat = {
    {"en", "English"}, {"fr", "French"}, {"de", "German"}, {"it", "Italian"},
    {"ja", "Japanese"}, {"ko", "Korean"}, {"pt", "Portuguese"}, {"es", "Spanish"},
    {"zh", "Chinese"}};
  const auto it = iso_to_qichat.find(iso_language);
  return it == iso_to_qichat.end() ? "" : it->second;
}

/**
 * Checks a Listen goal before anything reaches ALDialog. Expected phrases are pasted
 * into a QiChat topic between double quotes, which cannot hold a quote or a line
 * break. An empty string means valid.
 */
inline std::string validateListenGoal(const naoqi_bridge_msgs::action::Listen::Goal & goal)
{
  for (const auto & phrase : goal.expected) {
    if (phrase.empty()) {
      return "expected phrases must not be empty";
    }
    if (phrase.find_first_of("\"\n\r") != std::string::npos) {
      return "expected phrases cannot contain quotes or line breaks: " + phrase;
    }
  }
  if (!goal.language.empty() && qichatLanguage(goal.language).empty()) {
    return "unsupported language code: " + goal.language;
  }
  return "";
}

}  // namespace action
}  // namespace naoqi

#endif
