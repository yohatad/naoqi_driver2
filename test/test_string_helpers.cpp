// The std-based string helpers replace boost::split and boost::algorithm::replace_all.
// Boost is kept as a test-only reference so every case is checked against the
// exact behaviour the driver had before the migration.

#include <random>
#include <string>
#include <vector>

#include <boost/algorithm/string.hpp>
#include <gtest/gtest.h>

#include "helpers/string_helpers.hpp"

namespace sh = naoqi::helpers::string;

namespace
{

std::vector<std::string> boostSplit(const std::string& str, char delimiter)
{
  std::vector<std::string> out;
  boost::split(out, str, boost::is_any_of(std::string(1, delimiter)));
  return out;
}

std::string boostReplaceAll(std::string str, const std::string& from, const std::string& to)
{
  boost::algorithm::replace_all(str, from, to);
  return str;
}

std::string stdReplaceAll(std::string str, const std::string& from, const std::string& to)
{
  sh::replaceAll(str, from, to);
  return str;
}

std::string randomString(std::mt19937& rng, const std::string& alphabet, std::size_t max_len)
{
  std::uniform_int_distribution<std::size_t> len_dist(0, max_len);
  std::uniform_int_distribution<std::size_t> char_dist(0, alphabet.size() - 1);
  std::string out(len_dist(rng), ' ');
  for (auto& c : out)
    c = alphabet[char_dist(rng)];
  return out;
}

} // namespace

TEST(StringHelpersSplit, MatchesBoostOnEdgeCases)
{
  for (const std::string input : {"", ":", "::", "a", "a:b", "a::b", ":a:", "a:b:c:d", "no delimiter"})
  {
    EXPECT_EQ(sh::split(input, ':'), boostSplit(input, ':')) << "input: '" << input << "'";
  }
}

TEST(StringHelpersSplit, ParsesQiLogSourceLikeLogConverter)
{
  // converters/log.cpp splits qi::LogMessage::source ("file:function:line").
  const auto parts = sh::split("src/foo.cpp:void bar():42", ':');
  ASSERT_EQ(parts.size(), 3u);
  EXPECT_EQ(parts[0], "src/foo.cpp");
  EXPECT_EQ(parts[1], "void bar()");
  EXPECT_EQ(parts[2], "42");
}

TEST(StringHelpersSplit, MatchesBoostOnRandomInputs)
{
  std::mt19937 rng(42);
  for (int i = 0; i < 5000; ++i)
  {
    const auto input = randomString(rng, "ab:", 24);
    ASSERT_EQ(sh::split(input, ':'), boostSplit(input, ':')) << "input: '" << input << "'";
  }
}

TEST(StringHelpersReplaceAll, MatchesBoostOnEdgeCases)
{
  struct Case { std::string input, from, to; };
  const std::vector<Case> cases = {
    {"", "-", ""},
    {"---", "-", ""},
    {"a-b-c", "-", ""},
    {"aaa", "aa", "b"},          // non-overlapping, left to right
    {"aaaa", "aa", "b"},
    {"abc", "a", "aa"},          // replacement contains the pattern: must not loop
    {"abc", "x", "y"},           // no match
    {"C:\\\\dir\\\\file", "\\\\", "\\"},  // filesystem_helpers getFoldersize
    {"abc", "", "x"},            // empty pattern is a no-op
  };
  for (const auto& c : cases)
  {
    EXPECT_EQ(stdReplaceAll(c.input, c.from, c.to), boostReplaceAll(c.input, c.from, c.to))
      << "input: '" << c.input << "' from: '" << c.from << "' to: '" << c.to << "'";
  }
}

TEST(StringHelpersReplaceAll, StripsDashesFromGoalUuidLikeListenAction)
{
  // actions/listen.cpp turns a goal UUID into a topic-safe name.
  std::string uuid = "123e4567-e89b-12d3-a456-426614174000";
  sh::replaceAll(uuid, "-", "");
  EXPECT_EQ(uuid, "123e4567e89b12d3a456426614174000");
}

TEST(StringHelpersReplaceAll, MatchesBoostOnRandomInputs)
{
  std::mt19937 rng(7);
  const std::vector<std::string> patterns = {"-", "a", "ab", "aa", "ba-"};
  const std::vector<std::string> replacements = {"", "-", "x", "aa", "ab"};
  for (int i = 0; i < 5000; ++i)
  {
    const auto input = randomString(rng, "ab-", 24);
    const auto& from = patterns[i % patterns.size()];
    const auto& to = replacements[(i / patterns.size()) % replacements.size()];
    ASSERT_EQ(stdReplaceAll(input, from, to), boostReplaceAll(input, from, to))
      << "input: '" << input << "' from: '" << from << "' to: '" << to << "'";
  }
}
