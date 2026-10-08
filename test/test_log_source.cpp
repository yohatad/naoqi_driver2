// LogConverter used to index split(source, ':')[1] and [2] unchecked.

#include <gtest/gtest.h>

#include "converters/log_source.hpp"

using naoqi::converter::parseLogSource;

TEST(LogSource, ParsesTheUsualFileFunctionLine)
{
  const auto s = parseLogSource("src/foo.cpp:bar:42");
  EXPECT_EQ(s.file, "src/foo.cpp");
  EXPECT_EQ(s.function, "bar");
  EXPECT_EQ(s.line, 42);
}

TEST(LogSource, KeepsQualifiedFunctionNamesWhole)
{
  const auto s = parseLogSource("src/foo.cpp:void naoqi::Driver::run():128");
  EXPECT_EQ(s.file, "src/foo.cpp");
  EXPECT_EQ(s.function, "void naoqi::Driver::run()");
  EXPECT_EQ(s.line, 128);
}

TEST(LogSource, ToleratesMissingFields)
{
  auto s = parseLogSource("");
  EXPECT_EQ(s.file, "");
  EXPECT_EQ(s.function, "");
  EXPECT_EQ(s.line, 0);

  s = parseLogSource("only_a_file");
  EXPECT_EQ(s.file, "only_a_file");
  EXPECT_EQ(s.function, "");
  EXPECT_EQ(s.line, 0);

  s = parseLogSource("file:function");
  EXPECT_EQ(s.file, "file");
  EXPECT_EQ(s.function, "function");
  EXPECT_EQ(s.line, 0);

  s = parseLogSource("file:function:");
  EXPECT_EQ(s.function, "function");
  EXPECT_EQ(s.line, 0);

  s = parseLogSource("file:function:not_a_number");
  EXPECT_EQ(s.line, 0);
}
