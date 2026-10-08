// helpers::onAsyncError() surfaces failures of fire-and-forget NAOqi calls.

#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <qi/future.hpp>

#include "helpers/async_helpers.hpp"

TEST(AsyncHelpers, ReportsAFailedCall)
{
  std::vector<std::string> errors;
  naoqi::helpers::onAsyncError<void>(
    qi::makeFutureError<void>("Unknown joint: BadJoint"),
    [&errors](const std::string & e) {errors.push_back(e);});
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_EQ(errors[0], "Unknown joint: BadJoint");
}

TEST(AsyncHelpers, StaysQuietOnSuccess)
{
  std::vector<std::string> errors;
  naoqi::helpers::onAsyncError<void>(
    qi::Future<void>(nullptr), [&errors](const std::string & e) {errors.push_back(e);});
  EXPECT_TRUE(errors.empty());
}

TEST(AsyncHelpers, ReportsAFailureThatHappensLater)
{
  std::vector<std::string> errors;
  qi::Promise<void> promise;
  naoqi::helpers::onAsyncError<void>(
    promise.future(), [&errors](const std::string & e) {errors.push_back(e);});
  EXPECT_TRUE(errors.empty());
  promise.setError("NAOqi went away");
  ASSERT_EQ(errors.size(), 1u);
  EXPECT_EQ(errors[0], "NAOqi went away");
}
