// AudioEventRegister must be owned by a boost::shared_ptr (libqi), while the driver holds
// std::shared_ptr<event::Event>. helpers::shareBoostOwned() bridges the two.

#include <memory>

#include <boost/enable_shared_from_this.hpp>
#include <boost/make_shared.hpp>
#include <gtest/gtest.h>

#include "helpers/boost_ownership.hpp"

using naoqi::helpers::shareBoostOwned;

namespace
{

struct Base
{
  virtual ~Base() = default;
};

struct Tracked : Base, boost::enable_shared_from_this<Tracked>
{
  explicit Tracked(bool & alive)
  : alive_(alive)
  {
    alive_ = true;
  }
  ~Tracked() override {alive_ = false;}
  bool & alive_;
};

}  // namespace

TEST(BoostOwnership, StdViewKeepsTheObjectAfterTheBoostOwnerIsGone)
{
  bool alive = false;
  std::shared_ptr<Tracked> view;
  {
    auto owner = boost::make_shared<Tracked>(alive);
    view = shareBoostOwned(owner);
    EXPECT_EQ(view.get(), owner.get());
  }
  EXPECT_TRUE(alive);
  view.reset();
  EXPECT_FALSE(alive);
}

TEST(BoostOwnership, BoostOwnerKeepsTheObjectAfterTheViewIsGone)
{
  bool alive = false;
  auto owner = boost::make_shared<Tracked>(alive);
  {
    auto view = shareBoostOwned(owner);
  }
  EXPECT_TRUE(alive);
  owner.reset();
  EXPECT_FALSE(alive);
}

TEST(BoostOwnership, BoostSharedFromThisStillWorks)
{
  // AudioEventRegister::startProcess() passes shared_from_this() to libqi.
  bool alive = false;
  auto owner = boost::make_shared<Tracked>(alive);
  auto view = shareBoostOwned(owner);
  EXPECT_EQ(view->shared_from_this().get(), owner.get());
}

TEST(BoostOwnership, ViewConvertsToABaseInterface)
{
  bool alive = false;
  std::shared_ptr<Base> as_base = shareBoostOwned(boost::make_shared<Tracked>(alive));
  EXPECT_TRUE(alive);
  as_base.reset();
  EXPECT_FALSE(alive);
}
