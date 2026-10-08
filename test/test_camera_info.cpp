// camera_info_definitions.hpp: boost::assign::list_of(...) replaced by std::vector
// initializer lists. The expected values are the literals from the pre-migration code.

#include <vector>

#include <gtest/gtest.h>

#include "converters/camera_info_definitions.hpp"

namespace cid = naoqi::converter::camera_info_definitions;

namespace
{

void expectDistortion(const sensor_msgs::msg::CameraInfo& info, const std::vector<double>& expected)
{
  EXPECT_EQ(info.distortion_model, "plumb_bob");
  ASSERT_EQ(info.d.size(), expected.size());
  for (std::size_t i = 0; i < expected.size(); ++i)
    EXPECT_DOUBLE_EQ(info.d[i], expected[i]) << "coefficient " << i;
}

} // namespace

TEST(CameraInfoDefinitions, TopCameraDistortion)
{
  expectDistortion(cid::createCameraInfoTOPVGA(),
    {-0.0545211535376379, 0.0691973423510287, -0.00241094929163055, -0.00112245009306511, 0});
  expectDistortion(cid::createCameraInfoTOPQVGA(),
    {-0.0870160932911717, 0.128210165050533, 0.003379500659424, -0.00106205540818586, 0});
  expectDistortion(cid::createCameraInfoTOPQQVGA(),
    {-0.0843564504845967, 0.125733083790192, 0.00275901756247071, -0.00138645823460527, 0});
}

TEST(CameraInfoDefinitions, BottomCameraDistortion)
{
  expectDistortion(cid::createCameraInfoBOTTOMVGA(),
    {-0.0648763971625288, 0.0612520196884308, 0.0038281538281731, -0.00551104078371959, 0});
  expectDistortion(cid::createCameraInfoBOTTOMQVGA(),
    {-0.0481869853715082, 0.0201858398559121, 0.0030362056699177, -0.00172241952442813, 0});
  expectDistortion(cid::createCameraInfoBOTTOMQQVGA(),
    {-0.0688388724945936, 0.0697453843669642, 0.00309518737071049, -0.00570486993696543, 0});
}

TEST(CameraInfoDefinitions, ResolutionsUnchanged)
{
  EXPECT_EQ(cid::createCameraInfoTOPVGA().width, 640u);
  EXPECT_EQ(cid::createCameraInfoTOPVGA().height, 480u);
  EXPECT_EQ(cid::createCameraInfoBOTTOMQQVGA().width, 160u);
  EXPECT_EQ(cid::createCameraInfoBOTTOMQQVGA().height, 120u);
}
