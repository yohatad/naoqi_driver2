// fromAnyValueToNaoqiImage() now checks the field count and reports the buffer size,
// so CameraConverter can refuse buffers smaller than width * height * pixel size.

#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <qi/anyvalue.hpp>
#include <qi/buffer.hpp>

#include "tools/from_any_value.hpp"

namespace
{

/** Builds the 12-field list returned by ALVideoDevice.getImageRemote. */
std::vector<qi::AnyValue> imageFields(int width, int height, const std::string & pixels)
{
  qi::Buffer buffer;
  buffer.write(pixels.data(), pixels.size());
  return {
    qi::AnyValue::from(width),
    qi::AnyValue::from(height),
    qi::AnyValue::from(3),        // layers
    qi::AnyValue::from(13),       // colorspace
    qi::AnyValue::from(100),      // timestamp_s
    qi::AnyValue::from(200),      // timestamp_us
    qi::AnyValue::from(buffer),
    qi::AnyValue::from(0),        // cam_id
    qi::AnyValue::from(0.1f),     // fov_left
    qi::AnyValue::from(0.2f),     // fov_top
    qi::AnyValue::from(0.3f),     // fov_right
    qi::AnyValue::from(0.4f),     // fov_bottom
  };
}

}  // namespace

TEST(NaoqiImage, ReportsDimensionsAndBufferSize)
{
  const std::string pixels(4 * 2 * 3, '\x7f');
  qi::AnyValue value = qi::AnyValue::from(imageFields(4, 2, pixels));

  const auto image = naoqi::tools::fromAnyValueToNaoqiImage(value);
  EXPECT_EQ(image.width, 4);
  EXPECT_EQ(image.height, 2);
  EXPECT_EQ(image.buffer_size, pixels.size());
  ASSERT_NE(image.buffer, nullptr);
  EXPECT_EQ(static_cast<const char *>(image.buffer)[0], '\x7f');
  EXPECT_FLOAT_EQ(image.fov_bottom, 0.4f);
}

TEST(NaoqiImage, ShortBufferIsReportedNotHidden)
{
  // A truncated frame: the converter compares buffer_size with width*height*3.
  qi::AnyValue value = qi::AnyValue::from(imageFields(640, 480, std::string(100, '\0')));
  const auto image = naoqi::tools::fromAnyValueToNaoqiImage(value);
  EXPECT_EQ(image.buffer_size, 100u);
  EXPECT_LT(image.buffer_size, static_cast<size_t>(640 * 480 * 3));
}

TEST(NaoqiImage, RejectsTruncatedFieldList)
{
  auto fields = imageFields(4, 2, std::string(24, '\0'));
  fields.resize(7);  // previously read anyref[7..11] out of bounds
  qi::AnyValue value = qi::AnyValue::from(fields);
  EXPECT_THROW(naoqi::tools::fromAnyValueToNaoqiImage(value), std::runtime_error);
}

TEST(NaoqiImage, RejectsWrongFieldTypes)
{
  auto fields = imageFields(4, 2, std::string(24, '\0'));
  fields[0] = qi::AnyValue::from(std::string("not a width"));
  qi::AnyValue value = qi::AnyValue::from(fields);
  EXPECT_THROW(naoqi::tools::fromAnyValueToNaoqiImage(value), std::runtime_error);
}
