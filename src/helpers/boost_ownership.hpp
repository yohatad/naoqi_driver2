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

#ifndef BOOST_OWNERSHIP_HPP
#define BOOST_OWNERSHIP_HPP

#include <memory>

#include <boost/shared_ptr.hpp>

namespace naoqi
{
namespace helpers
{

/**
 * @brief Returns a std::shared_ptr that shares ownership of a boost-owned object.
 * Needed for objects that libqi requires to be owned by a boost::shared_ptr: the
 * returned pointer keeps a copy of the boost owner in its deleter, so the object lives
 * until both the boost owners and the std owners are gone.
 */
template<typename T>
std::shared_ptr<T> shareBoostOwned(const boost::shared_ptr<T> & owner)
{
  return std::shared_ptr<T>(owner.get(), [owner](T *) {});
}

}  // namespace helpers
}  // namespace naoqi

#endif
