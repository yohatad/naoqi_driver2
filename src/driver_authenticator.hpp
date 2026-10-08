/*
 * Copyright 2015 Aldebaran
 *
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

#ifndef DRIVER_AUTHENTICATOR_HPP
#define DRIVER_AUTHENTICATOR_HPP

#include <boost/make_shared.hpp>
#include <qi/messaging/authprovider.hpp>

namespace naoqi
{

class DriverAuthenticator : public qi::ClientAuthenticator
{
public:
  inline static const std::string user_key = "user";
  inline static const std::string pass_key = "token";

  DriverAuthenticator(
    const std::string & user,
    const std::string & pass)
  : user_(user), password_(pass) {}

  qi::CapabilityMap initialAuthData() override
  {
    qi::CapabilityMap result;
    result[DriverAuthenticator::user_key] = qi::AnyValue::from(user_);
    result[DriverAuthenticator::pass_key] = qi::AnyValue::from(password_);
    return result;
  }

  qi::CapabilityMap _processAuth(const qi::CapabilityMap &) override
  {
    return qi::CapabilityMap();
  }

private:
  std::string user_;
  std::string password_;
};

class DriverAuthenticatorFactory : public qi::ClientAuthenticatorFactory
{
public:
  std::string user;
  std::string pass;

  qi::ClientAuthenticatorPtr newAuthenticator() override
  {
    // qi::ClientAuthenticatorPtr is a boost::shared_ptr, so this must stay boost.
    return boost::make_shared<DriverAuthenticator>(user, pass);
  }
};

}  // namespace naoqi

#endif // DRIVER_AUTHENTICATOR
