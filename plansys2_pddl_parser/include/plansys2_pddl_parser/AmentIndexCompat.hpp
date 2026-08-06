// Copyright 2026 Intelligent Robotics Lab
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef PLANSYS2_PDDL_PARSER__AMENTINDEXCOMPAT_HPP_
#define PLANSYS2_PDDL_PARSER__AMENTINDEXCOMPAT_HPP_

// Locating a package's share directory across ROS 2 distros.
//
// Rolling removed ament_index_cpp/get_package_share_directory.hpp; the
// replacement is get_package_share_path.hpp, which returns a
// std::filesystem::path rather than a std::string. Humble ships only the
// former, rolling only the latter, so no single include works on both and the
// choice has to be made at compile time.
//
// This lives in plansys2_pddl_parser because it is the only C++ package in the
// tree with no plansys2 dependency of its own beyond the messages: putting it
// in plansys2_core would be circular, since core depends on this package.
//
// Upstream PlanSys2's rolling branch migrated straight to
// get_package_share_path. This fork keeps both distros building instead, which
// is why it diverges here — expect conflicts when merging upstream, and prefer
// upstream's version if the fork ever drops Humble support.

#include <string>

#if __has_include("ament_index_cpp/get_package_share_path.hpp")
#define PLANSYS2_HAS_PACKAGE_SHARE_PATH 1
#include "ament_index_cpp/get_package_share_path.hpp"
#else
#define PLANSYS2_HAS_PACKAGE_SHARE_PATH 0
#include "ament_index_cpp/get_package_share_directory.hpp"
#endif

namespace plansys2
{

/// Absolute path to a package's share directory, as a string on every distro.
inline std::string get_package_share_dir(const std::string & package_name)
{
#if PLANSYS2_HAS_PACKAGE_SHARE_PATH
  return ament_index_cpp::get_package_share_path(package_name).string();
#else
  return ament_index_cpp::get_package_share_directory(package_name);
#endif
}

}  // namespace plansys2

#endif  // PLANSYS2_PDDL_PARSER__AMENTINDEXCOMPAT_HPP_
