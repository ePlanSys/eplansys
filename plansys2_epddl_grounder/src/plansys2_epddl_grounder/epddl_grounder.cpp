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

#include "plansys2_epddl_grounder/epddl_grounder.hpp"

#include <sys/wait.h>
#include <unistd.h>

#include <array>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <system_error>
#include <vector>

#include "plansys2_pddl_parser/AmentIndexCompat.hpp"

namespace plansys2
{

namespace
{

/// Locate an executable the way a shell would, without going through one:
/// a name containing a slash is a path, anything else is searched on PATH.
/// Running plank through a shell would mean quoting file paths correctly, and
/// a domain path with a space in it is a user's mistake to make, not a reason
/// for the grounder to run the wrong command.
std::string which(const std::string & name)
{
  std::error_code ec;

  if (name.find('/') != std::string::npos) {
    const std::filesystem::path p(name);
    if (std::filesystem::is_regular_file(p, ec) && ::access(name.c_str(), X_OK) == 0) {
      return std::filesystem::absolute(p, ec).string();
    }
    return "";
  }

  const char * path_env = std::getenv("PATH");
  if (path_env == nullptr) {
    return "";
  }

  std::stringstream paths(path_env);
  std::string dir;
  while (std::getline(paths, dir, ':')) {
    if (dir.empty()) {
      continue;
    }
    const auto candidate = std::filesystem::path(dir) / name;
    if (std::filesystem::is_regular_file(candidate, ec) &&
      ::access(candidate.c_str(), X_OK) == 0)
    {
      return candidate.string();
    }
  }
  return "";
}

/// Run @p argv, collecting stdout and stderr together into @p output.
/// @return the wait status, or -1 when the process could not be started.
///
/// The two streams are merged deliberately: plank writes its progress to
/// stdout and its diagnostics to whichever stream the failure came from, and
/// the interleaved text is what a user would have seen running it by hand,
/// which is what makes a grounding error readable when it surfaces in a ROS
/// log line.
int run(const std::vector<std::string> & argv, std::string & output)
{
  int pipe_fd[2];
  if (::pipe(pipe_fd) != 0) {
    return -1;
  }

  const pid_t pid = ::fork();
  if (pid < 0) {
    ::close(pipe_fd[0]);
    ::close(pipe_fd[1]);
    return -1;
  }

  if (pid == 0) {
    // Child: both streams to the pipe, then exec. Anything after execv means
    // exec failed; _exit rather than exit, so no parent atexit handler (or
    // rclcpp shutdown hook) runs twice.
    ::close(pipe_fd[0]);
    ::dup2(pipe_fd[1], STDOUT_FILENO);
    ::dup2(pipe_fd[1], STDERR_FILENO);
    ::close(pipe_fd[1]);

    std::vector<char *> raw;
    raw.reserve(argv.size() + 1);
    for (const auto & a : argv) {
      raw.push_back(const_cast<char *>(a.c_str()));
    }
    raw.push_back(nullptr);

    ::execv(raw[0], raw.data());
    ::_exit(127);
  }

  ::close(pipe_fd[1]);

  std::array<char, 4096> buffer{};
  ssize_t n = 0;
  while ((n = ::read(pipe_fd[0], buffer.data(), buffer.size())) > 0) {
    output.append(buffer.data(), static_cast<std::size_t>(n));
  }
  ::close(pipe_fd[0]);

  int status = 0;
  if (::waitpid(pid, &status, 0) < 0) {
    return -1;
  }
  return status;
}

/// A one-line account of how the process ended, for the error message.
std::string describe(int status)
{
  if (WIFSIGNALED(status)) {
    return "plank died on signal " + std::to_string(WTERMSIG(status));
  }
  if (WIFEXITED(status)) {
    return "plank exited with status " + std::to_string(WEXITSTATUS(status));
  }
  return "plank ended abnormally";
}

/// Fingerprint of the sources: paths and modification times. Editing a domain
/// and re-planning has to re-ground, and touching nothing has to not.
std::string fingerprint(const EpddlSpec & spec)
{
  std::string key;
  const auto add = [&key](const std::string & path) {
      std::error_code ec;
      const auto t = std::filesystem::last_write_time(path, ec);
      key += path;
      key += '@';
      key += ec ? "?" : std::to_string(t.time_since_epoch().count());
      key += ';';
    };

  add(spec.domain);
  add(spec.problem);
  for (const auto & lib : spec.libraries) {
    add(lib);
  }
  return key;
}

/// A directory of our own for plank's output, since it names the file after
/// the problem and two tasks could otherwise collide.
std::filesystem::path make_output_dir()
{
  static unsigned counter = 0;
  const auto dir = std::filesystem::temp_directory_path() /
    ("eplansys-epddl-" + std::to_string(::getpid()) + "-" + std::to_string(counter++));

  std::error_code ec;
  std::filesystem::remove_all(dir, ec);
  std::filesystem::create_directories(dir, ec);
  return dir;
}

/// plank names the output after the problem file's stem. Read that file, or
/// the only JSON in the directory if the naming ever changes.
std::string read_output(const std::filesystem::path & dir, const std::string & problem)
{
  std::error_code ec;
  const auto expected = dir / (std::filesystem::path(problem).stem().string() + ".json");

  std::filesystem::path found;
  if (std::filesystem::is_regular_file(expected, ec)) {
    found = expected;
  } else {
    for (const auto & entry : std::filesystem::directory_iterator(dir, ec)) {
      if (entry.path().extension() == ".json") {
        if (!found.empty()) {
          return "";      // ambiguous: say nothing rather than pick one
        }
        found = entry.path();
      }
    }
  }

  if (found.empty()) {
    return "";
  }

  std::ifstream in(found);
  std::stringstream contents;
  contents << in.rdbuf();
  return contents.str();
}

}  // namespace

EpddlGrounder::EpddlGrounder(const std::string & command)
: requested_command_(command)
{
}

std::string EpddlGrounder::packaged_library()
{
  try {
    const auto path = std::filesystem::path(get_package_share_dir("plansys2_epddl_grounder")) /
      "libraries" / "intermediate.epddl";
    std::error_code ec;
    return std::filesystem::is_regular_file(path, ec) ? path.string() : "";
  } catch (const std::exception &) {
    return "";     // not installed, e.g. a unit test run from the build tree
  }
}

GroundResult EpddlGrounder::ground(const EpddlSpec & spec)
{
  GroundResult result;

  if (!spec.complete()) {
    result.error = spec.empty() ?
      "no EPDDL sources configured: set both a domain and a problem" :
      "an EPDDL specification needs both a domain and a problem; only " +
      std::string(spec.domain.empty() ? "the problem" : "the domain") + " was given";
    return result;
  }

  std::error_code ec;
  for (const auto & path : {spec.domain, spec.problem}) {
    if (!std::filesystem::is_regular_file(path, ec)) {
      result.error = "no such EPDDL file: " + path;
      return result;
    }
  }

  EpddlSpec effective = spec;
  if (effective.libraries.empty()) {
    // Every domain in this repository declares `(:action-type-libraries
    // intermediate)`, and plank resolves that name only against the library
    // files it is handed. Supplying the packaged copy is what lets a domain
    // and a problem alone be enough.
    const auto packaged = packaged_library();
    if (!packaged.empty()) {
      effective.libraries.push_back(packaged);
    }
  }
  for (const auto & lib : effective.libraries) {
    if (!std::filesystem::is_regular_file(lib, ec)) {
      result.error = "no such action-type library: " + lib;
      return result;
    }
  }

  const auto key = fingerprint(effective);
  if (!cache_json_.empty() && key == cache_key_) {
    result.ok = true;
    result.task_json = cache_json_;
    return result;
  }

  if (resolved_command_.empty()) {
    const char * env = std::getenv("PLANK");
    const std::string requested = !requested_command_.empty() ? requested_command_ :
      (env != nullptr && *env != '\0' ? std::string(env) : std::string("plank"));

    resolved_command_ = which(requested);
    if (resolved_command_.empty()) {
      result.error =
        "the plank grounder was not found as '" + requested +
        "'. plank turns EPDDL into the grounded task this planner reads; build "
        "it in the workspace (it is listed in dependency_repos.repos) or set "
        "the plank_command parameter, or the PLANK environment variable, to "
        "its path.";
      return result;
    }
  }

  const auto out_dir = make_output_dir();

  std::vector<std::string> argv{
    resolved_command_, "export",
    "-d", effective.domain,
    "-p", effective.problem};
  if (!effective.libraries.empty()) {
    // One -l flag takes every library path after it; repeating the flag makes
    // plank read "-l" itself as a file name.
    argv.push_back("-l");
    for (const auto & lib : effective.libraries) {
      argv.push_back(lib);
    }
  }
  argv.push_back("-o");
  argv.push_back(out_dir.string());

  std::string output;
  const int status = run(argv, output);

  if (status < 0) {
    std::filesystem::remove_all(out_dir, ec);
    result.error = "could not run " + resolved_command_;
    return result;
  }

  result.task_json = read_output(out_dir, effective.problem);
  std::filesystem::remove_all(out_dir, ec);

  // plank reports a bad specification by printing the offending line and then
  // dying on a signal rather than exiting cleanly, so a clean exit is not
  // something to rely on; the file it was asked to write is. Its output goes
  // into the error either way, because that is where the syntax error is.
  if (result.task_json.empty()) {
    result.error = "grounding failed (" + describe(status) + "):\n" + output;
    return result;
  }

  cache_key_ = key;
  cache_json_ = result.task_json;

  result.ok = true;
  return result;
}

}  // namespace plansys2
