#include "asr_sdm_log_collector/path_utils.hpp"

#include <cctype>
#include <cstdlib>

namespace asr_sdm::log
{

namespace
{

const char * environmentValue(const std::string & name)
{
  const char * value = std::getenv(name.c_str());
  return value != nullptr ? value : "";
}

bool isNameCharacter(char character)
{
  return std::isalnum(static_cast<unsigned char>(character)) != 0 || character == '_';
}

}  // namespace

std::string expandUserPath(const std::string & path)
{
  std::string result;
  result.reserve(path.size());

  std::size_t index = 0;

  // A tilde only means the home directory at the very start of a path.
  if (path == "~" || path.rfind("~/", 0) == 0) {
    result.append(environmentValue("HOME"));
    index = 1;
  }

  while (index < path.size()) {
    if (path[index] != '$') {
      result.push_back(path[index++]);
      continue;
    }

    // A trailing '$' has nothing to expand.
    if (index + 1 >= path.size()) {
      result.push_back(path[index++]);
      continue;
    }

    const bool braced = path[index + 1] == '{';
    const std::size_t name_start = braced ? index + 2 : index + 1;
    std::size_t name_end = name_start;

    if (braced) {
      name_end = path.find('}', name_start);
      if (name_end == std::string::npos) {
        // Unbalanced brace: keep the remainder verbatim rather than guessing.
        result.append(path, index, std::string::npos);
        break;
      }
    } else {
      while (name_end < path.size() && isNameCharacter(path[name_end])) {
        ++name_end;
      }
      // A '$' not followed by a name is an ordinary character.
      if (name_end == name_start) {
        result.push_back(path[index++]);
        continue;
      }
    }

    result.append(environmentValue(path.substr(name_start, name_end - name_start)));
    index = braced ? name_end + 1 : name_end;
  }

  return result;
}

}  // namespace asr_sdm::log
