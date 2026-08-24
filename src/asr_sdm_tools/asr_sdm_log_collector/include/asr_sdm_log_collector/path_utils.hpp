#ifndef ASR_SDM_LOG_COLLECTOR__PATH_UTILS_HPP_
#define ASR_SDM_LOG_COLLECTOR__PATH_UTILS_HPP_

#include <string>

namespace asr_sdm::log
{

/// Expands a leading `~` plus `$VAR` and `${VAR}` references in a configured path.
///
/// ROS parameter files are plain YAML with no shell involved, so a value such as
/// "$HOME/log/vehicle" arrives verbatim and would otherwise be created as a
/// directory literally named "$HOME". Unset variables expand to nothing, which
/// matches shell behaviour; text that cannot be parsed as a reference is left
/// exactly as written.
std::string expandUserPath(const std::string & path);

}  // namespace asr_sdm::log

#endif  // ASR_SDM_LOG_COLLECTOR__PATH_UTILS_HPP_
