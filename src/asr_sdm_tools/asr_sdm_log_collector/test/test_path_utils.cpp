#include "asr_sdm_log_collector/path_utils.hpp"

#include <gtest/gtest.h>

#include <cstdlib>
#include <optional>
#include <string>

namespace
{

/// Restores the variables the tests overwrite, so ordering cannot matter.
class PathUtilsTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    saved_home_ = readVariable("HOME");
    ::setenv("HOME", "/home/tester", 1);
    ::setenv("ASR_SDM_TEST_DIR", "/data/logs", 1);
    ::unsetenv("ASR_SDM_TEST_MISSING");
  }

  void TearDown() override
  {
    if (saved_home_.has_value()) {
      ::setenv("HOME", saved_home_->c_str(), 1);
    } else {
      ::unsetenv("HOME");
    }
    ::unsetenv("ASR_SDM_TEST_DIR");
  }

private:
  static std::optional<std::string> readVariable(const char * name)
  {
    const char * value = std::getenv(name);
    if (value == nullptr) {
      return std::nullopt;
    }
    return std::string{value};
  }

  std::optional<std::string> saved_home_;
};

TEST_F(PathUtilsTest, LeavesPlainPathsAlone)
{
  EXPECT_EQ(asr_sdm::log::expandUserPath("/run/vehicle/log.sock"), "/run/vehicle/log.sock");
  EXPECT_EQ(asr_sdm::log::expandUserPath(""), "");
}

TEST_F(PathUtilsTest, ExpandsBareAndBracedVariables)
{
  EXPECT_EQ(asr_sdm::log::expandUserPath("$HOME/log/vehicle"), "/home/tester/log/vehicle");
  EXPECT_EQ(asr_sdm::log::expandUserPath("${HOME}/log/vehicle"), "/home/tester/log/vehicle");
  EXPECT_EQ(asr_sdm::log::expandUserPath("$ASR_SDM_TEST_DIR/run"), "/data/logs/run");
  // A variable at the very end has no separator to stop the name scan.
  EXPECT_EQ(asr_sdm::log::expandUserPath("$HOME"), "/home/tester");
}

TEST_F(PathUtilsTest, ExpandsLeadingTildeOnly)
{
  EXPECT_EQ(asr_sdm::log::expandUserPath("~/log/vehicle"), "/home/tester/log/vehicle");
  EXPECT_EQ(asr_sdm::log::expandUserPath("~"), "/home/tester");
  // Anywhere else a tilde is an ordinary character, as are backup file names.
  EXPECT_EQ(asr_sdm::log::expandUserPath("/var/log/all.log~"), "/var/log/all.log~");
  EXPECT_EQ(asr_sdm::log::expandUserPath("/var/~/log"), "/var/~/log");
}

TEST_F(PathUtilsTest, DropsUnsetVariablesLikeAShell)
{
  EXPECT_EQ(asr_sdm::log::expandUserPath("$ASR_SDM_TEST_MISSING/log"), "/log");
  EXPECT_EQ(asr_sdm::log::expandUserPath("${ASR_SDM_TEST_MISSING}"), "");
}

TEST_F(PathUtilsTest, KeepsTextThatIsNotAReference)
{
  // A '$' with no name after it, and an unbalanced brace, stay as written rather
  // than silently eating the rest of the path.
  EXPECT_EQ(asr_sdm::log::expandUserPath("/var/log/$"), "/var/log/$");
  EXPECT_EQ(asr_sdm::log::expandUserPath("/var/$/log"), "/var/$/log");
  EXPECT_EQ(asr_sdm::log::expandUserPath("/var/${HOME/log"), "/var/${HOME/log");
}

}  // namespace
