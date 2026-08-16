#ifndef ASR_SDM_CONTROL_MSGS_ENCODE_MSGS_H
#define ASR_SDM_CONTROL_MSGS_ENCODE_MSGS_H

#include <stdint.h>
#include <vector>
#include <asr_sdm_control_msgs/msg/so3_command.hpp>
#include <asr_sdm_control_msgs/msg/trpy_command.hpp>
#include <asr_sdm_control_msgs/msg/gains.hpp>

namespace asr_sdm_control_msgs
{

void encodeSO3Command(const asr_sdm_control_msgs::msg::SO3Command & so3_command,
                      std::vector<uint8_t> & output);
void encodeTRPYCommand(const asr_sdm_control_msgs::msg::TRPYCommand & trpy_command,
                       std::vector<uint8_t> & output);

void encodePPRGains(const asr_sdm_control_msgs::msg::Gains & gains,
                    std::vector<uint8_t> & output);
}

#endif
