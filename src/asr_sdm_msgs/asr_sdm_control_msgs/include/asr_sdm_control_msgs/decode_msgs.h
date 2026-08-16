#ifndef ASR_SDM_CONTROL_MSGS_DECODE_MSGS_H
#define ASR_SDM_CONTROL_MSGS_DECODE_MSGS_H

#include <stdint.h>
#include <vector>
#include <asr_sdm_control_msgs/msg/output_data.hpp>
#include <asr_sdm_control_msgs/msg/status_data.hpp>
#include <asr_sdm_control_msgs/msg/ppr_output_data.hpp>

namespace asr_sdm_control_msgs
{

bool decodeOutputData(const std::vector<uint8_t> & data,
                      asr_sdm_control_msgs::msg::OutputData & output);

bool decodeStatusData(const std::vector<uint8_t> & data,
                      asr_sdm_control_msgs::msg::StatusData & status);

bool decodePPROutputData(const std::vector<uint8_t> & data,
                         asr_sdm_control_msgs::msg::PPROutputData & output);
}

#endif
