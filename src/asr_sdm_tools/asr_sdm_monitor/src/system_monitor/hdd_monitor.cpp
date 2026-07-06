#include "asr_sdm_monitor/system_monitor/hdd_monitor.hpp"

#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>

#include <set>

#include "asr_sdm_monitor/system_monitor/monitor_utils.hpp"

namespace asr_sdm_monitor
{
namespace system_monitor
{

HddMonitor::HddMonitor(
    const std::string & hostname,
    const std::string & diag_hostname,
    const rclcpp::NodeOptions & options)
: Node("hdd_monitor", options)
{
    no_temp_ = declare_parameter("no_hw_temp", true);
    no_temp_warn_ = declare_parameter("no_hw_temp_warn", false);
    hdd_level_warn_ = declare_parameter("hdd_level_warn", 0.95);
    hdd_level_error_ = declare_parameter("hdd_level_error", 0.99);
    hdd_temp_warn_ = declare_parameter("hdd_temp_warn", 55.0);
    hdd_temp_error_ = declare_parameter("hdd_temp_error", 70.0);

    updater_ = std::make_shared<diagnostic_updater::Updater>(this);
    updater_->setHardwareID(hostname);

    if (!no_temp_) {
        temp_stat_.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
        temp_stat_.message = "No Data";
        temp_stat_.values = {
            diagnostic_msgs::msg::KeyValue().set__key("Update Status").set__value("No Data"),
            diagnostic_msgs::msg::KeyValue().set__key("Time Since Last Update").set__value("N/A"),
        };
        updater_->add(
            "HW Temperature (" + diag_hostname + ")",
            this, &HddMonitor::updateTempStatus);
    }

    usage_stat_.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    usage_stat_.message = "No Data";
    usage_stat_.values = {
        diagnostic_msgs::msg::KeyValue().set__key("Update Status").set__value("No Data"),
        diagnostic_msgs::msg::KeyValue().set__key("Time Since Last Update").set__value("N/A"),
    };
    updater_->add(
        "HDD Usage (" + diag_hostname + ")",
        this, &HddMonitor::updateUsageStatus);

    if (!no_temp_) {
        temp_timer_ = create_wall_timer(
            std::chrono::milliseconds(10000),
            [this]() { checkTemps(); });
        checkTemps();
    }

    usage_timer_ = create_wall_timer(
        std::chrono::milliseconds(5000),
        [this]() { checkDiskUsage(); });
    checkDiskUsage();
}

void HddMonitor::extractTempInputs(
    const QJsonObject & object,
    const std::string & prefix,
    std::vector<std::string> & paths,
    std::vector<double> & temps) const
{
    for (auto it = object.begin(); it != object.end(); ++it) {
        const std::string key = it.key().toStdString();
        const QJsonValue value = it.value();
        if (value.isDouble()) {
            if (key.size() >= 5 && key.substr(key.size() - 5) == "input") {
                paths.push_back(prefix.empty() ? key : prefix + "/" + key);
                temps.push_back(value.toDouble());
            }
        } else if (value.isObject()) {
            const std::string next_prefix = prefix.empty() ? key : prefix + "/" + key;
            extractTempInputs(value.toObject(), next_prefix, paths, temps);
        }
    }
}

void HddMonitor::checkTemps()
{
    std::vector<diagnostic_msgs::msg::KeyValue> diag_vals = {
        diagnostic_msgs::msg::KeyValue().set__key("Update Status").set__value("OK"),
        diagnostic_msgs::msg::KeyValue().set__key("Time Since Last Update").set__value("0.0s"),
    };
    std::set<std::string> diag_msgs;
    uint8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    bool sensors_ok = true;

    const auto cmd = runShellCommand("sensors -j");
    if (cmd.return_code != 0) {
        if (!has_warned_sensors_) {
            RCLCPP_ERROR(
                get_logger(),
                "'sensors' failed to run for hdd_monitor. Return code %d.",
                cmd.return_code);
            has_warned_sensors_ = true;
        }
        diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diag_vals.push_back(
            diagnostic_msgs::msg::KeyValue()
                .set__key("\"sensors -j\" Call Error")
                .set__value(std::to_string(cmd.return_code)));
        diag_msgs.insert("Unable to Check HW Temp");
        sensors_ok = false;
    } else {
        QJsonParseError parse_error;
        const QJsonDocument document = QJsonDocument::fromJson(
            QByteArray::fromStdString(cmd.stdout_text), &parse_error);
        if (parse_error.error != QJsonParseError::NoError || !document.isObject()) {
            diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            diag_vals.push_back(
                diagnostic_msgs::msg::KeyValue()
                    .set__key("Sensors Exception")
                    .set__value(parse_error.errorString().toStdString()));
            sensors_ok = false;
        } else {
            std::vector<std::string> hw_paths;
            std::vector<double> temps;
            extractTempInputs(document.object(), "", hw_paths, temps);
            for (size_t index = 0; index < hw_paths.size(); ++index) {
                const double temp = temps[index];
                uint8_t temp_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
                if (temp >= hdd_temp_error_) {
                    temp_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
                } else if (temp >= hdd_temp_warn_) {
                    temp_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
                }

                diag_level = maxLevel(diag_level, temp_level);
                diag_vals.push_back(
                    diagnostic_msgs::msg::KeyValue()
                        .set__key(hw_paths[index] + " Temperature Status")
                        .set__value(tempDict(temp_level)));
                diag_vals.push_back(
                    diagnostic_msgs::msg::KeyValue()
                        .set__key(hw_paths[index] + " Temperature")
                        .set__value(std::to_string(temp) + "DegC"));
            }
        }
    }

    std::string message;
    if (!diag_msgs.empty()) {
        message = *diag_msgs.begin();
        for (auto it = std::next(diag_msgs.begin()); it != diag_msgs.end(); ++it) {
            message += ", " + *it;
        }
    } else {
        message = tempDict(diag_level);
    }

    if (no_temp_warn_ && sensors_ok) {
        diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    }

    std::lock_guard<std::mutex> lock(mutex_);
    last_temp_time_ = get_clock()->now();
    temp_stat_.values = std::move(diag_vals);
    temp_stat_.message = message;
    temp_stat_.level = diag_level;
}

void HddMonitor::checkDiskUsage()
{
    std::vector<diagnostic_msgs::msg::KeyValue> diag_vals = {
        diagnostic_msgs::msg::KeyValue().set__key("Update Status").set__value("OK"),
        diagnostic_msgs::msg::KeyValue().set__key("Time Since Last Update").set__value("0.0s"),
    };
    uint8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    std::string diag_message = "OK";

    const auto cmd = runCommand({"df", "-Pht", "ext4"});
    if (cmd.return_code == 0 || cmd.return_code == 1) {
        diag_vals.push_back(
            diagnostic_msgs::msg::KeyValue().set__key("Disk Space Reading").set__value("OK"));

        auto rows = splitLines(cmd.stdout_text);
        if (!rows.empty()) {
            rows.erase(rows.begin());
        }

        int row_count = 0;
        for (const auto & row : rows) {
            const auto parts = splitWhitespace(row);
            if (parts.size() < 2 || parts[0] == "none") {
                continue;
            }
            if (parts.size() < 6) {
                continue;
            }

            ++row_count;
            const std::string & name = parts[0];
            const std::string & size = parts[1];
            const std::string & g_available = parts[parts.size() - 3];
            const std::string g_use = parts[parts.size() - 2];
            const std::string & mount_pt = parts.back();

            std::string use_value = g_use;
            if (!use_value.empty() && use_value.back() == '%') {
                use_value.pop_back();
            }
            const double hdd_usage = std::stod(use_value) * 0.01;

            uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
            if (hdd_usage >= hdd_level_error_) {
                level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            } else if (hdd_usage >= hdd_level_warn_) {
                level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
            }

            diag_vals.push_back(
                diagnostic_msgs::msg::KeyValue()
                    .set__key("Disk " + std::to_string(row_count) + " Name")
                    .set__value(name));
            diag_vals.push_back(
                diagnostic_msgs::msg::KeyValue()
                    .set__key("Disk " + std::to_string(row_count) + " Size")
                    .set__value(size));
            diag_vals.push_back(
                diagnostic_msgs::msg::KeyValue()
                    .set__key("Disk " + std::to_string(row_count) + " Available")
                    .set__value(g_available));
            diag_vals.push_back(
                diagnostic_msgs::msg::KeyValue()
                    .set__key("Disk " + std::to_string(row_count) + " Use")
                    .set__value(g_use));
            diag_vals.push_back(
                diagnostic_msgs::msg::KeyValue()
                    .set__key("Disk " + std::to_string(row_count) + " Status")
                    .set__value(statDict(level)));
            diag_vals.push_back(
                diagnostic_msgs::msg::KeyValue()
                    .set__key("Disk " + std::to_string(row_count) + " Mount Point")
                    .set__value(mount_pt));

            diag_level = maxLevel(diag_level, level);
            diag_message = usageDict(diag_level);
        }
    } else {
        diag_vals.push_back(
            diagnostic_msgs::msg::KeyValue().set__key("Disk Space Reading").set__value("Failed"));
        diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
        diag_message = statDict(diag_level);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    last_usage_time_ = get_clock()->now();
    usage_stat_.values = std::move(diag_vals);
    usage_stat_.message = diag_message;
    usage_stat_.level = diag_level;
}

void HddMonitor::updateTempStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto temp_copy = temp_stat_;
    updateStatusStale(temp_copy, *get_clock(), last_temp_time_);
    stat.summary(temp_copy.level, temp_copy.message);
    for (const auto & value : temp_copy.values) {
        stat.add(value.key, value.value);
    }
}

void HddMonitor::updateUsageStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
    std::lock_guard<std::mutex> lock(mutex_);
    auto usage_copy = usage_stat_;
    updateStatusStale(usage_copy, *get_clock(), last_usage_time_);
    stat.summary(usage_copy.level, usage_copy.message);
    for (const auto & value : usage_copy.values) {
        stat.add(value.key, value.value);
    }
}

}  // namespace system_monitor
}  // namespace asr_sdm_monitor
