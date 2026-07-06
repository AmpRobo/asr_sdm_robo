#include "system_monitor/net_monitor.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <fstream>
#include <functional>
#include <map>
#include <regex>
#include <set>
#include <sstream>
#include <thread>

#include "system_monitor/monitor_utils.hpp"

namespace asr_sdm_monitor
{
namespace system_monitor
{

namespace
{

struct InterfaceByteCounter
{
    std::string name;
    uint64_t rx_bytes = 0;
    uint64_t tx_bytes = 0;
};

std::string trimCopy(const std::string & text)
{
    const auto begin = std::find_if_not(
        text.begin(), text.end(),
        [](unsigned char ch) { return std::isspace(ch) != 0; });
    const auto end = std::find_if_not(
        text.rbegin(), text.rend(),
        [](unsigned char ch) { return std::isspace(ch) != 0; }).base();
    if (begin >= end) {
        return {};
    }
    return std::string(begin, end);
}

std::vector<InterfaceByteCounter> readProcNetDev()
{
    std::vector<InterfaceByteCounter> counters;
    std::ifstream input("/proc/net/dev");
    if (!input) {
        return counters;
    }

    std::string line;
    std::getline(input, line);
    std::getline(input, line);

    while (std::getline(input, line)) {
        const auto colon = line.find(':');
        if (colon == std::string::npos) {
            continue;
        }

        InterfaceByteCounter counter;
        counter.name = trimCopy(line.substr(0, colon));
        if (counter.name.empty()) {
            continue;
        }

        std::istringstream fields(line.substr(colon + 1));
        uint64_t rx_packets = 0;
        uint64_t rx_errs = 0;
        uint64_t rx_drop = 0;
        uint64_t rx_fifo = 0;
        uint64_t rx_frame = 0;
        uint64_t rx_compressed = 0;
        uint64_t rx_multicast = 0;
        uint64_t tx_packets = 0;
        uint64_t tx_errs = 0;
        uint64_t tx_drop = 0;
        uint64_t tx_fifo = 0;
        uint64_t tx_colls = 0;
        uint64_t tx_carrier = 0;
        uint64_t tx_compressed = 0;

        fields >> counter.rx_bytes >> rx_packets >> rx_errs >> rx_drop >> rx_fifo >>
            rx_frame >> rx_compressed >> rx_multicast >> counter.tx_bytes >> tx_packets >>
            tx_errs >> tx_drop >> tx_fifo >> tx_colls >> tx_carrier >> tx_compressed;

        if (fields.fail()) {
            continue;
        }

        counters.push_back(counter);
    }

    return counters;
}

std::string joinStrings(const std::vector<std::string> & parts, const std::string & delimiter)
{
    std::string result;
    for (size_t i = 0; i < parts.size(); ++i) {
        if (i > 0) {
            result += delimiter;
        }
        result += parts[i];
    }
    return result;
}

std::string readInterfaceIpv4(const std::string & iface)
{
    const auto cmd = runCommand({"ip", "-4", "-o", "addr", "show", "dev", iface});
    if (cmd.return_code != 0) {
        return {};
    }

    static const std::regex inet_pattern(R"(\binet\s+(\d+\.\d+\.\d+\.\d+))");
    std::vector<std::string> addresses;
    for (const auto & line : splitLines(cmd.stdout_text)) {
        std::smatch match;
        if (std::regex_search(line, match, inet_pattern)) {
            addresses.push_back(match.str(1));
        }
    }

    return joinStrings(addresses, ", ");
}

std::tuple<uint8_t, std::string, std::vector<diagnostic_msgs::msg::KeyValue>>
buildNetworkDiagnostics(
    const std::vector<std::string> & ifaces,
    const std::vector<double> & kb_in,
    const std::vector<double> & kb_out,
    double net_level_warn,
    double net_capacity,
    const std::function<std::pair<int, std::string>(const std::string &, const std::string &)> &
        read_sys_net,
    const std::function<std::pair<int, std::string>(const std::string &, const std::string &)> &
        read_sys_net_stat)
{
    std::vector<diagnostic_msgs::msg::KeyValue> values;
    uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    static const std::regex eth_pattern(R"(eth[0-9]+)");

    for (size_t i = 0; i < ifaces.size() && i < kb_in.size() && i < kb_out.size(); ++i) {
        values.push_back(
            diagnostic_msgs::msg::KeyValue().set__key("Interface Name").set__value(ifaces[i]));

        const auto [state_code, state_out] = read_sys_net(ifaces[i], "operstate");
        if (state_code == 0) {
            values.push_back(
                diagnostic_msgs::msg::KeyValue().set__key("State").set__value(state_out));
            if (std::regex_match(ifaces[i], eth_pattern) &&
                (state_out == "down" || state_out == "dormant"))
            {
                level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
            }
        }

        const std::string ip_address = readInterfaceIpv4(ifaces[i]);
        values.push_back(
            diagnostic_msgs::msg::KeyValue()
                .set__key("IP Address")
                .set__value(ip_address.empty() ? "N/A" : ip_address));

        const double in_mbps = kb_in[i] / 1024.0;
        const double out_mbps = kb_out[i] / 1024.0;
        values.push_back(
            diagnostic_msgs::msg::KeyValue()
                .set__key("Input Traffic")
                .set__value(std::to_string(in_mbps) + " (MB/s)"));
        values.push_back(
            diagnostic_msgs::msg::KeyValue()
                .set__key("Output Traffic")
                .set__value(std::to_string(out_mbps) + " (MB/s)"));

        const double net_usage_in = in_mbps / net_capacity;
        const double net_usage_out = out_mbps / net_capacity;
        if (net_usage_in > net_level_warn || net_usage_out > net_level_warn) {
            level = maxLevel(level, diagnostic_msgs::msg::DiagnosticStatus::WARN);
        }

        const auto [mtu_code, mtu_out] = read_sys_net(ifaces[i], "mtu");
        if (mtu_code == 0) {
            values.push_back(
                diagnostic_msgs::msg::KeyValue().set__key("MTU").set__value(mtu_out));
        }

        const auto [rx_code, rx_out] = read_sys_net_stat(ifaces[i], "rx_bytes");
        if (rx_code == 0) {
            values.push_back(
                diagnostic_msgs::msg::KeyValue()
                    .set__key("Total received MB")
                    .set__value(std::to_string(std::stod(rx_out) / 1024.0 / 1024.0)));
        }

        const auto [tx_code, tx_out] = read_sys_net_stat(ifaces[i], "tx_bytes");
        if (tx_code == 0) {
            values.push_back(
                diagnostic_msgs::msg::KeyValue()
                    .set__key("Total transmitted MB")
                    .set__value(std::to_string(std::stod(tx_out) / 1024.0 / 1024.0)));
        }

        const auto [collision_code, collision_out] = read_sys_net_stat(ifaces[i], "collisions");
        if (collision_code == 0) {
            values.push_back(
                diagnostic_msgs::msg::KeyValue().set__key("Collisions").set__value(collision_out));
        }

        const auto [rx_err_code, rx_err_out] = read_sys_net_stat(ifaces[i], "rx_errors");
        if (rx_err_code == 0) {
            values.push_back(
                diagnostic_msgs::msg::KeyValue().set__key("Rx Errors").set__value(rx_err_out));
        }

        const auto [tx_err_code, tx_err_out] = read_sys_net_stat(ifaces[i], "tx_errors");
        if (tx_err_code == 0) {
            values.push_back(
                diagnostic_msgs::msg::KeyValue().set__key("Tx Errors").set__value(tx_err_out));
        }
    }

    return {level, netDict(level), values};
}

}  // namespace

NetMonitor::NetMonitor(
    const std::string & hostname,
    const std::string & diag_hostname,
    const rclcpp::NodeOptions & options)
: Node("net_monitor", options)
{
    net_level_warn_ = declare_parameter("net_level_warn", 0.95);
    net_capacity_ = declare_parameter("net_capacity", 128.0);

    updater_ = std::make_shared<diagnostic_updater::Updater>(this);
    updater_->setHardwareID(hostname);

    usage_stat_.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
    usage_stat_.message = "No Data";
    usage_stat_.values = {
        diagnostic_msgs::msg::KeyValue().set__key("Update Status").set__value("No Data"),
        diagnostic_msgs::msg::KeyValue().set__key("Time Since Last Update").set__value("N/A"),
    };

    updater_->add(
        "Network Usage (" + diag_hostname + ")",
        this, &NetMonitor::updateUsageStatus);

    usage_timer_ = create_wall_timer(
        std::chrono::milliseconds(5000),
        [this]() { checkUsage(); });

    checkUsage();
}

std::pair<int, std::string> NetMonitor::readSysNetStat(
    const std::string & iface,
    const std::string & stat_name)
{
    const auto cmd = runShellCommand("cat /sys/class/net/" + iface + "/statistics/" + stat_name);
    if (cmd.return_code != 0) {
        return {cmd.return_code, cmd.stderr_text};
    }

    std::string output = cmd.stdout_text;
    while (!output.empty() && (output.back() == '\n' || output.back() == '\r')) {
        output.pop_back();
    }
    return {0, output};
}

std::pair<int, std::string> NetMonitor::readSysNet(
    const std::string & iface,
    const std::string & field)
{
    const auto cmd = runShellCommand("cat /sys/class/net/" + iface + "/" + field);
    if (cmd.return_code != 0) {
        return {cmd.return_code, cmd.stderr_text};
    }

    std::string output = cmd.stdout_text;
    while (!output.empty() && (output.back() == '\n' || output.back() == '\r')) {
        output.pop_back();
    }
    return {0, output};
}

std::tuple<uint8_t, std::string, std::vector<diagnostic_msgs::msg::KeyValue>>
NetMonitor::checkNetwork()
{
    const auto read_sys_net = [this](const std::string & iface, const std::string & field)
    {
        return readSysNet(iface, field);
    };
    const auto read_sys_net_stat = [this](const std::string & iface, const std::string & stat_name)
    {
        return readSysNetStat(iface, stat_name);
    };

    const auto cmd = runShellCommand("ifstat -q -S 1 1");
    if (cmd.return_code == 0) {
        const auto rows = splitLines(cmd.stdout_text);
        if (rows.size() >= 3) {
            const auto ifaces = splitWhitespace(rows[0]);
            const auto data = splitWhitespace(rows[2]);
            std::vector<double> kb_in;
            std::vector<double> kb_out;
            for (size_t i = 0; i + 1 < data.size(); i += 2) {
                kb_in.push_back(std::stod(data[i]));
                kb_out.push_back(std::stod(data[i + 1]));
            }

            if (!ifaces.empty() && kb_in.size() == ifaces.size()) {
                return buildNetworkDiagnostics(
                    ifaces, kb_in, kb_out, net_level_warn_, net_capacity_,
                    read_sys_net, read_sys_net_stat);
            }
        }
    }

    const auto first_sample = readProcNetDev();
    if (first_sample.empty()) {
        std::vector<diagnostic_msgs::msg::KeyValue> values;
        values.push_back(
            diagnostic_msgs::msg::KeyValue()
                .set__key("Network Usage Check Error")
                .set__value("Unable to read /proc/net/dev"));
        return {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Network Usage Check Error", values};
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
    const auto second_sample = readProcNetDev();
    if (second_sample.empty()) {
        std::vector<diagnostic_msgs::msg::KeyValue> values;
        values.push_back(
            diagnostic_msgs::msg::KeyValue()
                .set__key("Network Usage Check Error")
                .set__value("Unable to resample /proc/net/dev"));
        return {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Network Usage Check Error", values};
    }

    std::map<std::string, InterfaceByteCounter> first_by_name;
    for (const auto & counter : first_sample) {
        first_by_name[counter.name] = counter;
    }

    std::vector<std::string> ifaces;
    std::vector<double> kb_in;
    std::vector<double> kb_out;
    for (const auto & counter : second_sample) {
        if (counter.name == "lo") {
            continue;
        }

        const auto previous = first_by_name.find(counter.name);
        if (previous == first_by_name.end()) {
            continue;
        }

        const double rx_kbps = static_cast<double>(counter.rx_bytes - previous->second.rx_bytes) / 1024.0;
        const double tx_kbps = static_cast<double>(counter.tx_bytes - previous->second.tx_bytes) / 1024.0;

        ifaces.push_back(counter.name);
        kb_in.push_back(std::max(rx_kbps, 0.0));
        kb_out.push_back(std::max(tx_kbps, 0.0));
    }

    if (ifaces.empty()) {
        std::vector<diagnostic_msgs::msg::KeyValue> values;
        values.push_back(
            diagnostic_msgs::msg::KeyValue()
                .set__key("Network Usage Check Error")
                .set__value("No active network interfaces found"));
        return {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Network Usage Check Error", values};
    }

    return buildNetworkDiagnostics(
        ifaces, kb_in, kb_out, net_level_warn_, net_capacity_, read_sys_net, read_sys_net_stat);
}

void NetMonitor::checkUsage()
{
    std::vector<diagnostic_msgs::msg::KeyValue> diag_vals = {
        diagnostic_msgs::msg::KeyValue().set__key("Update Status").set__value("OK"),
        diagnostic_msgs::msg::KeyValue().set__key("Time Since Last Update").set__value("0.0s"),
    };
    std::set<std::string> diag_msgs;
    uint8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;

    const auto [net_level, net_msg, net_vals] = checkNetwork();
    diag_vals.insert(diag_vals.end(), net_vals.begin(), net_vals.end());
    if (net_level != diagnostic_msgs::msg::DiagnosticStatus::OK) {
        diag_msgs.insert(net_msg);
    }
    diag_level = maxLevel(diag_level, net_level);

    std::string usage_msg;
    if (!diag_msgs.empty() && diag_level != diagnostic_msgs::msg::DiagnosticStatus::OK) {
        usage_msg = *diag_msgs.begin();
        for (auto it = std::next(diag_msgs.begin()); it != diag_msgs.end(); ++it) {
            usage_msg += ", " + *it;
        }
    } else {
        usage_msg = statDict(diag_level);
    }

    std::lock_guard<std::mutex> lock(mutex_);
    last_usage_time_ = get_clock()->now();
    usage_stat_.level = diag_level;
    usage_stat_.values = std::move(diag_vals);
    usage_stat_.message = usage_msg;
}

void NetMonitor::updateUsageStatus(diagnostic_updater::DiagnosticStatusWrapper & stat)
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
