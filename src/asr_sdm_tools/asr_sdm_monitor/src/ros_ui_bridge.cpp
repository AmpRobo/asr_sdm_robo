#include "asr_sdm_monitor/ros_ui_bridge.hpp"

#include <QMetaObject>
#include <QStringList>
#include <Qt>

#include <algorithm>
#include <map>
#include <numeric>
#include <regex>
#include <sstream>
#include <vector>

RosUiBridge::RosUiBridge(QObject *parent)
    : QObject(parent),
      ros_status_("Waiting for /diagnostics ...")
{
    node_ = std::make_shared<rclcpp::Node>("diagnostics_qml_ui_node");

    diagnostics_sub_ = node_->create_subscription<diagnostic_msgs::msg::DiagnosticArray>(
        "/diagnostics", 20,
        [this](const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
        {
            diagnosticsCallback(msg);
        });

    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);

    ros_status_ = "Subscribed: /diagnostics";
    emit rosStatusChanged();

    ros_thread_ = std::thread([this]()
    {
        executor_->spin();
    });
}

RosUiBridge::~RosUiBridge()
{
    if (executor_) {
        executor_->cancel();
    }

    if (ros_thread_.joinable()) {
        ros_thread_.join();
    }
}

QString RosUiBridge::rosStatus() const
{
    return ros_status_;
}

QVariantMap RosUiBridge::cpuSummary() const
{
    return cpu_summary_;
}

QVariantList RosUiBridge::cpuHistory() const
{
    return cpu_history_;
}

QVariantList RosUiBridge::cpuCoreRows() const
{
    return cpu_core_rows_;
}

QVariantMap RosUiBridge::memorySummary() const
{
    return memory_summary_;
}

QVariantList RosUiBridge::memoryHistory() const
{
    return memory_history_;
}

QVariantList RosUiBridge::memoryRows() const
{
    return memory_rows_;
}

QVariantMap RosUiBridge::hddSummary() const
{
    return hdd_summary_;
}

QVariantList RosUiBridge::hddRows() const
{
    return hdd_rows_;
}

QVariantMap RosUiBridge::netSummary() const
{
    return net_summary_;
}

QVariantList RosUiBridge::netInHistory() const
{
    return net_in_history_;
}

QVariantList RosUiBridge::netOutHistory() const
{
    return net_out_history_;
}

QVariantList RosUiBridge::netInterfaceRows() const
{
    return net_interface_rows_;
}

QVariantMap RosUiBridge::ntpSummary() const
{
    return ntp_summary_;
}

QVariantList RosUiBridge::ntpRows() const
{
    return ntp_rows_;
}

void RosUiBridge::diagnosticsCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg)
{
    QMetaObject::invokeMethod(
        this,
        [this]()
        {
            ros_status_ = "Receiving: /diagnostics";
            emit rosStatusChanged();
        },
        Qt::QueuedConnection);

    for (const auto &status : msg->status) {
        const QString name = QString::fromStdString(status.name).toLower();

        if (name.contains("cpu usage")) {
            updateCpu(status);
        } else if (name.contains("memory usage")) {
            updateMemory(status);
        } else if (name.contains("hdd usage")) {
            updateHdd(status);
        } else if (name.contains("network usage")) {
            updateNet(status);
        } else if (name.contains("ntp offset")) {
            updateNtp(status);
        }
    }
}

QString RosUiBridge::valueForKey(const diagnostic_msgs::msg::DiagnosticStatus &status, const QString &key)
{
    for (const auto &item : status.values) {
        if (QString::fromStdString(item.key) == key) {
            return QString::fromStdString(item.value);
        }
    }
    return {};
}

double RosUiBridge::extractNumber(const QString &text)
{
    static const std::regex pattern(R"((-?\d+(?:\.\d+)?))");
    std::smatch match;
    const std::string input = text.toStdString();
    if (std::regex_search(input, match, pattern)) {
        return std::stod(match.str(1));
    }
    return 0.0;
}

QString RosUiBridge::formatNumber(double value, int precision)
{
    return QString::number(value, 'f', precision);
}

QString RosUiBridge::levelToText(unsigned char level)
{
    switch (level) {
    case 0:
        return "OK";
    case 1:
        return "WARN";
    case 2:
        return "ERROR";
    case 3:
        return "STALE";
    default:
        return "UNKNOWN";
    }
}

void RosUiBridge::appendHistory(QVariantList &history, double value, int maxPoints)
{
    history.append(value);
    while (history.size() > maxPoints) {
        history.removeFirst();
    }
}

void RosUiBridge::updateCpu(const diagnostic_msgs::msg::DiagnosticStatus &status)
{
    struct CoreData {
        int index = -1;
        QString status;
        QString clock;
        QString user;
        QString system;
        QString idle;
        double usage = 0.0;
        double clockValue = 0.0;
    };

    std::map<int, CoreData> cores;
    static const std::regex core_pattern(R"(Core\s+(\d+)\s+(.+))");

    for (const auto &item : status.values) {
        std::smatch match;
        const std::string key = item.key;
        if (!std::regex_match(key, match, core_pattern)) {
            continue;
        }

        const int idx = std::stoi(match.str(1));
        const QString field = QString::fromStdString(match.str(2));
        auto &core = cores[idx];
        core.index = idx;

        const QString value = QString::fromStdString(item.value);
        if (field == "Status") {
            core.status = value;
        } else if (field == "Clock Speed") {
            core.clock = value;
            core.clockValue = extractNumber(value);
        } else if (field == "User") {
            core.user = value;
        } else if (field == "System") {
            core.system = value;
        } else if (field == "Idle") {
            core.idle = value;
            core.usage = std::clamp(100.0 - extractNumber(value), 0.0, 100.0);
        }
    }

    QVariantList rows;
    std::vector<double> usages;
    std::vector<double> clocks;
    double max_usage = 0.0;

    for (const auto &[index, core] : cores) {
        Q_UNUSED(index);
        QVariantMap row;
        row["core"] = QString::number(core.index);
        row["usage"] = formatNumber(core.usage, 1) + "%";
        row["clock"] = core.clock;
        row["user"] = core.user;
        row["system"] = core.system;
        row["idle"] = core.idle;
        row["status"] = core.status;
        rows.append(row);

        usages.push_back(core.usage);
        clocks.push_back(core.clockValue);
        max_usage = std::max(max_usage, core.usage);
    }

    const double avg_usage = usages.empty()
                                 ? 0.0
                                 : std::accumulate(usages.begin(), usages.end(), 0.0) / usages.size();
    const double avg_clock = clocks.empty()
                                 ? 0.0
                                 : std::accumulate(clocks.begin(), clocks.end(), 0.0) / clocks.size();

    QVariantMap summary;
    summary["state"] = QString::fromStdString(status.message);
    summary["level"] = levelToText(status.level);
    summary["avgUsage"] = formatNumber(avg_usage, 1) + "%";
    summary["maxUsage"] = formatNumber(max_usage, 1) + "%";
    summary["avgClock"] = formatNumber(avg_clock, 0) + " MHz";
    summary["load1"] = valueForKey(status, "Load Average (1min)");
    summary["load5"] = valueForKey(status, "Load Average (5min)");
    summary["load15"] = valueForKey(status, "Load Average (15min)");
    summary["coreCount"] = static_cast<int>(rows.size());

    QMetaObject::invokeMethod(
        this,
        [this, summary, rows, avg_usage]()
        {
            cpu_summary_ = summary;
            appendHistory(cpu_history_, avg_usage / 100.0);
            cpu_core_rows_ = rows;
            emit cpuSummaryChanged();
            emit cpuHistoryChanged();
            emit cpuCoreRowsChanged();
        },
        Qt::QueuedConnection);
}

void RosUiBridge::updateMemory(const diagnostic_msgs::msg::DiagnosticStatus &status)
{
    const QString total_physical = valueForKey(status, "Total Memory (Physical)");
    const QString used_physical = valueForKey(status, "Used Memory (Physical)");
    const QString free_physical = valueForKey(status, "Free Memory (Physical)");
    const QString total_swap = valueForKey(status, "Total Memory (Swap)");
    const QString used_swap = valueForKey(status, "Used Memory (Swap)");
    const QString free_swap = valueForKey(status, "Free Memory (Swap)");
    const QString total_all = valueForKey(status, "Total Memory");
    const QString used_all = valueForKey(status, "Used Memory");
    const QString free_all = valueForKey(status, "Free Memory");

    const double total_physical_num = extractNumber(total_physical);
    const double used_physical_num = extractNumber(used_physical);
    const double usage_percent = total_physical_num > 0.0
                                     ? (used_physical_num / total_physical_num) * 100.0
                                     : 0.0;

    QVariantMap summary;
    summary["state"] = QString::fromStdString(status.message);
    summary["level"] = levelToText(status.level);
    summary["usedPhysical"] = used_physical;
    summary["totalPhysical"] = total_physical;
    summary["freePhysical"] = free_physical;
    summary["usedSwap"] = used_swap;
    summary["totalSwap"] = total_swap;
    summary["usagePercent"] = formatNumber(usage_percent, 1) + "%";
    summary["updateStatus"] = valueForKey(status, "Update Status");

    QVariantList rows;
    rows.append(QVariantMap{{"item", "Physical"}, {"total", total_physical}, {"used", used_physical}, {"free", free_physical}});
    rows.append(QVariantMap{{"item", "Swap"}, {"total", total_swap}, {"used", used_swap}, {"free", free_swap}});
    rows.append(QVariantMap{{"item", "Combined"}, {"total", total_all}, {"used", used_all}, {"free", free_all}});

    QMetaObject::invokeMethod(
        this,
        [this, summary, rows, usage_percent]()
        {
            memory_summary_ = summary;
            appendHistory(memory_history_, usage_percent / 100.0);
            memory_rows_ = rows;
            emit memorySummaryChanged();
            emit memoryHistoryChanged();
            emit memoryRowsChanged();
        },
        Qt::QueuedConnection);
}

void RosUiBridge::updateHdd(const diagnostic_msgs::msg::DiagnosticStatus &status)
{
    QVariantList rows;
    QVariantMap current_disk;
    bool has_disk = false;
    double worst_use = 0.0;

    for (const auto &item : status.values) {
        const QString key = QString::fromStdString(item.key);
        const QString value = QString::fromStdString(item.value);

        if (key.contains("Name") && key.startsWith("Disk")) {
            if (has_disk) {
                rows.append(current_disk);
            }
            current_disk.clear();
            current_disk["disk"] = value;
            has_disk = true;
        } else if (has_disk && key.contains("Size")) {
            current_disk["size"] = value;
        } else if (has_disk && key.contains("Available")) {
            current_disk["available"] = value;
        } else if (has_disk && key.contains("Use")) {
            current_disk["use"] = value;
            worst_use = std::max(worst_use, extractNumber(value));
        } else if (has_disk && key.contains("Status")) {
            current_disk["status"] = value;
        } else if (has_disk && key.contains("Mount Point")) {
            current_disk["mount"] = value;
        }
    }

    if (has_disk) {
        rows.append(current_disk);
    }

    QVariantMap summary;
    summary["state"] = QString::fromStdString(status.message);
    summary["level"] = levelToText(status.level);
    summary["diskCount"] = static_cast<int>(rows.size());
    summary["worstUse"] = formatNumber(worst_use, 0) + "%";

    QMetaObject::invokeMethod(
        this,
        [this, summary, rows]()
        {
            hdd_summary_ = summary;
            hdd_rows_ = rows;
            emit hddSummaryChanged();
            emit hddRowsChanged();
        },
        Qt::QueuedConnection);
}

void RosUiBridge::updateNet(const diagnostic_msgs::msg::DiagnosticStatus &status)
{
    QVariantList rows;
    QVariantMap current_if;
    bool has_interface = false;
    double total_in = 0.0;
    double total_out = 0.0;
    int total_errors = 0;
    QStringList interfaces;

    for (const auto &item : status.values) {
        const QString key = QString::fromStdString(item.key);
        const QString value = QString::fromStdString(item.value);

        if (key == "Interface Name") {
            if (has_interface) {
                rows.append(current_if);
            }
            current_if.clear();
            current_if["interface"] = value;
            interfaces << value;
            has_interface = true;
        } else if (has_interface && key == "State") {
            current_if["state"] = value;
        } else if (has_interface && key == "Input Traffic") {
            current_if["input"] = value;
            total_in += extractNumber(value);
        } else if (has_interface && key == "Output Traffic") {
            current_if["output"] = value;
            total_out += extractNumber(value);
        } else if (has_interface && key == "MTU") {
            current_if["mtu"] = value;
        } else if (has_interface && key == "Total received MB") {
            current_if["totalRx"] = value;
        } else if (has_interface && key == "Total transmitted MB") {
            current_if["totalTx"] = value;
        } else if (has_interface && key == "Collisions") {
            current_if["collisions"] = value;
        } else if (has_interface && key == "Rx Errors") {
            current_if["rxErrors"] = value;
            total_errors += static_cast<int>(extractNumber(value));
        } else if (has_interface && key == "Tx Errors") {
            current_if["txErrors"] = value;
            total_errors += static_cast<int>(extractNumber(value));
        }
    }

    if (has_interface) {
        rows.append(current_if);
    }

    QVariantMap summary;
    summary["state"] = QString::fromStdString(status.message);
    summary["level"] = levelToText(status.level);
    summary["input"] = formatNumber(total_in, 4) + " MB/s";
    summary["output"] = formatNumber(total_out, 4) + " MB/s";
    summary["interfaces"] = interfaces.join(", ");
    summary["interfaceCount"] = interfaces.size();
    summary["errors"] = total_errors;

    QMetaObject::invokeMethod(
        this,
        [this, summary, rows, total_in, total_out]()
        {
            net_summary_ = summary;
            appendHistory(net_in_history_, total_in);
            appendHistory(net_out_history_, total_out);
            net_interface_rows_ = rows;
            emit netSummaryChanged();
            emit netInHistoryChanged();
            emit netOutHistoryChanged();
            emit netInterfaceRowsChanged();
        },
        Qt::QueuedConnection);
}

void RosUiBridge::updateNtp(const diagnostic_msgs::msg::DiagnosticStatus &status)
{
    const QString offset = valueForKey(status, "Offset (us)");
    const QString tolerance = valueForKey(status, "Offset tolerance (us)");
    const QString error_tolerance = valueForKey(status, "Offset tolerance (us) for Error");

    QVariantMap summary;
    summary["state"] = QString::fromStdString(status.message);
    summary["level"] = levelToText(status.level);
    summary["offset"] = offset + " us";
    summary["tolerance"] = tolerance + " us";
    summary["errorTolerance"] = error_tolerance + " us";

    QVariantList rows;
    rows.append(QVariantMap{{"name", "Offset (us)"}, {"value", offset}});
    rows.append(QVariantMap{{"name", "Tolerance (us)"}, {"value", tolerance}});
    rows.append(QVariantMap{{"name", "Error Tolerance (us)"}, {"value", error_tolerance}});

    QMetaObject::invokeMethod(
        this,
        [this, summary, rows]()
        {
            ntp_summary_ = summary;
            ntp_rows_ = rows;
            emit ntpSummaryChanged();
            emit ntpRowsChanged();
        },
        Qt::QueuedConnection);
}
