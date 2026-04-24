#pragma once

#include <QObject>
#include <QString>
#include <QVariant>
#include <memory>
#include <thread>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>

class RosUiBridge : public QObject
{
    Q_OBJECT

    Q_PROPERTY(QString rosStatus READ rosStatus NOTIFY rosStatusChanged)

    Q_PROPERTY(QVariantMap cpuSummary READ cpuSummary NOTIFY cpuSummaryChanged)
    Q_PROPERTY(QVariantList cpuHistory READ cpuHistory NOTIFY cpuHistoryChanged)
    Q_PROPERTY(QVariantList cpuCoreRows READ cpuCoreRows NOTIFY cpuCoreRowsChanged)

    Q_PROPERTY(QVariantMap memorySummary READ memorySummary NOTIFY memorySummaryChanged)
    Q_PROPERTY(QVariantList memoryHistory READ memoryHistory NOTIFY memoryHistoryChanged)
    Q_PROPERTY(QVariantList memoryRows READ memoryRows NOTIFY memoryRowsChanged)

    Q_PROPERTY(QVariantMap hddSummary READ hddSummary NOTIFY hddSummaryChanged)
    Q_PROPERTY(QVariantList hddRows READ hddRows NOTIFY hddRowsChanged)

    Q_PROPERTY(QVariantMap netSummary READ netSummary NOTIFY netSummaryChanged)
    Q_PROPERTY(QVariantList netInHistory READ netInHistory NOTIFY netInHistoryChanged)
    Q_PROPERTY(QVariantList netOutHistory READ netOutHistory NOTIFY netOutHistoryChanged)
    Q_PROPERTY(QVariantList netInterfaceRows READ netInterfaceRows NOTIFY netInterfaceRowsChanged)

    Q_PROPERTY(QVariantMap ntpSummary READ ntpSummary NOTIFY ntpSummaryChanged)
    Q_PROPERTY(QVariantList ntpRows READ ntpRows NOTIFY ntpRowsChanged)

public:
    explicit RosUiBridge(QObject *parent = nullptr);
    ~RosUiBridge() override;

    QString rosStatus() const;

    QVariantMap cpuSummary() const;
    QVariantList cpuHistory() const;
    QVariantList cpuCoreRows() const;

    QVariantMap memorySummary() const;
    QVariantList memoryHistory() const;
    QVariantList memoryRows() const;

    QVariantMap hddSummary() const;
    QVariantList hddRows() const;

    QVariantMap netSummary() const;
    QVariantList netInHistory() const;
    QVariantList netOutHistory() const;
    QVariantList netInterfaceRows() const;

    QVariantMap ntpSummary() const;
    QVariantList ntpRows() const;

signals:
    void rosStatusChanged();

    void cpuSummaryChanged();
    void cpuHistoryChanged();
    void cpuCoreRowsChanged();

    void memorySummaryChanged();
    void memoryHistoryChanged();
    void memoryRowsChanged();

    void hddSummaryChanged();
    void hddRowsChanged();

    void netSummaryChanged();
    void netInHistoryChanged();
    void netOutHistoryChanged();
    void netInterfaceRowsChanged();

    void ntpSummaryChanged();
    void ntpRowsChanged();

private:
    void diagnosticsCallback(const diagnostic_msgs::msg::DiagnosticArray::SharedPtr msg);

    static QString valueForKey(const diagnostic_msgs::msg::DiagnosticStatus &status, const QString &key);
    static double extractNumber(const QString &text);
    static QString formatNumber(double value, int precision = 1);
    static QString levelToText(unsigned char level);
    static void appendHistory(QVariantList &history, double value, int maxPoints = 60);

    void updateCpu(const diagnostic_msgs::msg::DiagnosticStatus &status);
    void updateMemory(const diagnostic_msgs::msg::DiagnosticStatus &status);
    void updateHdd(const diagnostic_msgs::msg::DiagnosticStatus &status);
    void updateNet(const diagnostic_msgs::msg::DiagnosticStatus &status);
    void updateNtp(const diagnostic_msgs::msg::DiagnosticStatus &status);

    QString ros_status_;

    QVariantMap cpu_summary_;
    QVariantList cpu_history_;
    QVariantList cpu_core_rows_;

    QVariantMap memory_summary_;
    QVariantList memory_history_;
    QVariantList memory_rows_;

    QVariantMap hdd_summary_;
    QVariantList hdd_rows_;

    QVariantMap net_summary_;
    QVariantList net_in_history_;
    QVariantList net_out_history_;
    QVariantList net_interface_rows_;

    QVariantMap ntp_summary_;
    QVariantList ntp_rows_;

    rclcpp::Node::SharedPtr node_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread ros_thread_;
};
