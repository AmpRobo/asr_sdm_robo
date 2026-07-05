#pragma once

#include <QObject>
#include <QImage>
#include <QFile>
#include <QTimer>
#include <QMap>
#include <QString>
#include <QStringList>
#include <QVariant>
#include <QVariantMap>
#include <QJsonArray>
#include <QJsonObject>
#include <array>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <rosbag2_cpp/writer.hpp>

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

    Q_PROPERTY(QVariantList videoTopics READ videoTopics NOTIFY videoTopicsChanged)
    Q_PROPERTY(QVariantList videoSlots READ videoSlots NOTIFY videoSlotsChanged)
    Q_PROPERTY(QString videoTopic0 READ videoTopic0 NOTIFY videoSlot0Changed)
    Q_PROPERTY(QString videoTopic1 READ videoTopic1 NOTIFY videoSlot1Changed)
    Q_PROPERTY(QString videoTopic2 READ videoTopic2 NOTIFY videoSlot2Changed)
    Q_PROPERTY(QString videoTopic3 READ videoTopic3 NOTIFY videoSlot3Changed)
    Q_PROPERTY(QString videoStatus0 READ videoStatus0 NOTIFY videoSlot0Changed)
    Q_PROPERTY(QString videoStatus1 READ videoStatus1 NOTIFY videoSlot1Changed)
    Q_PROPERTY(QString videoStatus2 READ videoStatus2 NOTIFY videoSlot2Changed)
    Q_PROPERTY(QString videoStatus3 READ videoStatus3 NOTIFY videoSlot3Changed)
    Q_PROPERTY(int videoFrame0Revision READ videoFrame0Revision NOTIFY videoSlot0Changed)
    Q_PROPERTY(int videoFrame1Revision READ videoFrame1Revision NOTIFY videoSlot1Changed)
    Q_PROPERTY(int videoFrame2Revision READ videoFrame2Revision NOTIFY videoSlot2Changed)
    Q_PROPERTY(int videoFrame3Revision READ videoFrame3Revision NOTIFY videoSlot3Changed)

    Q_PROPERTY(QVariantList plotTopics READ plotTopics NOTIFY plotTopicsChanged)
    Q_PROPERTY(QString plotTopicsStatus READ plotTopicsStatus NOTIFY plotTopicsChanged)
    Q_PROPERTY(QVariantList plotFieldOptions READ plotFieldOptions NOTIFY plotFieldOptionsChanged)
    Q_PROPERTY(QVariantList imuPlotSamples READ imuPlotSamples NOTIFY imuPlotSamplesChanged)
    Q_PROPERTY(QString plotStatus READ plotStatus NOTIFY plotStatusChanged)

    Q_PROPERTY(QString plotDataSource READ plotDataSource WRITE setPlotDataSource NOTIFY plotDataSourceChanged)
    Q_PROPERTY(QVariantList recordedPlotFieldOptions READ recordedPlotFieldOptions NOTIFY recordedPlotFieldOptionsChanged)
    Q_PROPERTY(QVariantList recordedPlotSamples READ recordedPlotSamples NOTIFY recordedPlotSamplesChanged)
    Q_PROPERTY(bool plotRecording READ plotRecording NOTIFY plotRecordingChanged)
    Q_PROPERTY(QString plotRecordingPath READ plotRecordingPath NOTIFY plotRecordingChanged)
    Q_PROPERTY(QString recordedFilePath READ recordedFilePath NOTIFY recordedPlaybackChanged)
    Q_PROPERTY(QString recordedStatus READ recordedStatus NOTIFY recordedPlaybackChanged)
    Q_PROPERTY(double playbackStartTimeMs READ playbackStartTimeMs NOTIFY recordedPlaybackChanged)
    Q_PROPERTY(double playbackEndTimeMs READ playbackEndTimeMs NOTIFY recordedPlaybackChanged)
    Q_PROPERTY(double playbackCurrentTimeMs READ playbackCurrentTimeMs NOTIFY playbackCurrentTimeMsChanged)
    Q_PROPERTY(double playbackSpeed READ playbackSpeed NOTIFY playbackSpeedChanged)
    Q_PROPERTY(bool playbackPlaying READ playbackPlaying NOTIFY playbackPlayingChanged)

public:
    explicit RosUiBridge(QObject *parent = nullptr);
    ~RosUiBridge() override;

    void addNode(const rclcpp::Node::SharedPtr & node);
    void startRosExecutor();

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

    QVariantList videoTopics() const;
    QVariantList videoSlots() const;
    QString videoTopic0() const;
    QString videoTopic1() const;
    QString videoTopic2() const;
    QString videoTopic3() const;
    QString videoStatus0() const;
    QString videoStatus1() const;
    QString videoStatus2() const;
    QString videoStatus3() const;
    int videoFrame0Revision() const;
    int videoFrame1Revision() const;
    int videoFrame2Revision() const;
    int videoFrame3Revision() const;
    QImage videoFrameImage(int slotIndex) const;

    QVariantList plotTopics() const;
    QString plotTopicsStatus() const;
    QVariantList plotFieldOptions() const;
    QVariantList imuPlotSamples() const;
    QString plotStatus() const;

    QString plotDataSource() const;
    QVariantList recordedPlotFieldOptions() const;
    QVariantList recordedPlotSamples() const;
    bool plotRecording() const;
    QString plotRecordingPath() const;
    QString recordedFilePath() const;
    QString recordedStatus() const;
    double playbackStartTimeMs() const;
    double playbackEndTimeMs() const;
    double playbackCurrentTimeMs() const;
    double playbackSpeed() const;
    bool playbackPlaying() const;

    Q_INVOKABLE void setVideoTopic(int slotIndex, const QString &topicName);
    Q_INVOKABLE void refreshPlotTopics();
    Q_INVOKABLE void setPlotTopicSelected(const QString &topicName, bool selected);
    Q_INVOKABLE void setPlotDataSource(const QString &dataSource);
    Q_INVOKABLE QString defaultPlotRecordingPath() const;
    Q_INVOKABLE bool startPlotRecording(const QString &filePath);
    Q_INVOKABLE void stopPlotRecording();
    Q_INVOKABLE bool openRecordedPlotFile(const QString &filePath);
    Q_INVOKABLE void setPlaybackStartTimeMs(double startTimeMs);
    Q_INVOKABLE void setPlaybackEndTimeMs(double endTimeMs);
    Q_INVOKABLE void setPlaybackCurrentTimeMs(double currentTimeMs);
    Q_INVOKABLE void setPlaybackSpeed(double speed);
    Q_INVOKABLE void setPlaybackPlaying(bool playing);
    Q_INVOKABLE QString pickExistingDirectory(const QString &title, const QString &startDir = QString());
    Q_INVOKABLE QString pickColor(const QString &title, const QString &initialColor = QString());

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

    void videoTopicsChanged();
    void videoSlotsChanged();
    void videoSlot0Changed();
    void videoSlot1Changed();
    void videoSlot2Changed();
    void videoSlot3Changed();

    void plotTopicsChanged();
    void imuPlotSamplesChanged();
    void plotFieldOptionsChanged();
    void plotStatusChanged();

    void plotDataSourceChanged();
    void recordedPlotFieldOptionsChanged();
    void recordedPlotSamplesChanged();
    void plotRecordingChanged();
    void recordedPlaybackChanged();
    void playbackCurrentTimeMsChanged();
    void playbackSpeedChanged();
    void playbackPlayingChanged();

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

    void handleVideoTopicsMessage(const std_msgs::msg::String::SharedPtr msg);
    void handleVideoSlotStatusMessage(int slotIndex, const std_msgs::msg::String::SharedPtr msg);
    void publishVideoSelection(int slotIndex, const QString &topicName);
    void discoverVideoTopics();
    void applyDiscoveredVideoTopics(const QStringList &topicNames, const QMap<QString, QString> &topicTypes);
    void stopVideoSlot(int slotIndex);
    void startVideoSlot(int slotIndex);
    void imageCallback(int slotIndex, const sensor_msgs::msg::Image::SharedPtr msg);
    void compressedImageCallback(int slotIndex, const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    void updateVideoFrame(int slotIndex, const QImage &image, const QString &status);
    void updateVideoStatus(int slotIndex, const QString &status);
    void emitVideoSlotChanged(int slotIndex);

    void discoverPlotTopics();
    void applyDiscoveredPlotTopics(const QStringList &topicNames, const QMap<QString, QString> &topicTypes);
    void rebuildPlotTopicsModel();
    void rebuildPlotFieldOptions();
    QVariantList filterPlotFieldOptionsForSelectedTopics(const QVariantList &fields) const;
    void refreshPlotSubscriptions();
    void stopPlotSubscriptions();
    void startPlotSubscriptionForTopic(const QString &topicName, const QString &topicType);
    void appendLivePlotSample(const QVariantMap &sample, const QString &topicName);
    void flushLivePlotSamples();
    template<typename MessageT>
    void startTypedPlotSubscription(const QString &topicName);
    template<typename MessageT>
    void writePlotRecordingSample(const QString &topicName, const MessageT &msg, double absoluteTimeMs);
    static QVariantList plotFieldOptionsForTopic(const QString &topicName, const QString &topicType);
    static QString plotFieldPath(const QString &topicName, const QString &fieldName);
    static bool isSupportedPlotType(const QString &topicType);
    static QString normalizeLocalPath(const QString &filePath);
    bool loadRecordedRosbag(const QString &filePath);
    void updateRecordedPlaybackBounds();
    void playbackTick();

    static bool isPerceptionImageTopic(const QString &topicName, const QString &topicType);
    static QImage imageMessageToQImage(const sensor_msgs::msg::Image &msg, QString *errorMessage);

    struct VideoSlot {
        QString topic;
        QString topic_type;
        QString status = "No topic selected";
        int frame_revision = 0;
        QImage frame;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
        rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_sub;
    };

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

    QVariantList video_topics_;
    QStringList video_topic_names_;
    QMap<QString, QString> video_topic_types_;
    QString video_topic_namespace_ = QStringLiteral("/system_monitor/video");
    std::array<VideoSlot, 4> video_slots_;
    mutable std::mutex video_frame_mutex_;

    QVariantList plot_topics_;
    QStringList plot_topic_names_;
    QMap<QString, QString> plot_topic_types_;
    QStringList selected_plot_topic_names_;
    QString plot_topics_status_ = QStringLiteral("Waiting for ROS topics ...");
    QMap<QString, rclcpp::SubscriptionBase::SharedPtr> plot_subscriptions_;

    QVariantList plot_field_options_;
    QVariantList recorded_available_plot_field_options_;
    QVariantList imu_plot_samples_;
    QVariantList pending_live_plot_samples_;
    QString pending_live_plot_status_topic_;
    std::mutex live_plot_mutex_;
    QString plot_status_ = QStringLiteral("Waiting for selected plot topics ...");
    double plot_start_time_ = -1.0;

    QString plot_data_source_ = QStringLiteral("live");
    QVariantList recorded_plot_field_options_;
    QVariantList recorded_plot_samples_;
    QString recorded_file_path_;
    QString recorded_status_ = QStringLiteral("No recorded file loaded");
    bool plot_recording_ = false;
    QString plot_recording_path_;
    std::unique_ptr<rosbag2_cpp::Writer> plot_bag_writer_;
    std::mutex plot_recording_mutex_;
    size_t plot_recorded_message_count_ = 0;
    double recorded_bag_start_time_ms_ = 0.0;
    double recorded_bag_end_time_ms_ = 0.0;
    double playback_start_time_ms_ = 0.0;
    double playback_end_time_ms_ = 0.0;
    double playback_current_time_ms_ = 0.0;
    double playback_speed_ = 1.0;
    bool playback_playing_ = false;
    QTimer *playback_timer_ = nullptr;
    QTimer *plot_update_timer_ = nullptr;
    std::chrono::steady_clock::time_point playback_last_tick_;

    rclcpp::Node::SharedPtr node_;
    std::vector<rclcpp::Node::SharedPtr> owned_nodes_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr video_topics_sub_;
    rclcpp::TimerBase::SharedPtr topic_discovery_timer_;
    std::array<rclcpp::Subscription<std_msgs::msg::String>::SharedPtr, 4> video_status_subs_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr video_select_pub_;
    std::unique_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    std::thread ros_thread_;
    bool ros_executor_started_ = false;
};
