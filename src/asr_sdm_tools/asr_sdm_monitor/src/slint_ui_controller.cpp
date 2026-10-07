#include "asr_sdm_monitor/slint_ui_controller.hpp"

#include "asr_sdm_monitor/ros_ui_bridge.hpp"

#include <QDateTime>
#include <QGuiApplication>
#include <QRegularExpression>
#include <QScreen>
#include <QVariant>

#include <algorithm>
#include <cmath>
#include <utility>

namespace
{
slint::SharedString toSs(const QString &text)
{
    const QByteArray utf8 = text.toUtf8();
    return slint::SharedString(std::string_view(utf8.constData(), static_cast<size_t>(utf8.size())));
}

QString fromSs(const slint::SharedString &text)
{
    return QString::fromUtf8(text.data(), static_cast<int>(text.size()));
}

QString displayValue(const QVariant &value)
{
    const QString text = value.toString();
    return text.isEmpty() ? QStringLiteral("--") : text;
}

struct ChartPalette
{
    QColor surface;
    QColor grid;
    QColor text;
    QColor text_secondary;
    QColor accent;
    QColor accent_fill;
    QColor secondary;
    QColor secondary_fill;
};

ChartPalette chartPalette(bool dark)
{
    if (!dark) {
        return {
            QColor(QStringLiteral("#ffffff")),
            QColor(QStringLiteral("#e8eef6")),
            QColor(QStringLiteral("#1e2b3a")),
            QColor(QStringLiteral("#607285")),
            QColor(QStringLiteral("#2a6df4")),
            QColor(42, 109, 244, 36),
            QColor(QStringLiteral("#009688")),
            QColor(0, 150, 136, 26),
        };
    }
    return {
        QColor(QStringLiteral("#16263d")),
        QColor(QStringLiteral("#2c4060")),
        QColor(QStringLiteral("#e7eef8")),
        QColor(QStringLiteral("#8fa3bf")),
        QColor(QStringLiteral("#5b9dff")),
        QColor(91, 157, 255, 46),
        QColor(QStringLiteral("#3ddec4")),
        QColor(61, 222, 196, 36),
    };
}

QVector<double> asDoubles(const QVariantList &values)
{
    QVector<double> result;
    result.reserve(values.size());
    for (const QVariant &value : values) {
        result.append(value.toDouble());
    }
    return result;
}

double historyPeak(const QVariantList &first, const QVariantList &second)
{
    double peak = 0.0;
    for (const QVariant &value : first) {
        peak = std::max(peak, value.toDouble());
    }
    for (const QVariant &value : second) {
        peak = std::max(peak, value.toDouble());
    }
    return peak;
}

double netMaxY(const QVariantList &first, const QVariantList &second)
{
    const double peak = historyPeak(first, second);
    if (peak <= 1.0) {
        return 1.0;
    }
    if (peak <= 10.0) {
        return 10.0;
    }
    if (peak <= 100.0) {
        return 100.0;
    }
    return std::ceil(peak / 100.0) * 100.0;
}

QString formatClock(double milliseconds)
{
    if (!(milliseconds > 0.0)) {
        return QStringLiteral("--");
    }
    return QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(milliseconds))
        .toString(QStringLiteral("HH:mm:ss.zzz"));
}

double parsePlaybackTime(const QString &text, double fallback_ms, double start_ms)
{
    const QString raw = text.trimmed();
    if (raw.isEmpty()) {
        return fallback_ms;
    }
    bool ok = false;
    const double numeric = raw.toDouble(&ok);
    if (ok) {
        if (numeric > 100000000000.0) {
            return numeric;
        }
        return start_ms + numeric * 1000.0;
    }

    const QRegularExpression expression(
        QStringLiteral("^(\\d{1,2}):(\\d{1,2}):(\\d{1,2})(?:\\.(\\d{1,3}))?$"));
    const QRegularExpressionMatch match = expression.match(raw);
    if (!match.hasMatch()) {
        return fallback_ms;
    }
    QDateTime base = QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(fallback_ms));
    const QString milliseconds = (match.captured(4) + QStringLiteral("000")).left(3);
    base.setTime(QTime(match.captured(1).toInt(), match.captured(2).toInt(),
                       match.captured(3).toInt(), milliseconds.toInt()));
    return static_cast<double>(base.toMSecsSinceEpoch());
}

slint::Image toImage(const QImage &source)
{
    if (source.isNull() || source.width() <= 0 || source.height() <= 0) {
        return {};
    }
    const QImage rgba = source.convertToFormat(QImage::Format_RGBA8888);
    slint::SharedPixelBuffer<slint::Rgba8Pixel> buffer(
        static_cast<uint32_t>(rgba.width()), static_cast<uint32_t>(rgba.height()));
    auto *destination = buffer.begin();
    for (int y = 0; y < rgba.height(); ++y) {
        const auto *line = reinterpret_cast<const slint::Rgba8Pixel *>(rgba.constScanLine(y));
        std::copy(line, line + rgba.width(), destination + static_cast<size_t>(y) * rgba.width());
    }
    return slint::Image(buffer);
}

asr_sdm_monitor_ui::TextRow textRow(const QStringList &cells)
{
    asr_sdm_monitor_ui::TextRow row;
    const auto cell = [&cells](int index) {
        return index < cells.size() ? toSs(cells.at(index)) : slint::SharedString();
    };
    row.c0 = cell(0);
    row.c1 = cell(1);
    row.c2 = cell(2);
    row.c3 = cell(3);
    row.c4 = cell(4);
    row.c5 = cell(5);
    row.c6 = cell(6);
    row.c7 = cell(7);
    row.c8 = cell(8);
    return row;
}

QString rowFingerprint(const QVariantList &rows, const QStringList &keys)
{
    QString fingerprint;
    for (const QVariant &item : rows) {
        const QVariantMap map = item.toMap();
        for (const QString &key : keys) {
            fingerprint += map.value(key).toString();
            fingerprint += QLatin1Char('\t');
        }
        fingerprint += QLatin1Char('\n');
    }
    return fingerprint;
}

const QColor kDarkSeries[] = {
    QColor("#3794ff"), QColor("#4ec9b0"), QColor("#f2cc60"), QColor("#c586c0"),
    QColor("#ce9178"), QColor("#b5cea8"), QColor("#9cdcfe"), QColor("#d7ba7d"),
    QColor("#ff8c00"), QColor("#f44747"), QColor("#9cdcfe"), QColor("#569cd6"),
    QColor("#d16969"), QColor("#00b7c3"), QColor("#b267e6"), QColor("#86c232"),
};
const QColor kLightSeries[] = {
    QColor("#2a6df4"), QColor("#009688"), QColor("#e69500"), QColor("#8e44ad"),
    QColor("#d14b4b"), QColor("#2e7d32"), QColor("#0078a8"), QColor("#6d4c41"),
    QColor("#ef6c00"), QColor("#c2185b"), QColor("#455a64"), QColor("#5d7b00"),
    QColor("#3949ab"), QColor("#00897b"), QColor("#ad1457"), QColor("#827717"),
};
}

SlintUiController::SlintUiController(
    RosUiBridge &bridge, slint::ComponentHandle<asr_sdm_monitor_ui::MainWindow> window)
    : bridge_(bridge),
      window_(std::move(window))
{
    ensureSeries(live_settings_);
    ensureSeries(recorded_settings_);
    recording_path_ = bridge_.defaultPlotRecordingPath();
    bind();
    state().set_recording_path(toSs(recording_path_));
    applySettings(live_settings_);
    sync();
}

const asr_sdm_monitor_ui::UiState &SlintUiController::state() const
{
    return window_->global<asr_sdm_monitor_ui::UiState>();
}

bool SlintUiController::zh() const
{
    return window_->global<asr_sdm_monitor_ui::AppText>().get_zh();
}

bool SlintUiController::dark() const
{
    return window_->global<asr_sdm_monitor_ui::AppTheme>().get_dark();
}

QString SlintUiController::trKey(const char *key) const
{
    const bool chinese = zh();
    const QString name = QString::fromLatin1(key);
    if (name == QLatin1String("none")) return chinese ? QStringLiteral("无") : QStringLiteral("None");
    if (name == QLatin1String("plottable")) return chinese ? QStringLiteral("可绘图") : QStringLiteral("Plottable");
    if (name == QLatin1String("recordableOnly")) return chinese ? QStringLiteral("可录制") : QStringLiteral("Recordable");
    if (name == QLatin1String("unsupportedPlotTopic")) return chinese ? QStringLiteral("不可绘图") : QStringLiteral("Not plottable");
    if (name == QLatin1String("ros2LiveSource")) return chinese ? QStringLiteral("ROS 2 实时") : QStringLiteral("ROS 2 Live");
    if (name == QLatin1String("ros2BagPlaySource")) return chinese ? QStringLiteral("ROS 2 数据包播放") : QStringLiteral("ROS 2 Bag Play");
    if (name == QLatin1String("topicSummaryTopics")) return chinese ? QStringLiteral("话题") : QStringLiteral("Topics");
    if (name == QLatin1String("topicSummarySelected")) return chinese ? QStringLiteral("已选择") : QStringLiteral("Selected");
    if (name == QLatin1String("topicSummaryFields")) return chinese ? QStringLiteral("可绘图字段") : QStringLiteral("Plottable fields");
    if (name == QLatin1String("noTopicSelected")) {
        return chinese ? QStringLiteral("请选择 /perception* 或 /sensing* 话题以开始显示画面")
                       : QStringLiteral("Select a /perception* or /sensing* topic to start streaming");
    }
    if (name == QLatin1String("waitingVideoFrame")) {
        return chinese ? QStringLiteral("等待视频帧 ...") : QStringLiteral("Waiting for video frame ...");
    }
    if (name == QLatin1String("currentScalePrefix")) return chinese ? QStringLiteral("当前量程: 0 - ") : QStringLiteral("Current Scale: 0 - ");
    if (name == QLatin1String("currentScaleSuffix")) return QStringLiteral(" MB/s");
    if (name == QLatin1String("relativeTime")) return chinese ? QStringLiteral("相对时间") : QStringLiteral("Relative Time");
    if (name == QLatin1String("absoluteTime")) return chinese ? QStringLiteral("绝对时间") : QStringLiteral("Absolute Time");
    if (name == QLatin1String("color")) return chinese ? QStringLiteral("颜色") : QStringLiteral("Color");
    if (name == QLatin1String("recordingFile")) return chinese ? QStringLiteral("录制文件") : QStringLiteral("Recording Bag");
    if (name == QLatin1String("recordedFile")) return chinese ? QStringLiteral("回放文件") : QStringLiteral("Recorded Bag");
    if (name == QLatin1String("notRecording")) return chinese ? QStringLiteral("未记录") : QStringLiteral("Not recording");
    if (name == QLatin1String("simulatorStateIdle")) return chinese ? QStringLiteral("未运行") : QStringLiteral("Not running");
    if (name == QLatin1String("simulatorStateRunning")) return chinese ? QStringLiteral("正在运行，RViz 在单独窗口中。") : QStringLiteral("Running. RViz is open in a separate window.");
    if (name == QLatin1String("simulatorStateStopping")) return chinese ? QStringLiteral("正在停止...") : QStringLiteral("Stopping...");
    if (name == QLatin1String("simulatorStateFailed")) return chinese ? QStringLiteral("仿真异常退出。") : QStringLiteral("The simulator stopped with an error.");
    if (name == QLatin1String("simulatorStartFailed")) {
        return chinese ? QStringLiteral("无法启动 ros2。请先 source ROS 2 环境，再打开监控界面。")
                       : QStringLiteral("Could not start ros2. Source the ROS 2 environment before opening the monitor.");
    }
    return name;
}

SlintUiController::PlotSettings &SlintUiController::activeSettings()
{
    return plot_section_ == 2 ? recorded_settings_ : live_settings_;
}

QColor SlintUiController::seriesColor(int index) const
{
    const QColor *palette = dark() ? kDarkSeries : kLightSeries;
    return palette[index % 16];
}

void SlintUiController::ensureSeries(PlotSettings &settings)
{
    settings.series_count = std::clamp(settings.series_count, 1, 16);
    while (static_cast<int>(settings.series.size()) < settings.series_count) {
        SeriesSetting series;
        series.color = seriesColor(static_cast<int>(settings.series.size()));
        settings.series.push_back(series);
    }
}

void SlintUiController::bumpEditors()
{
    ++editor_generation_;
    state().set_editor_generation(editor_generation_);
}

void SlintUiController::readSettingsFromUi(PlotSettings &settings)
{
    const auto &ui = state();
    settings.x_mode = ui.get_x_mode_index();
    settings.timestamp = ui.get_timestamp_index();
    settings.axis_scale = ui.get_axis_scale_index();
    settings.show_x_ticks = ui.get_show_x_ticks();
    settings.show_y_ticks = ui.get_show_y_ticks();
    settings.series_count = std::clamp(ui.get_series_count(), 1, 16);
    if (!x_fields_.empty()) {
        const int index = std::clamp(ui.get_x_field_index(), 0, static_cast<int>(x_fields_.size()) - 1);
        settings.x_field = x_fields_[static_cast<size_t>(index)].path;
    }
    ensureSeries(settings);
}

void SlintUiController::applySettings(const PlotSettings &settings)
{
    applying_ = true;
    const auto &ui = state();
    ui.set_x_mode_index(settings.x_mode);
    ui.set_timestamp_index(settings.timestamp);
    ui.set_axis_scale_index(settings.axis_scale);
    ui.set_show_x_ticks(settings.show_x_ticks);
    ui.set_show_y_ticks(settings.show_y_ticks);
    ui.set_series_count(settings.series_count);
    int x_index = 0;
    for (int index = 0; index < static_cast<int>(x_fields_.size()); ++index) {
        if (x_fields_[static_cast<size_t>(index)].path == settings.x_field) {
            x_index = index;
        }
    }
    ui.set_x_field_index(x_index);
    ui.set_time_window_text(toSs(QString::number(settings.time_window, 'f', 2)));
    applying_ = false;
    pushSeries();
    bumpEditors();
}

int SlintUiController::yIndexForPath(const QString &path) const
{
    if (path.isEmpty()) {
        return 0;
    }
    for (int index = 1; index < static_cast<int>(y_fields_.size()); ++index) {
        if (y_fields_[static_cast<size_t>(index)].path == path) {
            return index;
        }
    }
    return 0;
}

QString SlintUiController::yPathForIndex(int index) const
{
    if (index <= 0 || index >= static_cast<int>(y_fields_.size())) {
        return {};
    }
    return y_fields_[static_cast<size_t>(index)].path;
}

void SlintUiController::rebuildFields()
{
    const QVariantList fields = plot_section_ == 2 ? bridge_.recordedPlotFieldOptions() : bridge_.plotFieldOptions();
    QString fingerprint = zh() ? QStringLiteral("zh") : QStringLiteral("en");
    for (const QVariant &item : fields) {
        const QVariantMap map = item.toMap();
        fingerprint += map.value(QStringLiteral("path")).toString();
        fingerprint += map.value(QStringLiteral("unit")).toString();
        fingerprint += QLatin1Char('\n');
    }
    if (fingerprint == fields_fp_) {
        return;
    }
    fields_fp_ = fingerprint;

    y_fields_.clear();
    x_fields_.clear();
    y_fields_.push_back(FieldInfo{QString(), trKey("none"), QString()});
    std::vector<slint::SharedString> y_labels;
    std::vector<slint::SharedString> x_labels;
    y_labels.emplace_back(toSs(trKey("none")));
    for (const QVariant &item : fields) {
        const QVariantMap map = item.toMap();
        FieldInfo info;
        info.path = map.value(QStringLiteral("path")).toString();
        info.label = map.value(QStringLiteral("label")).toString();
        info.unit = map.value(QStringLiteral("unit")).toString();
        if (info.path.isEmpty()) {
            continue;
        }
        y_fields_.push_back(info);
        x_fields_.push_back(info);
        const QString caption = info.unit.isEmpty() ? info.label : info.label + QStringLiteral(" (") + info.unit + QLatin1Char(')');
        y_labels.emplace_back(toSs(caption));
        x_labels.emplace_back(toSs(caption));
    }
    if (x_labels.empty()) {
        x_labels.emplace_back(toSs(trKey("none")));
    }

    const auto &ui = state();
    ui.set_y_field_labels(std::make_shared<slint::VectorModel<slint::SharedString>>(std::move(y_labels)));
    ui.set_x_field_labels(std::make_shared<slint::VectorModel<slint::SharedString>>(std::move(x_labels)));

    auto &settings = activeSettings();
    bool x_found = false;
    for (const FieldInfo &field : x_fields_) {
        if (field.path == settings.x_field) {
            x_found = true;
        }
    }
    if (!x_found) {
        settings.x_field = x_fields_.empty() ? QString() : x_fields_.front().path;
    }
    for (SeriesSetting &series : settings.series) {
        if (yIndexForPath(series.field_path) == 0) {
            series.field_path.clear();
        }
    }
    series_fp_.clear();
    pushSeries();
}

void SlintUiController::pushSeries()
{
    auto &settings = activeSettings();
    ensureSeries(settings);
    QString fingerprint;
    std::vector<asr_sdm_monitor_ui::SeriesRow> rows;
    for (int index = 0; index < settings.series_count; ++index) {
        const SeriesSetting &series = settings.series[static_cast<size_t>(index)];
        asr_sdm_monitor_ui::SeriesRow row;
        row.field_index = yIndexForPath(series.field_path);
        row.swatch = slint::Color::from_rgb_uint8(series.color.red(), series.color.green(), series.color.blue());
        row.line_width = static_cast<float>(series.line_width);
        rows.push_back(row);
        fingerprint += QString::number(row.field_index);
        fingerprint += series.color.name();
        fingerprint += QLatin1Char('|');
    }
    if (fingerprint == series_fp_) {
        return;
    }
    series_fp_ = fingerprint;
    state().set_series_rows(std::make_shared<slint::VectorModel<asr_sdm_monitor_ui::SeriesRow>>(std::move(rows)));
}

QVariantList SlintUiController::filteredSamples(const PlotSettings &settings) const
{
    const QVariantList samples = plot_section_ == 2 ? bridge_.recordedPlotSamples() : bridge_.imuPlotSamples();
    if (settings.x_mode != 0 || samples.isEmpty()) {
        return samples;
    }
    const double current = plot_section_ == 2
                               ? bridge_.playbackCurrentTimeMs()
                               : samples.last().toMap().value(QStringLiteral("absoluteTimeMs")).toDouble();
    const double half = std::max(0.05, settings.time_window) * 500.0;
    QVariantList filtered;
    for (const QVariant &item : samples) {
        const double stamp = item.toMap().value(QStringLiteral("absoluteTimeMs")).toDouble();
        if (stamp >= current - half && stamp <= current + half) {
            filtered.append(item);
        }
    }
    return filtered;
}

void SlintUiController::redrawPlot()
{
    if (plot_section_ == 0 || plot_width_ < 8 || plot_height_ < 8) {
        return;
    }
    auto &settings = activeSettings();
    const QVariantList samples = filteredSamples(settings);
    bool has_series = false;
    for (int index = 0; index < settings.series_count && index < static_cast<int>(settings.series.size()); ++index) {
        if (!settings.series[static_cast<size_t>(index)].field_path.isEmpty()) {
            has_series = true;
        }
    }
    const auto &ui = state();
    ui.set_plot_has_data(has_series && !samples.isEmpty());

    PlotDrawRequest request;
    request.samples = samples;
    request.time_mode = settings.x_mode == 0;
    request.absolute_time = settings.timestamp == 1;
    request.x_field = settings.x_field;
    request.square = settings.axis_scale == 1;
    request.show_x_ticks = settings.show_x_ticks;
    request.show_y_ticks = settings.show_y_ticks;
    request.view = &settings.view;
    const ChartPalette palette = chartPalette(dark());
    request.surface = palette.surface;
    request.grid = palette.grid;
    request.text = palette.text;
    request.text_secondary = palette.text_secondary;
    request.accent = palette.accent;
    if (request.time_mode) {
        request.x_label = request.absolute_time ? trKey("absoluteTime") : trKey("relativeTime");
        if (plot_section_ == 2) {
            request.show_marker = true;
            const double current = bridge_.playbackCurrentTimeMs();
            request.marker_x = request.absolute_time ? current : (current - bridge_.playbackStartTimeMs()) / 1000.0;
        }
    } else {
        request.x_label = settings.x_field;
        for (const FieldInfo &field : x_fields_) {
            if (field.path == settings.x_field) {
                request.x_label = field.label;
            }
        }
    }
    for (int index = 0; index < settings.series_count && index < static_cast<int>(settings.series.size()); ++index) {
        const SeriesSetting &series = settings.series[static_cast<size_t>(index)];
        if (series.field_path.isEmpty()) {
            continue;
        }
        PlotSeriesStyle style;
        style.field = series.field_path;
        style.legend = series.field_path;
        style.color = series.color;
        style.line_width = series.line_width;
        for (const FieldInfo &field : y_fields_) {
            if (field.path == series.field_path) {
                style.legend = field.unit.isEmpty() ? field.label : field.label + QStringLiteral(" (") + field.unit + QLatin1Char(')');
            }
        }
        request.series.push_back(style);
    }
    ui.set_plot_chart(toImage(renderPlotChart(request, QSize(plot_width_, plot_height_))));
}

void SlintUiController::refreshHardware()
{
    const auto &ui = state();
    const auto assign_values = [&](const QStringList &values, QString &cache, auto setter) {
        const QString fingerprint = values.join(QLatin1Char('\n'));
        if (fingerprint == cache) {
            return;
        }
        cache = fingerprint;
        std::vector<slint::SharedString> model;
        model.reserve(static_cast<size_t>(values.size()));
        for (const QString &value : values) {
            model.emplace_back(toSs(value));
        }
        setter(std::make_shared<slint::VectorModel<slint::SharedString>>(std::move(model)));
    };
    const auto assign_rows = [&](const QVariantList &rows, const QStringList &keys, QString &cache, auto setter) {
        const QString fingerprint = rowFingerprint(rows, keys);
        if (fingerprint == cache) {
            return;
        }
        cache = fingerprint;
        std::vector<asr_sdm_monitor_ui::TextRow> model;
        model.reserve(static_cast<size_t>(rows.size()));
        for (const QVariant &item : rows) {
            const QVariantMap map = item.toMap();
            QStringList cells;
            for (const QString &key : keys) {
                cells.append(displayValue(map.value(key)));
            }
            model.push_back(textRow(cells));
        }
        setter(std::make_shared<slint::VectorModel<asr_sdm_monitor_ui::TextRow>>(std::move(model)));
    };

    const QVariantMap cpu = bridge_.cpuSummary();
    assign_values({
        displayValue(cpu.value(QStringLiteral("avgUsage"))),
        displayValue(cpu.value(QStringLiteral("maxUsage"))),
        displayValue(cpu.value(QStringLiteral("avgClock"))),
        displayValue(cpu.value(QStringLiteral("load1"))),
        displayValue(cpu.value(QStringLiteral("load5"))),
        displayValue(cpu.value(QStringLiteral("load15"))),
        displayValue(cpu.value(QStringLiteral("coreCount"))),
        displayValue(cpu.value(QStringLiteral("level"))) + QStringLiteral(" / ") + displayValue(cpu.value(QStringLiteral("state"))),
    }, cpu_values_fp_, [&](auto model) { ui.set_cpu_values(std::move(model)); });
    assign_rows(bridge_.cpuCoreRows(),
                {QStringLiteral("core"), QStringLiteral("usage"), QStringLiteral("clock"), QStringLiteral("user"),
                 QStringLiteral("system"), QStringLiteral("idle"), QStringLiteral("status")},
                cpu_rows_fp_, [&](auto model) { ui.set_cpu_rows(std::move(model)); });

    const QVariantMap memory = bridge_.memorySummary();
    assign_values({
        displayValue(memory.value(QStringLiteral("usedPhysical"))),
        displayValue(memory.value(QStringLiteral("totalPhysical"))),
        displayValue(memory.value(QStringLiteral("freePhysical"))),
        displayValue(memory.value(QStringLiteral("usagePercent"))),
        displayValue(memory.value(QStringLiteral("usedSwap"))),
        displayValue(memory.value(QStringLiteral("totalSwap"))),
        displayValue(memory.value(QStringLiteral("updateStatus"))),
        displayValue(memory.value(QStringLiteral("level"))) + QStringLiteral(" / ") + displayValue(memory.value(QStringLiteral("state"))),
    }, memory_values_fp_, [&](auto model) { ui.set_memory_values(std::move(model)); });
    assign_rows(bridge_.memoryRows(),
                {QStringLiteral("item"), QStringLiteral("total"), QStringLiteral("used"), QStringLiteral("free")},
                memory_rows_fp_, [&](auto model) { ui.set_memory_rows(std::move(model)); });

    const QVariantMap hdd = bridge_.hddSummary();
    assign_values({
        displayValue(hdd.value(QStringLiteral("diskCount"))),
        displayValue(hdd.value(QStringLiteral("worstUse"))),
        displayValue(hdd.value(QStringLiteral("level"))),
        displayValue(hdd.value(QStringLiteral("state"))),
    }, hdd_values_fp_, [&](auto model) { ui.set_hdd_values(std::move(model)); });
    assign_rows(bridge_.hddRows(),
                {QStringLiteral("disk"), QStringLiteral("mount"), QStringLiteral("size"),
                 QStringLiteral("available"), QStringLiteral("use"), QStringLiteral("status")},
                hdd_rows_fp_, [&](auto model) { ui.set_hdd_rows(std::move(model)); });

    const QVariantMap net = bridge_.netSummary();
    assign_values({
        displayValue(net.value(QStringLiteral("input"))),
        displayValue(net.value(QStringLiteral("output"))),
        displayValue(net.value(QStringLiteral("interfaceCount"))),
        displayValue(net.value(QStringLiteral("errors"))),
        displayValue(net.value(QStringLiteral("interfaces"))),
        displayValue(net.value(QStringLiteral("ipAddresses"))),
        displayValue(net.value(QStringLiteral("level"))),
        displayValue(net.value(QStringLiteral("state"))),
    }, net_values_fp_, [&](auto model) { ui.set_net_values(std::move(model)); });
    assign_rows(bridge_.netInterfaceRows(),
                {QStringLiteral("interface"), QStringLiteral("ip"), QStringLiteral("state"), QStringLiteral("input"),
                 QStringLiteral("output"), QStringLiteral("rxErrors"), QStringLiteral("txErrors"),
                 QStringLiteral("totalRx"), QStringLiteral("totalTx")},
                net_rows_fp_, [&](auto model) { ui.set_net_rows(std::move(model)); });

    const QVariantMap ntp = bridge_.ntpSummary();
    assign_values({
        displayValue(ntp.value(QStringLiteral("offset"))),
        displayValue(ntp.value(QStringLiteral("tolerance"))),
        displayValue(ntp.value(QStringLiteral("errorTolerance"))),
        displayValue(ntp.value(QStringLiteral("level"))) + QStringLiteral(" / ") + displayValue(ntp.value(QStringLiteral("state"))),
    }, ntp_values_fp_, [&](auto model) { ui.set_ntp_values(std::move(model)); });
    assign_rows(bridge_.ntpRows(), {QStringLiteral("name"), QStringLiteral("value")}, ntp_rows_fp_,
                [&](auto model) { ui.set_ntp_rows(std::move(model)); });

    const QString cpu_chart_key = QString::number(bridge_.cpuHistory().size()) + QLatin1Char(':')
                                  + (bridge_.cpuHistory().isEmpty() ? QString() : bridge_.cpuHistory().last().toString())
                                  + (dark() ? QStringLiteral(":d") : QStringLiteral(":l"));
    if (cpu_chart_key != cpu_chart_fp_) {
        cpu_chart_fp_ = cpu_chart_key;
        const ChartPalette palette = chartPalette(dark());
        ui.set_cpu_chart(toImage(renderAreaChart(asDoubles(bridge_.cpuHistory()), {}, 1.0,
                                                 palette.surface, palette.grid,
                                                 palette.accent, palette.accent_fill,
                                                 palette.accent, palette.accent_fill, QSize(960, 200))));
    }
    const QString memory_chart_key = QString::number(bridge_.memoryHistory().size()) + QLatin1Char(':')
                                     + (bridge_.memoryHistory().isEmpty() ? QString() : bridge_.memoryHistory().last().toString())
                                     + (dark() ? QStringLiteral(":d") : QStringLiteral(":l"));
    if (memory_chart_key != memory_chart_fp_) {
        memory_chart_fp_ = memory_chart_key;
        const ChartPalette palette = chartPalette(dark());
        ui.set_memory_chart(toImage(renderAreaChart(asDoubles(bridge_.memoryHistory()), {}, 1.0,
                                                    palette.surface, palette.grid,
                                                    palette.accent, palette.accent_fill,
                                                    palette.accent, palette.accent_fill, QSize(960, 200))));
    }
    const double max_y = netMaxY(bridge_.netInHistory(), bridge_.netOutHistory());
    const QString net_chart_key = QString::number(bridge_.netInHistory().size()) + QLatin1Char(':')
                                  + QString::number(bridge_.netOutHistory().size()) + QLatin1Char(':')
                                  + QString::number(max_y) + (dark() ? QStringLiteral(":d") : QStringLiteral(":l"))
                                  + (zh() ? QStringLiteral(":zh") : QStringLiteral(":en"));
    if (net_chart_key != net_chart_fp_) {
        net_chart_fp_ = net_chart_key;
        ui.set_net_scale(toSs(trKey("currentScalePrefix") + QString::number(max_y, 'f', 0) + trKey("currentScaleSuffix")));
        const ChartPalette palette = chartPalette(dark());
        ui.set_net_chart(toImage(renderAreaChart(
            asDoubles(bridge_.netInHistory()), asDoubles(bridge_.netOutHistory()), max_y,
            palette.surface, palette.grid,
            palette.accent, palette.accent_fill, palette.secondary, palette.secondary_fill, QSize(960, 200))));
    }
}

void SlintUiController::refreshVideo()
{
    QStringList labels;
    labels << trKey("none");
    video_names_.clear();
    for (const QVariant &topic : bridge_.videoTopics()) {
        const QString name = topic.toString();
        if (!name.isEmpty()) {
            video_names_ << name;
            labels << name;
        }
    }
    const QString fingerprint = labels.join(QLatin1Char('\n')) + (zh() ? QStringLiteral("#zh") : QStringLiteral("#en"));
    const auto &ui = state();
    if (fingerprint != video_topics_fp_) {
        video_topics_fp_ = fingerprint;
        std::vector<slint::SharedString> model;
        for (const QString &label : labels) {
            model.emplace_back(toSs(label));
        }
        ui.set_video_topics(std::make_shared<slint::VectorModel<slint::SharedString>>(std::move(model)));
    }

    const auto localize = [this](const QString &status) {
        if (status == QLatin1String("No topic selected")) {
            return trKey("noTopicSelected");
        }
        if (status == QLatin1String("Waiting for video frame")) {
            return trKey("waitingVideoFrame");
        }
        return status;
    };
    const QString topics[] = {bridge_.videoTopic0(), bridge_.videoTopic1(), bridge_.videoTopic2(), bridge_.videoTopic3()};
    const QString statuses[] = {localize(bridge_.videoStatus0()), localize(bridge_.videoStatus1()),
                                localize(bridge_.videoStatus2()), localize(bridge_.videoStatus3())};
    const int revisions[] = {bridge_.videoFrame0Revision(), bridge_.videoFrame1Revision(),
                             bridge_.videoFrame2Revision(), bridge_.videoFrame3Revision()};
    ui.set_video_topic_0(toSs(topics[0].isEmpty() ? trKey("none") : topics[0]));
    ui.set_video_topic_1(toSs(topics[1].isEmpty() ? trKey("none") : topics[1]));
    ui.set_video_topic_2(toSs(topics[2].isEmpty() ? trKey("none") : topics[2]));
    ui.set_video_topic_3(toSs(topics[3].isEmpty() ? trKey("none") : topics[3]));
    ui.set_video_status_0(toSs(statuses[0]));
    ui.set_video_status_1(toSs(statuses[1]));
    ui.set_video_status_2(toSs(statuses[2]));
    ui.set_video_status_3(toSs(statuses[3]));

    const int indexes[] = {ui.get_video_index_0(), ui.get_video_index_1(), ui.get_video_index_2(), ui.get_video_index_3()};
    const auto set_index = [&](int slot, int index) {
        applying_ = true;
        if (slot == 0) ui.set_video_index_0(index);
        else if (slot == 1) ui.set_video_index_1(index);
        else if (slot == 2) ui.set_video_index_2(index);
        else ui.set_video_index_3(index);
        applying_ = false;
    };
    for (int slot = 0; slot < 4; ++slot) {
        int index = 0;
        const int found = video_names_.indexOf(topics[slot]);
        if (!topics[slot].isEmpty() && found >= 0) {
            index = found + 1;
        }
        if (indexes[slot] != index) {
            set_index(slot, index);
        }
        if (revisions[slot] != video_revision_[static_cast<size_t>(slot)]) {
            video_revision_[static_cast<size_t>(slot)] = revisions[slot];
            const QImage frame = bridge_.videoFrameImage(slot);
            const bool has_frame = !frame.isNull();
            if (slot == 0) {
                ui.set_video_has_frame_0(has_frame);
                if (has_frame) ui.set_video_frame_0(toImage(frame));
            } else if (slot == 1) {
                ui.set_video_has_frame_1(has_frame);
                if (has_frame) ui.set_video_frame_1(toImage(frame));
            } else if (slot == 2) {
                ui.set_video_has_frame_2(has_frame);
                if (has_frame) ui.set_video_frame_2(toImage(frame));
            } else {
                ui.set_video_has_frame_3(has_frame);
                if (has_frame) ui.set_video_frame_3(toImage(frame));
            }
        }
    }
}

void SlintUiController::refreshTopics()
{
    QVariantList topics = bridge_.plotTopics();
    std::sort(topics.begin(), topics.end(), [this](const QVariant &left_value, const QVariant &right_value) {
        const QVariantMap left = left_value.toMap();
        const QVariantMap right = right_value.toMap();
        const QString left_name = left.value(QStringLiteral("name")).toString();
        const QString right_name = right.value(QStringLiteral("name")).toString();
        if (topic_sort_ == 1 && left_name != right_name) {
            return left_name > right_name;
        }
        if (topic_sort_ == 2) {
            const int left_rank = left.value(QStringLiteral("sourceKind")).toString() == QLatin1String("ros2_bag_play") ? 1 : 0;
            const int right_rank = right.value(QStringLiteral("sourceKind")).toString() == QLatin1String("ros2_bag_play") ? 1 : 0;
            if (left_rank != right_rank) {
                return left_rank < right_rank;
            }
        }
        if (topic_sort_ == 3) {
            const int left_rank = left.value(QStringLiteral("plottable")).toBool() ? 0 : 1;
            const int right_rank = right.value(QStringLiteral("plottable")).toBool() ? 0 : 1;
            if (left_rank != right_rank) {
                return left_rank < right_rank;
            }
        }
        if (left_name != right_name) {
            return left_name < right_name;
        }
        return left.value(QStringLiteral("key")).toString() < right.value(QStringLiteral("key")).toString();
    });

    QString fingerprint = zh() ? QStringLiteral("zh\n") : QStringLiteral("en\n");
    fingerprint += QString::number(topic_sort_);
    int selected = 0;
    int field_count = 0;
    topic_keys_.clear();
    std::vector<asr_sdm_monitor_ui::TopicRow> rows;
    for (const QVariant &item : topics) {
        const QVariantMap map = item.toMap();
        const bool is_selected = map.value(QStringLiteral("selected")).toBool();
        const bool plottable = map.value(QStringLiteral("plottable")).toBool();
        const int fields = map.value(QStringLiteral("fieldCount")).toInt();
        if (is_selected) {
            ++selected;
            field_count += fields;
        }
        QString source = map.value(QStringLiteral("sourceKind")).toString() == QLatin1String("ros2_bag_play")
                             ? trKey("ros2BagPlaySource") : trKey("ros2LiveSource");
        const QString source_name = map.value(QStringLiteral("sourceName")).toString();
        if (!source_name.isEmpty()) {
            source += QStringLiteral(" (") + source_name + QLatin1Char(')');
        }
        QString capability = trKey("unsupportedPlotTopic");
        if (plottable) {
            capability = trKey("plottable") + QStringLiteral(" · ") + QString::number(fields);
            if (map.value(QStringLiteral("recordable")).toBool()) {
                capability += QStringLiteral(" · ") + trKey("recordableOnly");
            }
        } else if (map.value(QStringLiteral("recordable")).toBool()) {
            capability = trKey("recordableOnly");
        }

        asr_sdm_monitor_ui::TopicRow row;
        row.key = toSs(map.value(QStringLiteral("key")).toString());
        row.name = toSs(map.value(QStringLiteral("name")).toString());
        row.type = toSs(map.value(QStringLiteral("type")).toString());
        row.source = toSs(source);
        row.capability = toSs(capability);
        row.selected = is_selected;
        rows.push_back(row);
        topic_keys_.push_back(map.value(QStringLiteral("key")).toString());
        fingerprint += map.value(QStringLiteral("key")).toString();
        fingerprint += is_selected ? QLatin1Char('1') : QLatin1Char('0');
        fingerprint += source;
        fingerprint += capability;
        fingerprint += QLatin1Char('\n');
    }
    const QString summary = trKey("topicSummaryTopics") + QStringLiteral(": ") + QString::number(topics.size())
                            + QStringLiteral("   ") + trKey("topicSummarySelected") + QStringLiteral(": ") + QString::number(selected)
                            + QStringLiteral("   ") + trKey("topicSummaryFields") + QStringLiteral(": ") + QString::number(field_count);
    if (fingerprint == topics_fp_) {
        return;
    }
    topics_fp_ = fingerprint;
    const auto &ui = state();
    ui.set_topic_summary(toSs(summary));
    ui.set_plot_topics(std::make_shared<slint::VectorModel<asr_sdm_monitor_ui::TopicRow>>(std::move(rows)));
}

void SlintUiController::refreshPlot()
{
    rebuildFields();
    pushSeries();
    const auto &settings = activeSettings();
    const QVariantList samples = plot_section_ == 2 ? bridge_.recordedPlotSamples() : bridge_.imuPlotSamples();
    const QString last = samples.isEmpty() ? QString() : samples.last().toMap().value(QStringLiteral("absoluteTimeMs")).toString();
    const QString fingerprint = QString::number(plot_section_) + QLatin1Char(':') + QString::number(samples.size())
                                + QLatin1Char(':') + last + QLatin1Char(':') + QString::number(bridge_.playbackCurrentTimeMs(), 'f', 0)
                                + QLatin1Char(':') + QString::number(plot_width_) + QLatin1Char('x') + QString::number(plot_height_)
                                + QLatin1Char(':') + QString::number(settings.x_mode) + QString::number(settings.timestamp)
                                + QString::number(settings.axis_scale) + settings.x_field + QString::number(settings.series_count)
                                + QString::number(settings.time_window, 'f', 2)
                                + (settings.view.manual ? QStringLiteral(":m") : QStringLiteral(":a"))
                                + QString::number(settings.view.x_span, 'f', 3)
                                + (dark() ? QStringLiteral(":d") : QStringLiteral(":l"));
    if (fingerprint != plot_fp_) {
        plot_fp_ = fingerprint;
        redrawPlot();
    }
}

void SlintUiController::refreshPlayback()
{
    const auto &ui = state();
    const QString opened = bridge_.recordedFilePath();
    if (!opened.isEmpty() && opened != recorded_path_) {
        recorded_path_ = opened;
        ui.set_recorded_path(toSs(recorded_path_));
        bumpEditors();
    }
    ui.set_recording(bridge_.plotRecording());
    ui.set_recording_status(toSs(bridge_.plotRecording() ? bridge_.plotRecordingPath() : trKey("notRecording")));
    ui.set_plot_status(toSs(bridge_.plotStatus()));
    ui.set_recorded_status(toSs(bridge_.recordedStatus()));
    ui.set_playback_playing(bridge_.playbackPlaying());
    ui.set_playback_start_text(toSs(formatClock(bridge_.playbackStartTimeMs())));
    ui.set_playback_end_text(toSs(formatClock(bridge_.playbackEndTimeMs())));
    ui.set_playback_current_text(toSs(formatClock(bridge_.playbackCurrentTimeMs())));
    const double start = bridge_.playbackStartTimeMs();
    const double end = bridge_.playbackEndTimeMs();
    const double current = bridge_.playbackCurrentTimeMs();
    const float position = end > start ? static_cast<float>((current - start) / (end - start)) : 0.0f;
    ui.set_playback_position(std::clamp(position, 0.0f, 1.0f));

    const double speed = bridge_.playbackSpeed();
    const double speeds[] = {0.25, 0.5, 1.0, 2.0, 4.0};
    int speed_index = 2;
    for (int index = 0; index < 5; ++index) {
        if (std::abs(speed - speeds[index]) < 0.01) {
            speed_index = index;
        }
    }
    if (ui.get_playback_speed_index() != speed_index) {
        applying_ = true;
        ui.set_playback_speed_index(speed_index);
        applying_ = false;
    }
}

void SlintUiController::refreshSimulator()
{
    const QString state_name = bridge_.planningSimulatorState();
    QString text = trKey("simulatorStateIdle");
    if (state_name == QLatin1String("running")) {
        text = trKey("simulatorStateRunning");
    } else if (state_name == QLatin1String("stopping")) {
        text = trKey("simulatorStateStopping");
    } else if (state_name == QLatin1String("failed")) {
        text = trKey("simulatorStateFailed");
    }
    QString detail = bridge_.planningSimulatorDetail();
    if (detail == QLatin1String("start_failed")) {
        detail = trKey("simulatorStartFailed");
    }
    const auto &ui = state();
    ui.set_simulator_state(toSs(text));
    ui.set_simulator_detail(toSs(detail));
    ui.set_simulator_busy(state_name == QLatin1String("running") || state_name == QLatin1String("stopping"));
    const QString log = bridge_.planningSimulatorLog();
    if (log != simulator_log_fp_) {
        simulator_log_fp_ = log;
        ui.set_simulator_log(toSs(log));
    }
}

void SlintUiController::sync()
{
    state().set_ros_status(toSs(bridge_.rosStatus()));
    refreshHardware();
    refreshVideo();
    refreshTopics();
    refreshPlot();
    refreshPlayback();
    refreshSimulator();
}

void SlintUiController::bind()
{
    const auto &actions = window_->global<asr_sdm_monitor_ui::UiActions>();
    actions.on_appearance_changed([this] {
        cpu_chart_fp_.clear();
        memory_chart_fp_.clear();
        net_chart_fp_.clear();
        video_topics_fp_.clear();
        topics_fp_.clear();
        fields_fp_.clear();
        series_fp_.clear();
        plot_fp_.clear();
        sync();
    });
    actions.on_video_count_changed([this](int count) {
        for (int slot = std::max(0, count); slot < 4; ++slot) {
            bridge_.setVideoTopic(slot, QString());
        }
    });
    actions.on_video_topic_picked([this](int slot, int index) {
        if (applying_ || slot < 0 || slot > 3) {
            return;
        }
        QString topic;
        if (index > 0 && index <= video_names_.size()) {
            topic = video_names_.at(index - 1);
        }
        bridge_.setVideoTopic(slot, topic);
    });
    actions.on_refresh_plot_topics([this] {
        bridge_.refreshPlotTopics();
        topics_fp_.clear();
        refreshTopics();
    });
    actions.on_topic_sort_changed([this](int index) {
        topic_sort_ = std::clamp(index, 0, 3);
        topics_fp_.clear();
        refreshTopics();
    });
    actions.on_plot_topic_toggled([this](int index, bool selected) {
        if (index < 0 || index >= topic_keys_.size()) {
            return;
        }
        bridge_.setPlotTopicSelected(topic_keys_.at(index), selected);
        topics_fp_.clear();
        fields_fp_.clear();
        plot_fp_.clear();
        refreshTopics();
        rebuildFields();
        redrawPlot();
    });
    actions.on_plot_section_changed([this](int section) {
        if (applying_ || section == plot_section_) {
            return;
        }
        readSettingsFromUi(activeSettings());
        plot_section_ = section;
        if (section == 2) {
            bridge_.setPlotDataSource(QStringLiteral("recorded"));
            fields_fp_.clear();
            rebuildFields();
            applySettings(recorded_settings_);
        } else if (section == 1) {
            bridge_.setPlotDataSource(QStringLiteral("live"));
            fields_fp_.clear();
            rebuildFields();
            applySettings(live_settings_);
        }
        plot_fp_.clear();
        redrawPlot();
    });
    actions.on_plot_settings_changed([this] {
        if (applying_) {
            return;
        }
        readSettingsFromUi(activeSettings());
        if (activeSettings().x_mode == 1 && activeSettings().x_field.isEmpty() && !x_fields_.empty()) {
            activeSettings().x_field = x_fields_.front().path;
        }
        plotResetView(activeSettings().view);
        plot_fp_.clear();
        redrawPlot();
    });
    actions.on_series_count_changed([this](int count) {
        if (applying_) {
            return;
        }
        activeSettings().series_count = std::clamp(count, 1, 16);
        ensureSeries(activeSettings());
        series_fp_.clear();
        pushSeries();
        plot_fp_.clear();
        redrawPlot();
    });
    actions.on_series_field_changed([this](int index, int field_index) {
        if (applying_) {
            return;
        }
        auto &settings = activeSettings();
        ensureSeries(settings);
        if (index < 0 || index >= static_cast<int>(settings.series.size())) {
            return;
        }
        const QString path = yPathForIndex(field_index);
        if (!path.isEmpty()) {
            for (int other = 0; other < settings.series_count && other < static_cast<int>(settings.series.size()); ++other) {
                if (other != index && settings.series[static_cast<size_t>(other)].field_path == path) {
                    settings.series[static_cast<size_t>(other)].field_path.clear();
                }
            }
        }
        settings.series[static_cast<size_t>(index)].field_path = path;
        series_fp_.clear();
        pushSeries();
        plot_fp_.clear();
        redrawPlot();
    });
    actions.on_series_color_requested([this](int index) {
        auto &settings = activeSettings();
        if (index < 0 || index >= static_cast<int>(settings.series.size())) {
            return;
        }
        const QString picked = bridge_.pickColor(trKey("color"), settings.series[static_cast<size_t>(index)].color.name(QColor::HexRgb));
        if (picked.isEmpty()) {
            return;
        }
        settings.series[static_cast<size_t>(index)].color = QColor(picked);
        series_fp_.clear();
        pushSeries();
        plot_fp_.clear();
        redrawPlot();
    });
    actions.on_series_width_changed([this](int index, float width) {
        auto &settings = activeSettings();
        if (index < 0 || index >= static_cast<int>(settings.series.size())) {
            return;
        }
        settings.series[static_cast<size_t>(index)].line_width = std::clamp(static_cast<double>(width), 0.1, 8.0);
        plot_fp_.clear();
        redrawPlot();
    });
    actions.on_time_window_accepted([this](const slint::SharedString &text) {
        bool ok = false;
        const double value = fromSs(text).toDouble(&ok);
        activeSettings().time_window = ok && value > 0.0 ? std::max(0.05, value) : 4.0;
        plot_fp_.clear();
        redrawPlot();
    });
    actions.on_browse_recording([this] {
        const QString picked = bridge_.pickExistingDirectory(trKey("recordingFile"), recording_path_);
        if (picked.isEmpty()) {
            return;
        }
        recording_path_ = picked;
        state().set_recording_path(toSs(recording_path_));
        bumpEditors();
    });
    actions.on_recording_path_accepted([this](const slint::SharedString &text) {
        const QString path = fromSs(text).trimmed();
        if (!path.isEmpty()) {
            recording_path_ = path;
        }
    });
    actions.on_start_recording([this] {
        if (recording_path_.isEmpty()) {
            recording_path_ = bridge_.defaultPlotRecordingPath();
            state().set_recording_path(toSs(recording_path_));
            bumpEditors();
        }
        bridge_.startPlotRecording(recording_path_);
    });
    actions.on_stop_recording([this] { bridge_.stopPlotRecording(); });
    actions.on_browse_recorded([this] {
        const QString picked = bridge_.pickExistingDirectory(trKey("recordedFile"), recorded_path_);
        if (picked.isEmpty()) {
            return;
        }
        recorded_path_ = picked;
        state().set_recorded_path(toSs(recorded_path_));
        bumpEditors();
    });
    actions.on_recorded_path_accepted([this](const slint::SharedString &text) {
        const QString path = fromSs(text).trimmed();
        if (!path.isEmpty()) {
            recorded_path_ = path;
        }
    });
    actions.on_open_recorded([this] {
        if (recorded_path_.isEmpty()) {
            return;
        }
        if (bridge_.openRecordedPlotFile(recorded_path_)) {
            bridge_.setPlotDataSource(QStringLiteral("recorded"));
            plot_section_ = 2;
            fields_fp_.clear();
            plot_fp_.clear();
            rebuildFields();
            redrawPlot();
        }
    });
    actions.on_toggle_playback([this] {
        bridge_.setPlaybackPlaying(!bridge_.playbackPlaying());
    });
    actions.on_playback_bound_accepted([this](const slint::SharedString &which, const slint::SharedString &text) {
        const QString field = fromSs(which);
        const double start = bridge_.playbackStartTimeMs();
        const double fallback = field == QLatin1String("end") ? bridge_.playbackEndTimeMs()
                                : field == QLatin1String("current") ? bridge_.playbackCurrentTimeMs()
                                                                    : start;
        const double parsed = parsePlaybackTime(fromSs(text), fallback, start);
        if (field == QLatin1String("end")) {
            bridge_.setPlaybackEndTimeMs(parsed);
        } else if (field == QLatin1String("current")) {
            bridge_.setPlaybackCurrentTimeMs(parsed);
        } else {
            bridge_.setPlaybackStartTimeMs(parsed);
        }
    });
    actions.on_playback_seek([this](float position) {
        const double start = bridge_.playbackStartTimeMs();
        const double end = bridge_.playbackEndTimeMs();
        if (end > start) {
            bridge_.setPlaybackCurrentTimeMs(start + (end - start) * std::clamp(static_cast<double>(position), 0.0, 1.0));
        }
    });
    actions.on_playback_speed_changed([this](int index) {
        if (applying_) {
            return;
        }
        const double speeds[] = {0.25, 0.5, 1.0, 2.0, 4.0};
        bridge_.setPlaybackSpeed(speeds[std::clamp(index, 0, 4)]);
    });
    actions.on_reset_plot_view([this] {
        plotResetView(activeSettings().view);
        plot_fp_.clear();
        redrawPlot();
    });
    actions.on_plot_resized([this](float width, float height) {
        plot_width_ = std::max(0, static_cast<int>(std::lround(width)));
        plot_height_ = std::max(0, static_cast<int>(std::lround(height)));
        plot_fp_.clear();
        redrawPlot();
    });
    actions.on_plot_pointer([this](float x, float y, int kind) {
        auto &settings = activeSettings();
        const bool time_mode = settings.x_mode == 0;
        if (kind == 0) {
            plotPanStart(settings.view, x, y, time_mode);
        } else if (kind == 1) {
            plotPanUpdate(settings.view, x, y, time_mode);
        } else {
            settings.view.panning = false;
        }
        if (kind != 2) {
            plot_fp_.clear();
            redrawPlot();
        }
    });
    actions.on_plot_scroll([this](float x, float y, float delta) {
        plotZoom(activeSettings().view, x, y, delta, activeSettings().x_mode == 0);
        plot_fp_.clear();
        redrawPlot();
    });
    actions.on_simulator_start([this](int kind, int control, int teleop, int planning) {
        const auto flag = [](int index) {
            return index == 0 ? QStringLiteral("enable") : QStringLiteral("disable");
        };
        bridge_.startPlanningSimulator(
            kind == 1 ? QStringLiteral("logging") : QStringLiteral("planning"),
            flag(control), flag(teleop), flag(planning));
    });
    actions.on_simulator_stop([this] { bridge_.stopPlanningSimulator(); });
    actions.on_toggle_window_maximized([this] { toggleWindowMaximized(); });
    actions.on_window_edge_drag([this](int edges, int kind, float x, float y) {
        dragWindowEdge(edges, kind, x, y);
    });
}

namespace
{
constexpr int kEdgeNorth = 1;
constexpr int kEdgeSouth = 2;
constexpr int kEdgeWest = 4;
constexpr int kEdgeEast = 8;
constexpr float kMinWindowWidth = 960.f;
constexpr float kMinWindowHeight = 640.f;

int cursorScreen(int origin, float local, float scale)
{
    return origin + static_cast<int>(std::lround(local * scale));
}
}

void SlintUiController::toggleWindowMaximized()
{
    auto &win = window_->window();
    if (!window_maximized_) {
        restore_position_ = win.position();
        restore_size_ = win.size();
        QScreen *screen = QGuiApplication::primaryScreen();
        const qreal ratio = screen != nullptr ? screen->devicePixelRatio() : 1.0;
        const QPoint center(
            static_cast<int>(std::lround((restore_position_.x + static_cast<double>(restore_size_.width) / 2.0) / ratio)),
            static_cast<int>(std::lround((restore_position_.y + static_cast<double>(restore_size_.height) / 2.0) / ratio)));
        if (QScreen *at = QGuiApplication::screenAt(center)) {
            screen = at;
        }
        if (screen == nullptr) {
            return;
        }
        const QRect area = screen->availableGeometry();
        const qreal dpr = screen->devicePixelRatio();
        const int x = static_cast<int>(std::lround(area.x() * dpr));
        const int y = static_cast<int>(std::lround(area.y() * dpr));
        const auto width = static_cast<uint32_t>(std::max(1, static_cast<int>(std::lround(area.width() * dpr))));
        const auto height = static_cast<uint32_t>(std::max(1, static_cast<int>(std::lround(area.height() * dpr))));
        win.set_position(slint::PhysicalPosition({x, y}));
        win.set_size(slint::PhysicalSize({width, height}));
        window_maximized_ = true;
    } else {
        win.set_position(restore_position_);
        win.set_size(restore_size_);
        window_maximized_ = false;
        edge_drag_.active = false;
    }
    state().set_window_maximized(window_maximized_);
}

void SlintUiController::dragWindowEdge(int edges, int kind, float x, float y)
{
    if (window_maximized_) {
        return;
    }
    auto &win = window_->window();
    const float scale = std::max(win.scale_factor(), 0.01f);
    const slint::PhysicalPosition position = win.position();
    const int screen_x = cursorScreen(position.x, x, scale);
    const int screen_y = cursorScreen(position.y, y, scale);
    if (kind == 0) {
        const slint::PhysicalSize size = win.size();
        edge_drag_.active = true;
        edge_drag_.edges = edges;
        edge_drag_.start_cursor_x = screen_x;
        edge_drag_.start_cursor_y = screen_y;
        edge_drag_.start_x = position.x;
        edge_drag_.start_y = position.y;
        edge_drag_.start_width = size.width;
        edge_drag_.start_height = size.height;
        return;
    }
    if (kind != 1 || !edge_drag_.active) {
        edge_drag_.active = false;
        return;
    }

    const int dx = screen_x - edge_drag_.start_cursor_x;
    const int dy = screen_y - edge_drag_.start_cursor_y;
    int left = edge_drag_.start_x;
    int top = edge_drag_.start_y;
    int width = static_cast<int>(edge_drag_.start_width);
    int height = static_cast<int>(edge_drag_.start_height);
    if ((edges & kEdgeWest) != 0) {
        left += dx;
        width -= dx;
    }
    if ((edges & kEdgeEast) != 0) {
        width += dx;
    }
    if ((edges & kEdgeNorth) != 0) {
        top += dy;
        height -= dy;
    }
    if ((edges & kEdgeSouth) != 0) {
        height += dy;
    }

    const int min_width = std::max(1, static_cast<int>(std::lround(kMinWindowWidth * scale)));
    const int min_height = std::max(1, static_cast<int>(std::lround(kMinWindowHeight * scale)));
    if (width < min_width) {
        if ((edges & kEdgeWest) != 0) {
            left -= min_width - width;
        }
        width = min_width;
    }
    if (height < min_height) {
        if ((edges & kEdgeNorth) != 0) {
            top -= min_height - height;
        }
        height = min_height;
    }
    win.set_position(slint::PhysicalPosition({left, top}));
    win.set_size(slint::PhysicalSize({static_cast<uint32_t>(width), static_cast<uint32_t>(height)}));
}
