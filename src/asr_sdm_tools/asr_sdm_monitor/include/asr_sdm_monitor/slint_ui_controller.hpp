#pragma once

#include "main_window.h"
#include "asr_sdm_monitor/chart_image.hpp"

#include <QString>
#include <QStringList>
#include <QVariant>
#include <array>
#include <vector>

class RosUiBridge;

// Pushes RosUiBridge data into the Slint window and routes UI callbacks back
// into the bridge. Chart pixels are produced here; Slint only displays them.
class SlintUiController
{
public:
    SlintUiController(
        RosUiBridge &bridge,
        slint::ComponentHandle<asr_sdm_monitor_ui::MainWindow> window);
    void sync();

private:
    struct SeriesSetting
    {
        QString field_path;
        QColor color;
        double line_width = 1.0;
    };

    struct PlotSettings
    {
        int x_mode = 0;
        int timestamp = 0;
        int axis_scale = 0;
        QString x_field;
        int series_count = 1;
        bool show_x_ticks = true;
        bool show_y_ticks = true;
        double time_window = 4.0;
        std::vector<SeriesSetting> series;
        PlotView view;
    };

    struct FieldInfo
    {
        QString path;
        QString label;
        QString unit;
    };

    void bind();
    bool zh() const;
    bool dark() const;
    QString trKey(const char *key) const;
    const asr_sdm_monitor_ui::UiState &state() const;
    PlotSettings &activeSettings();
    void readSettingsFromUi(PlotSettings &settings);
    void applySettings(const PlotSettings &settings);
    void bumpEditors();
    void ensureSeries(PlotSettings &settings);
    QColor seriesColor(int index) const;
    void rebuildFields();
    void pushSeries();
    int yIndexForPath(const QString &path) const;
    QString yPathForIndex(int index) const;
    void refreshHardware();
    void refreshVideo();
    void refreshTopics();
    void refreshPlot();
    void refreshPlayback();
    void refreshSimulator();
    QVariantList filteredSamples(const PlotSettings &settings) const;
    void redrawPlot();
    void toggleWindowMaximized();
    void dragWindowEdge(int edges, int kind, float x, float y);

    RosUiBridge &bridge_;
    slint::ComponentHandle<asr_sdm_monitor_ui::MainWindow> window_;
    bool applying_ = false;
    int editor_generation_ = 0;
    int topic_sort_ = 0;
    int plot_section_ = 1;
    int plot_width_ = 0;
    int plot_height_ = 0;
    QString recording_path_;
    QString recorded_path_;
    PlotSettings live_settings_;
    PlotSettings recorded_settings_;
    std::vector<FieldInfo> y_fields_;
    std::vector<FieldInfo> x_fields_;
    QStringList video_names_;
    std::vector<QString> topic_keys_;
    QString cpu_values_fp_;
    QString cpu_rows_fp_;
    QString cpu_chart_fp_;
    QString memory_values_fp_;
    QString memory_rows_fp_;
    QString memory_chart_fp_;
    QString hdd_values_fp_;
    QString hdd_rows_fp_;
    QString net_values_fp_;
    QString net_rows_fp_;
    QString net_chart_fp_;
    QString ntp_values_fp_;
    QString ntp_rows_fp_;
    QString video_topics_fp_;
    QString topics_fp_;
    QString fields_fp_;
    QString series_fp_;
    QString plot_fp_;
    QString simulator_log_fp_;
    std::array<int, 4> video_revision_{{-1, -1, -1, -1}};
    bool window_maximized_ = false;
    slint::PhysicalPosition restore_position_{};
    slint::PhysicalSize restore_size_{};
    struct WindowEdgeDrag
    {
        bool active = false;
        int edges = 0;
        int start_cursor_x = 0;
        int start_cursor_y = 0;
        int start_x = 0;
        int start_y = 0;
        uint32_t start_width = 0;
        uint32_t start_height = 0;
    } edge_drag_;
};
