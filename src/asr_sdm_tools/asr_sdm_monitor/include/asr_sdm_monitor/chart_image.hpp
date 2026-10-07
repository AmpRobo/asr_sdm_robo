#pragma once

#include <QColor>
#include <QImage>
#include <QString>
#include <QVariant>
#include <QVector>

#include <vector>

// View state for the plot chart. The renderer updates the automatic range and the
// pixel rectangle; pointer handlers use that rectangle to zoom and pan.
struct PlotView
{
    bool manual = false;
    double min_x = 0;
    double max_x = 1;
    double min_y = 0;
    double max_y = 1;
    double x_span = 1;
    double x_center_offset = 0;
    double auto_min_x = 0;
    double auto_max_x = 1;
    double auto_min_y = 0;
    double auto_max_y = 1;
    double chart_left = 0;
    double chart_right = 0;
    double chart_top = 0;
    double chart_bottom = 0;
    bool panning = false;
    double pan_mouse_x = 0;
    double pan_mouse_y = 0;
    double pan_min_x = 0;
    double pan_max_x = 1;
    double pan_min_y = 0;
    double pan_max_y = 1;
};

struct PlotSeriesStyle
{
    QString field;
    QString legend;
    QColor color;
    double line_width = 1.0;
};

struct PlotDrawRequest
{
    QVariantList samples;
    bool time_mode = true;
    bool absolute_time = false;
    QString x_field;
    QString x_label;
    std::vector<PlotSeriesStyle> series;
    bool square = false;
    bool show_x_ticks = true;
    bool show_y_ticks = true;
    bool show_marker = false;
    double marker_x = 0;
    QColor surface;
    QColor grid;
    QColor text;
    QColor text_secondary;
    QColor accent;
    PlotView *view = nullptr;
};

QImage renderAreaChart(
    const QVector<double> &primary,
    const QVector<double> &secondary,
    double max_y,
    const QColor &background,
    const QColor &grid,
    const QColor &primary_color,
    const QColor &primary_fill,
    const QColor &secondary_color,
    const QColor &secondary_fill,
    const QSize &size);

QImage renderPlotChart(const PlotDrawRequest &request, const QSize &size);

void plotResetView(PlotView &view);
void plotZoom(PlotView &view, double mouse_x, double mouse_y, double delta_y, bool time_mode);
void plotPanStart(PlotView &view, double mouse_x, double mouse_y, bool time_mode);
void plotPanUpdate(PlotView &view, double mouse_x, double mouse_y, bool time_mode);
