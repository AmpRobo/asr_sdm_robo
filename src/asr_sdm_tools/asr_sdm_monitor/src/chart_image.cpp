#include "asr_sdm_monitor/chart_image.hpp"

#include <QDateTime>
#include <QFont>
#include <QFontMetrics>
#include <QPainter>
#include <QPainterPath>
#include <QPen>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
bool finiteNumber(double value)
{
    return std::isfinite(value);
}

bool validRange(double min_value, double max_value)
{
    return finiteNumber(min_value) && finiteNumber(max_value) && std::abs(max_value - min_value) > 1e-12;
}

std::pair<double, double> paddedRange(double min_value, double max_value)
{
    if (!finiteNumber(min_value) || !finiteNumber(max_value)) {
        return {-1.0, 1.0};
    }
    if (std::abs(max_value - min_value) < 1e-9) {
        const double pad = std::max(1.0, std::abs(max_value) * 0.1);
        return {min_value - pad, max_value + pad};
    }
    const double pad = (max_value - min_value) * 0.08;
    return {min_value - pad, max_value + pad};
}

double clamp01(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

QString timeLabel(double value, bool time_mode, bool absolute_time)
{
    if (time_mode && absolute_time) {
        return QDateTime::fromMSecsSinceEpoch(static_cast<qint64>(value)).toString(QStringLiteral("HH:mm:ss.zzz"));
    }
    return QString::number(value, 'f', 2);
}

QString fitText(const QFontMetrics &metrics, const QString &text, int max_width)
{
    if (metrics.horizontalAdvance(text) <= max_width) {
        return text;
    }
    QString result = text;
    while (result.size() > 1 && metrics.horizontalAdvance(result + QStringLiteral("…")) > max_width) {
        result.chop(1);
    }
    return result + QStringLiteral("…");
}

bool effectiveManualRange(const PlotView &view, bool time_mode,
                          double &min_x, double &max_x, double &min_y, double &max_y)
{
    if (!view.manual || !validRange(view.min_x, view.max_x) || !validRange(view.min_y, view.max_y)) {
        return false;
    }

    min_x = view.min_x;
    max_x = view.max_x;
    min_y = view.min_y;
    max_y = view.max_y;
    if (time_mode && validRange(view.auto_min_x, view.auto_max_x)
        && finiteNumber(view.x_span) && view.x_span > 1e-12 && finiteNumber(view.x_center_offset)) {
        const double auto_center = (view.auto_min_x + view.auto_max_x) / 2.0;
        const double center = auto_center + view.x_center_offset;
        min_x = center - view.x_span / 2.0;
        max_x = center + view.x_span / 2.0;
    }
    return validRange(min_x, max_x) && validRange(min_y, max_y);
}

bool currentRange(const PlotView &view, bool time_mode,
                  double &min_x, double &max_x, double &min_y, double &max_y)
{
    if (effectiveManualRange(view, time_mode, min_x, max_x, min_y, max_y)) {
        return true;
    }
    if (!validRange(view.auto_min_x, view.auto_max_x) || !validRange(view.auto_min_y, view.auto_max_y)) {
        return false;
    }
    min_x = view.auto_min_x;
    max_x = view.auto_max_x;
    min_y = view.auto_min_y;
    max_y = view.auto_max_y;
    return true;
}

void setManualRange(PlotView &view, double min_x, double max_x, double min_y, double max_y, bool time_mode)
{
    if (!validRange(min_x, max_x) || !validRange(min_y, max_y)) {
        return;
    }
    view.manual = true;
    view.min_x = min_x;
    view.max_x = max_x;
    view.min_y = min_y;
    view.max_y = max_y;
    if (time_mode && validRange(view.auto_min_x, view.auto_max_x)) {
        view.x_span = max_x - min_x;
        view.x_center_offset = (min_x + max_x) / 2.0 - (view.auto_min_x + view.auto_max_x) / 2.0;
    }
}

void drawSeries(QPainter &painter, const QVector<double> &data, double max_y,
                const QRectF &chart, const QColor &line, const QColor &fill)
{
    if (data.size() < 2 || max_y <= 0.0) {
        return;
    }

    QPainterPath stroke;
    QPainterPath area;
    for (int i = 0; i < data.size(); ++i) {
        const double value = std::max(0.0, std::min(max_y, data.at(i)));
        const double x = chart.left() + chart.width() * i / std::max(1, static_cast<int>(data.size()) - 1);
        const double y = chart.bottom() - chart.height() * (value / max_y);
        if (i == 0) {
            stroke.moveTo(x, y);
            area.moveTo(x, y);
        } else {
            stroke.lineTo(x, y);
            area.lineTo(x, y);
        }
    }
    area.lineTo(chart.right(), chart.bottom());
    area.lineTo(chart.left(), chart.bottom());
    area.closeSubpath();
    painter.fillPath(area, fill);
    painter.setPen(QPen(line, 2.5));
    painter.drawPath(stroke);
}
}

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
    const QSize &size)
{
    QImage image(std::max(1, size.width()), std::max(1, size.height()), QImage::Format_RGBA8888);
    image.fill(background);
    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing, true);

    const QRectF chart(18, 12, std::max(1.0, image.width() - 36.0), std::max(1.0, image.height() - 30.0));
    painter.setPen(QPen(grid, 1));
    for (int gx = 0; gx < 7; ++gx) {
        const double x = chart.left() + chart.width() * gx / 6.0;
        painter.drawLine(QPointF(x, chart.top()), QPointF(x, chart.bottom()));
    }
    for (int gy = 0; gy < 5; ++gy) {
        const double y = chart.top() + chart.height() * gy / 4.0;
        painter.drawLine(QPointF(chart.left(), y), QPointF(chart.right(), y));
    }

    drawSeries(painter, primary, std::max(0.000001, max_y), chart, primary_color, primary_fill);
    drawSeries(painter, secondary, std::max(0.000001, max_y), chart, secondary_color, secondary_fill);
    return image;
}

QImage renderPlotChart(const PlotDrawRequest &request, const QSize &size)
{
    QImage image(std::max(1, size.width()), std::max(1, size.height()), QImage::Format_RGBA8888);
    image.fill(request.surface);
    if (request.view) {
        request.view->chart_left = 0;
        request.view->chart_right = 0;
        request.view->chart_top = 0;
        request.view->chart_bottom = 0;
    }
    if (request.series.empty() || request.samples.isEmpty()) {
        return image;
    }

    struct DrawnSeries
    {
        QString legend;
        QColor color;
        double line_width = 1.0;
        QVector<QPointF> points;
    };

    const int max_points = 1800;
    const int step = std::max(1, static_cast<int>(request.samples.size() / max_points));
    QVector<DrawnSeries> series;
    double min_x = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double min_y = min_x;
    double max_y = max_x;

    for (const PlotSeriesStyle &style : request.series) {
        if (style.field.isEmpty()) {
            continue;
        }
        DrawnSeries drawn;
        drawn.legend = style.legend.isEmpty() ? style.field : style.legend;
        drawn.color = style.color;
        drawn.line_width = style.line_width > 0.0 ? style.line_width : 1.0;
        for (int i = 0; i < request.samples.size(); i += step) {
            const QVariantMap sample = request.samples.at(i).toMap();
            const double x = request.time_mode
                                 ? (request.absolute_time
                                        ? sample.value(QStringLiteral("absoluteTimeMs")).toDouble()
                                        : sample.value(QStringLiteral("relativeTime")).toDouble())
                                 : sample.value(request.x_field).toDouble();
            const double y = sample.value(style.field).toDouble();
            if (!finiteNumber(x) || !finiteNumber(y)) {
                continue;
            }
            drawn.points.append(QPointF(x, y));
            min_x = std::min(min_x, x);
            max_x = std::max(max_x, x);
            min_y = std::min(min_y, y);
            max_y = std::max(max_y, y);
        }
        if (!drawn.points.isEmpty()) {
            series.append(drawn);
        }
    }

    if (series.isEmpty() || !finiteNumber(min_x) || !finiteNumber(max_x) || !finiteNumber(min_y) || !finiteNumber(max_y)) {
        return image;
    }

    QPainter painter(&image);
    painter.setRenderHint(QPainter::Antialiasing, true);
    QFont font(QStringLiteral("sans-serif"), 12);
    painter.setFont(font);
    const QFontMetrics metrics(font);

    const double width = image.width();
    const double height = image.height();
    const double left = request.show_y_ticks ? std::max(62.0, std::min(96.0, width * 0.12))
                                             : std::max(30.0, std::min(44.0, width * 0.06));
    const double right = width - std::max(18.0, std::min(36.0, width * 0.04));
    const double legend_gap = 18.0;
    const double legend_row_height = 20.0;
    const double available_legend = std::max(1.0, right - left);
    double max_item_width = 120.0;
    for (const DrawnSeries &item : series) {
        max_item_width = std::max(max_item_width, metrics.horizontalAdvance(item.legend) + 34.0);
    }
    max_item_width = std::min(max_item_width, std::max(120.0, available_legend));
    int columns = std::max(1, static_cast<int>((available_legend + legend_gap) / (max_item_width + legend_gap)));
    columns = std::max(1, std::min(static_cast<int>(series.size()), columns));
    const int legend_rows = static_cast<int>(std::ceil(series.size() / static_cast<double>(columns)));
    const double cell_width = std::max(120.0, (available_legend - legend_gap * (columns - 1)) / columns);
    const double legend_height = legend_rows * legend_row_height;
    const double bottom = height - (request.show_x_ticks ? std::max(46.0, std::min(70.0, height * 0.12))
                                                         : std::max(30.0, std::min(42.0, height * 0.08)));
    double top = std::max(42.0, 8.0 + legend_height + 14.0);
    if (bottom - top < 80.0) {
        top = std::max(42.0, bottom - 80.0);
    }
    const double chart_w = std::max(1.0, right - left);
    const double chart_h = std::max(1.0, bottom - top);

    const auto xr = paddedRange(min_x, max_x);
    const auto yr = paddedRange(min_y, max_y);
    min_x = xr.first;
    max_x = xr.second;
    min_y = yr.first;
    max_y = yr.second;

    if (!request.time_mode && request.square) {
        const double x_center = (min_x + max_x) / 2.0;
        const double y_center = (min_y + max_y) / 2.0;
        const double unit = std::max((max_x - min_x) / chart_w, (max_y - min_y) / chart_h);
        min_x = x_center - unit * chart_w / 2.0;
        max_x = x_center + unit * chart_w / 2.0;
        min_y = y_center - unit * chart_h / 2.0;
        max_y = y_center + unit * chart_h / 2.0;
    }

    if (request.view) {
        request.view->auto_min_x = min_x;
        request.view->auto_max_x = max_x;
        request.view->auto_min_y = min_y;
        request.view->auto_max_y = max_y;
        request.view->chart_left = left;
        request.view->chart_right = right;
        request.view->chart_top = top;
        request.view->chart_bottom = bottom;
    }

    double view_min_x = min_x;
    double view_max_x = max_x;
    double view_min_y = min_y;
    double view_max_y = max_y;
    if (request.view) {
        double manual_min_x = 0;
        double manual_max_x = 0;
        double manual_min_y = 0;
        double manual_max_y = 0;
        if (effectiveManualRange(*request.view, request.time_mode,
                                 manual_min_x, manual_max_x, manual_min_y, manual_max_y)) {
            view_min_x = manual_min_x;
            view_max_x = manual_max_x;
            view_min_y = manual_min_y;
            view_max_y = manual_max_y;
        }
    }

    const auto px = [&](double x) {
        return left + chart_w * (x - view_min_x) / std::max(1e-12, view_max_x - view_min_x);
    };
    const auto py = [&](double y) {
        return bottom - chart_h * (y - view_min_y) / std::max(1e-12, view_max_y - view_min_y);
    };

    painter.setPen(QPen(request.grid, 1));
    painter.setFont(QFont(QStringLiteral("sans-serif"), 11));
    for (int tick = 0; tick < 5; ++tick) {
        const double t = tick / 4.0;
        const double x = left + chart_w * t;
        const double y = top + chart_h * t;
        painter.drawLine(QPointF(x, top), QPointF(x, bottom));
        painter.drawLine(QPointF(left, y), QPointF(right, y));
        painter.setPen(request.text_secondary);
        if (request.show_x_ticks) {
            const QString label = timeLabel(view_min_x + (view_max_x - view_min_x) * t, request.time_mode, request.absolute_time);
            painter.drawText(QRectF(x - 48, bottom + 6, 96, 18), Qt::AlignHCenter | Qt::AlignTop, label);
        }
        if (request.show_y_ticks) {
            const QString label = QString::number(view_max_y - (view_max_y - view_min_y) * t, 'f', 3);
            painter.drawText(QRectF(4, y - 8, left - 12, 16), Qt::AlignRight | Qt::AlignVCenter, label);
        }
        painter.setPen(QPen(request.grid, 1));
    }

    painter.setPen(QPen(request.text_secondary, 1.2));
    painter.drawLine(QPointF(left, top), QPointF(left, bottom));
    painter.drawLine(QPointF(left, bottom), QPointF(right, bottom));

    if (request.show_marker && request.time_mode && finiteNumber(request.marker_x)
        && request.marker_x >= view_min_x && request.marker_x <= view_max_x) {
        painter.setPen(QPen(request.accent, 1.4));
        painter.drawLine(QPointF(px(request.marker_x), top), QPointF(px(request.marker_x), bottom));
    }

    painter.save();
    painter.setClipRect(QRectF(left, top, chart_w, chart_h));
    for (const DrawnSeries &line : series) {
        if (line.points.isEmpty()) {
            continue;
        }
        QPainterPath path;
        path.moveTo(px(line.points.first().x()), py(line.points.first().y()));
        for (int k = 1; k < line.points.size(); ++k) {
            path.lineTo(px(line.points.at(k).x()), py(line.points.at(k).y()));
        }
        painter.setPen(QPen(line.color, line.line_width));
        painter.drawPath(path);
        if (!request.time_mode) {
            painter.setPen(Qt::NoPen);
            painter.setBrush(line.color);
            const int dot_step = std::max(1, static_cast<int>(line.points.size() / 80));
            for (int p = 0; p < line.points.size(); p += dot_step) {
                painter.drawEllipse(QPointF(px(line.points.at(p).x()), py(line.points.at(p).y())), 2.3, 2.3);
            }
        }
    }
    painter.restore();

    painter.setPen(request.text_secondary);
    painter.setFont(QFont(QStringLiteral("sans-serif"), 12));
    painter.drawText(QRectF(left, height - 20, chart_w, 18), Qt::AlignHCenter | Qt::AlignBottom, request.x_label);

    painter.setFont(QFont(QStringLiteral("sans-serif"), 12));
    for (int index = 0; index < series.size(); ++index) {
        const int row = index / columns;
        const int column = index % columns;
        const double x = left + column * (cell_width + legend_gap);
        const double y = 8.0 + row * legend_row_height;
        painter.fillRect(QRectF(x, y + 7, 18, std::max(2.0, std::min(6.0, series.at(index).line_width))), series.at(index).color);
        painter.setPen(request.text);
        painter.drawText(QRectF(x + 26, y, std::max(20.0, cell_width - 26), legend_row_height),
                         Qt::AlignLeft | Qt::AlignVCenter,
                         fitText(metrics, series.at(index).legend, static_cast<int>(std::max(20.0, cell_width - 26))));
    }
    return image;
}

void plotResetView(PlotView &view)
{
    view.manual = false;
    view.panning = false;
}

void plotZoom(PlotView &view, double mouse_x, double mouse_y, double delta_y, bool time_mode)
{
    if (delta_y == 0.0 || view.chart_right <= view.chart_left || view.chart_bottom <= view.chart_top) {
        return;
    }
    double min_x = 0;
    double max_x = 0;
    double min_y = 0;
    double max_y = 0;
    if (!currentRange(view, time_mode, min_x, max_x, min_y, max_y)) {
        return;
    }
    const double chart_w = std::max(1e-12, view.chart_right - view.chart_left);
    const double chart_h = std::max(1e-12, view.chart_bottom - view.chart_top);
    const double tx = clamp01((mouse_x - view.chart_left) / chart_w);
    const double ty = clamp01((view.chart_bottom - mouse_y) / chart_h);
    const double anchor_x = min_x + (max_x - min_x) * tx;
    const double anchor_y = min_y + (max_y - min_y) * ty;
    const double factor = delta_y > 0.0 ? 0.82 : 1.22;
    setManualRange(view,
                   anchor_x - (anchor_x - min_x) * factor,
                   anchor_x + (max_x - anchor_x) * factor,
                   anchor_y - (anchor_y - min_y) * factor,
                   anchor_y + (max_y - anchor_y) * factor,
                   time_mode);
}

void plotPanStart(PlotView &view, double mouse_x, double mouse_y, bool time_mode)
{
    if (mouse_x < view.chart_left || mouse_x > view.chart_right || mouse_y < view.chart_top || mouse_y > view.chart_bottom) {
        view.panning = false;
        return;
    }
    double min_x = 0;
    double max_x = 0;
    double min_y = 0;
    double max_y = 0;
    if (!currentRange(view, time_mode, min_x, max_x, min_y, max_y)) {
        view.panning = false;
        return;
    }
    view.panning = true;
    view.pan_mouse_x = mouse_x;
    view.pan_mouse_y = mouse_y;
    view.pan_min_x = min_x;
    view.pan_max_x = max_x;
    view.pan_min_y = min_y;
    view.pan_max_y = max_y;
}

void plotPanUpdate(PlotView &view, double mouse_x, double mouse_y, bool time_mode)
{
    if (!view.panning || view.chart_right <= view.chart_left || view.chart_bottom <= view.chart_top) {
        return;
    }
    const double chart_w = std::max(1e-12, view.chart_right - view.chart_left);
    const double chart_h = std::max(1e-12, view.chart_bottom - view.chart_top);
    const double offset_x = -(mouse_x - view.pan_mouse_x) / chart_w * (view.pan_max_x - view.pan_min_x);
    const double offset_y = (mouse_y - view.pan_mouse_y) / chart_h * (view.pan_max_y - view.pan_min_y);
    setManualRange(view,
                   view.pan_min_x + offset_x,
                   view.pan_max_x + offset_x,
                   view.pan_min_y + offset_y,
                   view.pan_max_y + offset_y,
                   time_mode);
}
