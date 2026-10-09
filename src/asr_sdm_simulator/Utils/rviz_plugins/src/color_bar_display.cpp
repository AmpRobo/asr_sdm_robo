// Copyright (c) Amphibious Robotics.
// Screen-space color bar legend for rainbow-colored PointCloud2 displays.

#include "color_bar_display.h"

#include <OgreHardwarePixelBuffer.h>
#include <OgreMaterialManager.h>
#include <OgrePass.h>
#include <OgreTechnique.h>
#include <OgreTextureManager.h>
#include <OgreTextureUnitState.h>
#include <Overlay/OgreOverlay.h>
#include <Overlay/OgreOverlayManager.h>
#include <Overlay/OgrePanelOverlayElement.h>

#include <QFontMetrics>
#include <QGuiApplication>
#include <QPainter>
#include <QScreen>

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>
#include <vector>

#include "rviz_common/properties/bool_property.hpp"
#include "rviz_common/properties/enum_property.hpp"
#include "rviz_common/properties/float_property.hpp"
#include "rviz_common/properties/int_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "rviz_default_plugins/displays/pointcloud/point_cloud_helpers.hpp"
#include "rviz_rendering/render_system.hpp"

namespace rviz_plugins
{
namespace
{
enum Corner { TOP_LEFT, TOP_RIGHT, BOTTOM_LEFT, BOTTOM_RIGHT };

// Overlays are sized in device pixels; scale them like RViz's own Qt text.
double uiScale()
{
  const QScreen * screen = QGuiApplication::primaryScreen();
  return screen ? screen->logicalDotsPerInch() / 96.0 : 1.0;
}

// Fewest decimals (at most 4) that print `v` exactly.
int decimalsFor(double v)
{
  int decimals = 0;
  double scaled = std::fabs(v);
  while (decimals < 4 && std::fabs(scaled - std::round(scaled)) > 1e-6) {
    scaled *= 10.0;
    ++decimals;
  }
  return decimals;
}
}  // namespace

ColorBarDisplay::ColorBarDisplay()
{
  title_property_ = new rviz_common::properties::StringProperty(
    "Title", "Value", "Text shown above the bar.", this, SLOT(redraw()));
  min_property_ = new rviz_common::properties::FloatProperty(
    "Min Value", 0.0, "Min Intensity of the colored display.", this, SLOT(redraw()));
  max_property_ = new rviz_common::properties::FloatProperty(
    "Max Value", 1.0, "Max Intensity of the colored display.", this, SLOT(redraw()));
  invert_property_ = new rviz_common::properties::BoolProperty(
    "Invert Rainbow", false, "Invert Rainbow setting of the colored display.", this,
    SLOT(redraw()));
  hidden_below_property_ = new rviz_common::properties::FloatProperty(
    "Hidden Below", 0.0,
    "Values below this are never published, so this part of the bar is hatched. "
    "No effect at or below Min Value.",
    this, SLOT(redraw()));
  ticks_property_ = new rviz_common::properties::IntProperty(
    "Ticks", 6, "Number of labelled values, including both ends.", this, SLOT(redraw()));
  ticks_property_->setMin(2);
  ticks_property_->setMax(21);
  position_property_ = new rviz_common::properties::EnumProperty(
    "Position", "Top Right", "Corner of the 3D view.", this, SLOT(redraw()));
  position_property_->addOption("Top Left", TOP_LEFT);
  position_property_->addOption("Top Right", TOP_RIGHT);
  position_property_->addOption("Bottom Left", BOTTOM_LEFT);
  position_property_->addOption("Bottom Right", BOTTOM_RIGHT);
  length_property_ = new rviz_common::properties::IntProperty(
    "Length", 200, "Bar length in pixels at 96 DPI.", this, SLOT(redraw()));
  length_property_->setMin(20);
  width_property_ = new rviz_common::properties::IntProperty(
    "Width", 14, "Bar width in pixels at 96 DPI.", this, SLOT(redraw()));
  width_property_->setMin(2);
  font_size_property_ = new rviz_common::properties::IntProperty(
    "Font Size", 11, "Font size in pixels at 96 DPI.", this, SLOT(redraw()));
  font_size_property_->setMin(4);
}

ColorBarDisplay::~ColorBarDisplay()
{
  if (!overlay_) {
    return;
  }
  auto & overlays = Ogre::OverlayManager::getSingleton();
  overlays.destroy(overlay_);
  overlays.destroyOverlayElement(panel_);
  Ogre::MaterialManager::getSingleton().remove(material_);
  if (texture_) {
    Ogre::TextureManager::getSingleton().remove(texture_);
  }
}

void ColorBarDisplay::onInitialize()
{
  // RViz renders no overlays until this is called. Calling it twice for the same
  // scene manager draws every overlay twice per frame.
  static std::set<Ogre::SceneManager *> prepared;
  if (prepared.insert(scene_manager_).second) {
    rviz_rendering::RenderSystem::get()->prepareOverlays(scene_manager_);
  }

  static int count = 0;
  name_ = "ColorBarDisplay" + std::to_string(count++);
  auto & overlays = Ogre::OverlayManager::getSingleton();
  overlay_ = overlays.create(name_);
  panel_ = static_cast<Ogre::PanelOverlayElement *>(
    overlays.createOverlayElement("Panel", name_ + "Panel"));
  panel_->setMetricsMode(Ogre::GMM_PIXELS);

  material_ = Ogre::MaterialManager::getSingleton().create(
    name_ + "Material", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  Ogre::Pass * pass = material_->getTechnique(0)->getPass(0);
  pass->setLightingEnabled(false);
  pass->setDepthCheckEnabled(false);
  pass->setDepthWriteEnabled(false);
  pass->setSceneBlending(Ogre::SBT_TRANSPARENT_ALPHA);
  panel_->setMaterialName(material_->getName());
  overlay_->add2D(panel_);
  overlay_->setZOrder(500);

  redraw();
  if (isEnabled()) {
    overlay_->show();
  }
}

void ColorBarDisplay::onEnable()
{
  if (overlay_) {
    overlay_->show();
  }
}

void ColorBarDisplay::onDisable()
{
  if (overlay_) {
    overlay_->hide();
  }
}

void ColorBarDisplay::redraw()
{
  if (!panel_) {
    return;
  }
  const QImage image = render();
  upload(image);

  const float width = image.width();
  const float height = image.height();
  const float margin = 10.0f * uiScale();
  const int corner = position_property_->getOptionInt();
  const bool right = corner == TOP_RIGHT || corner == BOTTOM_RIGHT;
  const bool bottom = corner == BOTTOM_LEFT || corner == BOTTOM_RIGHT;
  panel_->setHorizontalAlignment(right ? Ogre::GHA_RIGHT : Ogre::GHA_LEFT);
  panel_->setVerticalAlignment(bottom ? Ogre::GVA_BOTTOM : Ogre::GVA_TOP);
  panel_->setPosition(right ? -width - margin : margin, bottom ? -height - margin : margin);
  panel_->setDimensions(width, height);
}

QImage ColorBarDisplay::render() const
{
  const double s = uiScale();
  const double min = min_property_->getFloat();
  const double max = std::max<double>(max_property_->getFloat(), min + 1e-6);
  const double hidden_below = hidden_below_property_->getFloat();
  const bool invert = invert_property_->getBool();
  const int ticks = ticks_property_->getInt();
  const QString title = title_property_->getString();

  QFont font;
  font.setPixelSize(std::max(1, static_cast<int>(std::lround(font_size_property_->getInt() * s))));
  QFont title_font = font;
  title_font.setBold(true);
  title_font.setPixelSize(
    std::max(1, static_cast<int>(std::lround((font_size_property_->getInt() - 3) * s))));
  const QFontMetrics fm(font);
  const QFontMetrics title_fm(title_font);

  const int pad_x = std::lround(3 * s);
  const int pad_y = std::lround(4 * s);
  const int gap = std::lround(3 * s);
  const int tick_len = std::lround(3 * s);
  const int bar_w = std::lround(width_property_->getInt() * s);
  const int bar_len = std::lround(length_property_->getInt() * s);
  auto rows_apart = [&](double a, double b) {
      return std::fabs(a - b) / (max - min) * bar_len;
    };

  const double step = (max - min) / (ticks - 1);
  const int decimals = std::max(decimalsFor(min), decimalsFor(step));
  std::vector<std::pair<double, QString>> labels;
  for (int i = 0; i < ticks; ++i) {
    const double v = min + step * i;
    labels.emplace_back(v, QString::number(v, 'f', decimals));
  }
  const bool has_hidden = hidden_below > min && hidden_below < max;
  if (has_hidden) {
    const bool clashes = std::any_of(
      labels.begin(), labels.end(), [&](const auto & label) {
        return rows_apart(label.first, hidden_below) < fm.height();
      });
    if (!clashes) {
      labels.emplace_back(
        hidden_below,
        QString::number(hidden_below, 'f', std::max(decimals, decimalsFor(hidden_below))));
    }
  }

  int label_w = 0;
  for (const auto & label : labels) {
    label_w = std::max(label_w, fm.horizontalAdvance(label.second));
  }
  // The bar and its labels set the width. A long title wraps instead of
  // stretching the black background out past them.
  const int column = bar_w + tick_len + gap + label_w;
  std::vector<QString> title_lines;
  if (!title.isEmpty()) {
    QString line;
    for (const QString & word : title.split(' ', Qt::SkipEmptyParts)) {
      const QString trial = line.isEmpty() ? word : line + QLatin1Char(' ') + word;
      if (!line.isEmpty() && title_fm.horizontalAdvance(trial) > column) {
        title_lines.push_back(line);
        line = word;
      } else {
        line = trial;
      }
    }
    if (!line.isEmpty()) {
      title_lines.push_back(line);
    }
  }
  int title_w = 0;
  for (const QString & line : title_lines) {
    title_w = std::max(title_w, title_fm.horizontalAdvance(line));
  }
  const int bar_x = pad_x;
  const int title_block =
    title_lines.empty() ? 0 : static_cast<int>(title_lines.size()) * title_fm.height() + gap;
  const int bar_top = pad_y + title_block + fm.height() / 2;
  auto row_of = [&](double v) {return bar_top + (max - v) / (max - min) * bar_len;};
  const int width = pad_x + std::max(column, title_w) + pad_x;
  const int height = bar_top + bar_len + fm.height() / 2 + pad_y;

  QImage image(width, height, QImage::Format_ARGB32);
  image.fill(Qt::transparent);
  QPainter painter(&image);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.setRenderHint(QPainter::TextAntialiasing);
  painter.setPen(Qt::NoPen);
  painter.setBrush(QColor(0, 0, 0, 150));
  painter.drawRoundedRect(QRectF(0, 0, width, height), 4 * s, 4 * s);

  for (int row = 0; row < bar_len; ++row) {
    const double v = max - (row + 0.5) / bar_len * (max - min);
    // Same mapping as rviz_default_plugins::IntensityPCTransformer.
    float value = 1.0f - static_cast<float>((v - min) / (max - min));
    if (invert) {
      value = 1.0f - value;
    }
    Ogre::ColourValue color;
    rviz_default_plugins::getRainbowColor(value, color);
    const double alpha = has_hidden && v < hidden_below ? 0.3 : 1.0;
    painter.fillRect(
      QRectF(bar_x, bar_top + row, bar_w, 1), QColor::fromRgbF(color.r, color.g, color.b, alpha));
  }
  if (has_hidden) {
    const double top = row_of(hidden_below);
    painter.fillRect(
      QRectF(bar_x, top, bar_w, bar_top + bar_len - top),
      QBrush(QColor(255, 255, 255, 110), Qt::BDiagPattern));
  }

  painter.setBrush(Qt::NoBrush);
  painter.setPen(QPen(QColor(255, 255, 255, 200), std::max(1.0, s)));
  painter.drawRect(QRectF(bar_x, bar_top, bar_w, bar_len));
  painter.setFont(font);
  for (const auto & [v, text] : labels) {
    const double y = row_of(v);
    painter.drawLine(QPointF(bar_x + bar_w, y), QPointF(bar_x + bar_w + tick_len, y));
    painter.drawText(
      QPointF(bar_x + bar_w + tick_len + gap, y + (fm.ascent() - fm.descent()) / 2.0), text);
  }
  if (!title_lines.empty()) {
    painter.setFont(title_font);
    for (size_t i = 0; i < title_lines.size(); ++i) {
      painter.drawText(
        QPointF(pad_x, pad_y + title_fm.ascent() + static_cast<int>(i) * title_fm.height()),
        title_lines[i]);
    }
  }
  painter.end();
  return image;
}

void ColorBarDisplay::upload(const QImage & image)
{
  const auto width = static_cast<Ogre::uint32>(image.width());
  const auto height = static_cast<Ogre::uint32>(image.height());
  if (!texture_ || texture_->getWidth() != width || texture_->getHeight() != height) {
    Ogre::Pass * pass = material_->getTechnique(0)->getPass(0);
    pass->removeAllTextureUnitStates();
    if (texture_) {
      Ogre::TextureManager::getSingleton().remove(texture_);
    }
    texture_ = Ogre::TextureManager::getSingleton().createManual(
      name_ + "Texture", Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      Ogre::TEX_TYPE_2D, width, height, 0, Ogre::PF_A8R8G8B8, Ogre::TU_STATIC_WRITE_ONLY);
    Ogre::TextureUnitState * unit = pass->createTextureUnitState();
    unit->setTexture(texture_);
    unit->setTextureFiltering(Ogre::TFO_NONE);
  }
  // QImage::Format_ARGB32 and PF_A8R8G8B8 are both native-endian 0xAARRGGBB.
  const Ogre::PixelBox box(
    width, height, 1, Ogre::PF_A8R8G8B8, const_cast<uchar *>(image.constBits()));
  texture_->getBuffer()->blitFromMemory(box);
}

}  // namespace rviz_plugins

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(rviz_plugins::ColorBarDisplay, rviz_common::Display)
