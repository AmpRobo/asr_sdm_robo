// Copyright (c) Amphibious Robotics.
// Screen-space color bar legend for rainbow-colored PointCloud2 displays.

#ifndef RVIZ_PLUGINS_COLOR_BAR_DISPLAY_H
#define RVIZ_PLUGINS_COLOR_BAR_DISPLAY_H

#ifndef Q_MOC_RUN
# include <OgreMaterial.h>
# include <OgreTexture.h>

# include <QImage>

# include <string>

# include "rviz_common/display.hpp"
#endif

namespace Ogre
{
class Overlay;
class PanelOverlayElement;
}  // namespace Ogre

namespace rviz_common
{
namespace properties
{
class BoolProperty;
class EnumProperty;
class FloatProperty;
class IntProperty;
class StringProperty;
}  // namespace properties
}  // namespace rviz_common

namespace rviz_plugins
{

// Draws the palette of a PointCloud2 display that uses the Intensity color
// transformer with "Use rainbow" on. Min Value, Max Value and Invert Rainbow
// must be set to the same values as that display.
class ColorBarDisplay : public rviz_common::Display
{
Q_OBJECT
public:
  ColorBarDisplay();
  ~ColorBarDisplay() override;

protected:
  void onInitialize() override;
  void onEnable() override;
  void onDisable() override;

private Q_SLOTS:
  void redraw();

private:
  QImage render() const;
  void upload(const QImage & image);

  rviz_common::properties::StringProperty * title_property_;
  rviz_common::properties::FloatProperty * min_property_;
  rviz_common::properties::FloatProperty * max_property_;
  rviz_common::properties::BoolProperty * invert_property_;
  rviz_common::properties::FloatProperty * hidden_below_property_;
  rviz_common::properties::IntProperty * ticks_property_;
  rviz_common::properties::EnumProperty * position_property_;
  rviz_common::properties::IntProperty * length_property_;
  rviz_common::properties::IntProperty * width_property_;
  rviz_common::properties::IntProperty * font_size_property_;

  std::string name_;
  Ogre::Overlay * overlay_ = nullptr;
  Ogre::PanelOverlayElement * panel_ = nullptr;
  Ogre::MaterialPtr material_;
  Ogre::TexturePtr texture_;
};

}  // namespace rviz_plugins

#endif
