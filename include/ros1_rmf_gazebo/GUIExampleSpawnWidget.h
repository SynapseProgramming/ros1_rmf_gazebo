#ifndef _GUI_EXAMPLE_SPAWN_WIDGET_HH_
#define _GUI_EXAMPLE_SPAWN_WIDGET_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
// moc parsing error of tbb headers
#ifndef Q_MOC_RUN
#include <gazebo/transport/transport.hh>
#endif

namespace gazebo {
class GAZEBO_VISIBLE GUIExampleSpawnWidget : public GUIPlugin {
  Q_OBJECT

  /// \brief Constructor
  /// \param[in] _parent Parent widget
public:
  GUIExampleSpawnWidget();

  /// \brief Destructor
public:
  virtual ~GUIExampleSpawnWidget();

  /// \brief Callback trigged when the button is pressed.
protected slots:
  void OnButton();

  /// \brief Counter used to create unique model names
private:
  unsigned int counter;

  /// \brief Node used to establish communication with gzserver.
private:
  transport::NodePtr node;

  /// \brief Publisher of factory messages.
private:
  transport::PublisherPtr factoryPub;
};
} // namespace gazebo
#endif
