#ifndef ___ROBOT_DART_ADAPTER_H___
#define ___ROBOT_DART_ADAPTER_H___

#include <QtCore>
#include <QMutex>
#include "KsGlobal.h"

#include <cstdio>
#include <cstdarg>

#include "dart/dart.hpp"
#include "dart/gui/gui.hpp"
//#include <rviz/robot/robot.h>

class RobotDartAdapter : public QObject {
    Q_OBJECT

public:
    static RobotDartAdapter* getInstance();
    ~RobotDartAdapter();
    static void deleteInstance();

    void initDart(int argc, char* argv[]);
private:
    RobotDartAdapter();
    static RobotDartAdapter* _instance;
    //
    QTimer timer;
    QMutex* _pMutex;
};

class MyWindow : public dart::gui::SimWindow {
public:
  /// \brief The constructor - set the position of the skeleton
  explicit MyWindow(dart::dynamics::SkeletonPtr _skel): SimWindow(), skel(_skel) {
    mTrans[1] = 200.f;
    mZoom = 0.3;
  }

  /// \brief Draw the skeleton
  void draw() override;

  /// \brief Move the joints with the {1,2,3} keys and '-' to change direction
  void keyboard(unsigned char _key, int _x, int _y) override;

  /// \brief Hardcoded skeleton
  dart::dynamics::SkeletonPtr skel;
};

#endif // ___ROBOT_DART_ADAPTER_H___

