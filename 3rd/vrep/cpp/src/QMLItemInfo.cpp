#include <QtGlobal> // Q_OS_WIN
#ifdef Q_OS_WIN
// KEV_QT_DIALOG_SET_ENABLE_CLOSE_BTN
#include <qt_windows.h>  // Resolve Qt5 - qdatetime - std::numeric_limits<qint64>::min()
#endif
#include <QQmlProperty>
#include "QMLItemInfo.h"
#include "RbGlobal.h"
