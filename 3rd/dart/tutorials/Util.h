#include <QtCore>

#include "MyWindow.h"

class Util : public QObject
{
    Q_OBJECT
public:
    Util(MyWindow* window, QObject* parent = nullptr){
        _window = window;
    }
    ~Util(){}

    MyWindow* _window;
public slots:
    void qDoIdleTask() {
        if(_window)
            _window->doIdleTasks();
    }
};

