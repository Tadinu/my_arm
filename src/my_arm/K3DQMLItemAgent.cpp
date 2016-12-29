#include <QtGlobal> // Q_OS_WIN
#ifdef Q_OS_WIN
// KEV_QT_DIALOG_SET_ENABLE_CLOSE_BTN
#include <qt_windows.h>  // Resolve Qt5 - qdatetime - std::numeric_limits<qint64>::min()
#endif
#include <QQmlProperty>
#include "K3DQMLItemAgent.h"
#include "KsGlobal.h"

static const double K_DIALOG_TRANSPARENT_LEVEL = 0.5;
    
/* SINGLETON =====================================================================
K3DQMLItemAgent *K3DQMLItemAgent::getInstance()
{
    if (nullptr == _instance) {
        _instance = new MaterialDialog();*

        Q_ASSERT(_instance != nullptr);
    }

    return _instance;
}

void K3DQMLItemAgent::deleteInstance()
{
    if (nullptr != _instance) {
        delete _instance;
        _instance = nullptr; // NULLIFY _instance to mark it DELETED
    }
}
================================================================================== */

K3DQMLItemAgent::K3DQMLItemAgent(QObject *parent, bool isBusy,
                                 int itemId, QObject* itemUI)
            : QObject(parent),
              _itemId(itemId),
              _itemUI(itemUI),
              _posY(0),
              _posX(0),
              _isEscDisabled(false),
              _isBusy(isBusy),
              _isCloseCalledOnShownProgressBar(false)

{
    readSettings(); // Though this is virtual, calling it here in ctor does not invoke derived's readSettings()!
                    // Anw. since different agents, readSettings() and setupUI() should be called independently.
}

K3DQMLItemAgent::~K3DQMLItemAgent()
{
    // IF SINGLETON:
    //     s_instance = nullptr;
    // => Nullify _instance upon deletion (here in case it was forgotten doing so)!
}

void K3DQMLItemAgent::setupUI()
{

}

int K3DQMLItemAgent::itemId() const
{
    return _itemId;
}

void K3DQMLItemAgent::setupAgentAndUI(const QString& qmlPropertyName)
{
    // [QML Item UI]
    QQmlProperty prop(KsGlobal::k3dQMLCom(), qmlPropertyName);
    _itemUI = qvariant_cast<QObject *>(prop.read()); //!!!

    // [C++ Item Agent]
    QMetaObject::invokeMethod(_itemUI, "regItemAgent", 
                              Q_ARG(QVariant, QVariant::fromValue(this)));

    //
    setupUI();
}

QObject* K3DQMLItemAgent::UI() const
{
    return _itemUI;
}

void K3DQMLItemAgent::savePos(int posX, int posY)
{
    _posX = posX;
    _posY = posY;
    writeSettings(); // [Virtual]
}

void K3DQMLItemAgent::readSettings()
{
#if 0
    // 1 - Dialog Position (This is already done in QML - K3DDialog.qml)
    QRect frect = frameGeometry();
    move(Settings::getDialogPositon(K3DQMLAdapter::K3D_OBJ_MOVE_DIALOG, frect));
#endif
}

void K3DQMLItemAgent::writeSettings()
{
}
// -------------------------------------------------------------------------------

void K3DQMLItemAgent::formatText()
{
    //guiutil::formatAllItemsText(this);
    //this->fixBiggerTextMode(); // [virtual]
}

void K3DQMLItemAgent::popUp() {
    /*show();
    raise();
    activateWindow();*/
}

/**************************************************************************************************
 * @fn  bool K3DQMLItemAgent::event (QEvent * e)
 *
 * @brief   Events the given e.
 *          http://qt-project.org/doc/qt-4.8/eventsandfilters.html
 *          => 
 *          If you want to replace the base class's function, you must implement everything yourself.
 *          However, if you only want to extend the base class's functionality, then you implement what 
 *          you want and call the base class to obtain the default behavior for any cases you do not want to handle.
 *
 *         !NOTE: KSDIALOG -> : KDIALOG
 *          => KSDIALOG'SPECIFIC_EVENT' -> KDIALOG::event()[virtual] -> Base Call QWidget::event() 
 *                                                                   -> Specific_event() [virtual]
 *                                                                   -> KSDIALOG::specific_event()
 *          => So, in KSDIALOG::specific_event(), no need to call up to KDIALOG::specific_event() if already handled in KDIALOG's event()!
 * @author
 * @date
 *
 * @param [in,out]  e   If non-null, the QEvent * to process.
 *
 * @return  true if it succeeds, false if it fails.
 **************************************************************************************************/

bool K3DQMLItemAgent::event (QEvent * e)
{
    return true;
    //switch (e->type())
    //{
    //case QEvent::WindowActivate:
    //    //KLOG_INFO() << "Activate:" << metaObject()->className();
    //    setWindowOpacity(1.0);
    //    break;
    //case QEvent::WindowDeactivate:
    //    //KLOG_INFO() << "Deactivate:" << metaObject()->className();
    //    setWindowOpacity(K_DIALOG_TRANSPARENT_LEVEL);
    //    break;
    //case QEvent::Show:
    //    //KLOG_INFO() << "Show:" << metaObject()->className();
    //    _isActivated = true;
    //    break;
    //case QEvent::Close:
    //    //KLOG_INFO() << "Hide:" << metaObject()->className();
    //    _isActivated = false;
    //    // Since some dialog class's closeEvent has mistake of not calling K3DQMLItemAgent::closeEvent(),
    //    // , so reject() is not called!

    //    //! Note that: Only have the dialog possibly ignored in time of _isBusy.
    //    // When we ignore Close event, we understand that we may be also ignoring all possible
    //    // Qt built-in closeEvent().
    //    if(_isBusy)
    //    {
    //        // Only Accept close event from this->close() (Eg: from self-defined 'Close/X' button)
    //        if(_isCloseCalledOnShownProgressBar)
    //        {
    //            // Default as e->accept()
    //            _isCloseCalledOnShownProgressBar = false;
    //        }
    //        else // Other close events (from title bar) or 'Alt+F4' are ignored
    //        {
    //            e->ignore();
    //            return true;
    //        }
    //    }

    //    // GP-
    //    /* IF SINGLETON:
    //           if (s_instance->testAttribute(Qt::WA_DeleteOnClose))
    //               // IT's BETTER TO PUT s_instance = nullptr; in ~DTOR()
    //               // SINCE, FROM NOW UNTIL s_instance DELETION, it may be used.
    //       ELSE NOT SINGLETON:
    //           if (this->testAttribute(Qt::WA_DeleteOnClose))
    //               KSTUDIO: MainWindow::getInstance()->clearOwningPointer();
    //               K3PM: MainDialog::getInstance()->clearOwningPoiner();
    //    */
    //    break;

    //case QEvent::KeyPress:
    //    if ( (static_cast<QKeyEvent*>(e)->key() == Qt::Key_Escape) &&
    //         _isEscDisabled)
    //    {
    //        e->ignore();
    //        return true;
    //    }
    //    break;
    //}

    //return QDialog::event(e); 
    //// -> invoke QWidget::event(e), from which depending on e-type(), invoke the Virtual Event Handler
}

/**************************************************************************************************
 * @fn  void K3DQMLItemAgent::setDisableEsc(bool)
 *
 * @brief   Sets disable escape.
 *
 * @author  Duc Than
 * @date    5/13/2014
 *
 * @param   parameter1  true to parameter 1.
 **************************************************************************************************/

void K3DQMLItemAgent::setDisableEsc(bool isTrue)
{
    _isEscDisabled = isTrue;
}

/**************************************************************************************************
 * @fn  bool K3DQMLItemAgent::isEscDisabled()
 *
 * @brief   Queries if an escape is disabled.
 *
 * @author  Duc Than
 * @date    5/13/2014
 *
 * @return  true if an escape is disabled, false if not.
 **************************************************************************************************/

bool K3DQMLItemAgent::isEscDisabled()
{
    return _isEscDisabled;
}


/**************************************************************************************************
 * @fn  bool K3DQMLItemAgent::isUsing()
 *
 * @brief   Query if this object is being shown.
 *
 * @author  Unknown
 * @date
 *
 * @return  true if using, false if not.
 **************************************************************************************************/
bool K3DQMLItemAgent::isClosed()
{
    QVariant ret;
    QMetaObject::invokeMethod(_itemUI, "isClosed",
                              Q_RETURN_ARG(QVariant, ret));
    return ret.toBool();
}

bool K3DQMLItemAgent::isActivated()
{
    QVariant ret;
    QMetaObject::invokeMethod(_itemUI, "isActivated",
        Q_RETURN_ARG(QVariant, ret));
    return ret.toBool();
}

bool K3DQMLItemAgent::isLastFocused()
{
    QVariant ret;
    QMetaObject::invokeMethod(_itemUI, "isLastFocused",
        Q_RETURN_ARG(QVariant, ret));
    return ret.toBool();
}

bool K3DQMLItemAgent::isModalDialog()
{
    QVariant ret;
    QMetaObject::invokeMethod(_itemUI, "isModal",
                              Q_RETURN_ARG(QVariant, ret));
    return ret.toBool();
}


void K3DQMLItemAgent::resize(qreal width, qreal height)
{
    QMetaObject::invokeMethod(_itemUI, "resize",
        Q_ARG(QVariant, width),
        Q_ARG(QVariant, height));
}

void K3DQMLItemAgent::resize(const QSizeF& size)
{
    QMetaObject::invokeMethod(_itemUI, "resize",
        Q_ARG(QVariant, size.width()),
        Q_ARG(QVariant, size.height()));
}

QSizeF K3DQMLItemAgent::size()
{
    QVariant var;
    QMetaObject::invokeMethod(_itemUI, "size",
        Q_RETURN_ARG(QVariant, var));
    return var.toSizeF();
}

qreal K3DQMLItemAgent::width()
{
    return size().width();
}

qreal K3DQMLItemAgent::height()
{
    return size().height();
}

void K3DQMLItemAgent::move(int posX, int posY)
{
    QMetaObject::invokeMethod(_itemUI, "move",
        Q_ARG(QVariant, posX),
        Q_ARG(QVariant, posY));
}

QPoint K3DQMLItemAgent::pos()
{
    QVariant var;
    QMetaObject::invokeMethod(_itemUI, "pos",
        Q_RETURN_ARG(QVariant, var));
    return var.toPoint();
}

bool K3DQMLItemAgent::start()
{
    QMetaObject::invokeMethod(_itemUI, "setActivated",
        Q_ARG(QVariant,true));
    return true;
}

/**************************************************************************************************
 * @fn  bool K3DQMLItemAgent::end()
 *
 * @brief   Ends this object.
 *          A self-defined 'Close' button's click should be connected to this!
 *
 * @author  Duc Than
 * @date    5/21/2015
 *
 * @return  true if it succeeds, false if it fails.
 **************************************************************************************************/

bool K3DQMLItemAgent::end()
{
    QMetaObject::invokeMethod(_itemUI, "setActivated",
        Q_ARG(QVariant, false));

    // - IF [_isEscDiabled]: HANDLED IN K3DQMLItemAgent::event(QEvent*e)
    //
    // - IF [_isBusy]: DO NOTHING,
    // NO MATTER WHICH this->reject() COMES FROM (INCLUDING Qt::Key_Escape)
    //if (false == _isBusy);
    //QDialog::reject();
    // 
    // We only care about _isQWidgetCloseCalled in condition: _isBusy

    if(_isBusy)
        _isCloseCalledOnShownProgressBar = true;

    writeSettings(); // [Virtual]
    return true;
}

void K3DQMLItemAgent::showDialog(bool isStart)
{
    if (_itemUI == nullptr) return;

    QMetaObject::invokeMethod(_itemUI, "show"); // By default, show with highlight!
    if (isStart) {
        this->start();
    }
}

void K3DQMLItemAgent::hideDialog()
{
    if (_itemUI == nullptr) return;
    
    QMetaObject::invokeMethod(_itemUI, "hide");
}

void K3DQMLItemAgent::showHideDialog(bool isShown, bool isStart)
{
    if (isShown)
        this->showDialog(isStart);
    else
        this->hideDialog();
}

void K3DQMLItemAgent::closeDialog()
{
    if (_itemUI == nullptr) return;

    QMetaObject::invokeMethod(_itemUI, "close"); // QML::close() -> invokes this->end()
}

bool K3DQMLItemAgent::isHidden()
{
    if (_itemUI == nullptr) return true;

    QVariant ret;
    QMetaObject::invokeMethod(_itemUI, "isHidden",
                              Q_RETURN_ARG(QVariant, ret));
    return ret.toBool();
}

bool K3DQMLItemAgent::isVisible()
{
    if (_itemUI == nullptr) return false;

    QVariant ret;
    QMetaObject::invokeMethod(_itemUI, "isVisible",
                              Q_RETURN_ARG(QVariant, ret));
    return ret.toBool();
}

bool K3DQMLItemAgent::isDeletedOnClosed()
{
    if (_itemUI == nullptr) return false;

    QVariant ret;
    QMetaObject::invokeMethod(_itemUI, "isDeletedOnClosed",
                              Q_RETURN_ARG(QVariant, ret));
    return ret.toBool();
}

bool K3DQMLItemAgent::apply()
{
    return true;
}

/**************************************************************************************************
 * @fn  void K3DQMLItemAgent::setIsBusy(bool isBusy)
 *
 * @brief   Sets a K3DQMLItemAgent if currently showing progress bar or not.
 *          Put the codes using a progress bar between setIsBusy(true) & setIsBusy(false)
 *          Should any error happen in between, throw an exception and setIsBusy(fase) in catch {}
 * @author  Duc Than
 * @date    8/28/2013
 *
 * @param   isBusy true to isBusy.
 **************************************************************************************************/

void K3DQMLItemAgent::setIsBusy(bool isBusy)
{
    _isBusy = isBusy;

    //KEV_QT_DIALOG_SET_ENABLE_CLOSE_BTN(this, !isBusy);
}

void K3DQMLItemAgent::setMouseOptions(const QVector<int> mouseOptionList)
{
    _mouseOptionList = mouseOptionList;
}

void K3DQMLItemAgent::setMouseOptions(int mouseOption)
{
    _mouseOptionList << mouseOption;
}

#if 0 // Currently, this cannot apply to all dialogs yet (eg: Progress Dialog, due to same main thread residing)
/**************************************************************************************************
 * @fn  void K3DQMLItemAgent::mousePressEvent(QMouseEvent *event)
 *
 * @brief   Mouse press event.
 *
 * @author  Son Nguyen
 * @date    12/3/2013
 *
 * @param [in,out]  event   If non-null, the event.
 **************************************************************************************************/

void K3DDialogAgent::mousePressEvent(QMouseEvent *event)
{
    if (Qt::LeftButton == event->buttons()) {
        _lastPos = event->globalPos();
        KLOG_DEBUG() << QString("POS-CLICK:%1-%2").arg(_lastPos.x()).arg(_lastPos.y());
    }
}

/**************************************************************************************************
 * @fn  void K3DDialogAgent::mouseMoveEvent(QMouseEvent *event)
 *
 * @brief   Mouse move event.
 *
 * @author  Duc Than
 * @date    12/3/2013
 *
 * @param [in,out]  event   If non-null, the event.
 **************************************************************************************************/

void K3DDialogAgent::mouseMoveEvent(QMouseEvent *event)
{
    if (Qt::LeftButton == event->buttons()) {
        this->move(this->pos() + (event->globalPos() - _lastPos));
        _lastPos = event->globalPos();
    }
}
#endif
