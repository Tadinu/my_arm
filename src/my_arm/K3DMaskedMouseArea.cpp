/****************************************************************************
**
** Copyright (C) 2013 Digia Plc and/or its subsidiary(-ies).
** Contact: http://www.qt-project.org/legal
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of Digia Plc and its Subsidiary(-ies) nor the names
**     of its contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "K3DMaskedMouseArea.h"

#include <QStyleHints>
#include <QDesktopWidget>
#include <QGuiApplication>
#include <QTimer>
#include <qqmlfile.h>

#include <QEvent>
#include <QMouseEvent>
#include <QTimer>
#include <QQuickWindow>
#include <QScreen>
#include <QDebug>
#include <qmath.h>

K3DMaskedMouseArea::K3DMaskedMouseArea(QQuickItem *parent)
    : QQuickItem(parent),
      _widgetContainer(nullptr),
      _isModal(false),


      _isContainerTopLevel(false),
      _isMovingAgentRole(false),
      _isResizingAgentRole(false),

      _resizeTargetMinWidth(0),
      _resizeTargetMinHeight(0),
      _resizeTargetMaxWidth(0),
      _resizeTargetMaxHeight(0),

      _pressed(false),
      _propagateComposedEvents(true),
      m_alphaThreshold(0.0),
      _rotationAngle(0.0),
      m_containsMouse(false),

      m_cursorShape(-1),
      m_currentShapeSource(""),

      _dragTarget(nullptr),
      _dragMinimumX(-10000.0),
      _dragMaximumX(10000.0),
      _dragMinimumY(-10000.0),
      _dragMaximumY(10000.0),

      _dragRefRectX(0),
      _dragRefRectY(0),
      _dragRefRectWidth(0),
      _dragRefRectHeight(0),
      _dragReftRectVisible(false),

      m_pressAndHoldEvent(0),
      m_lastEvent(0),
      m_acceptedButtons(Qt::LeftButton)
{
    m_pressAndHoldTimer = new QTimer(this);
    m_pressAndHoldTimer->setSingleShot(true);
    connect(m_pressAndHoldTimer, SIGNAL(timeout()),
            this, SLOT(handlePressAndHold()));

    qmlRegisterType<K3DMouseEvent>();
    qmlRegisterType<K3DWheelEvent>();

    setFiltersChildMouseEvents(true);

    setAcceptedMouseButtons(Qt::LeftButton|Qt::RightButton|Qt::MidButton|Qt::XButton1|Qt::XButton2);
    //setAcceptedMouseButtons(Qt::AllButtons);
}

void K3DMaskedMouseArea::setQuickContainer(QWidget* widget, bool isTopLevel)
{
    _widgetContainer     = widget;
    _isContainerTopLevel = isTopLevel;
}

bool K3DMaskedMouseArea::event(QEvent * e)
{
    switch (e->type()) {
        case QEvent::MouseButtonPress:
        case QEvent::MouseButtonDblClick:
        case QEvent::MouseButtonRelease:
            //qDebug() << "Masked Mouse Area Event";
            if (!_propagateComposedEvents)
            {
                //qDebug() << "Reject!";
                e->setAccepted(true);
                return true;
            }
            break;

        default:
            break;
    }

    return QQuickItem::event(e);
}

void K3DMaskedMouseArea::setPressed(bool pressed)
{
    if (_pressed != pressed) {
        _pressed = pressed;
        emit pressedChanged();
    }
}

void K3DMaskedMouseArea::setContainsMouse(bool containsMouse)
{
    if (m_containsMouse != containsMouse) {
        m_containsMouse = containsMouse;
        emit containsMouseChanged(m_containsMouse);
    }
}

Qt::MouseButtons K3DMaskedMouseArea::acceptedButtons() const
{
    return m_acceptedButtons;
}

void K3DMaskedMouseArea::setAcceptedButtons(Qt::MouseButtons buttons)
{
    if (buttons == m_acceptedButtons) {
        return;
    }

    m_acceptedButtons = buttons;
    emit acceptedButtonsChanged();
}

bool K3DMaskedMouseArea::hoverEnabled() const
{
    return acceptHoverEvents();
}

void K3DMaskedMouseArea::setHoverEnabled(bool enable)
{
    if (enable == acceptHoverEvents()) {
        return;
    }

    setAcceptHoverEvents(enable);
    emit hoverEnabledChanged(enable);
}

void K3DMaskedMouseArea::setMaskSize(const QSize& size)
{ 
    if (size.width() > 0 && size.height() > 0 && !m_maskImage.isNull()) {
        m_maskImage = m_maskImage.scaled(size);
        emit maskSizeChanged(size);
    }
}

const qreal ROTATION_THRESHOLD = 0.01; // In degrees
void K3DMaskedMouseArea::rotateMaskImage(qreal angle)
{
    if (ROTATION_THRESHOLD > abs(angle))
        return;

    QTransform rotating;
    rotating.rotate(angle);
    m_maskImage = m_maskImage.transformed(rotating); // Works
}

void K3DMaskedMouseArea::setMaskSource(const QUrl &source)
{
    if (m_maskSource != source) {
        m_maskSource = source;
        //QString imgPath = QQmlFile::urlToLocalFileOrQrc(source);
        QSize currentMaskSize = m_maskImage.size();
        m_maskImage = QImage(QQmlFile::urlToLocalFileOrQrc(source));
        rotateMaskImage(_rotationAngle);
        setMaskSize(currentMaskSize);
        
        emit maskSourceChanged();
    }
}

void K3DMaskedMouseArea::setRotationAngle(qreal angle)
{
    if (ROTATION_THRESHOLD > abs(angle - _rotationAngle))
        return;

    _rotationAngle = angle;
    
    if (m_maskSource.isEmpty())
        return;

    m_maskImage = QImage(QQmlFile::urlToLocalFileOrQrc(m_maskSource));
    rotateMaskImage(_rotationAngle);

    emit rotationAngleChanged(angle);
}

void K3DMaskedMouseArea::setAlphaThreshold(qreal threshold)
{
    if (m_alphaThreshold != threshold) {
        m_alphaThreshold = threshold;
        emit alphaThresholdChanged(threshold);
    }
}

//!NOTE: THIS FUNCTION MUST RETURN TRUE TO MAKE MOUSE/KEY EVENT FUNCTION BE INVOKED IN THE CORECT ITEM IN THE INHERITANCE-POLYMORPHISM TREE!
// 
bool K3DMaskedMouseArea::contains(const QPointF &point) const
{
    bool containsMouse = QQuickItem::contains(point);

    // Return either NOT containing mouse or Null Image
    if (!containsMouse || m_maskImage.isNull())
        return containsMouse;

    QPoint p = point.toPoint();

    if (p.x() < 0 || p.x() >= m_maskImage.width() ||
        p.y() < 0 || p.y() >= m_maskImage.height())
        return false;

    qreal r = qBound<int>(0, m_alphaThreshold * 255, 255);
    return qAlpha(m_maskImage.pixel(p)) > r;
}

void K3DMaskedMouseArea::mousePressEvent(QMouseEvent *event)
{
    setPressed(true);
    m_pressPoint = event->pos();
    m_buttonDownPos = event->screenPos();
    
    //QPointF localPos = event->localPos();
    //QPointF globalPos = mapToScene(m_pressPoint);

    // ====================================================================
    if (m_lastEvent == event || !(event->buttons() & m_acceptedButtons)) {
        event->setAccepted(false);
        return;
    }

    //FIXME: when a popup window is visible: a click anywhere hides it: but the old QQuickitem will continue to think it's under the mouse
    //doesn't seem to be any good way to properly reset this.
    //this solution will still caused a missed click after the popup is gone, but gets the situation unblocked.
    QPointF viewPosition;
    if (_widgetContainer && _isContainerTopLevel) {
        viewPosition = _widgetContainer->pos();

        //QRectF ret = QRectF(mapToScene(QPoint(0, 0)) + viewPosition, QSizeF(width(), height()));
        if (!QRectF(mapToScene(QPointF(0.0f, 0.0f)) + viewPosition, QSizeF(width(), height())).contains(event->screenPos())) {
            event->ignore();
            return;
        }
    }
    else if (window()) {
        // viewPosition = window()->position();
        // 
        QRectF rect = QRectF(QPointF(0.0f, 0.0f), QSizeF(width(), height()));
        if (!rect.contains(event->pos())) {
            event->ignore();
            return;
        }
    }

    K3DMouseEvent dme(event->pos().x(), event->pos().y(), event->screenPos().x(), event->screenPos().y(), 
                      event->button(), event->buttons(), event->modifiers(), screenForGlobalPos(event->globalPos()));
    if (!m_pressAndHoldEvent) {
        m_pressAndHoldEvent = new K3DMouseEvent(event->pos().x(), event->pos().y(), event->screenPos().x(), event->screenPos().y(), 
                                                event->button(), event->buttons(), event->modifiers(), screenForGlobalPos(event->globalPos()));
    }

    emit pressed(&dme);
    _pressed = true;
    emit pressedChanged();

#if (QT_VERSION >= QT_VERSION_CHECK(5, 3, 0))
    m_pressAndHoldTimer->start(QGuiApplication::styleHints()->mousePressAndHoldInterval());
#else
    m_pressAndHoldTimer->start(1000);
#endif
}

void K3DMaskedMouseArea::mouseReleaseEvent(QMouseEvent *event)
{
    if (m_lastEvent == event) {
        event->setAccepted(false);
        return;
    }

    K3DMouseEvent dme(event->pos().x(), event->pos().y(), event->screenPos().x(), event->screenPos().y(), 
                      event->button(), event->buttons(), event->modifiers(), screenForGlobalPos(event->globalPos()));
    _pressed = false;
    emit released(&dme);
    emit pressedChanged();
    
    if (boundingRect().contains(event->pos()) /*&& m_pressAndHoldTimer->isActive()*/) { // NO NEED FOR PRESSED AND HOLD TIMER TO EMIT CLICKED()
        emit clicked(&dme);
        m_pressAndHoldTimer->stop();
    }
}

void K3DMaskedMouseArea::mouseDoubleClickEvent(QMouseEvent *event)
{
    if (m_lastEvent == event || !(event->buttons() & m_acceptedButtons)) {
        event->setAccepted(false);
        return;
    }

    K3DMouseEvent dme(event->pos().x(), event->pos().y(), event->screenPos().x(), event->screenPos().y(),
                      event->button(), event->buttons(), event->modifiers(), screenForGlobalPos(event->globalPos()));

    if (boundingRect().contains(event->pos()) /*&& m_pressAndHoldTimer->isActive()*/) { // NO NEED FOR PRESSED AND HOLD TIMER TO EMIT CLICKED()
        emit doubleClicked(&dme);
        m_pressAndHoldTimer->stop();
    }
}

void K3DMaskedMouseArea::mouseMoveEvent(QMouseEvent *event)
{
    if (m_lastEvent == event || !(event->buttons() & m_acceptedButtons)) {
        event->setAccepted(false);
        return;
    }

    if (QPointF(event->screenPos() - m_buttonDownPos).manhattanLength() > QGuiApplication::styleHints()->startDragDistance() && m_pressAndHoldTimer->isActive()) {
        m_pressAndHoldTimer->stop();
    }

#ifdef _DEBUG
    QPointF pos = event->pos();
    QPointF screenPos = event->screenPos();
    QPointF globalPos = event->globalPos();
    QPointF localPos  = event->localPos();
    QPointF posGlobal = _widgetContainer ? _widgetContainer->mapToGlobal(event->pos()) : QPointF(0,0);
#endif

    QPointF widgetPos = _widgetContainer ? _widgetContainer->pos() : QPointF(0.0f,0.0f);
    QPointF screenPosDistance = event->screenPos() - m_buttonDownPos;
    QPointF posDistance       = event->pos() - m_pressPoint;
    widgetPos += posDistance;

    K3DMouseEvent dme(event->pos().x(), event->pos().y(), event->screenPos().x(), event->screenPos().y(),
                      event->button(), event->buttons(), event->modifiers(), screenForGlobalPos(event->globalPos()));

    // [WIDGET CONTAINER: MOVING/RESIZING] -- (Drag Target null means Widget Container is used)
    // 
    if (nullptr == _dragTarget || !_pressed) {
        //
        if (_pressed) { // [DragTarget: NULL]
            // SIGNAL [pressMoved(me)] --
            //
            emit pressedMoved(&dme); // !!! - To order _widgetContainer to turn to showNormal first then move!

            // Resizing Quick Container --
            // 
            if (!_widgetContainer)
                return;
            //
            bool isMoving = true;
            //
            int newWidth  = _widgetContainer->width()  - event->pos().x() + m_pressPoint.x();
            int newHeight = _widgetContainer->height() - event->pos().y() + m_pressPoint.y();
            if (_isResizingAgentRole) {
                if (m_cursorShape == Qt::SizeFDiagCursor || m_cursorShape == Qt::SizeBDiagCursor) {
                    _widgetContainer->resize(newWidth,
                                             newHeight);
                    isMoving = newWidth  < _widgetContainer->maximumWidth() ||
                               newHeight < _widgetContainer->maximumHeight();
                }
                else if (m_cursorShape == Qt::SizeHorCursor) {
                    _widgetContainer->resize(newWidth, _widgetContainer->height());
                    isMoving = newWidth < _widgetContainer->maximumWidth();
                }
                else if (m_cursorShape == Qt::SizeVerCursor) {
                    _widgetContainer->resize(_widgetContainer->width(), newHeight);
                    isMoving = newHeight < _widgetContainer->maximumHeight();
                }
            }

            // Moving Widget Container --
            // 
            if (isMoving &&
                _isMovingAgentRole) {
                if (_dragMaximumX > _dragMinimumX || _dragMaximumY > _dragMinimumY) {
                    QPointF dragPos;
                    dragPos.setX(qBound(_dragMinimumX,
                                        widgetPos.x(),
                                        _dragMaximumX));

                    dragPos.setY(qBound(_dragMinimumY,
                                        widgetPos.y(),
                                        _dragMaximumY));
                    _widgetContainer->move(dragPos.x(), dragPos.y());
                }
                else {
                    _widgetContainer->move(widgetPos.x(), widgetPos.y());
                }
            }
        } // if (_pressed)

        // Anyway: RETURN!
        return;
    }

    // [DRAG TARGET: QUICK ITEM]
    //
    if (!_pressed) {
        event->setAccepted(false);
        return;
    }
    emit pressedMoved(&dme);
    //
    QPointF targetPos = _dragTarget->position();
    targetPos += posDistance;
    QPointF dragPos;
    
    // [RESIZING DRAG TARGET] --
    //
    if (_isResizingAgentRole) {
        int newWidth  = _dragTarget->width()  - event->pos().x() + m_pressPoint.x();
        int newHeight = _dragTarget->height() - event->pos().y() + m_pressPoint.y();
        if ((newWidth  < _resizeTargetMinWidth  || newWidth  > _resizeTargetMaxWidth) ||
            (newHeight < _resizeTargetMinHeight || newHeight > _resizeTargetMaxHeight)) {
            return;
        }
        if (m_cursorShape == Qt::SizeFDiagCursor || m_cursorShape == Qt::SizeBDiagCursor) {
            _dragTarget->setSize(QSizeF(newWidth, newHeight));
        }
        else if (m_cursorShape == Qt::SizeHorCursor) {
            _dragTarget->setSize(QSizeF(newWidth, _dragTarget->height()));
        }
        else if (m_cursorShape == Qt::SizeVerCursor) {
            _dragTarget->setSize(QSizeF(_dragTarget->width(), newHeight));
        }
        
        // RESIZING ALSO MEANS MOVING AS WELL!!
        // 
        _isMovingAgentRole = true;
    }
    
    // [MOVING DRAG TARGET] --
    // 
    if (_isMovingAgentRole) {
        dragPos.setX(qBound(_dragMinimumX,
                            targetPos.x(),
                            _dragMaximumX));

        dragPos.setY(qBound(_dragMinimumY,
                            targetPos.y(),
                            _dragMaximumY));

        static QPointF lastDragPos(QDesktopWidget().availableGeometry().center());
        //
        // --------------------------------------------------------------------------------------------------
        // ! NOTE: This helps detect collision between _dragTarget(as a circle only) with the Ref Rect:
        // 
        bool isRejected = false;

        // IF REFT RECTANGLE VISIBLE ONLY:
        if (!_isModal && _dragReftRectVisible) {
            bool isXInside = (dragPos.x() >= _dragRefRectX - 1 - _offsetMaxX &&
                              dragPos.x() <= _dragRefRectX + _dragRefRectWidth  + _offsetMinX);
            
            bool isYInside = (dragPos.y() >= _dragRefRectY - 1 - _offsetMaxY &&
                              dragPos.y() <= _dragRefRectY + _dragRefRectHeight + _offsetMinY);

            // [X,Y inside]
            if (isXInside && isYInside) {
                isRejected = true;
            }

            // [X inside only]
            else if (isXInside) {
                qreal yDistance = qMin(qAbs(dragPos.y() + _dragTarget->height() / 2 - _dragRefRectY),
                                       qAbs(dragPos.y() + _dragTarget->height() / 2 - _dragRefRectY - _dragRefRectHeight));
                
                isRejected      = yDistance < _dragTarget->height() / 2 + _offsetMinY;
            }

            // [Y inside only]
            else if (isYInside) {
                qreal xDistance = qMin(qAbs(dragPos.x() + _dragTarget->width() / 2 - _dragRefRectX),
                                       qAbs(dragPos.x() + _dragTarget->width() / 2 - _dragRefRectX - _dragRefRectWidth));
                
                isRejected      = xDistance < _dragTarget->width() / 2  + _offsetMinX;
            }

            // [X, Y outside]
            else {
                qreal d1, d2, d3, d4;

                d1 = qPow(dragPos.x() + _dragTarget->width()  / 2 - _dragRefRectX, 2) +
                     qPow(dragPos.y() + _dragTarget->height() / 2 - _dragRefRectY, 2);

                d2 = qPow(dragPos.x() + _dragTarget->width()  / 2 - _dragRefRectX - _dragRefRectWidth, 2) +
                     qPow(dragPos.y() + _dragTarget->height() / 2 - _dragRefRectY, 2);

                d3 = qPow(dragPos.x() + _dragTarget->width()  / 2 - _dragRefRectX, 2) +
                     qPow(dragPos.y() + _dragTarget->height() / 2 - _dragRefRectY - _dragRefRectHeight, 2);

                d4 = qPow(dragPos.x() + _dragTarget->width()  / 2 - _dragRefRectX - _dragRefRectWidth, 2) +
                     qPow(dragPos.y() + _dragTarget->height() / 2 - _dragRefRectY - _dragRefRectHeight, 2);

                isRejected = qRound(qMin(qMin(qMin(d1, d2), d3), d4)) < qRound(qPow(_dragTarget->width() / 2, 2));
            }
        }
        else {
            isRejected = false;
        }
        // --------------------------------------------------------------------------------------------------
        //
        if (!isRejected) {
            lastDragPos = dragPos;
        }

        _dragTarget->setPosition(lastDragPos);
        emit positionChanged(&dme);
    } // if(_isMovingAgentRole)
}

void K3DMaskedMouseArea::wheelEvent(QWheelEvent *we)
{
    if (m_lastEvent == we) {
        return;
    }

    K3DWheelEvent dwe(we->pos(), we->globalPos(), we->angleDelta(), we->delta(), 
                      we->buttons(), we->modifiers(), we->orientation());
    emit wheel(&dwe);
}

void K3DMaskedMouseArea::handlePressAndHold()
{
    if (_pressed) {
        emit pressAndHold(m_pressAndHoldEvent);

        delete m_pressAndHoldEvent;
        m_pressAndHoldEvent = 0;
    }
}

void K3DMaskedMouseArea::hoverEnterEvent(QHoverEvent *event)
{
    Q_UNUSED(event);
    setContainsMouse(true);
    emit entered();
}

void K3DMaskedMouseArea::hoverLeaveEvent(QHoverEvent *event)
{
    Q_UNUSED(event);
    setContainsMouse(false);
    emit exited();
}

void K3DMaskedMouseArea::hoverMoveEvent(QHoverEvent * event)
{
    if (m_lastEvent == event) {
        return;
    }

    QQuickWindow *w = window();
    QPoint screenPos;
    if (w) {
        screenPos = w->mapToGlobal(event->pos());
    }

    K3DMouseEvent dme(event->pos().x(), event->pos().y(), screenPos.x(), screenPos.y(), Qt::NoButton, Qt::NoButton, event->modifiers(), 0);
    emit positionChanged(&dme);
}

bool K3DMaskedMouseArea::childMouseEventFilter(QQuickItem *item, QEvent *event)
{
    if (!isEnabled()) {
        return false;
    }

    //don't filter other mouseeventlisteners
    if (qobject_cast<K3DMaskedMouseArea *>(item)) {
        return false;
    }

    switch (event->type()) {
    case QEvent::MouseButtonPress: {
        m_lastEvent = event;
        QMouseEvent *me = static_cast<QMouseEvent *>(event);

        if (!(me->buttons() & m_acceptedButtons)) {
            break;
        }

        //the parent will receive events in its own coordinates
        const QPointF myPos = item->mapToItem(this, me->pos());

        K3DMouseEvent dme(myPos.x(), myPos.y(), me->screenPos().x(), me->screenPos().y(), me->button(), me->buttons(), me->modifiers(), screenForGlobalPos(me->globalPos()));
        delete m_pressAndHoldEvent;
        m_pressAndHoldEvent = new K3DMouseEvent(myPos.x(), myPos.y(), me->screenPos().x(), me->screenPos().y(), me->button(), me->buttons(), me->modifiers(), screenForGlobalPos(me->globalPos()));

        //qDebug() << "pressed in sceneEventFilter";
        m_buttonDownPos = me->screenPos();
        emit pressed(&dme);
        _pressed = true;
        emit pressedChanged();

#if (QT_VERSION >= QT_VERSION_CHECK(5, 3, 0))
        m_pressAndHoldTimer->start(QGuiApplication::styleHints()->mousePressAndHoldInterval());
#else
        m_pressAndHoldTimer->start(1000);
#endif
        break;
    }
    case QEvent::HoverMove: {
        if (!acceptHoverEvents()) {
            break;
        }
        m_lastEvent = event;
        QHoverEvent *he = static_cast<QHoverEvent *>(event);
        const QPointF myPos = item->mapToItem(this, he->pos());

        QQuickWindow *w = window();
        QPoint screenPos;
        if (w) {
            screenPos = w->mapToGlobal(myPos.toPoint());
        }

        K3DMouseEvent dme(myPos.x(), myPos.y(), screenPos.x(), screenPos.y(), Qt::NoButton, Qt::NoButton, he->modifiers(), 0);
        //qDebug() << "positionChanged..." << dme.x() << dme.y();
        //emit positionChanged(&dme);
        break;
    }
    case QEvent::MouseMove: {
        m_lastEvent = event;
        QMouseEvent *me = static_cast<QMouseEvent *>(event);
        if (!(me->buttons() & m_acceptedButtons)) {
            break;
        }

        const QPointF myPos = item->mapToItem(this, me->pos());
        K3DMouseEvent dme(myPos.x(), myPos.y(), me->screenPos().x(), me->screenPos().y(), me->button(), me->buttons(), me->modifiers(), screenForGlobalPos(me->globalPos()));
        //qDebug() << "positionChanged..." << dme.x() << dme.y();

        //stop the pressandhold if mouse moved enough
        if (QPointF(me->screenPos() - m_buttonDownPos).manhattanLength() > QGuiApplication::styleHints()->startDragDistance() && m_pressAndHoldTimer->isActive()) {
            m_pressAndHoldTimer->stop();

        //if the mouse moves and we are waiting to emit a press and hold event, update the co-ordinates
        //as there is no update function, delete the old event and create a new one
        } else if (m_pressAndHoldEvent) {
            delete m_pressAndHoldEvent;
            m_pressAndHoldEvent = new K3DMouseEvent(myPos.x(), myPos.y(), me->screenPos().x(), me->screenPos().y(), me->button(), me->buttons(), me->modifiers(), screenForGlobalPos(me->globalPos()));
        }
        emit positionChanged(&dme);
        break;
    }
    case QEvent::MouseButtonRelease: {
        m_lastEvent = event;
        QMouseEvent *me = static_cast<QMouseEvent *>(event);

        const QPointF myPos = item->mapToItem(this, me->pos());
        K3DMouseEvent dme(myPos.x(), myPos.y(), me->screenPos().x(), me->screenPos().y(), me->button(), me->buttons(), me->modifiers(), screenForGlobalPos(me->globalPos()));
        _pressed = false;

        emit released(&dme);
        emit pressedChanged();

        if (QPointF(me->screenPos() - m_buttonDownPos).manhattanLength() <= QGuiApplication::styleHints()->startDragDistance() && m_pressAndHoldTimer->isActive()) {
            emit clicked(&dme);
            m_pressAndHoldTimer->stop();
        }
        break;
    }

    case QEvent::MouseButtonDblClick: {
        break;
    }

    case QEvent::Wheel: {
        m_lastEvent = event;
        QWheelEvent *we = static_cast<QWheelEvent *>(event);
        K3DWheelEvent dwe(we->pos(), we->globalPos(), we->angleDelta(), we->delta(), we->buttons(), we->modifiers(), we->orientation());
        emit wheel(&dwe);
        break;
    }
    default:
        break;
    }

    return QQuickItem::childMouseEventFilter(item, event);
//    return false;
}

QScreen* K3DMaskedMouseArea::screenForGlobalPos(const QPoint& globalPos)
{
    foreach(QScreen *screen, QGuiApplication::screens()) {
        if (screen->geometry().contains(globalPos)) {
            return screen;
        }
    }
    return 0;
}

void K3DMaskedMouseArea::mouseUngrabEvent()
{
    handleUngrab();

    QQuickItem::mouseUngrabEvent();
}

void K3DMaskedMouseArea::touchUngrabEvent()
{
    handleUngrab();

    QQuickItem::touchUngrabEvent();
}

void K3DMaskedMouseArea::handleUngrab()
{
    m_pressAndHoldTimer->stop();

    setPressed(false);
    emit canceled();
}

Qt::CursorShape K3DMaskedMouseArea::cursorShape() const
{
    return (Qt::CursorShape)m_cursorShape;
}

QString K3DMaskedMouseArea::cursorShapeSource() const
{
    return m_currentShapeSource;
}

void K3DMaskedMouseArea::setCursorShape(Qt::CursorShape cursorShape)
{
    if (m_cursorShape == (int)cursorShape)
        return;

    QCursor cursor(cursorShape);

    setCursor(cursor);

    // [1]
    emit cursorShapeChanged(cursorShape);

    // [2]
    m_cursorShape = cursorShape;
}

void K3DMaskedMouseArea::setCursorShape(QString cursorShapeSource)
{
    if (m_currentShapeSource.compare(cursorShapeSource, Qt::CaseSensitive) == 0)
        return;
    QCursor cursor(cursorShapeSource);

    setCursor(cursor);

    // [1]
    emit cursorShapeSourceChanged(cursorShapeSource);

    // [2]
    m_currentShapeSource = cursorShapeSource;
}
