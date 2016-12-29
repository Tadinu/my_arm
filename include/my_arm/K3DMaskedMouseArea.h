/*
REFERENCE:
http://api.kde.org/frameworks-api/frameworks5-apidocs/kdeclarative/html/mouseeventlistener_8h_source.html
http://api.kde.org/frameworks-api/frameworks5-apidocs/kdeclarative/html/mouseeventlistener_8cpp_source.html
*/

#ifndef MASKEDMOUSEAREA_H
#define MASKEDMOUSEAREA_H

#include <QImage>
#include <QQuickItem>
#include <QGuiApplication>

// Signaled to QML only
// 
class K3DMouseEvent : public QObject
{
    Q_OBJECT
    Q_PROPERTY(int x READ x)
    Q_PROPERTY(int y READ y)
    Q_PROPERTY(int screenX READ screenX)
    Q_PROPERTY(int screenY READ screenY)
    Q_PROPERTY(int button READ button)
    Q_PROPERTY(Qt::MouseButtons buttons READ buttons)
    Q_PROPERTY(Qt::KeyboardModifiers modifiers READ modifiers)
    Q_PROPERTY(QScreen* screen READ screen CONSTANT)

public:
    K3DMouseEvent(int x, int y, int screenX, int screenY,
        Qt::MouseButton button,
        Qt::MouseButtons buttons,
        Qt::KeyboardModifiers modifiers,
        QScreen* screen)
        : m_x(x),
          m_y(y),
          m_screenX(screenX),
          m_screenY(screenY),
          m_button(button),
          m_buttons(buttons),
          m_modifiers(modifiers),
          m_screen(screen)
    {}

    int x() const { return m_x; }
    int y() const { return m_y; }
    int screenX() const { return m_screenX; }
    int screenY() const { return m_screenY; }
    int button() const { return m_button; }
    Qt::MouseButtons buttons() const { return m_buttons; }
    Qt::KeyboardModifiers modifiers() const { return m_modifiers; }
    QScreen* screen() const { return m_screen; }

    // only for internal usage
    void setX(int x) { m_x = x; }
    void setY(int y) { m_y = y; }

private:
    int m_x;
    int m_y;
    int m_screenX;
    int m_screenY;
    Qt::MouseButton m_button;
    Qt::MouseButtons m_buttons;
    Qt::KeyboardModifiers m_modifiers;
    QScreen *m_screen;
};

// Signaled to QML only
// 
class K3DWheelEvent : public QObject
{
    Q_OBJECT
    Q_PROPERTY(int x READ x CONSTANT)
    Q_PROPERTY(int y READ y CONSTANT)
    Q_PROPERTY(int screenX READ screenX CONSTANT)
    Q_PROPERTY(int screenY READ screenY CONSTANT)
    Q_PROPERTY(int delta READ delta CONSTANT)
    Q_PROPERTY(QPoint angleDelta READ angleDelta CONSTANT)
    Q_PROPERTY(Qt::MouseButtons buttons READ buttons CONSTANT)
    Q_PROPERTY(Qt::KeyboardModifiers modifiers READ modifiers CONSTANT)
    Q_PROPERTY(Qt::Orientation orientation READ orientation CONSTANT)

public:
    K3DWheelEvent(QPointF pos, QPoint screenPos, QPoint angleDeta, int delta,
                  Qt::MouseButtons buttons,
                  Qt::KeyboardModifiers modifiers,
                  Qt::Orientation orientation)
        : m_x(pos.x()),
          m_y(pos.y()),
          m_screenX(screenPos.x()),
          m_screenY(screenPos.y()),
          m_angleDelta(angleDeta),
          m_delta(delta),
          m_buttons(buttons),
          m_modifiers(modifiers),
          m_orientation(orientation)
    {}

    int x() const { return m_x; }
    int y() const { return m_y; }
    int screenX() const { return m_screenX; }
    int screenY() const { return m_screenY; }
    int delta() const { return m_delta; }
    QPoint angleDelta() const { return m_angleDelta; }
    Qt::MouseButtons buttons() const { return m_buttons; }
    Qt::KeyboardModifiers modifiers() const { return m_modifiers; }
    Qt::Orientation orientation() { return m_orientation; }

    // only for internal usage
    void setX(int x) { m_x = x; }
    void setY(int y) { m_y = y; }

private:
    int m_x;
    int m_y;
    int m_screenX;
    int m_screenY;
    int m_delta;
    QPoint m_angleDelta;
    Qt::MouseButtons m_buttons;
    Qt::KeyboardModifiers m_modifiers;
    Qt::Orientation m_orientation;
};

class K3DMaskedMouseArea : public QQuickItem
{
    Q_OBJECT
    Q_PROPERTY(QSize maskSize READ maskSize WRITE setMaskSize NOTIFY maskSizeChanged)
    Q_PROPERTY(bool isMovingAgentRole MEMBER _isMovingAgentRole)
    Q_PROPERTY(bool isResizingAgentRole MEMBER _isResizingAgentRole)
    
    Q_PROPERTY(qreal resizeTargetMinWidth  MEMBER _resizeTargetMinWidth  NOTIFY resizeTargetMinWidthChanged)
    Q_PROPERTY(qreal resizeTargetMinHeight MEMBER _resizeTargetMinHeight NOTIFY resizeTargetMinHeightChanged)
    Q_PROPERTY(qreal resizeTargetMaxWidth  MEMBER _resizeTargetMaxWidth  NOTIFY resizeTargetMaxWidthChanged)
    Q_PROPERTY(qreal resizeTargetMaxHeight MEMBER _resizeTargetMaxHeight NOTIFY resizeTargetMaxHeightChanged)
    
    //Q_PROPERTY(bool isContainterTopLevel MEMBER _isContainerTopLevel)
    Q_PROPERTY(bool isModal MEMBER _isModal)

    Q_PROPERTY(bool pressed READ isPressed NOTIFY pressedChanged)
    Q_PROPERTY(bool hoverEnabled READ hoverEnabled WRITE setHoverEnabled NOTIFY hoverEnabledChanged)

    Q_PROPERTY(bool propagateComposedEvents MEMBER _propagateComposedEvents)
    Q_PROPERTY(bool containsMouse READ containsMouse NOTIFY containsMouseChanged)
    Q_PROPERTY(QUrl maskSource READ maskSource WRITE setMaskSource NOTIFY maskSourceChanged)
    Q_PROPERTY(qreal alphaThreshold READ alphaThreshold WRITE setAlphaThreshold NOTIFY alphaThresholdChanged)
    Q_PROPERTY(qreal rotationAngle READ rotationAngle WRITE setRotationAngle NOTIFY rotationAngleChanged)

    Q_PROPERTY(Qt::MouseButtons acceptedButtons READ acceptedButtons WRITE setAcceptedButtons NOTIFY acceptedButtonsChanged)

    // Cursor --
    //
    Q_PROPERTY(Qt::CursorShape cursorShape READ cursorShape WRITE setCursorShape NOTIFY cursorShapeChanged)
    Q_PROPERTY(QString   cursorShapeSource READ cursorShapeSource WRITE setCursorShape NOTIFY cursorShapeSourceChanged)

    // Dragging --
    //
    Q_PROPERTY(QQuickItem* dragTarget READ dragTarget WRITE setDragTarget NOTIFY dragTargetChanged)
    Q_PROPERTY(qreal dragMinimumX MEMBER _dragMinimumX NOTIFY dragMinimumXChanged)
    Q_PROPERTY(qreal dragMinimumY MEMBER _dragMinimumY NOTIFY dragMinimumYChanged)
    Q_PROPERTY(qreal dragMaximumX MEMBER _dragMaximumX NOTIFY dragMaximumXChanged)
    Q_PROPERTY(qreal dragMaximumY MEMBER _dragMaximumY NOTIFY dragMaximumYChanged)

    Q_PROPERTY(qreal offsetMinX MEMBER _offsetMinX NOTIFY offsetMinXChanged)
    Q_PROPERTY(qreal offsetMaxX MEMBER _offsetMaxX NOTIFY offsetMaxXChanged)
    Q_PROPERTY(qreal offsetMinY MEMBER _offsetMinY NOTIFY offsetMinYChanged)
    Q_PROPERTY(qreal offsetMaxY MEMBER _offsetMaxY NOTIFY offsetMaxYChanged)

    Q_PROPERTY(bool  dragReftRectVisible MEMBER _dragReftRectVisible)
    Q_PROPERTY(qreal dragRefRectX      MEMBER _dragRefRectX      NOTIFY dragReftRectXChanged)
    Q_PROPERTY(qreal dragRefRectY      MEMBER _dragRefRectY      NOTIFY dragReftRectYChanged)
    Q_PROPERTY(qreal dragRefRectWidth  MEMBER _dragRefRectWidth  NOTIFY dragReftRectWidthChanged)
    Q_PROPERTY(qreal dragRefRectHeight MEMBER _dragRefRectHeight NOTIFY dragReftRectHeightChanged)

public:
    K3DMaskedMouseArea(QQuickItem *parent = 0);

    bool contains(const QPointF &point) const;

    bool isPressed() const { return _pressed; }
    bool containsMouse() const { return m_containsMouse; }

    QSize maskSize() const { return m_maskImage.size(); }
    void setMaskSize(const QSize& size);

    QUrl maskSource() const { return m_maskSource; }
    void setMaskSource(const QUrl &source);

    qreal alphaThreshold() const { return m_alphaThreshold; }
    void setAlphaThreshold(qreal threshold);

    qreal rotationAngle() const { return _rotationAngle; }
    void setRotationAngle(qreal angle);

    QQuickItem* dragTarget() const { return _dragTarget; }

    Qt::MouseButtons acceptedButtons() const;
    void setAcceptedButtons(Qt::MouseButtons buttons);

    bool hoverEnabled() const;
    void setHoverEnabled(bool enable);

    Qt::CursorShape cursorShape() const;
    QString cursorShapeSource() const;
    Q_INVOKABLE void setCursorShape(Qt::CursorShape cursorShape);
    Q_INVOKABLE void setCursorShape(QString cursorShapeSource);

    Q_INVOKABLE void setQuickContainer(QWidget* widget, bool isTopLevel);

public slots:
    void setDragTarget(QQuickItem* target) {
        if (_dragTarget == target)
            return;
        //ungrabMouse(_dragTarget);
        //grabMouse(target);
        _dragTarget = target;
        emit dragTargetChanged();
    }

signals:
    void pressed(K3DMouseEvent *mouse);
    void released(K3DMouseEvent *mouse);
    void clicked(K3DMouseEvent *mouse);
    void doubleClicked(K3DMouseEvent *mouse);
    void positionChanged(K3DMouseEvent* mouse);
    void pressedMoved(K3DMouseEvent* mouse);
    void pressAndHold(K3DMouseEvent* mouse);
    void wheel(K3DWheelEvent *wheel);
    void entered();
    void exited();
    void canceled();
    void pressedChanged();
    void maskSizeChanged(const QSize&);
    void maskSourceChanged();
    void containsMouseChanged(bool containsMouseChanged);
    void hoverEnabledChanged(bool hoverEnabled);
    void acceptedButtonsChanged();

    void alphaThresholdChanged(qreal);
    void rotationAngleChanged(qreal);
    void dragTargetChanged();

    void cursorShapeChanged(int);
    void cursorShapeSourceChanged(QString);


    void resizeTargetMinWidthChanged(qreal);
    void resizeTargetMinHeightChanged(qreal);
    void resizeTargetMaxWidthChanged(qreal);
    void resizeTargetMaxHeightChanged(qreal);

    void offsetMinXChanged(qreal);
    void offsetMaxXChanged(qreal);
    void offsetMinYChanged(qreal);
    void offsetMaxYChanged(qreal);

    void dragMinimumXChanged(qreal);
    void dragMinimumYChanged(qreal);
    void dragMaximumXChanged(qreal);
    void dragMaximumYChanged(qreal);

    void dragReftRectXChanged(qreal);
    void dragReftRectYChanged(qreal);
    void dragReftRectWidthChanged(qreal);
    void dragReftRectHeightChanged(qreal);

protected:
    virtual bool event(QEvent * ev);
    virtual void wheelEvent(QWheelEvent * wheelEvent);
    virtual void mousePressEvent(QMouseEvent *event);
    virtual void mouseReleaseEvent(QMouseEvent *event);
    virtual void mouseDoubleClickEvent(QMouseEvent *event);
    virtual void mouseMoveEvent(QMouseEvent * event);
    virtual void hoverEnterEvent(QHoverEvent *event);
    virtual void hoverLeaveEvent(QHoverEvent *event);
    virtual void hoverMoveEvent(QHoverEvent * event);

    bool childMouseEventFilter(QQuickItem *item, QEvent *event);
    virtual void mouseUngrabEvent();
    virtual void touchUngrabEvent();

    void setPressed(bool pressed);
    void setContainsMouse(bool containsMouse);

    static void grabMouse(QQuickItem* target)
    {
        if (target)
        {
            target->grabMouse();
            QMouseEvent event(QEvent::MouseButtonPress, QPointF(), Qt::LeftButton, QGuiApplication::mouseButtons(), QGuiApplication::keyboardModifiers());
            QGuiApplication::sendEvent(target, &event);
        }
    }

    static void ungrabMouse(QQuickItem* target)
    {
        if (target)
            target->ungrabMouse();
    }

    void rotateMaskImage(qreal angle);
    qreal _resizeTargetMinWidth;
    qreal _resizeTargetMinHeight;
    qreal _resizeTargetMaxWidth;
    qreal _resizeTargetMaxHeight;

private slots:
    void handlePressAndHold();
    void handleUngrab();
   
private:
    bool _pressed;
    K3DMouseEvent* m_pressAndHoldEvent;
    bool _propagateComposedEvents;
    QUrl m_maskSource;
    QImage m_maskImage;
    QPointF m_pressPoint;
    qreal m_alphaThreshold;
    qreal _rotationAngle;
    bool m_containsMouse;

    // Container --
    // 
    QWidget* _widgetContainer;
    bool _isContainerTopLevel;
    bool _isMovingAgentRole; // Make Quick Container a Widget dragged Item
    bool _isResizingAgentRole;



    // Cursor --
    // 
    int m_cursorShape;
    QString m_currentShapeSource;

    // Dragging --
    // 
    QQuickItem* _dragTarget; // Quick Item dragged Target
    qreal _dragMinimumX;
    qreal _dragMinimumY;
    qreal _dragMaximumX;
    qreal _dragMaximumY;

    qreal _offsetMinX;
    qreal _offsetMaxX;
    qreal _offsetMinY;
    qreal _offsetMaxY;

    // Reft Rect to which the dragged item evades collision
    bool _dragReftRectVisible;
    qreal _dragRefRectX;
    qreal _dragRefRectY;
    qreal _dragRefRectWidth;
    qreal _dragRefRectHeight;

    static QScreen* screenForGlobalPos(const QPoint &globalPos);

    QPointF m_buttonDownPos;
    //Important: used only for comparison. If you will ever need to access this pointer, make it a QWeakPointer
    QEvent *m_lastEvent;
    QTimer *m_pressAndHoldTimer;
    Qt::MouseButtons m_acceptedButtons;

    bool _isModal;
};

#endif
