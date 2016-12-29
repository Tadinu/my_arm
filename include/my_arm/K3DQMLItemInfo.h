#ifndef K3D_QML_ITEM_INFO_H
#define K3D_QML_ITEM_INFO_H

#include <assert.h>
#include <QObject>
#include <QKeyEvent>
#include <QtQml>
#include <QQmlContext>
#include <QQmlListProperty>

class K3DQMLItemInfo : public QObject
{
    Q_OBJECT
        Q_PROPERTY(QString text                   READ text                      WRITE setText                   NOTIFY textChanged            )
        Q_PROPERTY(QString tooltip                READ tooltip                   WRITE setText                   NOTIFY tooltipChanged         )
        Q_PROPERTY(int     opId                   READ opId                      WRITE setOpId                   NOTIFY opIdChanged            )
        Q_PROPERTY(int     itemId                 READ itemId                    WRITE setItemId                 NOTIFY itemIdChanged          )
        Q_PROPERTY(bool    checked                READ checked                   WRITE setChecked                NOTIFY toggled                )
                                                                                                                 
        Q_PROPERTY(bool    visible                READ visible                   WRITE setVisible                NOTIFY visibleChanged         )
        Q_PROPERTY(bool    enabled                READ enabled                   WRITE setEnabled                NOTIFY enabledChanged         )
        Q_PROPERTY(bool    clicked                READ clicked                   WRITE setClicked                NOTIFY clickedChanged         )

        Q_PROPERTY(QString productName            READ productName               WRITE setProductName            NOTIFY productNameChanged     )
        Q_PROPERTY(QString vendorType             READ vendorType                WRITE setVendorType             NOTIFY vendorTypeChanged      )
        Q_PROPERTY(QString owner                  READ owner                     WRITE setOwner                  NOTIFY ownerChanged           )
        Q_PROPERTY(QString userLevel              READ userLevel                 WRITE setUserLevel              NOTIFY userLevelChanged       )
        Q_PROPERTY(QString registerDate           READ registerDate              WRITE setRegisterDate           NOTIFY registerDateChanged    )
        Q_PROPERTY(QString expireDate             READ expireDate                WRITE setExpireDate             NOTIFY expireDateChanged      )

        Q_PROPERTY(int     extendedListType MEMBER _extendedListType)
        //Q_PROPERTY(QQmlListProperty<K3DQMLItemInfo> extendedList READ extendedList)
public:
    K3DQMLItemInfo(QObject* parent = nullptr) : QObject(parent) {
        initialize();
    }

    K3DQMLItemInfo(int itemId,
        QObject* parent = nullptr) : QObject(parent) {
        initialize();

        _itemId = itemId;
    }

    K3DQMLItemInfo(const QString& text, int opId,
        QObject* parent = nullptr) : QObject(parent) {
        initialize();

        _text = text;
        _opId = opId;
    }

    K3DQMLItemInfo(const QString& text, int itemId, 
                   int opId, bool checked,
                   bool visible, bool enabled,
                   QObject* parent = nullptr) : QObject(parent),
                   _text(text),
                   _tooltip(""),
                   _opId(opId),
                   _itemId(itemId),
                   _checked(checked),
                   _visible(visible),

                   _enabled(enabled) 
    {
    }

    void initialize() {
        _text    = "";
        _tooltip = "";
        _opId = -1;
        _itemId = -1;
        _checked = false;
        _visible = true;
        _enabled = true;
        _clicked = false;
        _productName   = "" ;
        _vendorType    = "" ;
        _owner         = "" ;
        _userLevel     = "" ;
        _registerDate  = "" ;
        _expireDate    = "" ;
    }

    /*
    // extendedList --
    QQmlListProperty<K3DQMLItemInfo> extendedList() {
        return QQmlListProperty<K3DQMLItemInfo>(this, _extendedList.toList());
        //return QQmlListProperty<K3DQMLItemInfo>(this, 0, &K3DQMLItemInfo::append_item);
    }
    */

    static void append_item(QQmlListProperty<K3DQMLItemInfo> *list, K3DQMLItemInfo *item)
    {
        K3DQMLItemInfo *itemList = qobject_cast<K3DQMLItemInfo *>(list->object);
        if (item)
            itemList->_extendedList.append(item);
    }

    // text --
    QString text() { return _text; }

    void setText(const QString&  text) {
        if (_text != text) {
            _text = text;
            emit textChanged(_text);
        }
    }

    // tooltip --
    QString tooltip() { return _tooltip; }

    void setTooltip(const QString&  tooltip) {
        if (_tooltip != tooltip) {
            _tooltip = tooltip;
            emit tooltipChanged(_tooltip);
        }
    }

    // opId --
    int opId()     { return _opId; }
    void setOpId(int opId) {
        if (_opId != opId) {
            _opId = opId;
            emit opIdChanged(_opId);
        }
    }

    // itemId --
    int itemId()     { return _itemId; }
    void setItemId(int itemId) {
        if (_itemId != itemId) {
            _itemId = itemId;
            emit itemIdChanged(_itemId);
        }
    }

    // enabled --
    bool enabled()     { return _enabled; }
    void setEnabled(bool enabled) {
        // To make QML buttons Enabled state updated from C++ calls.
        _enabled = enabled;
        emit enabledChanged(_enabled);
    }

    // visible --
    bool visible()     { return _visible; }
    void setVisible(bool visible) {
        // To make QML buttons Visible state updated from C++ calls.
        _visible = visible;
        emit visibleChanged(_visible);
    }

    // checked --
    bool checked()     { return _checked; }
    void setChecked(bool checked) {
        // To make QML buttons Checked state updated from C++ calls.
        _checked = checked;
        emit toggled(_checked);
    }

    // clicked --
    bool clicked()     { return _clicked; }
    void setClicked(bool clicked) {
        if (_clicked != clicked) {
            _clicked = clicked;
            emit clickedChanged(_clicked);
        }
    }
    void invertClicked() { 
        _clicked = !_clicked; 
        emit clickedChanged(_clicked);
    }

    // Just for NyoLicense
    QString productName()  { return _productName;  }
    void setProductName(QString productName) {
        if (_productName != productName) {
            _productName = productName;
            emit productNameChanged(_productName);
        }
    }

    QString vendorType()   { return _vendorType;   }
    void setVendorType(QString vendorType) {
        if (_vendorType != vendorType) {
            _vendorType = vendorType;
            emit vendorTypeChanged(_vendorType);
        }
    }

    QString owner()        { return _owner;        }
    void setOwner(QString owner) {
        if (_owner != owner) {
            _owner = owner;
            emit ownerChanged(_owner);
        }
    }

    QString userLevel()    { return _userLevel;    }
    void setUserLevel(QString userLevel) {
        if (_userLevel != userLevel) {
            _userLevel = userLevel;
            emit userLevelChanged(_userLevel);
        }
    }

    QString registerDate() { return _registerDate; }
    void setRegisterDate(QString registerDate) {
        if (_registerDate != registerDate) {
            _registerDate = registerDate;
            emit registerDateChanged(_registerDate);
        }
    }

    QString expireDate()   { return _expireDate;   }
    void setExpireDate(QString expireDate) {
        if (_expireDate != expireDate) {
            _expireDate = expireDate;
            emit expireDateChanged(_expireDate);
        }
    }

signals:
    void textChanged(QString text);
    void tooltipChanged(QString tooltip);
    void opIdChanged(int opId);
    void itemIdChanged(int itemId);
    void toggled(bool checked);
    void visibleChanged(bool visible);
    void enabledChanged(bool enabled);
    void clickedChanged(bool clicked);

    void productNameChanged(QString productName);
    void vendorTypeChanged(QString vendorType);
    void ownerChanged(QString owner);
    void userLevelChanged(QString userLevel);
    void registerDateChanged(QString registerDate);
    void expireDateChanged(QString expireDate);

private:
    QString _text;
    QString _tooltip;
    int  _opId;
    int  _itemId;
    bool _checked;

    bool _enabled;
    bool _visible;
    bool _clicked;

    QString _productName  ;
    QString _vendorType   ;
    QString _owner        ;
    QString _userLevel    ;
    QString _registerDate ;
    QString _expireDate   ;


    QVector<K3DQMLItemInfo *> _extendedList;
    int  _extendedListType;

    /*
    int _listIndex;
    bool _operationNeedIndex;

    int  _subOpId;
    bool _visible;
    bool _enabled;
    int  _btnType;
    QString _tooltip;
    int _tooltipPos;
    QString _textColor;
    int _textPixelSize;
    int _textPosition;
    int _textRotation;
    int _textVerticalAlignment;
    int _textHorizontalAlignment;
    QString _normalSource;
    QString _hoveredSource;
    QString _pressedSource;
    QString _functionBtnSourceNormal;
    QString _functionBtnSourceHover;

    bool _isExtended;
    int _extendedDirection;
    */
};
#endif // K3D_QML_ITEM_INFO_H


