#ifndef QML_ITEM_AGENT_H
#define QML_ITEM_AGENT_H

#include <assert.h>
#include <QObject>
#include <QKeyEvent>

#define RB_DIALOG_PP(dialogP) reinterpret_cast<QMLItemAgent**>(&dialogP)

class QMLItemAgent : public QObject
{
    Q_OBJECT

public:
    explicit QMLItemAgent(QObject *parent = 0, bool isBusy = false, 
                             int itemId = -1, QObject* itemUI = nullptr);
    virtual ~QMLItemAgent();

    // ##############################################################################
    // UI MEMBER METHODS & PROPERTIES -----------------------------------------------
    //
    void savePos(int posX, int posY);
    bool isEscDisabled();
    virtual void retranslateUi()     {} // 1-Static Text(_ui->retranslateUi(this) & 2-Dynamic self-defined text later

    bool isActivated();
    bool isLastFocused();
    bool isHidden();
    bool isVisible();
    bool isClosed();
    bool isModalDialog();
    bool isDeletedOnClosed();
    void resize(qreal width, qreal height);
    void resize(const QSizeF& size);
    QSizeF size();
    qreal width();
    qreal height();
    void move(int posX, int posY);
    QPoint pos();

    Q_INVOKABLE bool isBusy() { return _isBusy; }

    Q_INVOKABLE virtual void onActivated() {} // <= [QML::activate()]
    Q_INVOKABLE virtual bool start(); // Replace old QDialog::showEvent(), <= [QML::show()]
    virtual bool end();   // Replace old QDialog::reject(), <= [QML::close()]
    Q_INVOKABLE virtual bool apply(); // <= [QML::apply()]
    //virtual void reject();   // "Cancel"

    void setupAgentAndUI(const QString& qmlPropertyName);
    void setItemUI(QObject* object) {
        assert(object);
        _itemUI = object;
    }

    int itemId() const;
    void setItemId(int itemId) {
        _itemId = itemId;
    }

    void setMouseOptions(const QVector<int>);
    void setMouseOptions(int);
    const QVector<int>& mouseOptions()   { return _mouseOptionList; }
    bool hasMouseOption(int mouseOption) { return _mouseOptionList.contains(mouseOption);  }

public slots:
    virtual void showDialog(bool isStart = true);  // => [QML::show()]
    virtual void hideDialog();  // => [QML::hide()]
    virtual void closeDialog(); // => [QML::close()] => this->end()
    virtual void showHideDialog(bool isShown, bool isStart = false);

signals:
    // void updateUIAttributes(type arg);

protected:
    QObject* UI() const;

    void formatText();
    void setIsBusy(bool);
    void setDisableEsc(bool);// Used by children

    virtual void setupUI(); // Call readSettings()
    virtual void fixBiggerTextMode() {}
    virtual void labelMetric()       {} // For inheriting Item agents, This should only be called after UI is initialized!
    virtual void metricConvert()     {}
    
    virtual void readSettings();
    virtual void writeSettings();

    virtual void validation(bool) {}
    virtual bool event (QEvent * e);
#if 0
    virtual void mousePressEvent(QMouseEvent *event);
    virtual void mouseMoveEvent(QMouseEvent *event);
#endif

    void popUp();

protected:
    QObject* _itemUI;
    int _itemId; // Put here for inheritance
    int _posX;
    int _posY;
    QVector<int> _mouseOptionList;

private:
    /* IF SINGLETON:
        explicit QMLItemAgent(QWidget *parent = 0);
        static QMLItemAgent *_instance;
    */
    // Is being shown:
    bool _isActivated;

    // Is Close just called amid progress bar running:
    bool _isCloseCalledOnShownProgressBar;

    // Is ESC disabled
    bool _isEscDisabled;

    // This member is to indicate a progress bar is being shown,
    // meaning that some busy program task is running. 
    // Meanwhile, disable QDialog affecting user intefering actions 
    // like: X or Esc button.
    
    // In the future, maybe actions like Cancel, Escape, Close amid 
    // a progress bar showing will be allowed. For such, progress bar
    // is closed and also the underneath running task.
    bool _isBusy;
#if 0
    QPoint _lastPos;
#endif

    // ##############################################################################
    // STATE MACHINE MEMBER METHODS -------------------------------------------------
    //
public:
    int getCurrentStateRuleId()                 { return _currentStateRuleId;        }
    void setCurrentStateRuleId(int stateRuleId) { _currentStateRuleId = stateRuleId; }

    bool fTrue() { return true; }
    virtual void init() {}
    virtual void initializeStateMachine() {}
    virtual void startStateMachineOperation() {}

private:
    // STATE MACHINE PROPERTIES -----------------------------------------------------
    //
    int _currentStateRuleId;
};

#endif // QML_ITEM_AGENT_H
