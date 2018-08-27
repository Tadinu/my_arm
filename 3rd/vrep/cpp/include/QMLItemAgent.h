#ifndef QML_ITEM_AGENT_H
#define QML_ITEM_AGENT_H

#include <assert.h>
#include <QObject>
#include <QKeyEvent>
#include <QThread>
#include <QMutex>

#define RB_DIALOG_PP(dialogP) reinterpret_cast<QMLItemAgent**>(&dialogP)

class QMLItemAgent : public QObject
{
    Q_OBJECT

public:
    explicit QMLItemAgent(QObject *parent = 0,
                          int itemId = -1, QObject* itemUI = nullptr);
    virtual ~QMLItemAgent();

    // ##############################################################################
    // UI MEMBER METHODS & PROPERTIES -----------------------------------------------
    //
    void setupAgentAndUI(const QString& qmlPropertyName);
    void setItemUI(QObject* object) {
        assert(object);
        _itemUI = object;
    }

    int itemId() const;
    void setItemId(int itemId) {
        _itemId = itemId;
    }

    virtual void init();
    void setRunningOnThread(bool isYes);

//protected:
    QObject* UI() const;
    void setupUI();
    virtual bool event (QEvent * e) {}


protected:
    QObject* _itemUI;
    int _itemId; // Put here for inheritance

private:
    /* IF SINGLETON:
        explicit QMLItemAgent(QWidget *parent = 0);
        static QMLItemAgent *_instance;
    */

    // ##############################################################################
    // STATE MACHINE MEMBER METHODS -------------------------------------------------
    //
public:
    int getCurrentStateRuleId()                 { return _currentStateRuleId;        }
    void setCurrentStateRuleId(int stateRuleId) { _currentStateRuleId = stateRuleId; }

    bool fTrue() { return true; }
    virtual void initState(){}
    virtual void initializeStateMachine() {}
    virtual void runStateMachineOperation() {}

    virtual bool isHalted() { return false;}
private:
    int _currentStateRuleId;

    // ##############################################################################
    // THREADING  -------------------------------------------------------------------
    //
public slots:
    virtual void runTask();
    void startThreading();

protected:
    QThread* _thread;
    QMutex* _mutex;
    bool _isRunningOnThread;

    int _priorityNo;
};

#endif // QML_ITEM_AGENT_H
