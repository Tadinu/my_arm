#ifndef GEOPAD_ROBOT_AGENT
#define GEOPAD_ROBOT_AGENT

//#include "obstacle.h"

#include <QList.h>
#include <qwidget.h>
#include <qapplication.h>
#include <QColor.h>

#include "K3DStatemachine.h"
#include "K3DQMLItemAgent.h"
#include "guiutil.h"

template<typename T>
class K3DRobotMachine
{
public:
    // Jump to a True check, run the action and halt! (1)
    // Else keep jumping to sis states until (1) or no sis else(2) and halt!

    // Later the state machine comes back to the prev next rule in case (1) or 
    // the first state of False check in case (2)!
    //
    static int run(T* object) 
    {
        K3DSMRule<T>* smRules = object->getStateMachine();
        int currentState      = object->getCurrentStateRuleId();
        bool (T::*pfCheck)();
        void (T::*pAction)();
        bool checkResult = true;
        bool fNot;
        int nextStateId = currentState;
        do {
            pfCheck = (*(smRules + nextStateId)).pfCheck;
            pAction = (*(smRules + nextStateId)).pAction;
            fNot    = (*(smRules + nextStateId)).fNot;
            checkResult = K3D_MEMFUNC_CALL(*object, pfCheck)();

            if (checkResult && !fNot ||
                !checkResult && fNot) {
                K3D_MEMFUNC_CALL(*object, pAction)();
                nextStateId = (*(smRules + nextStateId)).irulNextRule;
                return nextStateId;
            }
            else {
                nextStateId = (*(smRules + nextStateId)).irulNearRule;
            }
        } while (nextStateId != 0);
    
        return -1; // 0, No sis else! 
    }
};

class RobotAgent : public K3DQMLItemAgent
{
    Q_OBJECT
    Q_ENUMS(RB_STATE)
public:
    enum RB_STATE_MACHINE {
        RB_STATE1,
        RB_STATE2,
        RB_STATE3,

        RB_STATE_MACHINE_TOTAL
    };


    enum RB_STATE {
        IDLE,
        OPERATING,

        RB_STATE_TOTAL
    };
    RobotAgent() : _robotId(0), _currentStateRuleId(0) {
        getInfo();
        MaxOngleForLearn = 90;
        MaxOngleForExec  = 45;
    }

    ~RobotAgent() {
    }

    bool fTrue() { return true; }
    void init() {}

    static void startOperation(RobotAgent* robotAgent);

    K3DSMRule<RobotAgent>* getStateMachine() {
        return _stateMachine;
    }

    int getCurrentStateRuleId() {
        return _currentStateRuleId;
    }

    void setCurrentStateRuleId(int stateRuleId) {
        _currentStateRuleId = stateRuleId;
    }

    bool checkOperating();
    bool checkIdle();
    void operate();
    void goIdle();

    int     getCapteurDistance(int numCapter,int Sens);
    void    rotate(int degree);
    void    move(int distance);
    //void  setObstacle(QList<obstacle> *envrnemt0){envrnemt=envrnemt0;};
    void    getInfo();
    float   getDistance(QPoint deb,QPoint end);

public slots:
    void onStateChanged() {
        operate();
    }

private:
    int _robotId;
    int prevRot; //juste pour test

    int	nbrCapteur ;
    int MaxOngleForLearn;
    int MaxOngleForExec;
    int timetoAvance;
    int destanceToAvance;
    int ImpactDistance;//pour le superviseur

    //obstacle *obst[10];
    //QList<obstacle*> envrnemt;
    int		CurentRotation;
    QPoint	CurentPosition;


    static K3DSMRule<RobotAgent> _stateMachine[RB_STATE_MACHINE_TOTAL];
    int _currentStateRuleId;
};
#endif
