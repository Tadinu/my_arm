#include "K3DStateMachine.h"

template<typename T>
int K3DStateMachine<T>::run(const T* object) {
    /*
    K3DSMRule<T>* smRules = object->getStateMachine();
    int currentState      = object->getCurrentState();
    bool (T::*pfCheck)();
    void (T::*pAction)();
    bool checkResult = true;
    bool fNot;
    int nextStateId = currentState;
    do {
        pfCheck = (*(smRules + nextStateId)).pfCheck;
        pAction = (*(smRules + nextStateId)).pAction;
        fNot    = (*(smRules + nextStateId)).fNot;
        checkResult = pfCheck();

        if (checkResult && !fNot ||
            !checkResult && fNot) {
            pAction();
            nextStateId = (*(smRules + nextStateId)).irulNextRule;
            return nextStateId;
        }
        else {
            nextStateId = (*(smRules + nextStateId)).irulNearRule;
        }
    } while (nextStateId != 0);
    */
    return -1; // 0, No sis else!
}
