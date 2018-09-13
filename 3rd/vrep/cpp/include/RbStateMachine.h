#ifndef K3D_STATE_MACHINE_H
#define K3D_STATE_MACHINE_H
#include "RbGlobal.h"

/* Type rule (rul): */
template<typename T>
class RbSMRule
{
public:
    bool (T::*pfCheck)();  /* Address of transition function. */
    bool fNot;             /* Flag=True if inverted transition. */
    void (T::*pAction)();  /* Address of action function. */
    int irulNearRule;      /* Index of neighbor rule. */
    int irulNextRule;      /* Index of next rule. */
};

/* Type state machine (stm): */
#define SMEStateMachine RbSMRule<T>*     /* Array of rules. */

template<typename T>
class RbStateMachine
{
public:
    // Jump to a True check, run the action and halt! (1)
    // Else keep jumping to sis states until (1) or no sis else(2) and halt!

    // Later the state machine comes back to the prev next rule in case (1) or
    // the first state of False check in case (2)!
    //
    static int run(T* object)
    {
        SMEStateMachine smRules = object->getStateMachine();
        int currentState     = object->getCurrentStateRuleId();
        bool (T::*pfCheck)() = nullptr;
        void (T::*pAction)() = nullptr;
        bool checkResult     = true;
        bool fNot            = false;
        int nextStateId      = currentState;
        do {
            pfCheck = (*(smRules + nextStateId)).pfCheck;
            pAction = (*(smRules + nextStateId)).pAction;
            fNot    = (*(smRules + nextStateId)).fNot;
            checkResult = (pfCheck == nullptr) ? false :
                                                 RB_MEMFUNC_CALL(*object, pfCheck)();

            if (checkResult && !fNot ||
                !checkResult && fNot) {
                //printf("Check %d\n", pAction);
                if(pAction != nullptr) RB_MEMFUNC_CALL(*object, pAction)();
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

#endif
