#ifndef K3D_STATE_MACHINE_H
#define K3D_STATE_MACHINE_H

/* Type rule (rul): */
template<typename T>
class K3DSMRule
{
public:
    bool (T::*pfCheck)();  /* Address of transition function. */
    bool fNot;                  /* Flag=True if inverted transition. */
    void (T::*pAction)();       /* Address of action function. */
    int irulNearRule;           /* Index of neighbor rule. */
    int irulNextRule;           /* Index of next rule. */
};

/* Type state machine (stm): */
#define SMEStateMachine K3DSMRule<T>*     /* Array of rules. */

template<typename T>
class K3DStateMachine
{
public:
    // Jump to a True check, run the action and halt! (1)
    // Else keep jumping to sis states until (1) or no sis else(2) and halt!

    // Later the state machine comes back to the prev next rule in case (1) or 
    // the first state of False check in case (2)!
    //
    int run(const T* object);
};

//ERLError ErrSMERunCycle(void)
//{
//    UByte8 ubStaMac;
//
//    if (ubTotalNbrStaMac == 0)
//    {
//        return (ERR_ERL_SME_RULE_FAILURE);
//    }
//
//    for (ubStaMac = 0; ubStaMac < ubTotalNbrStaMac; ubStaMac++)
//    {
//        if ((tbinfStaMac[ubStaMac].ubStatus & tbubPriorityMaskCycle[ubCurrentStage]) &&
//            (tbinfStaMac[ubStaMac].ubStatus & UB_SME_ACTIVE))
//        {
//            FRunStateMachine(ubStaMac);
//        }
//    }
//
//    if (++ubCurrentStage == NB_STAGE)
//    {
//        ubCurrentStage = 0;
//    }
//
//    return (ERR_ERL_NO_ERROR);
//}
//
//Boolean FRunStateMachine(UByte8 ubStateMachineId)
//{
//    UByte8  irulRule;
//    Boolean(*pfCurrentTransition)(void);
//    Boolean fTransitionResult;
//    SMEStateMachine stmStaMac;
//#ifdef UB_STATE_MACHINE_ID
//    UByte8  ubTraceTable;
//    UByte8 *pubTracePos;
//#endif /* UB_STATE_MACHINE_ID */
//
//
//    if (ubStateMachineId < ubTotalNbrStaMac)
//    {
//        irulRule = tbinfStaMac[ubStateMachineId].irulCurrentRule;
//        stmStaMac = *(ptbstmStaMac + ubStateMachineId);
//        pfCurrentTransition = (stmStaMac + irulRule)->pfTransition;
//        do
//        {
//            /* if ( pfCurrentTransition() XOR NotFlag ) */
//            fTransitionResult = pfCurrentTransition();
//            if ((fTransitionResult && !((stmStaMac + irulRule)->fNot)) ||
//                (!fTransitionResult && ((stmStaMac + irulRule)->fNot)))
//            {
//#ifdef UB_STATE_MACHINE_ID
//                /* ************************************************** */
//                /* Define the table to use to store the curent action */
//                /* ************************************************** */
//                if (ubStateMachineId == UB_STATE_MACHINE_ID)
//                {
//                    ubTraceTable = 0;
//                    pubTracePos = &ubStMac1TracePos;
//                }
//#ifdef UB_STATE_MACHINE_ID_2
//                else if (ubStateMachineId == UB_STATE_MACHINE_ID_2)
//                {
//                    ubTraceTable = 1;
//                    pubTracePos = &ubStMac2TracePos;
//                }
//#endif /* UB_STATE_MACHINE_ID_2 */
//                else
//                    ubTraceTable = 0xff;
//
//                /* ************************************* */
//                /* If the curent state machine is one of */
//                /* the watch add the action to the table */
//                /* ************************************* */
//                if (ubTraceTable != 0xff)
//                {
//                    STEMTraceTable[ubTraceTable][(*pubTracePos)++] = irulRule;
//
//                    /* If we are at the end of the table go back to the begin */
//                    if ((*pubTracePos) == UB_SIZEOF_TRACE_TABLE)
//                        (*pubTracePos) = 0;
//
//                    /* Marke the end of the table */
//                    STEMTraceTable[ubTraceTable][(*pubTracePos)] = 0xFF;
//                }
//#endif /* UB_STATE_MACHINE_ID */
//
//                /* begin action time measure */
//                TPOStartMeasure(COM_MEASURE_STATE_MACHINE_TRACE);
//                /* end action time measure */
//
//                /* Juste before executing the action,
//                * print the rule on the serial line
//                */
//                TRC_STAMAC_TRACE(ubStateMachineId, irulRule);
//
//                (stmStaMac + irulRule)->pAction();
//
//                /* begin test action time measure */
//                udTimeMeasure = UdTPOTimeMeasure(COM_MEASURE_STATE_MACHINE_TRACE);
//                if (udTimeMeasure > 1) /* > 10 ms  */
//                {
//                    ubStaMacId = ubStateMachineId;
//                    ubStaMacRule = irulRule;
//                }
//                /* end test action time measure */
//
//                if ((stmStaMac + irulRule)->irulNextRule != UB_SME_NO_RULE)
//                {
//                    tbinfStaMac[ubStateMachineId].irulCurrentRule =
//                        (stmStaMac + irulRule)->irulNextRule;
//                }
//                else /* end of state machine, unactivation */
//                {
//                    tbinfStaMac[ubStateMachineId].ubStatus &= UB_SME_PRIORITY_MASK;
//                    tbinfStaMac[ubStateMachineId].ubStatus |= UB_SME_INACTIVE;
//                }
//                return (FTRUE);
//            }
//            else /* see the sister rule... */
//            {
//                if ((stmStaMac + irulRule)->irulNearRule != UB_SME_NO_RULE)
//                {
//                    irulRule = (stmStaMac + irulRule)->irulNearRule;
//                    pfCurrentTransition = (stmStaMac + irulRule)->pfTransition;
//                }
//                else /* no more sister rule */
//                {
//                    pfCurrentTransition = NULL;
//                }
//            }
//        } while (pfCurrentTransition != NULL);
//    }
//    return (FFALSE); /* all transition of the current rule are false. */
//}

/* end code for function FunctionName */

#endif
