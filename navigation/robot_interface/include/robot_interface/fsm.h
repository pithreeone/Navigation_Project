#ifndef _FSM_H_
#define _FSM_H_

#include "robot_interface/fsm_item.h"
#include <vector>
class FSM
{
public:

    /// @brief 
    FSM();

    /// @brief 
    /// @param cur_state 
    FSM(FSMItem::State cur_state);


    /// @brief transform state by the argument
    /// @param next_state the state which is the next state
    void transferState(FSMItem::State next_state);

    void printState();

    // according to different event, change the current state to next state
    void handleEvent(FSMItem::Events event);

private:
    FSMItem::State cur_state_;
    std::vector<FSMItem*> fsm_table_;
    /// @brief initial FSM　table
    void initFSMTable();

};

#endif