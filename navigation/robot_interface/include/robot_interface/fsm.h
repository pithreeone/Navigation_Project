#include "robot_interface/fsm_item.h"
#include <vector>
class FSM
{
public:
    FSM(FSMItem::State cur_state){

    }
    void transferState(FSMItem::State next_state);
    // according to different event, change the current state to next state
    void handleEvent(FSMItem::Events event){

    }

private:
    FSMItem::State cur_state_;
    std::vector<FSMItem*> fsm_table_;
    void initFSMTable();

};