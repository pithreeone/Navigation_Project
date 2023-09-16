#ifndef _FSM_ITEM_H_
#define _FSM_ITEM_H_

class FSMItem
{
public:
    // enumerate all states
    enum State
    {
        // waiting
        STOP = 0,
        // mission
        CONTROL_MOVING,
        MOVE_TO_GOAL,
        MOVE_TO_GOAL_KEY, // move to goal by position name
        RECORD_COORDINATE,
        // slam
        START_MAPPING,
        AUTO_MAPPING,
        CONTROL_MAPPING,
        CONFIRM_MAP,
    };
    // enumerate all events
    enum Events
    {
        E_NAN = 0,
        E_START_MAPPING,
        E_FAST_V,
        E_SLOW_V,
        E_FINISH_AUTO_MAPPING,
        E_FINISH_CONTROL_MAPPING,
        E_FINISH_CHECK_MAP,

        E_MOVE_TO_GOAL,
        E_MOVE_TO_GOAL_KEY,
        E_FINISH_MOVE,

        E_RECORD_COORDINATE,

    };

    FSMItem(State cur_state, Events event, State next_state)
    :cur_state_(cur_state), event_(event), next_state_(next_state)
    {
    }
    
    State cur_state_;
    State next_state_;
    Events event_;
    void(*action_)();

};

#endif