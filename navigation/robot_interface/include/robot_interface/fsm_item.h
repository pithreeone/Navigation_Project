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
        CONTROL_MOVING,
        // mission
        MOVE_TO_GOAL,
        RECORD_COORDINATE,
        // slam
        START_MAPPING,
        AUTO_MAPPING,
        CONTROL_MAPPING,
        FINISH_MAPPING,
    };
    // enumerate all events
    enum Events
    {
        E_NAN = 0,
        E_STOP_2_START_MAPPING,
        E_AUTO_MAPPING_2_CONTROL_MAPPING,
        E_CONTROL_MAPPING_2_AUTO_MAPPING,
        E_AUTO_MAPPING_2_FINISH_MAPPING,
        E_CONTROL_MAPPING_2_FINISH_MAPPING,
        E_FINISH_MAPPING_2_CONTROL_MAPPING,
        E_FINISH_MAPPING_2_STOP,

        E_STOP_2_CONTROL_MOVING,
        E_CONTROL_MOVING_2_STOP,

        E_STOP_2_MOVE_TO_GOAL,
        E_MOVE_TO_GOAL_2_STOP,

        E_STOP_2_RECORD_COORDINATE,

    };

    FSMItem(State cur_state, Events event, State next_state)
    :cur_state_(cur_state), event_(event), next_state_(next_state)
    {
    }
    
    State cur_state_;
    State next_state_;
    Events event_;


};

#endif