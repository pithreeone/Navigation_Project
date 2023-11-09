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
        // slam
        START_MAPPING,
        AUTO_MAPPING,
        CONTROL_MAPPING,
        CONFIRM_MAP,
        SAVE_MAP,
        // mission
        NAVIGATION_MODE,
        CONTROL_MOVING,
        MOVE_TO_GOAL,
        MOVE_TO_GOAL_KEY, // move to goal by position name
        RECORD_COORDINATE,
        // State for move to different floor
        TEMP_STOP,
        MOVE_TO_GOAL_1,
        RAISE_HAND,
        MOVE_TO_GOAL_2,
        GET_DOOR,
        MOVE_TO_GOAL_3,
        MOVE_INTO_ELEVATOR,
        SAY_FLOOR,
        WAIT_FOR_ELEVATOR,
        GET_OUT_OF_ELEVATOR,
        MOVE_TO_GOAL_4,
    };
    // enumerate all events
    enum Events
    {
        // slam
        E_NAN = 0,
        E_START_MAPPING,
        E_FAST_V,
        E_SLOW_V,
        E_FINISH_AUTO_MAPPING,
        E_FINISH_CONTROL_MAPPING,
        E_FINISH_CHECK_MAP,
        
        // mission
        E_CHOOSE_MAP,
        E_MOVE_TO_GOAL,
        E_MOVE_TO_GOAL_KEY,
        E_FINISH_MOVE_FAIL,
        E_FINISH_MOVE_SUCCESS,
        E_RECORD_COORDINATE,

        // mission for move different floor
        E_MOVE_TO_GOAL_FLOOR,
        E_DIFFERENT_FLOOR_MOVE,
        E_SAME_FLOOR_MOVE,
        E_FINISH_RAISE_HAND,
        E_GET_DOOR,
        E_FINISH_SAY,
        E_SUCCESS_UPDOWN,

        // debug
        E_DEBUG,
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