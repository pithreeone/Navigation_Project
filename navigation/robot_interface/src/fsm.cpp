#include "robot_interface/fsm.h"
#include "ros/ros.h"

FSM::FSM()
{
    *this = FSM(FSMItem::State::STOP);
    // cur_state_ = FSMItem::State::STOP;
}

FSM::FSM(FSMItem::State cur_state)
{
    cur_state_ = cur_state;
    finish_ = false;
    initFSMTable();
}

void FSM::transferState(FSMItem::State next_state)
{
    cur_state_ = next_state;
}

void FSM::printState()
{
    switch(cur_state_){
    case FSMItem::State::STOP:
        ROS_INFO("Robot_Interface: current status is 'STOP'");
        break;
    case FSMItem::State::CONTROL_MOVING:
        ROS_INFO("Robot_Interface: current status is 'CONTROL_MOVING'");
        break;
    case FSMItem::State::NAVIGATION_MODE:
        ROS_INFO("Robot_Interface: current status is 'NAVIGATION_MODE'");
        break;        
    case FSMItem::State::MOVE_TO_GOAL:
        ROS_INFO("Robot_Interface: current status is 'MOVE_TO_GOAL'");
        break;
    case FSMItem::State::MOVE_TO_GOAL_KEY:
        ROS_INFO("Robot_Interface: current status is 'MOVE_TO_GOAL_KEY'");
        break;
    case FSMItem::State::RECORD_COORDINATE:
        ROS_INFO("Robot_Interface: current status is 'RECORD_COORDINATE'");
        break;
    case FSMItem::State::START_MAPPING:
        ROS_INFO("Robot_Interface: current status is 'START_MAPPING'");
        break;
    case FSMItem::State::AUTO_MAPPING:
        ROS_INFO("Robot_Interface: current status is 'AUTO_MAPPING'");
        break;
    case FSMItem::State::CONTROL_MAPPING:
        ROS_INFO("Robot_Interface: current status is 'CONTROL_MAPPING'");
        break;
    case FSMItem::State::TEMP_STOP:
        ROS_INFO("Robot_Interface: current status is 'TEMP_STOP'");
        break;
    case FSMItem::State::MOVE_TO_GOAL_1:
        ROS_INFO("Robot_Interface: current status is 'MOVE_TO_GOAL_1'");
        break;
    case FSMItem::State::RAISE_HAND:
        ROS_INFO("Robot_Interface: current status is 'RAISE_HAND'");
        break;
    case FSMItem::State::MOVE_TO_GOAL_2:
        ROS_INFO("Robot_Interface: current status is 'MOVE_TO_GOAL_2'");
        break;
    case FSMItem::State::GET_DOOR:
        ROS_INFO("Robot_Interface: current status is 'GET_DOOR'");
        break;
    case FSMItem::State::MOVE_TO_GOAL_3:
        ROS_INFO("Robot_Interface: current status is 'MOVE_TO_GOAL_3'");
        break;
    case FSMItem::State::MOVE_INTO_ELEVATOR:
        ROS_INFO("Robot_Interface: current status is 'MOVE_INTO_ELEVATOR'");
        break;
    case FSMItem::State::SAY_FLOOR:
        ROS_INFO("Robot_Interface: current status is 'SAY_FLOOR'");
        break;    
    case FSMItem::State::WAIT_FOR_ELEVATOR:
        ROS_INFO("Robot_Interface: current status is 'WAIT_FOR_ELEVATOR'");
        break;
    case FSMItem::State::GET_OUT_OF_ELEVATOR:
        ROS_INFO("Robot_Interface: current status is 'GET_OUT_OF_ELEVATOR'");
        break;  
    case FSMItem::State::MOVE_TO_GOAL_4:
        ROS_INFO("Robot_Interface: current status is 'MOVE_TO_GOAL_4'");
        break;  
    
    
    }
    
}

void FSM::handleEvent(FSMItem::Events event)
{
    FSMItem::State cur_state = cur_state_;
    FSMItem::State next_state;
    bool flag = false;
    for (int i=0; i<fsm_table_.size(); i++){
        if(event == fsm_table_[i]->event_ && cur_state == fsm_table_[i]->cur_state_){
            flag = true;
            pre_state_ = cur_state;
            next_state = fsm_table_[i]->next_state_;
            // ROS_INFO("cur_state: %d, event: %d", cur_state, event);
            // only change the finish_ to false when changing current state.
            finish_ = false;
            break;
        }
    }

    // if find corresponding state
    if(flag){
        transferState(next_state);
    }
}

bool FSM::ifFinishMission()
{
    return finish_;
}

void FSM::setFinishMission(bool if_finish){
    finish_ = if_finish;
}

FSMItem::State FSM::getState()
{
    return cur_state_;
}

FSMItem::State FSM::getPreviousState()
{
    return pre_state_;
}

void FSM::initFSMTable()
{
    // SLAM-mode
    fsm_table_.push_back(new FSMItem(FSMItem::State::STOP, FSMItem::Events::E_START_MAPPING, FSMItem::State::START_MAPPING));
    fsm_table_.push_back(new FSMItem(FSMItem::State::START_MAPPING, FSMItem::Events::E_NAN, FSMItem::State::AUTO_MAPPING));
    fsm_table_.push_back(new FSMItem(FSMItem::State::AUTO_MAPPING, FSMItem::Events::E_FAST_V, FSMItem::State::CONTROL_MAPPING));
    fsm_table_.push_back(new FSMItem(FSMItem::State::CONTROL_MAPPING, FSMItem::Events::E_SLOW_V, FSMItem::State::AUTO_MAPPING));
    fsm_table_.push_back(new FSMItem(FSMItem::State::AUTO_MAPPING, FSMItem::Events::E_FINISH_AUTO_MAPPING, FSMItem::State::CONFIRM_MAP));
    fsm_table_.push_back(new FSMItem(FSMItem::State::CONTROL_MAPPING, FSMItem::Events::E_FINISH_CONTROL_MAPPING, FSMItem::State::CONFIRM_MAP));
    fsm_table_.push_back(new FSMItem(FSMItem::State::CONFIRM_MAP, FSMItem::Events::E_FAST_V, FSMItem::State::CONTROL_MAPPING));
    fsm_table_.push_back(new FSMItem(FSMItem::State::CONFIRM_MAP, FSMItem::Events::E_FINISH_CHECK_MAP, FSMItem::State::SAVE_MAP));
    fsm_table_.push_back(new FSMItem(FSMItem::State::SAVE_MAP, FSMItem::Events::E_NAN, FSMItem::State::STOP));

    // Navigation-mode
    fsm_table_.push_back(new FSMItem(FSMItem::State::STOP, FSMItem::Events::E_FAST_V, FSMItem::State::CONTROL_MOVING));
    fsm_table_.push_back(new FSMItem(FSMItem::State::CONTROL_MOVING, FSMItem::Events::E_SLOW_V, FSMItem::State::STOP));
    fsm_table_.push_back(new FSMItem(FSMItem::State::STOP, FSMItem::Events::E_CHOOSE_MAP, FSMItem::State::NAVIGATION_MODE));
    fsm_table_.push_back(new FSMItem(FSMItem::State::NAVIGATION_MODE, FSMItem::Events::E_MOVE_TO_GOAL, FSMItem::State::MOVE_TO_GOAL));
    fsm_table_.push_back(new FSMItem(FSMItem::State::MOVE_TO_GOAL, FSMItem::Events::E_FINISH_MOVE_SUCCESS, FSMItem::State::STOP));
    fsm_table_.push_back(new FSMItem(FSMItem::State::MOVE_TO_GOAL, FSMItem::Events::E_FINISH_MOVE_FAIL, FSMItem::State::STOP));
    fsm_table_.push_back(new FSMItem(FSMItem::State::NAVIGATION_MODE, FSMItem::Events::E_RECORD_COORDINATE, FSMItem::State::RECORD_COORDINATE));
    fsm_table_.push_back(new FSMItem(FSMItem::State::RECORD_COORDINATE, FSMItem::Events::E_NAN, FSMItem::State::NAVIGATION_MODE));
    fsm_table_.push_back(new FSMItem(FSMItem::State::STOP, FSMItem::Events::E_MOVE_TO_GOAL_KEY, FSMItem::State::MOVE_TO_GOAL_KEY));
    fsm_table_.push_back(new FSMItem(FSMItem::State::MOVE_TO_GOAL_KEY, FSMItem::Events::E_FINISH_MOVE_SUCCESS, FSMItem::State::STOP));
    fsm_table_.push_back(new FSMItem(FSMItem::State::MOVE_TO_GOAL_KEY, FSMItem::Events::E_FINISH_MOVE_FAIL, FSMItem::State::STOP));

    // Navigation-mode (More general. Accept different floor)
    fsm_table_.push_back(new FSMItem(FSMItem::State::STOP, FSMItem::Events::E_MOVE_TO_GOAL_FLOOR, FSMItem::State::TEMP_STOP));
    fsm_table_.push_back(new FSMItem(FSMItem::State::TEMP_STOP, FSMItem::Events::E_DIFFERENT_FLOOR_MOVE, FSMItem::State::MOVE_TO_GOAL_1));
    fsm_table_.push_back(new FSMItem(FSMItem::State::TEMP_STOP, FSMItem::Events::E_SAME_FLOOR_MOVE, FSMItem::State::MOVE_TO_GOAL_4));
    fsm_table_.push_back(new FSMItem(FSMItem::State::MOVE_TO_GOAL_1, FSMItem::Events::E_FINISH_MOVE_SUCCESS, FSMItem::State::RAISE_HAND));
    fsm_table_.push_back(new FSMItem(FSMItem::State::RAISE_HAND, FSMItem::Events::E_FINISH_RAISE_HAND, FSMItem::State::MOVE_TO_GOAL_2));
    fsm_table_.push_back(new FSMItem(FSMItem::State::MOVE_TO_GOAL_2, FSMItem::Events::E_FINISH_MOVE_SUCCESS, FSMItem::State::GET_DOOR));
    fsm_table_.push_back(new FSMItem(FSMItem::State::GET_DOOR, FSMItem::Events::E_GET_DOOR, FSMItem::State::MOVE_TO_GOAL_3));
    fsm_table_.push_back(new FSMItem(FSMItem::State::MOVE_TO_GOAL_3, FSMItem::Events::E_FINISH_MOVE_SUCCESS, FSMItem::State::MOVE_INTO_ELEVATOR));
    fsm_table_.push_back(new FSMItem(FSMItem::State::MOVE_INTO_ELEVATOR, FSMItem::Events::E_FINISH_MOVE_SUCCESS, FSMItem::State::SAY_FLOOR));
    fsm_table_.push_back(new FSMItem(FSMItem::State::SAY_FLOOR, FSMItem::Events::E_FINISH_SAY, FSMItem::State::WAIT_FOR_ELEVATOR));
    fsm_table_.push_back(new FSMItem(FSMItem::State::WAIT_FOR_ELEVATOR, FSMItem::Events::E_SUCCESS_UPDOWN, FSMItem::State::GET_OUT_OF_ELEVATOR));
    fsm_table_.push_back(new FSMItem(FSMItem::State::WAIT_FOR_ELEVATOR, FSMItem::Events::E_DEBUG, FSMItem::State::GET_OUT_OF_ELEVATOR));
    fsm_table_.push_back(new FSMItem(FSMItem::State::GET_OUT_OF_ELEVATOR, FSMItem::Events::E_FINISH_MOVE_SUCCESS, FSMItem::State::MOVE_TO_GOAL_4));
    fsm_table_.push_back(new FSMItem(FSMItem::State::MOVE_TO_GOAL_4, FSMItem::Events::E_FINISH_MOVE_SUCCESS, FSMItem::State::STOP));
    
    // debug
    // fsm_table_.push_back(new FSMItem(FSMItem::State::STOP, FSMItem::Events::E_DEBUG, FSMItem::State::RAISE_HAND));
    // fsm_table_.push_back(new FSMItem(FSMItem::State::RAISE_HAND, FSMItem::Events::E_DEBUG, FSMItem::State::MOVE_TO_GOAL_2));
    // fsm_table_.push_back(new FSMItem(FSMItem::State::STOP, FSMItem::Events::E_DEBUG, FSMItem::State::SAY_FLOOR));
    // fsm_table_.push_back(new FSMItem(FSMItem::State::GET_DOOR, FSMItem::Events::E_DEBUG, FSMItem::State::GET_OUT_OF_ELEVATOR));
    

}