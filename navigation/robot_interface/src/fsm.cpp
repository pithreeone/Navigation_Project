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
    case FSMItem::State::CONFIRM_MAP:
        ROS_INFO("Robot_Interface: current status is 'CONFIRM_MAP'");
        break;
    }
    
}

void FSM::handleEvent(FSMItem::Events event)
{
    FSMItem::State cur_state = cur_state_;
    FSMItem::State next_state;
    finish_ = false;
    bool flag = false;
    for (int i=0; i<fsm_table_.size(); i++){
        if(event == fsm_table_[i]->event_ && cur_state == fsm_table_[i]->cur_state_){
            flag = true;
            next_state = fsm_table_[i]->next_state_;
            // ROS_INFO("cur_state: %d, event: %d", cur_state, event);
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

void FSM::initFSMTable()
{
    fsm_table_.push_back(new FSMItem(FSMItem::State::STOP, FSMItem::Events::E_START_MAPPING, FSMItem::State::START_MAPPING));
    fsm_table_.push_back(new FSMItem(FSMItem::State::START_MAPPING, FSMItem::Events::E_NAN, FSMItem::State::AUTO_MAPPING));
    fsm_table_.push_back(new FSMItem(FSMItem::State::AUTO_MAPPING, FSMItem::Events::E_FAST_V, FSMItem::State::CONTROL_MAPPING));
    fsm_table_.push_back(new FSMItem(FSMItem::State::CONTROL_MAPPING, FSMItem::Events::E_SLOW_V, FSMItem::State::AUTO_MAPPING));
    fsm_table_.push_back(new FSMItem(FSMItem::State::AUTO_MAPPING, FSMItem::Events::E_FINISH_AUTO_MAPPING, FSMItem::State::CONFIRM_MAP));
    fsm_table_.push_back(new FSMItem(FSMItem::State::CONTROL_MAPPING, FSMItem::Events::E_FINISH_CONTROL_MAPPING, FSMItem::State::CONFIRM_MAP));
    fsm_table_.push_back(new FSMItem(FSMItem::State::CONFIRM_MAP, FSMItem::Events::E_FAST_V, FSMItem::State::CONTROL_MAPPING));
    fsm_table_.push_back(new FSMItem(FSMItem::State::CONFIRM_MAP, FSMItem::Events::E_FINISH_CHECK_MAP, FSMItem::State::STOP));
    fsm_table_.push_back(new FSMItem(FSMItem::State::STOP, FSMItem::Events::E_FAST_V, FSMItem::State::CONTROL_MOVING));
    fsm_table_.push_back(new FSMItem(FSMItem::State::CONTROL_MOVING, FSMItem::Events::E_SLOW_V, FSMItem::State::STOP));
    fsm_table_.push_back(new FSMItem(FSMItem::State::STOP, FSMItem::Events::E_MOVE_TO_GOAL, FSMItem::State::MOVE_TO_GOAL));
    fsm_table_.push_back(new FSMItem(FSMItem::State::MOVE_TO_GOAL, FSMItem::Events::E_FINISH_MOVE, FSMItem::State::STOP));
    fsm_table_.push_back(new FSMItem(FSMItem::State::STOP, FSMItem::Events::E_MOVE_TO_GOAL_KEY, FSMItem::State::MOVE_TO_GOAL_KEY));
    fsm_table_.push_back(new FSMItem(FSMItem::State::MOVE_TO_GOAL_KEY, FSMItem::Events::E_FINISH_MOVE, FSMItem::State::STOP));
    fsm_table_.push_back(new FSMItem(FSMItem::State::STOP, FSMItem::Events::E_RECORD_COORDINATE, FSMItem::State::RECORD_COORDINATE));
    fsm_table_.push_back(new FSMItem(FSMItem::State::RECORD_COORDINATE, FSMItem::Events::E_NAN, FSMItem::State::STOP));

}