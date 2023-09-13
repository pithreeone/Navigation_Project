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
    initFSMTable();
}

void FSM::transferState(FSMItem::State next_state)
{
    cur_state_ = next_state;
}

void FSM::printState()
{
    ROS_INFO("Robot_Interface: %d", cur_state_);
}

void FSM::handleEvent(FSMItem::Events event)
{
    FSMItem::State cur_state = cur_state_;
    FSMItem::State next_state;
    bool flag = false;
    for (int i=0; i<fsm_table_.size(); i++){
        if(event == fsm_table_[i]->event_ && cur_state == fsm_table_[i]->cur_state_){
            flag = true;
            next_state = fsm_table_[i]->next_state_;
            break;
        }
    }

    // if find corresponding state
    if(flag){
        transferState(next_state);
    }
}

void FSM::initFSMTable()
{
    fsm_table_.push_back(new FSMItem(FSMItem::State::STOP, FSMItem::Events::E_STOP_2_START_MAPPING, FSMItem::State::START_MAPPING));
    fsm_table_.push_back(new FSMItem(FSMItem::State::START_MAPPING, FSMItem::Events::E_NAN, FSMItem::State::AUTO_MAPPING));
    fsm_table_.push_back(new FSMItem(FSMItem::State::AUTO_MAPPING, FSMItem::Events::E_AUTO_MAPPING_2_CONTROL_MAPPING, FSMItem::State::CONTROL_MAPPING));
    fsm_table_.push_back(new FSMItem(FSMItem::State::CONTROL_MAPPING, FSMItem::Events::E_CONTROL_MAPPING_2_AUTO_MAPPING, FSMItem::State::AUTO_MAPPING));
    fsm_table_.push_back(new FSMItem(FSMItem::State::AUTO_MAPPING, FSMItem::Events::E_AUTO_MAPPING_2_FINISH_MAPPING, FSMItem::State::FINISH_MAPPING));
    fsm_table_.push_back(new FSMItem(FSMItem::State::CONTROL_MAPPING, FSMItem::Events::E_CONTROL_MAPPING_2_FINISH_MAPPING, FSMItem::State::FINISH_MAPPING));
    fsm_table_.push_back(new FSMItem(FSMItem::State::FINISH_MAPPING, FSMItem::Events::E_FINISH_MAPPING_2_CONTROL_MAPPING, FSMItem::State::CONTROL_MAPPING));
    fsm_table_.push_back(new FSMItem(FSMItem::State::FINISH_MAPPING, FSMItem::Events::E_FINISH_MAPPING_2_STOP, FSMItem::State::STOP));
    fsm_table_.push_back(new FSMItem(FSMItem::State::STOP, FSMItem::Events::E_STOP_2_CONTROL_MOVING, FSMItem::State::CONTROL_MOVING));
    fsm_table_.push_back(new FSMItem(FSMItem::State::CONTROL_MOVING, FSMItem::Events::E_CONTROL_MOVING_2_STOP, FSMItem::State::STOP));
    fsm_table_.push_back(new FSMItem(FSMItem::State::STOP, FSMItem::Events::E_STOP_2_MOVE_TO_GOAL, FSMItem::State::MOVE_TO_GOAL));
    fsm_table_.push_back(new FSMItem(FSMItem::State::MOVE_TO_GOAL, FSMItem::Events::E_MOVE_TO_GOAL_2_STOP, FSMItem::State::STOP));
    fsm_table_.push_back(new FSMItem(FSMItem::State::STOP, FSMItem::Events::E_STOP_2_RECORD_COORDINATE, FSMItem::State::RECORD_COORDINATE));
    fsm_table_.push_back(new FSMItem(FSMItem::State::RECORD_COORDINATE, FSMItem::Events::E_NAN, FSMItem::State::STOP));

}