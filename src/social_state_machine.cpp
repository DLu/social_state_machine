#include<social_state_machine/social_state_machine.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(social_state_machine::SocialStateMachine, move_base::StateMachine)

namespace social_state_machine
{

SocialStateMachine::SocialStateMachine()
{
}

SocialStateMachine::~SocialStateMachine() 
{
}

void SocialStateMachine::initialize(tf::TransformListener* tf, move_base::GlobalNavigator* planner, move_base::LocalNavigator* controller)
{
    StateMachine::initialize(tf, planner, controller);
    ros::NodeHandle private_nh("~");

    //private_nh.param("local_costmap/circumscribed_radius", base_radius_, 0.46);
    
    dsrv_ = new dynamic_reconfigure::Server<social_state_machine::SocialStateMachineConfig>(ros::NodeHandle("~/social_state_machine"));
    dynamic_reconfigure::Server<social_state_machine::SocialStateMachineConfig>::CallbackType cb = boost::bind(&SocialStateMachine::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
    
    reset();
}

void SocialStateMachine::reconfigureCB(social_state_machine::SocialStateMachineConfig &config, uint32_t level){
}

void SocialStateMachine::reset()
{
}

void SocialStateMachine::executeCycle(int* status, std::string* message)
{
    *status = 0;
    
}

};
