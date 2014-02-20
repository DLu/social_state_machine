#include<social_state_machine/social_state_machine.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(social_state_machine::SocialStateMachine, move_base::StateMachine)

namespace social_state_machine
{

SocialStateMachine::SocialStateMachine() :
    enabled_(NULL),
    head_("/head_traj_controller/joint_trajectory_action"), 
    spine_("/torso_controller/joint_trajectory_action")
{
    head_goal_.trajectory.joint_names.push_back("head_pan_joint");
    head_goal_.trajectory.joint_names.push_back("head_tilt_joint");
    spine_goal_.trajectory.joint_names.push_back("torso_lift_joint");
}

SocialStateMachine::~SocialStateMachine()
{
    if(enabled_) delete enabled_;
}

void SocialStateMachine::initialize(tf::TransformListener* tf, move_base::GlobalNavigator* planner, move_base::LocalNavigator* controller)
{
    StateMachine::initialize(tf, planner, controller);
    ros::NodeHandle private_nh("~");
    
    enabled_ = new bool[NUM_STATES];

    //private_nh.param("local_costmap/circumscribed_radius", base_radius_, 0.46);
    dsrv_ = new dynamic_reconfigure::Server<social_state_machine::SocialStateMachineConfig>(ros::NodeHandle("~/social_state_machine"));
    dynamic_reconfigure::Server<social_state_machine::SocialStateMachineConfig>::CallbackType cb = boost::bind(&SocialStateMachine::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
    
    
    if(false)
        head_.waitForServer();
    if(enabled_[3])
        spine_.waitForServer();
    
    
    reset();
}

void SocialStateMachine::reconfigureCB(social_state_machine::SocialStateMachineConfig &config, uint32_t level){
    enabled_[0] = true;                                 // S_PLANNING
    enabled_[1] = config.lookup_enabled;                // S_LOOKUP
    enabled_[2] = config.initial_lookaround_enabled;    // S_LOOK_AROUND
    enabled_[3] = config.spine_enabled;                 // S_SPINE_UP
    enabled_[4] = config.initial_path_look_enabled;     // S_LOOK_AT_PATH
    enabled_[5] = true;                                 // S_NORMAL
    enabled_[6] = true;                                 // S_NEAR_END
    enabled_[7] = config.spine_enabled;                 // S_SPINE_DOWN
    enabled_[8] = config.final_lookaround_enabled;      // S_END_LOOK_AROUND
    enabled_[9] = config.lookup_enabled;                // S_END_LOOK_DOWN
    enabled_[10]= true;                                 // S_AT_GOAL
    
    end_distance_ = config.end_distance;
}

void SocialStateMachine::reset()
{
    state_ = S_PLANNING;
}

void SocialStateMachine::advance()
{
    while(state_ < NUM_STATES - 1){
        state_ = state_ + 1;
        if( enabled_[state_] ){
            break;
        }
    }
}

void SocialStateMachine::executeCycle(int* status, std::string* message)
{
    *status = 0;
    
    switch(state_){
        case S_PLANNING:
            if(planner_->hasNewPlan())
                advance();
            else
                return;
            break;
        case S_NORMAL:
            if(planner_->getDistanceLeft() < end_distance_){
                advance();
            }else
                return;
            break;
        case S_NEAR_END:
            if(controller_->getState()==FINISHED){
                advance();
            }else{
                return;
            }
            break;
        default:
            if(true/*actions_complete*/){
                advance();
            }else{
                return;            
            }
            break;
    }
    
        
    switch(state_){
        case S_LOOKUP:
            lookup();
            break;
        case S_LOOK_AROUND:
        case S_END_LOOK_AROUND:
            lookaround();
            break;
        case S_SPINE_UP:
            spineup();
            break;
        case S_LOOK_AT_PATH:
            lookatpath();
            break;
        case S_NORMAL:
            controller_->setGlobalPlan( planner_->getPlan() );
            break;
            
        case S_SPINE_DOWN:
            spinedown();
            break;
        case S_END_LOOK_DOWN:
            lookdown();
            break;
        case S_AT_GOAL:
            reset();
            *status = 1;
            *message = "Goal Reached!";
            return;
        default:
            return;
        
            
    }
}

};
