#include<social_state_machine/social_state_machine.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(social_state_machine::SocialStateMachine, move_base::StateMachine)

void clear(control_msgs::JointTrajectoryGoal* goal)
{
    goal->trajectory.points.clear();
}

void add_point(control_msgs::JointTrajectoryGoal* goal, double a, double b, double d)
{
    trajectory_msgs::JointTrajectoryPoint p;
    p.positions.push_back(a);
    p.positions.push_back(b);
    p.time_from_start = ros::Duration(d);
    goal->trajectory.points.push_back(p);
}

void add_point(control_msgs::JointTrajectoryGoal* goal, double a, double d)
{
    trajectory_msgs::JointTrajectoryPoint p;
    p.positions.push_back(a);
    p.time_from_start = ros::Duration(d);
    goal->trajectory.points.push_back(p);
}



namespace social_state_machine
{

SocialStateMachine::SocialStateMachine() :
    enabled_(NULL),
    head_("/head_traj_controller/joint_trajectory_action"), 
    spine_("/torso_controller/joint_trajectory_action"),
    point_head_("/head_traj_controller/point_head_action")
{
    head_goal_.trajectory.joint_names.push_back("head_pan_joint");
    head_goal_.trajectory.joint_names.push_back("head_tilt_joint");
    spine_goal_.trajectory.joint_names.push_back("torso_lift_joint");
}

SocialStateMachine::~SocialStateMachine()
{
    if(enabled_) delete enabled_;
    if(active_) delete active_;
}

void SocialStateMachine::initialize(tf::TransformListener* tf, move_base::GlobalNavigator* planner, move_base::LocalNavigator* controller)
{
    StateMachine::initialize(tf, planner, controller);
    ros::NodeHandle private_nh("~");
    
    enabled_ = new bool[NUM_STATES];
    active_ = new bool[3];
    
    //private_nh.param("local_costmap/circumscribed_radius", base_radius_, 0.46);
    dsrv_ = new dynamic_reconfigure::Server<social_state_machine::SocialStateMachineConfig>(ros::NodeHandle("~/social_state_machine"));
    dynamic_reconfigure::Server<social_state_machine::SocialStateMachineConfig>::CallbackType cb = boost::bind(&SocialStateMachine::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);
    
    
    head_.waitForServer();
    if(enabled_[3])
        spine_.waitForServer();
    point_head_.waitForServer();
    
    spinedown();
    lookdown();
    
    
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
    side_point_ = config.side_point;
    down_angle_ = config.look_down_angle;
    base_time_ = config.base_time;
    up_spine_ = config.up_spine;
    down_spine_ = config.down_spine;
}

void SocialStateMachine::reset()
{
    state_ = S_PLANNING;
    active_[0] = active_[1] = active_[2] = false;
}

void SocialStateMachine::advance()
{
    if(active_[0])    head_.cancelGoal();
    if(active_[1])    spine_.cancelGoal();
    if(active_[2])    point_head_.cancelGoal();
    
    active_[0] = active_[1] = active_[2] = false;
    
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
                lookatpath();
                return;
            }
            break;
        default:
            if(actionsComplete()){
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
            plan_ = planner_->getPlan();
            lookatpath();
            break;
        case S_NORMAL:
            controller_->setGlobalPlan( plan_ );
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

bool SocialStateMachine::actionsComplete()
{
    return head_.getState().isDone() && spine_.getState().isDone() && point_head_.getState().isDone();
}

void SocialStateMachine::lookup()
{
    clear(&head_goal_);
    add_point(&head_goal_, 0, 0, base_time_);
    startJointAction(true);
}

void SocialStateMachine::lookdown()
{
    clear(&head_goal_);
    add_point(&head_goal_, 0, down_angle_, base_time_);
    startJointAction(true);

}

void SocialStateMachine::lookaround(){
    clear(&head_goal_);
    add_point(&head_goal_, -side_point_, 0, base_time_/2);
    add_point(&head_goal_, side_point_, 0, base_time_);
    add_point(&head_goal_, 0, 0, base_time_/2);
    startJointAction(true);
}

void SocialStateMachine::lookatpath(){
    unsigned int i=0;
    double d = 0.0;
    
    while(d<5.0 and i<plan_.size()-1){
        i += 1;
        d = 0.0; // CUrrent pose, plan_.get(i)
    }    
    
    point_goal_.target.header = plan_[i].header;
    point_goal_.target.point = plan_[i].pose.position;
    active_[2] = true;
    point_head_.sendGoal(point_goal_);
}

void SocialStateMachine::spineup()
{
    clear(&spine_goal_);
    add_point(&spine_goal_, up_spine_, base_time_);
    startJointAction(false);
}

void SocialStateMachine::spinedown(){
    clear(&spine_goal_);
    add_point(&spine_goal_, down_spine_, base_time_);
    startJointAction(false);
}


void SocialStateMachine::startJointAction(bool head)
{
    if(head){
        active_[0] = true;
        head_goal_.trajectory.header.stamp = ros::Time::now();
        head_.sendGoal(head_goal_);
    }else{
        active_[1] = true;
        spine_goal_.trajectory.header.stamp = ros::Time::now();
        spine_.sendGoal(spine_goal_);
    }
}

};
