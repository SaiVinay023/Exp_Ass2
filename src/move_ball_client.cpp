#include <ros/ros.h>
#include <exp_assignment2/PlanningAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <std_msgs/Float64.h>
#include <unistd.h>

#define min_x -8.0
#define max_x 8.0
#define min_y -8.0
#define max_y 8.0



float RandomFloat(float min, float max)
{
    //Function to calcualate a random float between a min and a max
    assert(max > min);
    float random = ((float) rand()) / (float) RAND_MAX;
    float range = max - min;
    return (random*range) + min;
}



int main(int argc, char** argv){
    ros::init(argc, argv, "move_ball");
    ros::NodeHandle nh;

    actionlib::SimpleActionClient<exp_assignment2::PlanningAction> ac("/reaching_goal", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }


    while(1){
        //define the first goal position
        exp_assignment2::PlanningGoal goal;


        //std_msgs::Float64 angle;
        //angle.data = 0.0;
        //we'll send the goal to move the robot
        //goal.target_pose.header.frame_id = "ball_link";
        //goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = RandomFloat(min_x, max_x);
        goal.target_pose.pose.position.y = RandomFloat(min_y, max_y);
        goal.target_pose.pose.position.z = 0.1;
        goal.target_pose.pose.orientation.w = 0.0;



        ROS_INFO("Sending first goal");
        ac.sendGoal(goal);

        sleep(20);
        ac.waitForResult();

        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Position 1 is reached!");


        //define the second goal position
        exp_assignment2::PlanningGoal goal2;


        //std_msgs::Float64 angle2;
        //angle2.data = 0.0;
        //we'll send the second goal to move the robot
        //goal2.target_pose.header.frame_id = "ball_link";
        //goal2.target_pose.header.stamp = ros::Time::now();

        goal2.target_pose.pose.position.x = RandomFloat(min_x, max_x);
        goal2.target_pose.pose.position.y = RandomFloat(min_y, max_y);
        goal2.target_pose.pose.position.z = 0.1;
        goal2.target_pose.pose.orientation.w = 0.0;

        ROS_INFO("Sending second goal");
        ac.sendGoal(goal2);

        sleep(20);
        ac.waitForResult();


        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Position 2 is reached!");
    }

    return 0;
}

