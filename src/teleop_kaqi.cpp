// From package: pr2_teleop
// http://wiki.ros.org/pr2_teleop
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <topic_tools/MuxSelect.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>


const int PUBLISH_FREQ = 10;

using namespace std;

class TeleopKaqi
{
public:
    geometry_msgs::Twist cmd;
    //joy::Joy joy;
    double req_vx, req_vy, req_vw;
    double max_vx, max_vy, max_vw, max_vx_run, max_vy_run, max_vw_run;
    int axis_vx, axis_vy, axis_vw;
    int deadman_button, run_button;
    bool deadman_no_publish_;

    bool deadman_;
    bool use_mux_, last_deadman_;
    std::string last_selected_topic_;

    ros::Time last_recieved_joy_message_time_;
    ros::Duration joy_msg_timeout_;

    ros::NodeHandle n_, n_private_;
    ros::Publisher vel_pub_;
    ros::Subscriber joy_sub_;
    ros::ServiceClient mux_client_;
    std::string mux_teleop_topic_;

    TeleopKaqi(bool deadman_no_publish = false) :
        max_vx(0.4), max_vy(0.4), max_vw(1.0),
        max_vx_run(0.8), max_vy_run(0.8), max_vw_run(1.2),
        deadman_no_publish_(deadman_no_publish),
        deadman_(false),
        use_mux_(false), last_deadman_(false),
        n_private_("~")
    { }

    void init()
    {
        cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;


        //parameters for interaction with a mux on cmd_vel topics
        n_private_.param("use_mux", use_mux_, false);
        n_private_.param<string>("mux_teleop_topic", mux_teleop_topic_, "mux_cmd_vel");

        n_private_.param("max_vx", max_vx, max_vx);
        n_private_.param("max_vy", max_vy, max_vy);
        n_private_.param("max_vw", max_vw, max_vw);

        // Set max speed while running
        n_private_.param("max_vx_run", max_vx_run, max_vx_run);
        n_private_.param("max_vy_run", max_vy_run, max_vy_run);
        n_private_.param("max_vw_run", max_vw_run, max_vw_run);

        n_private_.param("axis_vx", axis_vx, 1);
        n_private_.param("axis_vw", axis_vw, 0);
        n_private_.param("axis_vy", axis_vy, 2);

        n_private_.param("deadman_button", deadman_button, 10);
        n_private_.param("run_button", run_button, 8);

        double joy_msg_timeout;
        n_private_.param("joy_msg_timeout", joy_msg_timeout, 0.5); //default to 0.5 seconds timeout
        if (joy_msg_timeout <= 0)
        {
            joy_msg_timeout_ = ros::Duration().fromSec(9999999);//DURATION_MAX;
            ROS_DEBUG("joy_msg_timeout <= 0 -> no timeout");
        }
        else
        {
            joy_msg_timeout_.fromSec(joy_msg_timeout);
            ROS_DEBUG("joy_msg_timeout: %.3f", joy_msg_timeout_.toSec());
        }

        ROS_DEBUG("max_vx: %.3f m/s\n", max_vx);
        ROS_DEBUG("max_vy: %.3f m/s\n", max_vy);
        ROS_DEBUG("max_vw: %.3f deg/s\n", max_vw*180.0/M_PI);

        ROS_DEBUG("max_vx_run: %.3f m/s\n", max_vx_run);
        ROS_DEBUG("max_vy_run: %.3f m/s\n", max_vy_run);
        ROS_DEBUG("max_vw_run: %.3f deg/s\n", max_vw_run*180.0/M_PI);

        ROS_DEBUG("axis_vx: %d\n", axis_vx);
        ROS_DEBUG("axis_vy: %d\n", axis_vy);
        ROS_DEBUG("axis_vw: %d\n", axis_vw);

        ROS_DEBUG("deadman_button: %d\n", deadman_button);
        ROS_DEBUG("run_button: %d\n", run_button);


        vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        joy_sub_ = n_.subscribe("joy", 10, &TeleopKaqi::joy_cb, this);

        //if we're going to use the mux, then we'll subscribe to state changes on the mux
        if(use_mux_)
        {
            ros::NodeHandle mux_nh("mux");
            mux_client_ = mux_nh.serviceClient<topic_tools::MuxSelect>("select");
        }
    }

  ~TeleopKaqi() { }

    /** Callback for joy topic **/
    void joy_cb(const sensor_msgs::Joy::ConstPtr& joy_msg)
    {
        //Record this message reciept
        last_recieved_joy_message_time_ = ros::Time::now();

        deadman_ = (((unsigned int)deadman_button < joy_msg->buttons.size()) && joy_msg->buttons[deadman_button]);

        if (!deadman_)
            return;

        // Base
        bool running = (((unsigned int)run_button < joy_msg->buttons.size()) && joy_msg->buttons[run_button]);
        double vx = running ? max_vx_run : max_vx;
        double vy = running ? max_vy_run : max_vy;
        double vw = running ? max_vw_run : max_vw;

        if((axis_vx >= 0) && (((unsigned int)axis_vx) < joy_msg->axes.size()))
            req_vx = joy_msg->axes[axis_vx] * vx;
        else
            req_vx = 0.0;
        if((axis_vy >= 0) && (((unsigned int)axis_vy) < joy_msg->axes.size()))
            req_vy = joy_msg->axes[axis_vy] * vy;
        else
            req_vy = 0.0;
        if((axis_vw >= 0) && (((unsigned int)axis_vw) < joy_msg->axes.size()))
            req_vw = joy_msg->axes[axis_vw] * vw;
        else
            req_vw = 0.0;

        // Enforce max/mins for velocity
        // Joystick should be [-1, 1], but it might not be
        req_vx = max(min(req_vx, vx), -vx);
        req_vy = max(min(req_vy, vy), -vy);
        req_vw = max(min(req_vw, vw), -vw);
    }


    void send_cmd_vel()
    {
        if(deadman_  &&
           last_recieved_joy_message_time_ + joy_msg_timeout_ > ros::Time::now())
        {
            //check if we need to switch the mux to our topic for teleop
            if(use_mux_ && !last_deadman_)
            {
                topic_tools::MuxSelect select_srv;
                select_srv.request.topic = mux_teleop_topic_;
                if(mux_client_.call(select_srv))
                {
                    last_selected_topic_ = select_srv.response.prev_topic;
                    ROS_DEBUG("Setting mux to %s for teleop", select_srv.request.topic.c_str());
                }
                else
                {
                    ROS_ERROR("Failed to call select service %s on mux. Are you sure that it is up and connected correctly to the teleop node?", mux_client_.getService().c_str());
                }
            }

            // Base
            cmd.linear.x = req_vx;
            cmd.linear.y = req_vy;
            cmd.angular.z = req_vw;
            vel_pub_.publish(cmd);


            fprintf(stdout,"teleop_base:: %f, %f, %f\n",
                cmd.linear.x ,cmd.linear.y, cmd.angular.z);
        }
        else
        {
            //make sure to set the mux back to whatever topic it was on when we grabbed it if the deadman has just toggled
            if(use_mux_ && last_deadman_)
            {
                topic_tools::MuxSelect select_srv;
                select_srv.request.topic = last_selected_topic_;
                if(mux_client_.call(select_srv))
                {
                    ROS_DEBUG("Setting mux back to %s", last_selected_topic_.c_str());
                }
                else
                {
                    ROS_ERROR("Failed to call select service %s on mux. Are you sure that it is up and connected correctly to the teleop node?", mux_client_.getService().c_str());
                }
            }

            // Publish zero commands iff deadman_no_publish is false
            cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
            if (!deadman_no_publish_)
            {
                // Base
                vel_pub_.publish(cmd);
            }
        }

        //make sure we store the state of our last deadman
        last_deadman_ = deadman_;
    }

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "teleop_kaqi");
    const char* opt_no_publish    = "--deadman_no_publish";

    bool no_publish = false;
    for(int i=1;i<argc;i++)
    {
        if(!strncmp(argv[i], opt_no_publish, strlen(opt_no_publish)))
            no_publish = true;
    }

    TeleopKaqi teleop_kaqi(no_publish);
    teleop_kaqi.init();

    ros::Rate pub_rate(PUBLISH_FREQ);

    while (teleop_kaqi.n_.ok())
    {
        ros::spinOnce();
        teleop_kaqi.send_cmd_vel();
        pub_rate.sleep();
    }

    exit(0);
    return 0;
}


