// From package: pr2_teleop
// http://wiki.ros.org/pr2_teleop
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#define KEYCODE_U 0x75
#define KEYCODE_I 0x69
#define KEYCODE_O 0x6F
#define KEYCODE_J 0x6A
#define KEYCODE_K 0x6B
#define KEYCODE_L 0x6C
#define KEYCODE_M 0x6D
#define KEYCODE_COMMA 0x2C
#define KEYCODE_DOT 0x2E

#define KEYCODE_CAP_U 0x55
#define KEYCODE_CAP_I 0x49
#define KEYCODE_CAP_O 0x4F
#define KEYCODE_CAP_J 0x4A
#define KEYCODE_CAP_K 0x4B
#define KEYCODE_CAP_L 0x4C
#define KEYCODE_CAP_M 0x4D
#define KEYCODE_LESS 0x3C
#define KEYCODE_GREATER 0x3E

using namespace std;

class TeleopKaqiKeyboard
{
    private:
    double walk_vel, run_vel, yaw_rate, yaw_rate_run;
    geometry_msgs::Twist cmd;

    ros::NodeHandle n_;
    ros::Publisher vel_pub_;

public:
    void init()
    {
        cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

        vel_pub_ = n_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

        ros::NodeHandle n_private("~");
        n_private.param<double>("walk_vel", walk_vel, 0.3);
        n_private.param<double>("run_vel", run_vel, 0.6);
        n_private.param<double>("yaw_rate", yaw_rate, 0.6);
        n_private.param<double>("yaw_run_rate", yaw_rate_run, 1.2);
    }

    ~TeleopKaqiKeyboard()   { }
    void keyboardLoop();

};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
    tcsetattr(kfd, TCSANOW, &cooked);
    exit(0);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kaqi_base_keyboard");

    TeleopKaqiKeyboard tkk;
    tkk.init();

    signal(SIGINT, quit);

    tkk.keyboardLoop();

    return(0);
}

const char* help = "  \n"\
"Control Kaqi Base!\n" \
"-----------------------------\n" \
"Moving around: \n"\
"   u    i    o \n"\
"   j    k    l \n"\
"   m    ,    . \n"\
"               \n"\
"CTRL-C to quit  \n";


void TeleopKaqiKeyboard::keyboardLoop()
{
    ros::Time last_stamp = ros::Time::now();
    ros::Time stamp = last_stamp;
    char c;
    bool dirty=false;

    // get the console in raw mode
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);

    puts(help);

    for(;;)
    {
        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0)
        {
            perror("read():");
            exit(-1);
        }

        cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;

//        printf("char = 0x%X\n", c);

        switch(c)
        {
        // Walking
        case KEYCODE_U:
            cmd.linear.x = walk_vel;
            cmd.angular.z = yaw_rate;
            dirty = true;
            break;
        case KEYCODE_I:
            cmd.linear.x = walk_vel;
            dirty = true;
            break;
        case KEYCODE_O:
            cmd.linear.x = walk_vel;
            cmd.angular.z = -yaw_rate;
            dirty = true;
            break;
        case KEYCODE_J:
            cmd.angular.z = yaw_rate;
            dirty = true;
            break;
        case KEYCODE_K:
            cmd.linear.x = 0;
            cmd.linear.y = 0;
            cmd.angular.z = 0;
            dirty = true;
            break;
        case KEYCODE_L:
            cmd.angular.z = -yaw_rate;
            dirty = true;
            break;
        case KEYCODE_M:
            cmd.linear.x = -walk_vel;
            cmd.angular.z = -yaw_rate;
            dirty = true;
            break;
        case KEYCODE_COMMA:
            cmd.linear.x = -walk_vel;
            dirty = true;
            break;
        case KEYCODE_DOT:
            cmd.linear.x = -walk_vel;
            cmd.angular.z = yaw_rate;
            dirty = true;
            break;
        }


        if (dirty == true)
        {
            stamp = ros::Time::now();
            if( (stamp - last_stamp).toSec() > 0.15 )
            {
                cmd.linear.x = cmd.linear.y = cmd.angular.z = 0;
            }
            vel_pub_.publish(cmd);
            last_stamp = stamp;
            dirty = false;
        }

    }
}

