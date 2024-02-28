#include <stdio.h>
#include <signal.h>
#include <termios.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <interactive_customer_service/Conversation.h>
#include <interactive_customer_service/RobotStatus.h>
#include <nodelet/nodelet.h>

class InteractiveCustomerServiceTeleopKey
{
private:
  static const char KEYCODE_0 = 0x30;
  static const char KEYCODE_1 = 0x31;
  static const char KEYCODE_2 = 0x32;
  static const char KEYCODE_3 = 0x33;
  static const char KEYCODE_4 = 0x34;
  static const char KEYCODE_5 = 0x35;
  static const char KEYCODE_6 = 0x36;
  static const char KEYCODE_7 = 0x37;
  static const char KEYCODE_8 = 0x38;
  static const char KEYCODE_9 = 0x39;

  const std::string MSG_ARE_YOU_READY = "Are_you_ready?";

  const std::string MSG_I_AM_READY    = "I_am_ready";
  const std::string MSG_ROBOT_MESSAGE = "robot_message";
  const std::string MSG_TAKE_ITEM     = "take_item";
  const std::string MSG_GIVE_ITEM     = "give_item";
  const std::string MSG_GIVE_UP       = "Give_up";
  
  ros::Publisher pub_msg_;
  
  void showHelp()
  {
    puts("Operate by Keyboard");
    puts("---------------------------");
//    puts(("0 : Send "+MSG_I_AM_READY).c_str());
    puts( "1 : Send Message : 'There are several candidates. Is it green?'");
    puts( "2 : Send Message : 'Is it the bigger one?'");
    puts( "3 : Send Message : 'Is this what you want?'");
    puts( "4 : Take '11_xylitol-1000'");
    puts( "5 : Take 'chipstar_consomme-2000'");
    puts( "6 : Take 'irohasu-3000'");
    puts( "7 : Give Item");
    puts(("9 : Send "+MSG_GIVE_UP).c_str());
    puts("---------------------------");
  }

  void messageCallback(const interactive_customer_service::Conversation::ConstPtr& message)
  {
    if(message->type.c_str()==MSG_ARE_YOU_READY)
    {
      sendMessage(pub_msg_, MSG_I_AM_READY);
    }
    else
    {
      ROS_INFO("Subscribe message:%s, %s", message->type.c_str(), message->detail.c_str());
    }
  }

  void robotStatusCallback(const interactive_customer_service::RobotStatus::ConstPtr& status)
  {
    ROS_DEBUG("Subscribe robot status:%s, %d, %s", status->state.c_str(), status->speaking, status->grasped_item.c_str());
  }

  void sendMessage(ros::Publisher &publisher, const std::string &type, const std::string &detail="")
  {
    ROS_INFO("Send message:%s, %s", type.c_str(), detail.c_str());

    interactive_customer_service::Conversation message;
    message.type   = type;
    message.detail = detail;
    publisher.publish(message);
  }

  static void rosSigintHandler(int sig)
  {
    ros::shutdown();
  }

  static int canReceive(int fd)
  {
    fd_set fdset;
    int ret;
    struct timeval timeout;
    FD_ZERO( &fdset );
    FD_SET( fd , &fdset );

    timeout.tv_sec = 0;
    timeout.tv_usec = 0;

    return select( fd+1 , &fdset , NULL , NULL , &timeout );
  }

public:
  int run(int argc, char **argv)
  {
    ros::NodeHandle node_handle;

    char c;

    /////////////////////////////////////////////
    // get the console in raw mode
    int kfd = 0;
    struct termios cooked;

    struct termios raw;
    tcgetattr(kfd, &cooked);
    memcpy(&raw, &cooked, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
    /////////////////////////////////////////////

    showHelp();

    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, rosSigintHandler);
  
    ros::Rate loop_rate(10);

    std::string sub_customer_msg_topic_name;
    std::string pub_robot_msg_topic_name;
    std::string sub_robot_status_topic_name;

    node_handle.param<std::string>("sub_customer_msg_topic_name", sub_customer_msg_topic_name, "/interactive_customer_service/message/customer");
    node_handle.param<std::string>("pub_robot_msg_topic_name",    pub_robot_msg_topic_name,    "/interactive_customer_service/message/robot");
    node_handle.param<std::string>("sub_robot_status_topic_name", sub_robot_status_topic_name, "/interactive_customer_service/robot_status");

    ros::Time waiting_start_time;

    ros::Subscriber sub_msg   = node_handle.subscribe<interactive_customer_service::Conversation>(sub_customer_msg_topic_name, 100, &InteractiveCustomerServiceTeleopKey::messageCallback, this);
    ros::Subscriber sub_state = node_handle.subscribe<interactive_customer_service::RobotStatus >(sub_robot_status_topic_name, 100, &InteractiveCustomerServiceTeleopKey::robotStatusCallback, this);

    pub_msg_ = node_handle.advertise<interactive_customer_service::Conversation>(pub_robot_msg_topic_name, 10);

    while (ros::ok())
    {
      if(canReceive(kfd))
      {
        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0)
        {
          perror("read():");
          exit(EXIT_FAILURE);
        }

        switch(c)
        {
          case KEYCODE_1:
          {
            sendMessage(pub_msg_, MSG_ROBOT_MESSAGE, "There are several candidates. Is it green?");
            break;
          }
          case KEYCODE_2:
          {
            sendMessage(pub_msg_, MSG_ROBOT_MESSAGE, "Is it the bigger one?");
            break;
          }
          case KEYCODE_3:
          {
            sendMessage(pub_msg_, MSG_ROBOT_MESSAGE, "Is this what you want?");
            break;
          }
          case KEYCODE_4:
          {
            sendMessage(pub_msg_, MSG_TAKE_ITEM, "11_xylitol-1000");
            break;
          }
          case KEYCODE_5:
          {
            sendMessage(pub_msg_, MSG_TAKE_ITEM, "chipstar_consomme-2000");
            break;
          }
          case KEYCODE_6:
          {
            sendMessage(pub_msg_, MSG_TAKE_ITEM, "irohasu-3000");
            break;
          }
          case KEYCODE_7:
          {
            sendMessage(pub_msg_, MSG_GIVE_ITEM);
            break;
          }
          case KEYCODE_9:
          {
            sendMessage(pub_msg_, MSG_GIVE_UP);
            break;
          }
        }
      }

      ros::spinOnce();

      loop_rate.sleep();
    }

    /////////////////////////////////////////////
    // cooked mode
    tcsetattr(kfd, TCSANOW, &cooked);
    /////////////////////////////////////////////

    return EXIT_SUCCESS;
  }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "interactive_customer_teleop_key");

  InteractiveCustomerServiceTeleopKey teleopkey;
  return teleopkey.run(argc, argv);
};

