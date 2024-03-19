#include <stdio.h>
#include <signal.h>
#include <termios.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <interactive_customer_service/Conversation.h>
#include <interactive_customer_service/RobotStatus.h>
#include <nodelet/nodelet.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace cv;

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
  
  const std::string OBJ1 = "11_xylitol-1000";
  const std::string OBJ2 = "chipstar_consomme-2000";
  const std::string OBJ3 = "irohasu-3000";
  
  ros::Publisher pub_msg_;
  
  void showHelp()
  {
    puts("Operate by Keyboard");
    puts("---------------------------");
//    puts(("0 : Send "+MSG_I_AM_READY).c_str());
    puts( "1 : Send Message : 'There are several candidates. Is it green?'");
    puts( "2 : Send Message : 'Is it the bigger one?'");
    puts( "3 : Send Message : 'Is this what you want?'");
    puts(("4 : Take "+OBJ1).c_str());
    puts(("5 : Take "+OBJ2).c_str());
    puts(("6 : Take "+OBJ3).c_str());
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

  void customerImageCallback(const sensor_msgs::ImageConstPtr& image)
  {
    ROS_INFO("Subscribe Image");
    
    cv_bridge::CvImagePtr cv_ptr;
    
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::BGR8);

      // Display Image
      cv::Mat image_mat = cv_ptr->image;
      cv::imshow("Customer Image", image_mat);
      cv::waitKey(3000);
      cv::destroyAllWindows();

      // Save the customer image to home directory
      cv::imwrite("../CustomerImage.jpg", image_mat); // current path=/home/username/.ros
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
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

    ros::Time waiting_start_time;

    ros::Subscriber sub_msg   = node_handle.subscribe<interactive_customer_service::Conversation>("/interactive_customer_service/message/customer", 100, &InteractiveCustomerServiceTeleopKey::messageCallback, this);
    ros::Subscriber sub_state = node_handle.subscribe<interactive_customer_service::RobotStatus >("/interactive_customer_service/robot_status",     100, &InteractiveCustomerServiceTeleopKey::robotStatusCallback, this);
    ros::Subscriber sub_image = node_handle.subscribe<sensor_msgs::Image>                        ("/interactive_customer_service/customer_image",   100, &InteractiveCustomerServiceTeleopKey::customerImageCallback, this);

    pub_msg_ = node_handle.advertise<interactive_customer_service::Conversation>("/interactive_customer_service/message/robot", 10);

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
            sendMessage(pub_msg_, MSG_TAKE_ITEM, OBJ1);
            break;
          }
          case KEYCODE_5:
          {
            sendMessage(pub_msg_, MSG_TAKE_ITEM, OBJ2);
            break;
          }
          case KEYCODE_6:
          {
            sendMessage(pub_msg_, MSG_TAKE_ITEM, OBJ3);
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

