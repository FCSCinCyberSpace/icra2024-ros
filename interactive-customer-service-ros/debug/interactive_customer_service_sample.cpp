#include <ros/ros.h>
#include <std_msgs/String.h>
#include <interactive_customer_service/Conversation.h>
#include <interactive_customer_service/RobotStatus.h>
#include <nodelet/nodelet.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

class InteractiveCustomerServiceSample
{
private:
  enum Step
  {
    Initialize,
    Ready,
    WaitForInstruction,
    TakeItem,
    WaitToTakeItem,
    GiveItem,
    WaitForResult,
  };

  const std::string MSG_ARE_YOU_READY       = "Are_you_ready?";
  const std::string MSG_CUSTOMER_MESSAGE    = "customer_message";
  const std::string MSG_ROBOT_MSG_SUCCEEDED = "robot_message_succeeded";
  const std::string MSG_ROBOT_MSG_FAILED    = "robot_message_failed";
  const std::string MSG_TAKE_ITEM_SUCCEEDED = "take_item_succeeded";
  const std::string MSG_TAKE_ITEM_FAILED    = "take_item_failed";
  const std::string MSG_GIVE_ITEM_SUCCEEDED = "give_item_succeeded";
  const std::string MSG_GIVE_ITEM_FAILED    = "give_item_failed";
  const std::string MSG_TASK_SUCCEEDED      = "Task_succeeded";
  const std::string MSG_TASK_FAILED         = "Task_failed";
  const std::string MSG_MISSION_COMPLETE    = "Mission_complete";
  const std::string MSG_YES                 = "Yes";
  const std::string MSG_NO                  = "No";
  const std::string MSG_I_DONT_KNOW         = "I don't know";

  const std::string MSG_I_AM_READY     = "I_am_ready";
  const std::string MSG_ROBOT_MESSAGE  = "robot_message";
  const std::string MSG_TAKE_ITEM      = "take_item";
  const std::string MSG_GIVE_ITEM      = "give_item";
  const std::string MSG_GIVE_UP        = "Give_up";
  
  const std::string STATE_STANDBY              = "standby";
  const std::string STATE_IN_CONVERSATION      = "in_conversation";
  const std::string STATE_MOVING               = "moving";
  
  int step_;

  std::string instruction_msg_;
  std::string customer_msg_;
  std::string robot_state_;
  bool        speaking_;
  std::string grasped_item_;

  bool is_started_;
  bool is_succeeded_;
  bool is_failed_;

  void reset()
  {
	ROS_INFO("Reset Parameters");
	  
    instruction_msg_ = "";
    customer_msg_    = "";
    robot_state_     = "";
    speaking_        = false;
    grasped_item_    = "";
    is_started_   = false;
    is_succeeded_ = false;
    is_failed_    = false;
  }


  void messageCallback(const interactive_customer_service::Conversation::ConstPtr& message)
  {
    ROS_INFO("Subscribe message:%s, %s", message->type.c_str(), message->detail.c_str());

    if(message->type.c_str()==MSG_ARE_YOU_READY)
    {
      if(step_==Ready)
      {
        is_started_ = true;
      }
    }
    if(message->type.c_str()==MSG_CUSTOMER_MESSAGE)
    {
      if(step_==WaitForInstruction)
      {
        instruction_msg_ = message->detail.c_str();
      }
    }
    if(message->type.c_str()==MSG_TAKE_ITEM_FAILED || message->type.c_str()==MSG_GIVE_ITEM_FAILED)
    {
      is_failed_ = true;
    }
    if(message->type.c_str()==MSG_TASK_SUCCEEDED)
    {
      is_succeeded_ = true;
    }
    if(message->type.c_str()==MSG_TASK_FAILED)
    {
      is_failed_ = true;
    }
    if(message->type.c_str()==MSG_MISSION_COMPLETE)
    {
      exit(EXIT_SUCCESS);
    }
  }

  void robotStatusCallback(const interactive_customer_service::RobotStatus::ConstPtr& status)
  {
    ROS_DEBUG("Subscribe robot status:%s, %d, %s", status->state.c_str(), status->speaking, status->grasped_item.c_str());

    robot_state_  = status->state.c_str();
    speaking_     = status->speaking;
    grasped_item_ = status->grasped_item.c_str();
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


public:
  void getObjectList1(std::string object_list[99])
  {
    object_list[0] = "pan_meat-1000";
    object_list[1] = "pepper_steak-1000";
    object_list[2] = "tabasco-1000";
    object_list[3] = "sb_honwasabi-1000";
    object_list[4] = "house_ginger-1000";
    object_list[5] = "house_karasi-1000";
    object_list[6] = "kizami_aojiso-1000";
    object_list[7] = "kikkoman_soysauce-1000";
    object_list[8] = "kikkoman_genen_soysauce-1000";
    object_list[9] = "7i_ginger-1000";
    object_list[10] = "7i_garlic-1000";
    object_list[11] = "7i_neri_karasi-1000";
    object_list[12] = "donbee_udon-1000";
    object_list[13] = "donbee_soba";
    object_list[14] = "midorinotanuki-1000";
    object_list[15] = "midorinotanuki_mini-1000";
    object_list[16] = "akaikitsune-1000";
    object_list[17] = "akaikitsune_mini";
    object_list[18] = "sumire";
    object_list[19] = "santouka-1000";
    object_list[20] = "ippudou-1000";
    object_list[21] = "cupnoodle";
    object_list[22] = "cupnoodle_seafood";
    object_list[23] = "cupnoodle_curry";
    object_list[24] = "cupnoodle_chilitomato";
    object_list[25] = "seven_tappuriguzai_seafoodnoodle-3000";
    object_list[26] = "seven_syouyu";
    object_list[27] = "seven_curry";
    object_list[28] = "bubuka_aburasoba-1000";
    object_list[29] = "ippeichan";
    object_list[30] = "seven_shijimi";
    object_list[31] = "seven_nori";
    object_list[32] = "seven_asari";
    object_list[33] = "seven_hourensoutotamago";
    object_list[34] = "oi_ocha_525ml-3000";
    object_list[35] = "Oi_Ocha_350-3000";
    object_list[36] = "Suntory_iemon600ml-3000";
    object_list[37] = "16cha_660-3000";
    object_list[38] = "Sou_ken_bitya600ml-3000";
    object_list[39] = "ayataka_houjicha-3000";
    object_list[40] = "itoen_mugicha_670ml-3000";
    object_list[41] = "cocacola_300ml-3000";
    object_list[42] = "7i_sparkling_water_lemon-3000";
    object_list[43] = "irohasu-3000";
    object_list[44] = "ion_water-3000";
    object_list[45] = "7i_tennensui_550ml-3000";
    object_list[46] = "GOMAMUGICHA-3000";
    object_list[47] = "Kao_herusiaryokutya_350ml-3000";
    object_list[48] = "Karadasukoyakatya_W-3000";
    object_list[49] = "kuro_uroncha_2-3000";
    object_list[50] = "suntory_uron-3000";
    object_list[51] = "gogotea_straight-3000";
    object_list[52] = "mintia_yogurt-1000";
    object_list[53] = "halls_ocean_blue-1000";
    object_list[54] = "vc_3000_candy-1000";
    object_list[55] = "honey_peach_candy-2000";
    object_list[56] = "ryukakusan_nodoame_hukuro-1000";
    object_list[57] = "chupachups_cola-1000";
    object_list[58] = "chupachups_strawberry_cream-1000";
    object_list[59] = "chupachups_strawberry-1000";
    object_list[60] = "chupachups_yuzu-1000";
    object_list[61] = "chupachups_grape-1000";
    object_list[62] = "chupachups_soda-1000";
    object_list[63] = "chupachups_marron_latte-1000";
    object_list[64] = "glico_almond-1000";
    object_list[65] = "lotte_almond_crisp-1000";
    object_list[66] = "lotte_almond_choco-1000";
    object_list[67] = "macadamia-1000";
    object_list[68] = "takenoko-1000";
    object_list[69] = "21_mt_kinoko-1000";
    object_list[70] = "19_pocky-1000";
    object_list[71] = "pocky-1000";
    object_list[72] = "15_toppo-1000";
    object_list[73] = "7i_chocomill-1000";
    object_list[74] = "7i_choco_chip-1000";
    object_list[75] = "16_choco_rusk-1000";
    object_list[76] = "giant_caplico-1000";
    object_list[77] = "dars_white-1000";
    object_list[78] = "dars_milk-1000";
    object_list[79] = "ghana_milk-1000";
    object_list[80] = "super_big_choco-1000";
    object_list[81] = "20_meltykiss-1000";
    object_list[82] = "pie_no_mi-1000";
    object_list[83] = "28_koalas_march-1000";
    object_list[84] = "xylitol_freshmint-1000";
    object_list[85] = "11_xylitol-1000";
    object_list[86] = "13_clorets-1000";
    object_list[87] = "xylitol_peach-1000";
    object_list[88] = "xylitol_grape-1000";
    object_list[89] = "xylitol_white_pink-1000";
    object_list[90] = "xylitol_uruoi-3000";
    object_list[91] = "green_gum-1000";
    object_list[92] = "tooth_lemon_gum-1000";
    object_list[93] = "lotte_kiokuryoku_tsubugum-1000";
    object_list[94] = "7i_kaisengonomi-1000";
    object_list[95] = "7i_ebi_mirin_yaki-1000";
    object_list[96] = "7i_ikasenbei-1000";
    object_list[97] = "calbee_potatochips_norisio-3000";
    object_list[98] = "consomme_w_punch-3000";
  }

  void getObjectList2(std::string object_list[79])
  {
    object_list[0] = "22_pretz-1000";
    object_list[1] = "chipstar_consomme-2000";
    object_list[2] = "calbee_jagariko_bits_sarada-1000";
    object_list[3] = "calbee_jagariko_bits_jagabutter-1000";
    object_list[4] = "eda_mariko-1000";
    object_list[5] = "toumoriko-1000";
    object_list[6] = "miino_soramame_2-1000";
    object_list[7] = "17_butter_cookie-1000";
    object_list[8] = "7i_cheese_in_3-3000";
    object_list[9] = "seven_sauce_mayo_monja-1000";
    object_list[10] = "06_mentai_cheese-3000";
    object_list[11] = "26_ottotto-1000";
    object_list[12] = "bisco_0-3000";
    object_list[13] = "umaibou_cheese-1000";
    object_list[14] = "bigkatu-1000";
    object_list[15] = "goldencurry_tyu_kara-1000";
    object_list[16] = "javacurry_tyu_kara-1000";
    object_list[17] = "vermontcurry_tyu_kara-1000";
    object_list[18] = "vermontcurry_amakuchi-1000";
    object_list[19] = "seven_kokutoumamicurry_tyu_kara-1000";
    object_list[20] = "7i_kokutoumamicurry_amakuchi-1000";
    object_list[21] = "seven_kokutoumamicurry_karakuvhi-1000";
    object_list[22] = "7i_cream_stew-1000";
    object_list[23] = "house_mixtew_cream-1000";
    object_list[24] = "corn_potage-1000";
    object_list[25] = "7i_potato_potage-1000";
    object_list[26] = "18_wakame_soup-1000";
    object_list[27] = "oi_ocha_tea_bag-1000";
    object_list[28] = "07_green_tea-1000";
    object_list[29] = "7i_genmaicha-3000";
    object_list[30] = "7i_houjicha-1000";
    object_list[31] = "calorie_mate_fruit-1000";
    object_list[32] = "calorie_mate_choco-1000";
    object_list[33] = "calorie_mate_choco_2p-1000";
    object_list[34] = "calorie_mate_cheese_2p-1000";
    object_list[35] = "yamitsuki_hormone-1000";
    object_list[36] = "maruzen_cheese_chikuwa-1000";
    object_list[37] = "mituboshi_gurume_premium-1000";
    object_list[38] = "creap-1000";
    object_list[39] = "clinica_ad_rinse-3000";
    object_list[40] = "nonio_mintpaste-1000";
    object_list[41] = "gum_paste-1000";
    object_list[42] = "clinica_ad_hamigaki-1000";
    object_list[43] = "yuskina-1000";
    object_list[44] = "atrix_handcream_medicated-1000";
    object_list[45] = "mentholatum_handveil-1000";
    object_list[46] = "sekkisui_white_washing_cream-1000";
    object_list[47] = "sekkisui_sengan-1000";
    object_list[48] = "nivea_cream-1000";
    object_list[49] = "nivea_soft-1000";
    object_list[50] = "uno_serum-1000";
    object_list[51] = "awa_biore-3000";
    object_list[52] = "14_shampoo-3000";
    object_list[53] = "7i_conditioner-1000";
    object_list[54] = "myuzu_kokei-1000";
    object_list[55] = "7i_bathcleaner-1000";
    object_list[56] = "7_eleven_ofuronosenzai-1000";
    object_list[57] = "widehaiter_tumekae-1000";
    object_list[58] = "emal_ayus_relax-1000";
    object_list[59] = "emal_ayus_refresh-3000";
    object_list[60] = "attack_neo_1pack-1000";
    object_list[61] = "top_room25g-1000";
    object_list[62] = "7i_laundry-1000";
    object_list[63] = "7i_sentakuso_cleaner-3000";
    object_list[64] = "ccute_shokusenki_tumekae-1000";
    object_list[65] = "05_jif-1000";
    object_list[66] = "kukutto_clear_refill-1000";
    object_list[67] = "pocket_tissue-1000";
    object_list[68] = "7i_tissue-1000";
    object_list[69] = "kleenex_hadaururu_240-3000";
    object_list[70] = "tissue_miaou-1000";
    object_list[71] = "tissue_hoshitsu-1000";
    object_list[72] = "toilet_magiclean_tumekae-3000";
    object_list[73] = "7i_toiletcleaner-1000";
    object_list[74] = "work_gloves-1000";
    object_list[75] = "megrhythm_lavender-1000";
    object_list[76] = "led60_red-1000";
    object_list[77] = "rope-1000";
    object_list[78] = "iwatani_gas-3000";
  }
  
  int run(int argc, char **argv)
  {
    ros::NodeHandle node_handle;

    ros::Rate loop_rate(10);
    
    ros::Time waiting_start_time;

    ROS_INFO("Interactive Customer Service sample start!");

    ros::Subscriber sub_msg   = node_handle.subscribe<interactive_customer_service::Conversation>("/interactive_customer_service/message/customer", 100, &InteractiveCustomerServiceSample::messageCallback, this);
    ros::Publisher  pub_msg   = node_handle.advertise<interactive_customer_service::Conversation>("/interactive_customer_service/message/robot",     10);
    ros::Subscriber sub_state = node_handle.subscribe<interactive_customer_service::RobotStatus >("/interactive_customer_service/robot_status",     100, &InteractiveCustomerServiceSample::robotStatusCallback, this);
    ros::Subscriber sub_image = node_handle.subscribe<sensor_msgs::Image>                        ("/interactive_customer_service/customer_image",   100, &InteractiveCustomerServiceSample::customerImageCallback, this);

    // Use List1
    std::string object_list[99]; getObjectList1(object_list);
    // Use List2
//    std::string object_list[79]; getObjectList2(object_list);
    
    int objectNo = -1;
    
    reset();
    step_ = Initialize;

    while (ros::ok())
    {
      if(is_succeeded_)
      {
        ROS_INFO("Task succeeded!");
        step_ = Initialize;
      }

      if(is_failed_)
      {
        ROS_INFO("Task failed!");
        step_ = Initialize;
      }

      switch(step_)
      {
        case Initialize:
        {
          reset();
          objectNo++;
          step_++;
          break;
        }
        case Ready:
        {
          if(is_started_)
          {
            sendMessage(pub_msg, MSG_I_AM_READY);

            ROS_INFO("Task start!");

            step_++;
          }
          break;
        }
        case WaitForInstruction:
        {
          if(instruction_msg_!="")
          {
            ROS_INFO("Instruction: %s", instruction_msg_.c_str());

            step_++;
          }
          break;
        }
        case TakeItem:
        {
          if(robot_state_==STATE_IN_CONVERSATION)
          {
            sendMessage(pub_msg, MSG_TAKE_ITEM, object_list[objectNo]);
            step_++;
          }

          break;
        }
        case WaitToTakeItem:
        {
          if(robot_state_==STATE_IN_CONVERSATION && grasped_item_!="")
          {
            step_++;
          }

          break;
        }
        case GiveItem:
        {
          sendMessage(pub_msg, MSG_GIVE_ITEM);
          step_++;

          break;
        }
        case WaitForResult:
        {
          break;
        }
      }

      ros::spinOnce();

      loop_rate.sleep();
    }

    return EXIT_SUCCESS;
  }
};



int main(int argc, char **argv)
{
  ros::init(argc, argv, "interactive_customer_service_sample");

  InteractiveCustomerServiceSample sample;
  return sample.run(argc, argv);
};

