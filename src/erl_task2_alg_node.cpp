#include "erl_task2_alg_node.h"

ErlTask2AlgNode::ErlTask2AlgNode(void) :
  algorithm_base::IriBaseAlgorithm<ErlTask2Algorithm>(),
    classifier_module("classifier",ros::this_node::getName()),
    tts("tts_module",ros::this_node::getName()),
    head("head_module",ros::this_node::getName()),
    nav_module("nav_module",ros::this_node::getName()),
    devices_module("devices_module",ros::this_node::getName()),
    log_module("log_module",ros::this_node::getName()),
    image_diff("image_diff_module",ros::this_node::getName()),
    referee(roah_rsbb_comm_ros::Benchmark::HWV,"task2_referee",ros::this_node::getName())
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]
    this->hasCalled = false;
    this->t2_m_s =  T2_START;
    this->t2_a_s = act_greet;
    this->current_person = Unknown;
    this->visitors_counter = 0;
    this->isWaiting = false;
    this->classification_retries = 0;
    for (size_t i = 0; i<4; ++i) seen_people.push_back(false);

  // [init publishers]

  // [init subscribers]

  // [init services]

  // [init clients]

  // [init action servers]

  // [init action clients]
}

ErlTask2AlgNode::~ErlTask2AlgNode(void)
{
  // [free dynamic memory]
}
void ErlTask2AlgNode::retryOrGetHighest(const float acc){
  if (this->classification_retries < this -> config_.max_retries){
    if (this->classification_retries == 0){
      this->most_probable_person = this->current_person;
      this->highest_accuracy = acc;
    }else {
      if (acc>this->highest_accuracy){
        this->most_probable_person = this->current_person;
        this->highest_accuracy = acc;
      }
    }
    this->classification_retries ++;
    this->t2_m_s = T2_CLASSIFY;
  }
  else {
      this->t2_m_s = T2_ACT;
        this->current_person = this->most_probable_person;
        seen_people[this->current_person] = true;
  }

}
bool ErlTask2AlgNode::chooseIfCorrectPerson (const std::string &label,const float acc){
  if (acc>=0.9 && !seen_people[this->current_person] && this->current_person == Kimble){
    std::string p2,p3,err2,err3;
    float acc2,acc3;
    bool result1 = this->classifier_module.classify_current_person(p2,acc2,err2);
    bool result2 = this->classifier_module.classify_current_person(p3,acc3,err3);
    if (result1 && result2 && err2==err3 && err2 == ""){
      if (p2==p3 && p3 == currentPersonStr()){
        seen_people[this->current_person] = true;
        this->t2_m_s = T2_ACT;
      }
    }
  }
  else {
  if (acc>=0.9 && !seen_people[this->current_person]){
    //We assume the classification is correct.
    seen_people[this->current_person] = true;
    this->t2_m_s = T2_ACT;

  }
  else {
    if (seen_people[this->current_person]){
      //if the person has been seen, retry.
      retryOrGetHighest(acc);
    }
    else {
      //if the classification has more than half accuracy, retry only once, and compare
      if (acc>=0.5){
        std::string aux_lab, err_;
        float aux_acc;
        Person first_person = this->current_person;
        bool result = this->classifier_module.classify_current_person(aux_lab,aux_acc,err_);
        if (result and labelToPerson(aux_lab)){
          if (this->current_person == first_person && aux_acc>=0.5){
          seen_people[this->current_person] = true;
          this->t2_m_s = T2_ACT;

          }
          else {
            retryOrGetHighest(acc);
          }
        }
        else {
          retryOrGetHighest(acc);
        }


      }
      else {
        retryOrGetHighest(acc);
      }
    }
  }
}
  return true;
}

bool ErlTask2AlgNode::labelToPerson (const std::string & label){
  if (label==this->config_.person_unknown){
    this->current_person = Unknown;
    return true;
  } else if (label == this->config_.person_kimble){
    this->current_person = Kimble;
    return true;
  } else if (label == this->config_.person_deliman){
    this->current_person = Deliman;
    return true;
  } else if (label == this->config_.person_postman) {
    this->current_person = Postman;
    return true;
  }
  return false;
}


std::string ErlTask2AlgNode::currentPersonStr (){
  if (this->current_person==Unknown){
    return this->config_.person_unknown;
  } else if (this->current_person == Kimble){
    return this->config_.person_kimble;
  }else if (this->current_person == Deliman){
    return this->config_.person_deliman;
  }else if (this->current_person == Postman){
    return this->config_.person_postman;
  }
  else return "";

}

bool ErlTask2AlgNode::action_greet(){
  static bool is_sentence_sent = false;
  std::string sentence;
  if (!is_sentence_sent){
  switch (this->current_person){
    case Deliman:
      sentence = "Hello, I am coming to get the breakfast";
      break;
    case Postman:
      sentence = "Hello, I am coming to get the post mail";
      break;
    case Kimble:
      sentence = "Hello, Doctor Kimble, I am coming to open the door";
      break;
    default:
      sentence = "Sorry, I don't know you. I cannot open the door";
      this->current_person = Unknown;
      break;
  }
    this->log_module.start_logging_audio();
    tts.say(sentence);
    is_sentence_sent = true;
  }
  if (tts.is_finished()){
    if (tts.get_status()==TTS_MODULE_SUCCESS or this->current_action_retries >= this->config_.max_action_retries){
      is_sentence_sent  = false;
      this->current_action_retries = 0;
        this->log_module.stop_logging_audio();
      return true;

    }
    else {
      ROS_INFO ("[TASK2] Nav module finished unsuccessfully. Retrying");
      is_sentence_sent  = false;
      this->current_action_retries ++;
      return false;
    }

  }
  return false;
}
bool ErlTask2AlgNode::action_navigate(){

  std::string POI;
  static bool is_poi_sent = false;
  if (!is_poi_sent){
    nav_module.costmaps_clear();
    switch(this->current_person){
      case Deliman:
        POI = this->config_.kitchen_name;
        break;
      case Postman:
        POI = this->config_.halltable_name;
        break;
      case Kimble:
        POI = this->config_.bedroom_name;
        break;
      default:
        POI = "";
        break;
    }
    nav_module.go_to_poi(POI);
    is_poi_sent = true;
  }
  if (nav_module.is_finished()){
    if (nav_module.get_status()==NAV_MODULE_SUCCESS or this->current_action_retries >= this->config_.max_action_retries){
      is_poi_sent  = false;
      this->current_action_retries = 0;
      return true;

    }
    else {
      ROS_INFO ("[TASK2] Nav module finished unsuccessfully. Retrying");
      is_poi_sent  = false;
      this->current_action_retries ++;
      return false;
    }
  }
  else return false;
}

bool ErlTask2AlgNode::action_gotoIDLE(){
  std::string POI = this->config_.idle_name;
  static bool is_poi_sent = false;
  //first execution : send the poi.
  if (!is_poi_sent){
    nav_module.costmaps_clear();
    nav_module.go_to_poi(POI);
    is_poi_sent = true;
  }
  if (nav_module.is_finished()){
    if (nav_module.get_status()==NAV_MODULE_SUCCESS or this->current_action_retries >= this->config_.max_action_retries){
      is_poi_sent  = false;
      this->current_action_retries = 0;
      return true;

    }
    else {
      ROS_INFO ("[TASK2] Nav module finished unsuccessfully. Retrying");
      is_poi_sent  = false;
      this->current_action_retries ++;
      return false;
    }
  } else return false;
}
bool ErlTask2AlgNode::action_gotodoor(std::string &POI){
  static bool is_poi_sent = false;
  //first execution : send the poi.
  if (!is_poi_sent){

    nav_module.costmaps_clear();
    nav_module.go_to_poi(POI);
    is_poi_sent = true;
  }
  if (nav_module.is_finished()){
    if (nav_module.get_status()==NAV_MODULE_SUCCESS or this->current_action_retries >= this->config_.max_action_retries){
      is_poi_sent  = false;
      this->current_action_retries = 0;
      return true;

    }
    else {
      ROS_INFO ("[TASK2] Nav module finished unsuccessfully. Retrying");
      is_poi_sent  = false;
      this->current_action_retries ++;
      return false;
    }
  } else return false;
}
bool ErlTask2AlgNode::action_say_sentence(const std::string & sentence){
  static bool is_sentence_sent = false;
  if (!is_sentence_sent){
    this->log_module.start_logging_audio();
    tts.say(sentence);
    is_sentence_sent = true;
  }
  else {
    if (tts.is_finished()){
      if (tts.get_status()==TTS_MODULE_SUCCESS or this->current_action_retries >= this->config_.max_action_retries){
        is_sentence_sent  = false;
        this->current_action_retries = 0;
          this->log_module.stop_logging_audio();
        return true;

      }
      else {
        ROS_INFO ("[TASK2] TTS module finished unsuccessfully. Retrying");
        is_sentence_sent  = false;
        this->current_action_retries ++;
        return false;
      }

    }
  }
  return false;
}
bool ErlTask2AlgNode::action_wait_leave(){
  static bool isWaitingDoctor = false;
  static time_t firstTime = time(NULL);
  ROS_INFO ("[TASK2]:Waiting for %d seconds in the bedroom: elapsed:%.f ",this->config_.waiting_time_doctor, difftime(time(NULL),firstTime));
  if (isWaitingDoctor){
    if (difftime(time(NULL),firstTime)>=this->config_.waiting_time_doctor){
      isWaitingDoctor = false;
      return true;
    }
    else return false;
  }
  else {
    firstTime = time(NULL);
    isWaitingDoctor= true;
    return false;
  }
}
bool ErlTask2AlgNode::action_room(){
    bool return_value=false;
    switch(this->current_person){
      case Deliman:
        return (this->action_say_sentence("Please deliver the breakfast on the kitchen table"));
        break;
      case Postman:
        return (this->action_say_sentence("Please leave the mail here in the hall "));
        break;
      case Kimble:
        switch (this->t2_kimble){
          case kimble_reach_bedroom:
            if(this->action_say_sentence("We have arrived at the bedroom, I will wait for you outside")){
              // move to the outside of the room
              this->current_action_retries=0;
              this->nav_module.go_to_poi(this->config_.outside_bedroom_name);
              this->t2_kimble=kimble_go_outside;
            }
            else {
              this->t2_kimble=kimble_reach_bedroom;
            }
            break;
          case kimble_go_outside:
            if (nav_module.is_finished()){
              if (nav_module.get_status()==NAV_MODULE_SUCCESS or this->current_action_retries >= this->config_.max_action_retries){
                this->t2_kimble=kimble_move_head;
                this->current_action_retries=0;
                this->head.move_to(0.0,this->config_.tilt_angle);
              }
              else {
                this->current_action_retries++;
                this->nav_module.go_to_poi(this->config_.outside_bedroom_name);
                this->t2_kimble=kimble_go_outside;
              }
            }
            else
              this->t2_kimble=kimble_go_outside;
            break;
          case kimble_move_head:
            if(this->head.is_finished())
            {
              if (head.get_status()==HEAD_MODULE_SUCCESS or this->current_action_retries >= this->config_.max_action_retries){
                this->t2_kimble=kimble_wait_leave;
                this->current_action_retries=0;
                this->image_diff.set_reference_image();
                this->image_diff.clear_change();
              }
              else {
                this->current_action_retries++;
                this->head.move_to(0.0,this->config_.tilt_angle);
                this->t2_kimble=kimble_move_head;
              }
            }
            else
              this->t2_kimble=kimble_move_head;
            break;
          case kimble_wait_leave:
            if(this->image_diff.has_changed() || this->action_wait_leave())
            {
              // doctor kimble is living
              return_value=true;
              this->t2_kimble=kimble_reach_bedroom;
            }
            else
              this->t2_kimble=kimble_wait_leave;
            break;
        }
        break;
      default:
        return true;
        break;
    }
    return return_value;
}

bool ErlTask2AlgNode::action_algorithm(){
  bool return_value = false;
  switch (this->t2_a_s){
        case act_greet:
          ROS_INFO ("[TASK2]:Greeting the visitor");
          if (this->action_greet()){
            this->t2_a_s = act_gotodoor;
            if (this->current_person == Unknown) {
              this->t2_a_s =  act_greet;
              this->t2_m_s = T2_FINISH;
            }
          }
          break;
        case act_gotodoor:
          ROS_INFO ("[TASK2]:Going to open the door");
          if (this->action_gotodoor( this->config_.entrance_name)){
            this->t2_a_s = act_opendoor;
          }
          else {
            this->t2_a_s = act_gotodoor;
          }
          break;
        case act_opendoor:
          ROS_INFO ("[TASK2]:Requesting to open the door");
          if (this->action_say_sentence("Could you please open the door?")){
            this->t2_a_s = act_askfollow;
          }
          else this->t2_a_s = act_opendoor;
          break;
        case act_askfollow:
            ROS_INFO ("[TASK2]:Requesting to follow");
            if (this->action_say_sentence("Please follow me ")){
              this->t2_a_s = act_navigate;
              //TODO wait for a certain time.
            }
            else this->t2_a_s = act_askfollow;
            break;
        case act_navigate:
          ROS_INFO ("[TASK2]:Navigating to the room");
          if (this->action_navigate()){
            this->t2_a_s = act_actionroom;
          }
          break;
        case act_actionroom:
          ROS_INFO ("[TASK2]:Doing the action in the room");
          if (this->action_room()){
            this->t2_a_s = act_askfollowdoor;
          }
          break;
        case act_askfollowdoor:
            ROS_INFO ("[TASK2]:Requesting to follow");
            if (this->action_say_sentence("You are leaving, please let me walk with you")){
              this->t2_a_s = act_returndoor;
            }
            else this->t2_a_s = act_askfollowdoor;
            break;
        case act_returndoor:
          ROS_INFO ("[TASK2]:Going to the door");
          if (this->action_gotodoor( this->config_.farewell_name)){
            this->t2_a_s = act_saygoodbye;
          }
          else {
            this->t2_a_s = act_returndoor;
          }
          break;
        case act_saygoodbye:
          ROS_INFO ("[TASK2]:Going to the door");
          if (this->action_say_sentence("Here is the door. Goodbye!")){
            this->t2_a_s = act_greet;
            return_value = true;
          }
          else {
            this->t2_a_s = act_saygoodbye;
          }
          break;

  }
  return return_value;

}
void ErlTask2AlgNode::mainNodeThread(void)
{
  // [fill msg structures]

  std::string label;
  float acc;
  std::string error_msg;
  bool result;
  switch (this->t2_m_s){
    case T2_START:

      ROS_INFO("[TASK2] Wait start");
      if(this->referee.execute() or (this->startTask)){
        this->startTask = false;
       this->t2_m_s=T2_WAIT;
       this->log_module.start_data_logging();
      }
      else
        this->t2_m_s=T2_START;
      break;
    case T2_WAIT:
      if (devices_module.listen_bell() or (this->hasCalled)){
            this->t2_m_s = T2_ASKLOOK;
            this->hasCalled = false;
            this->log_module.log_command("ring_bell");
            this->log_module.start_logging_images_front_door();
      } else {
            this-> t2_m_s = T2_WAIT;
      }
      break;
    case T2_ASKLOOK:
        ROS_INFO ("[TASK2]:Requesting to look at camera");
        if (this->action_say_sentence("Please look at the camera")){
              this->t2_m_s = T2_CLASSIFY;
        }
        else this->t2_m_s = T2_ASKLOOK;
        break;
    case T2_CLASSIFY:
      result = this->classifier_module.classify_current_person(label,acc,error_msg);
      if (result) {
        ROS_INFO ("[TASK2]:Successful classifier module->classify_current_person function call");
        ROS_INFO ("[TASK2]:Error : %s\n",error_msg.c_str());
        ROS_INFO ("[TASK2]:Label : %s\n",label.c_str());
        ROS_INFO ("[TASK2]:Accuracy : %f\n",acc);
        if (error_msg == "" || this->classification_retries > this->config_.max_retries){
        if (labelToPerson(label)){
          chooseIfCorrectPerson (label,acc);
          if (this->t2_m_s == T2_ACT){
            this->log_module.log_visitor(currentPersonStr());
            this->log_module.stop_logging_images_front_door();
            this->classification_retries = 0;
          }
        }
        }
        else {
          this->t2_m_s = T2_ASKLOOK;
          this->classification_retries++;
        }

      }
      break;
    case T2_ACT:
      if (this->action_algorithm()){
        this->t2_m_s = T2_RETURNIDLE;
      }
      break;
    case T2_RETURNIDLE:
      if (this->action_gotoIDLE()){
        this->t2_m_s = T2_FINISH;
      }
      break;

    case T2_FINISH:
        this->visitors_counter ++;
        if (this->visitors_counter >= this->visitors_num){
          this->t2_m_s = T2_END;
        }
        else {
          this->t2_m_s = T2_WAIT;
        }
      break;
    case T2_END:
      this->log_module.stop_data_logging();
      ROS_INFO ("[TASK2]:Task2 client :: Finish!");
      this->referee.execution_done();
      break;
   }
  // [fill srv structure and make request to the server]

  // [fill action structure and make request to the action server]

  // [publish messages]
}

/*  [subscriber callbacks] */

/*  [service callbacks] */

/*  [action callbacks] */

/*  [action requests] */

void ErlTask2AlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();

  this->visitors_num = config.visitors_num;
  if (config.start_task){
     this->startTask = config.start_task;
     this->t2_m_s = T2_START;
     this->t2_a_s = act_greet;
  }
  this->hasCalled = config.ring_bell;
  if (config.ring_bell) config.ring_bell = false;
  if (config.start_actions_for_person){
    if (this->labelToPerson(config.person)){
      this->t2_m_s =  T2_ACT;
      this->t2_a_s = act_greet;
    }
    else {
      ROS_INFO ("[TASK2] Person not valid!");
    }
    config.start_actions_for_person = false;
  }
  this->config_=config;
  this->alg_.unlock();
}

void ErlTask2AlgNode::addNodeDiagnostics(void)
{
}

/* main function */
int main(int argc,char *argv[])
{
  return algorithm_base::main<ErlTask2AlgNode>(argc, argv, "erl_task2_alg_node");
}
