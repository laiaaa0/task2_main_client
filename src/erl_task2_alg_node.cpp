#include "erl_task2_alg_node.h"

ErlTask2AlgNode::ErlTask2AlgNode(void) :
  algorithm_base::IriBaseAlgorithm<ErlTask2Algorithm>(),
    tts("tts_module",ros::this_node::getName()),
    nav_module("nav_module",ros::this_node::getName()),
    head("head_module", ros::this_node::getName()),
    devices_module("devices_module",ros::this_node::getName()),
    log_module("log_module",ros::this_node::getName()),
    recognition_module("recognition_module",ros::this_node::getName()),
    referee(roah_rsbb_comm_ros::Benchmark::HWV,"task2_referee",ros::this_node::getName()),
    task2_actions_module("actions_module", ros::this_node::getName())
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]
    this->current_state_ =  T2_WAIT_SERVER_READY;
    this->current_visitor_ = Undefined;
    this->visitors_counter = 0;


    if (this->public_node_handle_.getParam("kimble_path", kimble_path_)){
         ROS_INFO("Kimble path is %s", kimble_path_.c_str());
    }
    if (this->public_node_handle_.getParam("postman_path", postman_path_)){
         ROS_INFO("Postman path is %s", postman_path_.c_str());
    }


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


std::string ErlTask2AlgNode::PersonToString(const Person & person){
    switch (person) {
        case Deliman:
            return this->config_.person_deliman;
        case Postman:
            return this->config_.person_postman;
        case Plumber:
            return this->config_.person_plumber;
        case Kimble:
            return this->config_.person_kimble;
        default :
            return "";
    }
    return "";
}


bool ErlTask2AlgNode::ActionGreet(){
  std::string sentence;
  switch (this->current_visitor_){
    case Deliman:
      sentence = "Hello, thanks for bringing the breakfast";
      break;
    case Postman:
      sentence = "Hello, I am coming to get the post mail";
      break;
    case Kimble:
      sentence = "Hello, Doctor Kimble, thanks for coming";
      break;
    case Plumber:
      sentence = "Hello plumber";
      break;
    case Undefined:
      sentence = "I was not able to recognise you";
      break;
  }
  return this->ActionSaySentence(sentence);
}


bool ErlTask2AlgNode::ActionNavigateToPOI(std::string & POI){
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
  }
  else return false;
}

bool ErlTask2AlgNode::ActionSaySentence(const std::string & sentence){
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


void ErlTask2AlgNode::mainNodeThread(void)
{
  // [fill msg structures]
  switch (this->current_state_){

    case T2_WAIT_SERVER_READY:
        ROS_INFO("[TASK2] Waiting store people");
        if (this->recognition_module.IsReady()){
            if (this->recognition_module.StorePostmanAndKimble(this->postman_path_,this->kimble_path_)){
                this->current_state_ = T2_START;
            }
            else {
                ROS_ERROR("[TASK2] Problem loading person images");
                this->current_state_ = T2_START;
            }
        }
        break;

    case T2_START:
      ROS_INFO("[TASK2] Wait start");
      if(this->referee.execute() or (this->config_.start_task)){
         this->config_.start_task = false;
        this->current_state_=T2_WAIT_BELL;
        this->log_module.start_data_logging();
      }
      else
        this->current_state_=T2_START;
      break;

    case T2_WAIT_BELL:
      ROS_INFO("[TASK2] Wait bell");
      if (devices_module.listen_bell() or (this->config_.ring_bell)){
            this->current_state_ = T2_GOTO_DOOR;
	        this->log_module.log_command("ring_bell");
            this->config_.ring_bell = false;
      } else {
            this-> current_state_ = T2_WAIT_BELL;
      }
      break;

    case T2_GOTO_DOOR:
        ROS_INFO("[TASK2] Going to door");
        if (this->ActionNavigateToPOI(this->config_.door_poi)){
            this->current_state_ = T2_OPENDOOR;
        }
        else {
            this->current_state_ = T2_GOTO_DOOR;
        }
        break;

    case T2_OPENDOOR:
        ROS_INFO ("[TASK2]:Requesting to open door");
        if (this->ActionSaySentence("Please open the door")){
              this->current_state_ = T2_LOOKUP;
              this->head.move_to(this->config_.pan_angle_straight, this->config_.tilt_angle_straight);
        }
        else this->current_state_ = T2_OPENDOOR;
        break;

    case T2_LOOKUP:
        if (this->head.is_finished()){
            this->current_state_ = T2_RECOGNISE;
            recognition_module.StartRecognition();
        }
        else {
            this->current_state_ = T2_LOOKUP;
        }

        break;
    case T2_RECOGNISE:
        if (recognition_module.is_finished()){
	    if (recognition_module.get_status() == T2_RECOGNITION_SUCCESS){
            	this->current_visitor_ = recognition_module.GetCurrentPerson();
            	this->log_module.log_visitor(this->PersonToString(this->current_visitor_));
            	this->current_state_ = T2_GREET;

	    }
        }
        else {
            this->current_state_ = T2_RECOGNISE;
        }
      break;

    case T2_GREET:
        if (this->ActionGreet()){
              this->current_state_ = T2_ACTION;
              task2_actions_module.StartActions(this->current_visitor_);
        }
        else this->current_state_ = T2_GREET;
        break;

    case T2_ACTION:
      if (task2_actions_module.is_finished()) {
          this->current_state_ = T2_RETURNIDLE;
      }
      else {
          this->current_state_ = T2_ACTION;
      }
      break;

    case T2_RETURNIDLE:
      if (this->ActionNavigateToPOI(this->config_.home_poi)){
        this->current_state_ = T2_FINISH;
      }
      break;

    case T2_FINISH:
        this->visitors_counter ++;
        if (this->visitors_counter >= this->config_.visitors_num){
          this->current_state_ = T2_END;
        }
        else {
          this->current_state_ = T2_WAIT_BELL;
        }
        break;

    case T2_END:
      this->log_module.stop_data_logging();
      ROS_INFO ("[TASK2]:Task2 client :: Finish!");
      this->referee.execution_done();
      break;
   }

}


void ErlTask2AlgNode::node_config_update(Config &config, uint32_t level)
{
  this->alg_.lock();
  this->config_ = config;
  //If either of these are set to true,
  // their value will be stored in config_ variable.
  //Set them to false again.
  config.start_task = false;
  config.ring_bell = false;
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
