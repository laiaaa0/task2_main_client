#include "erl_task2_alg_node.h"

ErlTask2AlgNode::ErlTask2AlgNode(void) :
  algorithm_base::IriBaseAlgorithm<ErlTask2Algorithm>(),
    classifier_module("classifier","task_2_client"),
    tts_module("tts_module","task_2_client"),
    nav_module("nav_module","task_2_client")
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]
    this->hasCalled = false;
    this->t2_m_s =  task2_Start;
    this->t2_a_s = act_greet;
    this->current_person = Unknown;
    this->visitors_counter = 0;
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
bool ErlTask2AlgNode::labelToPerson (const std::string & label){
  if (label=="Unknown"){
    this -> current_person = Unknown;
  } else if (label == "Kimble"){
    this -> current_person = Kimble;
  } else if (label == "Annie"){
    this -> current_person = Annie;
  } else if (label == "Deliman"){
    this -> current_person = Deliman;
  } else if (label == "Postman") {
      this -> current_person = Postman;
  }
  return true;
}

bool ErlTask2AlgNode::action_greet(){
  static bool sent = false;
  std::string sentence;
  if (!sent){
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
      break;
  }
    tts_module.say(sentence);
    sent = true;
  }
  if (tts_module.is_finished()){
    sent  = false;
    return true;

  }
  return false;
}
bool ErlTask2AlgNode::action_navigate(){
  std::string POI;
  bool is_poi_sent = false;
  if (!is_poi_sent){
    switch(this->current_person){
      case Deliman:
        POI = "Kitchen";
        break;
      case Postman:
        POI = "Entrance";
        break;
      case Kimble:
        POI = "Bedroom";
        break;
      default:
        POI = "";
        break;
    }
    nav_module.go_to_poi(POI);

  }
  if (nav_module.is_finished())
  	return true;
  else return false;


}
bool ErlTask2AlgNode::action_algorithm(){
  bool return_value = false;
  switch (this->t2_a_s){
        case act_greet:
          ROS_INFO ("Greeting the visitor");
          if (this->action_greet()){
            this->t2_a_s = act_gotodoor;
            if (this->current_person == Unknown) {
              this -> t2_a_s =  act_greet;
              this -> t2_m_s = task2_Finish_act;
            }
          }
          break;
        case act_gotodoor:
          ROS_INFO ("Going to open the door");
          this->t2_a_s = act_opendoor;
          break;
        case act_opendoor:
          ROS_INFO ("Requesting to open the door");
          tts_module.say("Could you please open the door?");
          if (tts_module.is_finished()){
            this -> t2_a_s = act_navigate;
          }
          
          this->t2_a_s = act_opendoor;
          break;
        case act_navigate:
          if (this->action_navigate()){
            this->t2_a_s = act_actionroom;
          }
          break;
        case act_actionroom:
          this->t2_a_s = act_wait;
          break;
        case act_wait:
          this->t2_a_s = act_returndoor;
          break;
        case act_returndoor:
          this->t2_a_s = act_greet;
          return_value = true;
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
    case task2_Start:
      // Wait from call from erl_utils > task_state_controller > start_task_2.
      this->t2_m_s = task2_Wait;
      break;
    case task2_Wait:
      // Wait from doorbell
      /*if (CDevicesManagerModule::listen_bell()) {*/
            this->t2_m_s = task2_Classify; /*
      } else {
            this-> t2_m_s = task2_Wait;

      }*/


      break;
    case task2_Classify:
      result = this->classifier_module.classify_current_person(label,acc,error_msg);
      if (result) {
        ROS_INFO("Suceessful classifier module -> classify_current_person function call");
        ROS_INFO("Error : %s\n",error_msg.c_str());
        ROS_INFO("Label : %s\n",label.c_str());
        ROS_INFO("Accuracy : %f\n",acc);
        if (labelToPerson(label)){
          this->t2_m_s = task2_Act;
        }
      }
      break;
    case task2_Act:

      if (this->action_algorithm()){
        this->t2_m_s = task2_Finish_act;

      }
      break;
    case task2_Finish_act:
        this->visitors_counter ++;
        if (this->visitors_counter >= 1){
          this->t2_m_s = task2_End;
        }
        else {
          this->t2_m_s = task2_Wait;
        }
      break;
    case task2_End:
      ROS_INFO ("Task2 client :: Finish!");
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
