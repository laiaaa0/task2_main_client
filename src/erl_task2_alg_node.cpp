#include "erl_task2_alg_node.h"

ErlTask2AlgNode::ErlTask2AlgNode(void) :
  algorithm_base::IriBaseAlgorithm<ErlTask2Algorithm>(),
    classifier_module("task_2_client")
{
  //init class attributes if necessary
  //this->loop_rate_ = 2;//in [Hz]
    this->hasCalled = false;
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

void ErlTask2AlgNode::mainNodeThread(void)
{
  // [fill msg structures]
   if (!hasCalled){
    hasCalled = true;
    std::string label;
    float acc;
    std::string error_msg;
    bool result = this->classifier_module.classify_current_person(label,acc,error_msg);
    if (!result){
        ROS_INFO("%s\n",error_msg.c_str());
        if (error_msg=="fail"){
            //Do something
        } else if (error_msg=="pending"){
        
        } else {
        
        }
        
        hasCalled = false;
    }
    else {
        ROS_INFO ("Suceessful classifier module -> classify_current_person function call");
        if (error_msg==""){
        ROS_INFO("Image classified successfuly");
        ROS_INFO("%s\n",label.c_str());
        ROS_INFO("%f\n",acc);
        }
        else {
        ROS_INFO("%s\n",error_msg.c_str());
        }
    }
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
