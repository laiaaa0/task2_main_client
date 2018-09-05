// Copyright (C) 2010-2011 Institut de Robotica i Informatica Industrial, CSIC-UPC.
// Author
// All rights reserved.
//
// This file is part of iri-ros-pkg
// iri-ros-pkg is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
// IMPORTANT NOTE: This code has been generated through a script from the
// iri_ros_scripts. Please do NOT delete any comments to guarantee the correctness
// of the scripts. ROS topics can be easly add by using those scripts. Please
// refer to the IRI wiki page for more information:
// http://wikiri.upc.es/index.php/Robotics_Lab

#ifndef _erl_task2_alg_node_h_
#define _erl_task2_alg_node_h_

#include <iri_base_algorithm/iri_base_algorithm.h>
#include "erl_task2_alg.h"

#include <tiago_modules/tts_module.h>
#include <tiago_modules/nav_module.h>
#include <tiago_modules/head_module.h>
#include "log_modules/log_module.h"
#include <task_state_controller/task_state_controller.h>
#include <devices_manager/devices_manager.h>
#include <time.h>
#include <task2_recognition.h>
#include <task2_visitor_actions.h>

// [publisher subscriber headers]

// [service client headers]

// [action server client headers]


typedef enum {
    T2_START, // Initialize everything with the referee. Go to idle position.
    T2_WAIT_BELL,  // Wait for the bell to ring
    T2_OPENDOOR,  // Ask the visitor to Open the door
    T2_RECOGNISE, // Call the recognition module to know who the visitor is
    T2_GREET, // State who the visitor is and greet him.
    T2_ACTION, // Call the visitor action module. Do different tasks for each visitor
    T2_RETURNIDLE, // State that returns the robot to the idle position. Then, decides if continue waiting for bell or end.
    T2_END} TASK2_MAIN_STATES;



#ifndef _PERSON_DEFINITION_
#define _PERSON_DEFINITION_
 typedef enum {Deliman, Postman, Kimble, Plumber, Undefined} Person;
#endif

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class ErlTask2AlgNode : public algorithm_base::IriBaseAlgorithm<ErlTask2Algorithm>
{
  private:

    Config config_;

  //Modules
    //Device manager module (bell)
    CDevicesManagerModule devices_module;
    //Text to speech module
    CTTSModule tts;
    // Head module
    CHeadModule head;
    //Navigation module
    CNavModule nav_module;
    //Referee module
    CTaskStateControllerModule referee;
    //Log module
    CLogModule log_module;
    //task2 recognition Module
    CTask2Recognition recognition_module;
    //task2 action module
    CTask2VisitorActions task2_actions_module;


    Person current_visitor_;
    int visitors_counter;
    int visitors_num;
    int classification_retries;
    int current_action_retries;

    //State machines
    TASK2_MAIN_STATES t2_m_s;




    bool action_greet();
    bool action_navigate();
    bool action_say_sentence(const std::string & sentence);
    bool action_gotoIDLE();
    bool labelToPerson (const std::string & label);


  public:
   /**
    * \brief Constructor
    *
    * This constructor initializes specific class attributes and all ROS
    * communications variables to enable message exchange.
    */
    ErlTask2AlgNode(void);

   /**
    * \brief Destructor
    *
    * This destructor frees all necessary dynamic memory allocated within this
    * this class.
    */
    ~ErlTask2AlgNode(void);


  protected:
   /**
    * \brief main node thread
    *
    * This is the main thread node function. Code written here will be executed
    * in every node loop while the algorithm is on running state. Loop frequency
    * can be tuned by modifying loop_rate attribute.
    *
    * Here data related to the process loop or to ROS topics (mainly data structs
    * related to the MSG and SRV files) must be updated. ROS publisher objects
    * must publish their data in this process. ROS client servers may also
    * request data to the corresponding server topics.
    */
    void mainNodeThread(void);

   /**
    * \brief dynamic reconfigure server callback
    *
    * This method is called whenever a new configuration is received through
    * the dynamic reconfigure. The derivated generic algorithm class must
    * implement it.
    *
    * \param config an object with new configuration from all algorithm
    *               parameters defined in the config file.
    * \param level  integer referring the level in which the configuration
    *               has been changed.
    */
    void node_config_update(Config &config, uint32_t level);

   /**
    * \brief node add diagnostics
    *
    * In this abstract function additional ROS diagnostics applied to the
    * specific algorithms may be added.
    */
    void addNodeDiagnostics(void);

    // [diagnostic functions]

    // [test functions]
};

#endif
