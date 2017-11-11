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

#include <erl_classification_modules/person_classification_module.h>
#include <tiago_modules/tts_module.h>
#include <tiago_modules/nav_module.h>
#include <ctime>
// [publisher subscriber headers]

// [service client headers]

// [action server client headers]


typedef enum {task2_Start,task2_Wait, task2_Classify, task2_Act,task2_Finish_act, task2_End} task2_main_states;
typedef enum {act_greet, act_gotodoor, act_opendoor, act_navigate, act_actionroom, act_wait, act_returndoor} task2_act_states;
typedef enum {Deliman, Postman, Kimble, Unknown, Annie} Person;

/**
 * \brief IRI ROS Specific Algorithm Class
 *
 */
class ErlTask2AlgNode : public algorithm_base::IriBaseAlgorithm<ErlTask2Algorithm>
{
  private:
    // [publisher attributes]

    // [subscriber attributes]

    // [service attributes]

    // [client attributes]

    // [action server attributes]

    // [action client attributes]

   /**
    * \brief config variable
    *
    * This variable has all the driver parameters defined in the cfg config file.
    * Is updated everytime function config_update() is called.
    */
    Config config_;
    //CDevicesManagerModule devices_module;
    CPersonClassificationModule classifier_module;
    CTTSModule tts;
    CNavModule nav_module;
    bool hasCalled;
    bool startTask;
    bool isWaiting;
    clock_t waitingTime;
    Person current_person;
    int visitors_counter;
    int visitors_num;
    task2_main_states t2_m_s;
    task2_act_states t2_a_s;
    std::string kitchen_name;
    std::string entrance_name;
    std::string bedroom_name;
    std::vector<bool>seen_people;
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

    bool action_algorithm();
    bool action_greet();
    bool action_opendoor();
    bool action_navigate();
    bool action_room();
    bool action_say_sentence(const std::string & sentence);
    bool action_wait_leave();
    bool action_gotodoor();

    bool wait_result();
    bool labelToPerson (const std::string & label);

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
