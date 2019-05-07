/**
 * @file planner_types.h
 * @brief Planner types.
 *
 * @author Levi Armstrong
 * @date April 18, 2018
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2017, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TESSERACT_PLANNING_PLANNER_TYPES_H
#define TESSERACT_PLANNING_PLANNER_TYPES_H

#include <tesseract_environment/core/environment.h>
#include <tesseract_kinematics/core/forward_kinematics.h>

namespace tesseract_planning
{
/** @brief Negative status code should be related to errors and positive should be used for status of covergenace */
typedef std::unordered_map<int, std::string> StatusCodeMap;

struct PlannerRequest
{
  std::string name;                                           /**< @brief The name of the planner to use */
  tesseract_environment::EnvironmentConstPtr env;             /**< @brief The environment to use for planning */
  tesseract_environment::EnvStateConstPtr start_state;        /**< @brief The start state to use for planning */
  tesseract_kinematics::ForwardKinematicsConstPtrMap kin_map; /**< @brief A map of kinematics objects */
  std::string config;                                         /**< @brief The configuration to use (json file) */
  std::string config_format;                                  /**< @brief The file extension used to parse config */
};

struct PlannerResponse
{
  std::vector<std::string> joint_names; /**< @brief The joint names */
  tesseract_environment::TrajArray trajectory; /**< @brief The generated trajectory */
  int status_code; /**< @brief Negative status code should be related to errors and positive should be used for status
                      of covergenace */
  std::string status_description; /**< @brief Provide a brief description about what the error code means */
};
}

#endif  // TESSERACT_PLANNING_PLANNER_TYPES_H