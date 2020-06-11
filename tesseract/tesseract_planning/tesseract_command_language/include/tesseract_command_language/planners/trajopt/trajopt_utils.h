#ifndef TESSERACT_COMMAND_LANGUAGE_TRAJOPT_UTILS_H
#define TESSERACT_COMMAND_LANGUAGE_TRAJOPT_UTILS_H

#include <tesseract_command_language/cartesian_waypoint.h>
#include <tesseract_command_language/joint_waypoint.h>
#include <trajopt/problem_description.hpp>

namespace tesseract_planning
{
trajopt::TermInfo::Ptr createCartesianWaypointTermInfo(const CartesianWaypoint& c_wp,
                                                       int index,
                                                       std::string working_frame,
                                                       Eigen::Isometry3d tcp,
                                                       const Eigen::VectorXd& coeffs,
                                                       std::string link,
                                                       trajopt::TermType type);

trajopt::TermInfo::Ptr createDynamicCartesianWaypointTermInfo(const CartesianWaypoint &c_wp,
                                                              int index,
                                                              std::string working_frame,
                                                              const Eigen::Isometry3d& tcp,
                                                              const Eigen::VectorXd& coeffs,
                                                              const std::string& link,
                                                              trajopt::TermType type);

trajopt::TermInfo::Ptr createNearJointStateTermInfo(const JointWaypoint& target,
                                                    const std::vector<std::string>& joint_names,
                                                    int index,
                                                    const Eigen::VectorXd& coeffs,
                                                    trajopt::TermType type);


trajopt::TermInfo::Ptr createJointWaypointTermInfo(const JointWaypoint& j_wp,
                                                   int index,
                                                   const Eigen::VectorXd& coeffs,
                                                   trajopt::TermType type);

trajopt::TermInfo::Ptr createCollisionTermInfo(
    int start_index,
    int end_index,
    double collision_safety_margin,
    double collision_safety_margin_buffer,
    trajopt::CollisionEvaluatorType evaluator_type,
    bool use_weighted_sum = false,
    double coeff = 20.0,
    tesseract_collision::ContactTestType contact_test_type = tesseract_collision::ContactTestType::ALL,
    double longest_valid_segment_length = 0.5,
    trajopt::TermType type = trajopt::TermType::TT_COST);

trajopt::TermInfo::Ptr createSmoothVelocityTermInfo(int start_index,
                                                    int end_index,
                                                    int n_joints,
                                                    double coeff = 5.0,
                                                    trajopt::TermType type = trajopt::TermType::TT_COST);

trajopt::TermInfo::Ptr createSmoothVelocityTermInfo(int start_index,
                                                    int end_index,
                                                    const Eigen::Ref<const Eigen::VectorXd>& coeff,
                                                    trajopt::TermType type = trajopt::TermType::TT_COST);

trajopt::TermInfo::Ptr createSmoothAccelerationTermInfo(int start_index,
                                                        int end_index,
                                                        int n_joints,
                                                        double coeff = 1.0,
                                                        trajopt::TermType type = trajopt::TermType::TT_COST);

trajopt::TermInfo::Ptr createSmoothAccelerationTermInfo(int start_index,
                                                        int end_index,
                                                        const Eigen::Ref<const Eigen::VectorXd>& coeff,
                                                        trajopt::TermType type = trajopt::TermType::TT_COST);

trajopt::TermInfo::Ptr createSmoothJerkTermInfo(int start_index,
                                                int end_index,
                                                int n_joints,
                                                double coeff = 1.0,
                                                trajopt::TermType type = trajopt::TermType::TT_COST);

trajopt::TermInfo::Ptr createSmoothJerkTermInfo(int start_index,
                                                int end_index,
                                                const Eigen::Ref<const Eigen::VectorXd>& coeff,
                                                trajopt::TermType type = trajopt::TermType::TT_COST);

trajopt::TermInfo::Ptr createUserDefinedTermInfo(int start_index,
                                                 int end_index,
                                                 sco::VectorOfVector::func error_function,
                                                 sco::MatrixOfVector::func jacobian_function,
                                                 trajopt::TermType type);

trajopt::TermInfo::Ptr createAvoidSingularityTermInfo(int start_index,
                                                      int end_index,
                                                      const std::string& link,
                                                      double coeff = 5.0,
                                                      trajopt::TermType type = trajopt::TermType::TT_COST);
}

#endif // TESSERACT_COMMAND_LANGUAGE_TRAJOPT_UTILS_H
