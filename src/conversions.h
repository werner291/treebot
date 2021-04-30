//
// Created by werner on 30-04-21.
//

#ifndef TREEBOT_CONVERSIONS_H
#define TREEBOT_CONVERSIONS_H

#include <ompl/base/MotionValidator.h>

/***
 * Write the SE3State information into a named floating joint in the given RobotState.
 *
 * @param st A state assumed to belong to `SE3StateSpace`
 * @param rs A RobotState so write the state information into.
 * @param name Name of the floating joint to write to.
 */
void setFloatingJointFromSE3(const ompl::base::State *st, moveit::core::RobotState &rs, const char *name);

/***
 * Convert an SE3 state to a MultiDOFJointTrajectoryPoint.
 */
trajectory_msgs::MultiDOFJointTrajectoryPoint stateToTrajectoryPoint(ompl::base::State *st);

/***
 * Convert a set of 7 joint positions into a SE3 state.
 *
 * @param space The state space
 * @param floating_joint_positions Exactly 7 doubles representing position [x,y,z], then orientation [x,y,z,w]
 * @return The SE3 state.
 */
ompl::base::ScopedState<ompl::base::SE3StateSpace> floatingJointPositionsToSE3(std::shared_ptr<ompl::base::StateSpace> space, const double floating_joint_positions[7]);

moveit_msgs::RobotTrajectory trajectoryToMoveit(const ompl::base::PathPtr &path);

#endif //TREEBOT_CONVERSIONS_H
