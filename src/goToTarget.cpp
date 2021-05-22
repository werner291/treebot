//
// Created by werner on 17-05-21.
//


#include "goToTarget.h"


void goToTarget(std::mutex &targets_mutex, const std::vector<Eigen::Vector3d> &latest_targets,
                const std::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> &psm,
                std::_MakeUniq<moveit_visual_tools::MoveItVisualTools>::__single_object &visual_tools,
                std::shared_ptr<ompl::control::SpaceInformation> &si,
                std::shared_ptr<trajectory_execution_manager::TrajectoryExecutionManager> &tem, bool taskCompleted) {
    do {

        visual_tools->deleteAllMarkers();
        visual_tools->trigger();

//        auto planner(std::make_shared<oc::SST>(si));
        auto planner(std::make_shared<ompl::control::PDST>(si));


        planning_scene::PlanningScenePtr ps = snapshotPlanningScene(psm);
        std::vector<Eigen::Vector3d> targets;
        {        const std::lock_guard<std::mutex> lock(targets_mutex);
            targets = latest_targets;
        }

        moveit::core::RobotState current_state = ps->getCurrentState();

        si->setStateValidityChecker(std::make_shared<MoveitStateChecker>(si, ps, current_state));
        si->setup();

        auto space = std::dynamic_pointer_cast<PositionAndHeadingSpace>(si->getStateSpace());

        auto start = moveItStateToPositionAndHeading(space, current_state);

        ompl::base::ScopedState<PositionAndHeadingSpace> goal(space);
        goal->setXYZH(0.0, 0.0, 0.4, 0.0);

        std::shared_ptr<ompl::base::ProblemDefinition> pdef = std::make_shared<ompl::base::ProblemDefinition>(si);
        pdef->setStartAndGoalStates(start, goal, 0.01);



        auto opt(std::make_shared<ompl::base::PathLengthOptimizationObjective>(si));

        pdef->setOptimizationObjective(opt);

        planner->setProblemDefinition(pdef);

        auto planner_start_time = ros::Time::now();

//        ob::PlannerStatus solved = planner->solve(ob::PlannerTerminationCondition([&pdef, planner_start_time]() {
//            printf("Difference: %f", pdef->getSolutionDifference());
//            return pdef->getSolutionDifference() < 0.5 && (ros::Time::now() - planner_start_time) > ros::Duration(20.0);
//        }));
        ompl::base::PlannerStatus solved = planner->ompl::base::Planner::solve(20.0);
//
//        ob::PlannerData pd(si);
//        planner->oc::PDST::getPlannerData(pd);
//        visualizePlannerStates(visual_tools, pd);
//        visual_tools->trigger();

        ROS_INFO("Planner finished with status: %s", solved.asString().c_str());

        if (solved == ompl::base::PlannerStatus::EXACT_SOLUTION || solved == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION) {

            // print the path to screen
            const ompl::base::PathPtr path = pdef->getSolutionPath();

            auto states = path->as<ompl::geometric::PathGeometric>()->getStates();

            path->print(std::cout);

            moveit_msgs::RobotTrajectory robotTrajectory = trajectoryToMoveit(path);

            displayMultiDoFTrajectory(visual_tools, robotTrajectory, current_state);

            tem->push(robotTrajectory);

            bool done = false;
            tem->execute([&done, &taskCompleted](auto status) {
                ROS_INFO("Execution finished.");

                if (status == moveit_controller_manager::ExecutionStatus::SUCCEEDED) {
                    taskCompleted = true;
                }

                done = true;
            });

            ros::Rate rate(5.0);

            while (!done) {

                rate.sleep();

                bool valid = isTrajectoryStillValid(psm, states, si);

                if (!valid) {
                    ROS_WARN("Trajectory invalidated!");
                    tem->stopExecution();
                }
            }

        } else {
            ROS_ERROR("OMPL planner error: %s. Retrying.", solved.asString().c_str());
        }

        // Wait a bit to let the robot settle.
        ros::Duration(2.0).sleep();

    } while (!taskCompleted);
}