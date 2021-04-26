
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <mutex>
#include <actionlib/server/simple_action_server.h>
#include <actionlib_msgs/GoalStatus.h>
#include <moveit_simple_controller_manager/MultiDofFollowJointTrajectoryAction.h>
#include <simLib.h>

#ifdef _WIN32
#define SIM_DLLEXPORT extern "C" __declspec(dllexport)
#endif /* _WIN32 */
#if defined (__linux) || defined (__APPLE__)
#define SIM_DLLEXPORT extern "C"

#endif /* __linux || __APPLE__ */

// The 3 required entry points of the plugin:
SIM_DLLEXPORT unsigned char simStart(void* reservedPointer,int reservedInt);
SIM_DLLEXPORT void simEnd();
SIM_DLLEXPORT void* simMessage(int message,int* auxiliaryData,void* customData,int* replyData);

static LIBRARY simLib;

namespace scm = moveit_simple_controller_manager;
typedef actionlib::SimpleActionServer<scm::MultiDofFollowJointTrajectoryAction> ActionServer;
static std::unique_ptr<ActionServer> as;
static scm::MultiDofFollowJointTrajectoryGoalConstPtr current_goal;
static double trajectory_start_seconds;




unsigned char simStart(void *reservedPointer, int reservedInt) {

    //region Load simLib
    char curDirAndFile[1024];
#ifdef _WIN32
    #ifdef QT_COMPIL
        _getcwd(curDirAndFile, sizeof(curDirAndFile));
    #else
        GetModuleFileName(NULL,curDirAndFile,1023);
        PathRemoveFileSpec(curDirAndFile);
    #endif
#elif defined (__linux) || defined (__APPLE__)
    getcwd(curDirAndFile, sizeof(curDirAndFile));
#endif

    std::string currentDirAndPath(curDirAndFile);
    std::string temp(currentDirAndPath);

#ifdef _WIN32
    temp+="\\coppeliaSim.dll";
#elif defined (__linux)
    temp+="/libcoppeliaSim.so";
#elif defined (__APPLE__)
    temp+="/libcoppeliaSim.dylib";
#endif /* __linux || __APPLE__ */

    simLib=loadSimLibrary(temp.c_str());
    if (simLib==NULL)
    {
        printf("TreeGen: error: could not find or correctly load coppeliaSim.dll. Cannot start the plugin.\n"); // cannot use simAddLog here.
        return(0); // Means error, CoppeliaSim will unload this plugin
    }
    if (getSimProcAddresses(simLib)==0)
    {
        printf("TreeGen: error: could not find all required functions in coppeliaSim.dll. Cannot start the plugin.\n"); // cannot use simAddLog here.
        unloadSimLibrary(simLib);
        return(0); // Means error, CoppeliaSim will unload this plugin
    }
    //endregion

    return 1;
}


void simEnd() {

}

void *simMessage(int message, int *auxiliaryData, void *customData, int *replyData) {

    switch (message) {
        case sim_message_eventcallback_simulationabouttostart: {

            simAddLog("treebot_controller", sim_verbosity_infos, "Hello world!");

            if (!as) {
                as = std::make_unique<ActionServer>("drone_controller/drone_controller", false);
                as->start();
            }

        } break;

        case sim_message_eventcallback_instancepass: {

            if (as) {
                if (as->isNewGoalAvailable()) {
                    current_goal = as->acceptNewGoal();
                    trajectory_start_seconds = simGetSimulationTime();
                }
                if (current_goal && as->isPreemptRequested()) {
                    current_goal = nullptr;
                    as->setPreempted();
                }
            }

            if (current_goal) {

                if (simGetSimulationState() != sim_simulation_advancing_running) {
                    simAddLog("TreebotController", sim_verbosity_warnings, "Action server has current goal while simulation not running!");
                }
                trajectory_msgs::MultiDOFJointTrajectoryPoint point_before;
                trajectory_msgs::MultiDOFJointTrajectoryPoint point_after;
                double current_time = simGetSimulationTime() - trajectory_start_seconds;
                int handle = simGetObjectHandle("Quadricopter_target");
                bool in_trajectory = false;
                for (int i = 0; i < current_goal->trajectory.points.size() - 1; i++) {

                    point_before = current_goal->trajectory.points[i];
                    point_after = current_goal->trajectory.points[i+1];
                    if (point_before.time_from_start.toSec() <= current_time && point_after.time_from_start.toSec() > current_time) {
                        in_trajectory = true;
                        break;
                    }
                }
                if (!in_trajectory) {
                    as->setSucceeded();
                    float position[3] = {
                            static_cast<float>(current_goal->trajectory.points.back().transforms[0].translation.x),
                            static_cast<float>(current_goal->trajectory.points.back().transforms[0].translation.y),
                            static_cast<float>(current_goal->trajectory.points.back().transforms[0].translation.z)
                    };
                    simSetObjectPosition(handle, -1, position);
                    current_goal = nullptr;

                } else {
                    double t = (current_time - point_before.time_from_start.toSec()) / (point_after.time_from_start.toSec() - point_before.time_from_start.toSec());

                    float position[3] = {
                            static_cast<float>(point_before.transforms[0].translation.x * (1.0-t) + point_after.transforms[0].translation.x * t),
                            static_cast<float>(point_before.transforms[0].translation.y * (1.0-t) + point_after.transforms[0].translation.y * t),
                            static_cast<float>(point_before.transforms[0].translation.z * (1.0-t) + point_after.transforms[0].translation.z * t)
                    };

                    simSetObjectPosition(handle, -1, position);
//                    scm::MultiDofFollowJointTrajectoryFeedback fb;
//                    fb.header.stamp = ros::Time::now();
//                    fb.joint_names.push_back("w")
//                    as->publishFeedback(fb);
                }

            }

        }



        default:
            // ignore
            break;
    }

    return nullptr;
}
