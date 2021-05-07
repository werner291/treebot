//
// Created by werner on 07-05-21.
//

#ifndef TREEBOT_FOURIERSERIESPLANNER_H
#define TREEBOT_FOURIERSERIESPLANNER_H

#include <ompl/base/Planner.h>

class FourierSeriesPlanner : public ompl::base::Planner {
public:
    ompl::base::PlannerStatus solve(const ompl::base::PlannerTerminationCondition &ptc) override;

};


#endif //TREEBOT_FOURIERSERIESPLANNER_H
