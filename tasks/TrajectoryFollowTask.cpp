/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "TrajectoryFollowTask.hpp"

using namespace motoman_mh12;

TrajectoryFollowTask::TrajectoryFollowTask(std::string const& name)
    : TrajectoryFollowTaskBase(name)
{
}

TrajectoryFollowTask::TrajectoryFollowTask(std::string const& name, RTT::ExecutionEngine* engine)
    : TrajectoryFollowTaskBase(name, engine)
{
}

TrajectoryFollowTask::~TrajectoryFollowTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See TrajectoryFollowTask.hpp for more detailed
// documentation about them.

bool TrajectoryFollowTask::configureHook()
{
    if (! TrajectoryFollowTaskBase::configureHook())
        return false;
    return true;
}
bool TrajectoryFollowTask::startHook()
{
    if (! TrajectoryFollowTaskBase::startHook())
        return false;
    return true;
}
void TrajectoryFollowTask::updateHook()
{
    TrajectoryFollowTaskBase::updateHook();
}
void TrajectoryFollowTask::errorHook()
{
    TrajectoryFollowTaskBase::errorHook();
}
void TrajectoryFollowTask::stopHook()
{
    TrajectoryFollowTaskBase::stopHook();
}
void TrajectoryFollowTask::cleanupHook()
{
    TrajectoryFollowTaskBase::cleanupHook();
}
