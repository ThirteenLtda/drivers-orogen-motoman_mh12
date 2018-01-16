/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "TrajectoryFollowTask.hpp"
#include <motoman_mh12/Msgs.hpp>

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
    Driver* driver = new Driver();
    if (!_io_port.get().empty())
        driver->openURI(_io_port.get());
    setDriver(driver);
    mDriver = driver;

    if (! TrajectoryFollowTaskBase::configureHook())
        return false;
    return true;
}


void TrajectoryFollowTask::processIO()
{

}


bool TrajectoryFollowTask::startHook()
{
    if (! TrajectoryFollowTaskBase::startHook())
        return false;

    msgs::MotionReply reply = mDriver->sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::START_TRAJ_MODE);
    //TODO check reply
    return true;
}
void TrajectoryFollowTask::updateHook()
{
    TrajectoryFollowTaskBase::updateHook();
    base::JointsTrajectory trajectory;
    _trajectory.read(trajectory);

    if (!trajectory.isValid())
    {
        exception(INVALID_TRAJECTORY);
        throw std::runtime_error("received invalid trajectory");
    }
    else if (!trajectory.times.empty() && !trajectory.times[0].isNull())
    {
        exception(TRAJECTORY_START_TIME_NON_NULL);
        throw std::runtime_error("received trajectory with a non-null start time");
    }

    base::samples::Joints joints;
    base::Time time;
    msgs::MotionReply reply;
    reply = mDriver->sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::CHECK_MOTION_READY);
    //TODO check reply
    std::vector<base::JointState> current_position;

    for(int step = 0; step < trajectory.getTimeSteps(); step++){
        trajectory.getJointsAtTimeStep(step, joints);
        joints.time = trajectory.times[step];
        reply = mDriver->sendJointTrajPTFullCmd(0, step, joints);
        // TODO check reply
    }
}
void TrajectoryFollowTask::errorHook()
{
    TrajectoryFollowTaskBase::errorHook();
}
void TrajectoryFollowTask::stopHook()
{
    mDriver->close();
    TrajectoryFollowTaskBase::stopHook();
}
void TrajectoryFollowTask::cleanupHook()
{
    TrajectoryFollowTaskBase::cleanupHook();
    setDriver(0);
    delete mDriver;
    mDriver = 0;
}
