/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WriterTask.hpp"
#include <motoman_mh12/Msgs.hpp>

using namespace motoman_mh12;

WriterTask::WriterTask(std::string const& name)
    : WriterTaskBase(name)
{
}

WriterTask::WriterTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WriterTaskBase(name, engine)
{
}

WriterTask::~WriterTask()
{
}



/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See WriterTask.hpp for more detailed
// documentation about them.

bool WriterTask::configureHook()
{
    Driver* driver = new Driver();
    if (!_io_port.get().empty())
        driver->openURI(_io_port.get());
    setDriver(driver);
    mDriver = driver;

    if (! WriterTaskBase::configureHook())
        return false;
    return true;
}


void WriterTask::processIO()
{

}


bool WriterTask::startHook()
{
    if (! WriterTaskBase::startHook())
        return false;

    msgs::MotionReply reply = mDriver->sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::START_TRAJ_MODE);
    //TODO check reply
    return true;
}
void WriterTask::updateHook()
{
    WriterTaskBase::updateHook();
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

    for(size_t step = 0; step < trajectory.getTimeSteps(); step++){
        trajectory.getJointsAtTimeStep(step, joints);
        joints.time = trajectory.times[step];
        reply = mDriver->sendJointTrajPTFullCmd(0, step, joints);
        // TODO check reply
    }
}
void WriterTask::errorHook()
{
    WriterTaskBase::errorHook();
}
void WriterTask::stopHook()
{
    mDriver->close();
    WriterTaskBase::stopHook();
}
void WriterTask::cleanupHook()
{
    WriterTaskBase::cleanupHook();
    setDriver(0);
    delete mDriver;
    mDriver = 0;
}
