/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "ReaderTask.hpp"
#include <base/commands/Joints.hpp>

using namespace motoman_mh12;

ReaderTask::ReaderTask(std::string const& name)
    : ReaderTaskBase(name)
{
}

ReaderTask::ReaderTask(std::string const& name, RTT::ExecutionEngine* engine)
    : ReaderTaskBase(name, engine)
{
}

ReaderTask::~ReaderTask()
{
}

/// The following lines are template definitions for the various state machine
// hooks defined by Orocos::RTT. See Task.hpp for more detailed
// documentation about them.

bool ReaderTask::configureHook()
{
    Driver* driver = new Driver();
    if (!_io_port.get().empty())
        driver->openURI(_io_port.get());
    setDriver(driver);
    mDriver = driver;
    
    if (!TaskBase::configureHook())
        return false;

    return true;
}
bool ReaderTask::startHook()
{
    if (! ReaderTaskBase::startHook())
        return false;

    return true;
}
void ReaderTask::updateHook()
{
    ReaderTaskBase::updateHook();
}

void ReaderTask::processIO()
{
    msgs::MotomanMsgType msg_type =  mDriver->read(_io_read_timeout.get()); 
    if (msg_type == msgs::MOTOMAN_ROBOT_STATUS)
    {
        msgs::MotomanStatus status = mDriver->getRobotStatus();
        _status.write(status);
    }
    else if(msg_type == msgs::MOTOMAN_JOINT_FEEDBACK)
    {
        msgs::MotomanJointFeedback joint_feedback = mDriver->getJointFeedback();
        base::samples::Joints joints;
        joints.elements = joint_feedback.joint_states;
        joints.time = joint_feedback.time;
        _joints.write(joints);
    }
}
void ReaderTask::errorHook()
{
    ReaderTaskBase::errorHook();
}
void ReaderTask::stopHook()
{
    ReaderTaskBase::stopHook();
}
void ReaderTask::cleanupHook()
{
    ReaderTaskBase::cleanupHook();
}
