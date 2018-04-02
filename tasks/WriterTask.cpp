/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WriterTask.hpp"
#include <motoman_mh12/Msgs.hpp>
#include <iodrivers_base/ConfigureGuard.hpp>

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
    iodrivers_base::ConfigureGuard guard(this);
    Driver* driver = new Driver();
    if (!_io_port.get().empty())
        driver->openURI(_io_port.get());
    setDriver(driver);
    mDriver = driver;

    if (! WriterTaskBase::configureHook())
        return false;
    
    guard.commit();
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
    running = false;
    return true;
}
void WriterTask::updateHook()
{
    WriterTaskBase::updateHook();
    //if buffer not empty:
    // if time ==0 : current_step = 0
    if(!running)
    {
        if( !(_trajectory.read(current_trajectory) == RTT::NewData) )
            return;
        if (!current_trajectory.isValid())
        {
            exception(INVALID_TRAJECTORY);
            throw std::runtime_error("received invalid trajectory");
        }
        else if (!current_trajectory.times.empty() && !current_trajectory.times[0].isNull())
        {
            exception(TRAJECTORY_START_TIME_NON_NULL);
            throw std::runtime_error("received trajectory with a non-null start time");
        }
        running = true;
        current_step = 0;
        std::cout << "NUMBER OF POINTS: " << current_trajectory.getTimeSteps() << std::endl;
    }

    base::samples::Joints joints;
    for(; current_step < current_trajectory.getTimeSteps(); current_step++)
    {
        current_trajectory.getJointsAtTimeStep(current_step, joints);
        joints.time = current_trajectory.times[current_step];
        msgs::MotionReply reply = mDriver->sendJointTrajPTFullCmd(0, current_step, joints);
        if (reply.result == msgs::motion_reply::MotionReplyResults::BUSY)
        {
            return;
        }

        if(reply.result != msgs::motion_reply::MotionReplyResults::SUCCESS)
        {
            RTT::log(RTT::Error) << "Trajectory command returned " << reply.result << " for command " << reply.command << RTT::endlog();
            exception(TRAJECTORY_CMD_ERROR);
            throw std::runtime_error("Trajectory command was not SUCCESS nor BUSY.");
        }
        else
    }

    running = false;
    msgs::MotionReply reply = mDriver->sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::CHECK_QUEUE_CNT);
    if(reply.result == msgs::motion_reply::MotionReplyResults::SUCCESS && reply.subcode == 0)
        report(TRAJECTORY_END);
}
void WriterTask::errorHook()
{
    WriterTaskBase::errorHook();
}
void WriterTask::stopHook()
{
    while(mDriver->sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::STOP_MOTION).result != msgs::motion_reply::MotionReplyResults::SUCCESS){};
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
