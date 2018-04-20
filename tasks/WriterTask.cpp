/* Generated from orogen/lib/orogen/templates/tasks/Task.cpp */

#include "WriterTask.hpp"
#include <motoman_mh12/Msgs.hpp>
#include <iodrivers_base/ConfigureGuard.hpp>

using namespace motoman_mh12;

WriterTask::WriterTask(std::string const& name)
    : WriterTaskBase(name), start_trajectory_deadline(base::Time::fromSeconds(2))
{
}

WriterTask::WriterTask(std::string const& name, RTT::ExecutionEngine* engine)
    : WriterTaskBase(name, engine), start_trajectory_deadline(base::Time::fromSeconds(2))
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
    gpios_addresses.resize(_gpios_addresses.get().size());
    gpios_addresses = _gpios_addresses.get();

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

    state(NOT_CHECKED);
    return true;
}

void WriterTask::checkInitialStatus()
{
    msgs::MotomanStatus status;
    if(_status.read(status) != RTT::NewData) 
        return;
    
    if(status.ln_error != 0)
    {
        RTT::log(RTT::Error) << "Alarm with code " << 
            status.error_code << " is on" << RTT::endlog();
        exception(ALARM_ERROR);
        throw std::runtime_error("Alarm on, to enable the robot please reset it"); 
    }
    if(status.mode != 0)
    {
        RTT::log(RTT::Error) << "Pendant is not on remote mode, please turn key to the correct mode"
            << RTT::endlog();
        exception(PENDANT_MODE_ERROR);
        throw;
    }
    state(STATUS_CHECKED);
}

void WriterTask::sendAndCheckMotionCmd(base::Time const& timeout, int cmd)
{
    base::Timeout deadline(timeout);
    while(!deadline.elapsed())
    {
        msgs::MotionReply stop_reply = mDriver
            ->sendMotionCtrl(0, 0, cmd);
        if(stop_reply.result == msgs::motion_reply::MotionReplyResults::SUCCESS)
            return;
    }            
}

void WriterTask::stopTrajectory()
{
    base::Time timeout = base::Time::fromSeconds(1);
    sendAndCheckMotionCmd(timeout, msgs::motion_ctrl::MotionControlCmds::STOP_MOTION);
    sendAndCheckMotionCmd(timeout, msgs::motion_ctrl::MotionControlCmds::STOP_TRAJ_MODE);
    state(NOT_CHECKED);
}

void WriterTask::startTrajectoryMode()
{
    base::Time timeout = base::Time::fromSeconds(1);
    msgs::MotionReply reply = mDriver
            ->sendMotionCtrl(0, 0, msgs::motion_ctrl::MotionControlCmds::START_TRAJ_MODE);
    if(reply.result == msgs::motion_reply::MotionReplyResult::SUCCESS)
    {
        start_trajectory_deadline.restart();
        state(CHECK_MOTION_READY);
        return;
    }
    else
        throw std::runtime_error("The controller could not enter Trajectory mode");
}

void WriterTask::checkMotionReady()
{
    msgs::MotomanStatus status;
    if(!start_trajectory_deadline.elapsed())
    {
        if(_status.read(status) == RTT::NewData) 
        {
            if(status.motion_possible == 1 && status.drives_powered == 1)
            {
                state(MOTION_READY);
                return;
            }
        }
    }
    else
        throw std::runtime_error("The controller could not enter Trajectory mode");
}

void WriterTask::readNewTrajectory()
{
    base::JointsTrajectory temp_trajectory;
    RTT::FlowStatus incoming_data = _trajectory.read(temp_trajectory);

    if(incoming_data == RTT::NoData)
    {
        stopTrajectory(); 
        return;
    } else if (incoming_data == RTT::NewData)
    {
        if(!temp_trajectory.isValid() || !temp_trajectory.isTimed())
        {
            exception(INVALID_TRAJECTORY);
            return;
        } else if (!temp_trajectory.times[0].isNull())
        {
            exception(TRAJECTORY_START_TIME_NON_NULL);
            return;
        }
        current_trajectory = temp_trajectory;
        startTrajectoryMode();
        current_step = 0;
        state(TRAJECTORY_EXECUTION);
    }
}

void WriterTask::executeTrajectory()
{
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
            RTT::log(RTT::Error) << "Trajectory command returned " << 
                reply.result << " for command " << reply.command << RTT::endlog();
            exception(TRAJECTORY_CMD_ERROR);
            throw std::runtime_error("Trajectory command was not SUCCESS nor BUSY.");
        }
    }

    msgs::MotionReply reply = mDriver->sendMotionCtrl(0, 0, 
        msgs::motion_ctrl::MotionControlCmds::CHECK_QUEUE_CNT);
    if(reply.result == msgs::motion_reply::MotionReplyResults::SUCCESS && reply.subcode == 0)
        report(TRAJECTORY_END);
}

void WriterTask::readGPIO()
{
    GPIOs gpios;
    for(size_t i; i < gpios_addresses.size(); i++)
    {
        msgs::ReadSingleIo single_io = mDriver->queryReadSingleIO(gpios_addresses[i]);
        GPIO current_gpio;
        current_gpio.address = gpios_addresses[i];
        current_gpio.value = single_io.value;
        gpios.ports.push_back(current_gpio);
    }
    gpios.timestamp= base::Time::now();
    _gpios.write(gpios);
}

void WriterTask::updateHook()
{
    WriterTaskBase::updateHook();

    switch(state())
    {
        case NOT_CHECKED:
            checkInitialStatus();
            break;
        case STATUS_CHECKED: 
            startTrajectoryMode();
            break;
        case CHECK_MOTION_READY:
            checkMotionReady();
            break;
        case MOTION_READY: 
            readNewTrajectory();
            break;
        case TRAJECTORY_EXECUTION: 
            executeTrajectory();
            break;
        default: 
            stopTrajectory();
            break;
    }
    readGPIO();
}

void WriterTask::errorHook()
{
    WriterTaskBase::errorHook();
}
void WriterTask::stopHook()
{
    stopTrajectory();
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
