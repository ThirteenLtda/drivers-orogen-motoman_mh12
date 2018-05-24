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

bool WriterTask::checkInitialStatus()
{
    msgs::MotomanStatus status;
    if(_status.read(status) != RTT::NewData)
    {
        return false;
    }

    if(status.e_stopped)
    {
        RTT::log(RTT::Error) << "Panic Button is pressed, release it before continuing"
            << RTT::endlog();
        exception(PANIC_BUTTON_ERROR);
        throw std::runtime_error("Panic Button is pressed, release it before continuing");
    }
    if(status.ln_error)
    {
        RTT::log(RTT::Error) << "Alarm with code " <<
            status.error_code << " is on" << RTT::endlog();
        exception(ALARM_ERROR);
        throw std::runtime_error("Alarm on, to enable the robot please reset it");
    }
    if(!status.mode)
    {
        RTT::log(RTT::Error) << "Pendant is not on remote mode, please turn key to the correct mode"
            << RTT::endlog();
        exception(PENDANT_MODE_ERROR);
        throw std::runtime_error("Pendant is not on remote mode, please turn key to the correct mode");
    }
    return true;
}

bool WriterTask::sendAndCheckMotionCmd(base::Time const& timeout, int cmd)
{
    base::Timeout deadline(timeout);
    while(!deadline.elapsed())
    {
        msgs::MotionReply stop_reply = mDriver
            ->sendMotionCtrl(0, 0, cmd);
        if(stop_reply.result == msgs::motion_reply::MotionReplyResults::SUCCESS)
            return true;
        RTT::log(RTT::Warning) << "Command #" << cmd << " was not sent properly"
            << RTT::endlog();
    }
    return false;
}

bool WriterTask::stopTrajectory()
{
    base::Time timeout = base::Time::fromSeconds(1);
    bool stop_motion
        = sendAndCheckMotionCmd(timeout, msgs::motion_ctrl::MotionControlCmds::STOP_MOTION);
    bool stop_traj_mode =
        sendAndCheckMotionCmd(timeout, msgs::motion_ctrl::MotionControlCmds::STOP_TRAJ_MODE);
    return stop_motion && stop_traj_mode;
}

void WriterTask::startTrajectoryMode()
{
    base::Time timeout = base::Time::fromSeconds(1);
    if(!sendAndCheckMotionCmd(timeout, msgs::motion_ctrl::MotionControlCmds::START_TRAJ_MODE))
        throw std::runtime_error("The controller could not enter Trajectory mode");
}

bool WriterTask::checkMotionReady()
{
    msgs::MotomanStatus status;
    if (_status.read(status) != RTT::NewData)
    {
        return false;
    else if (status.motion_possible && status.drives_powered)
        return true;
}

bool WriterTask::readNewTrajectory()
{
    base::JointsTrajectory temp_trajectory;
    RTT::FlowStatus incoming_data = _trajectory.read(temp_trajectory);

    if (incoming_data != RTT::NewData)
        return false;

    if(!temp_trajectory.isValid() || !temp_trajectory.isTimed())
    {
        exception(INVALID_TRAJECTORY);
        return false;
    }
    else if (!temp_trajectory.times[0].isNull())
    {
        exception(TRAJECTORY_START_TIME_NON_NULL);
        return false;
    }
    current_trajectory = temp_trajectory;
    current_step = 0;
    return true;
}

bool WriterTask::executeTrajectory()
{
    base::samples::Joints joints;
    for(; current_step < current_trajectory.getTimeSteps(); current_step++)
    {
        current_trajectory.getJointsAtTimeStep(current_step, joints);
        joints.time = current_trajectory.times[current_step];
        msgs::MotionReply reply = mDriver->sendJointTrajPTFullCmd(0, current_step, joints);
        if (reply.result == msgs::motion_reply::MotionReplyResults::BUSY)
        {
            return false;
        }

        if(reply.result != msgs::motion_reply::MotionReplyResults::SUCCESS)
        {
            RTT::log(RTT::Error) << "Trajectory command returned " <<
                reply.result << " for command " << reply.command << RTT::endlog();
            exception(TRAJECTORY_CMD_ERROR);
            throw std::runtime_error("Trajectory command was not SUCCESS nor BUSY.");
        }
    }
    return true;
}

bool WriterTask::checkTrajectoryEnd()
{
    msgs::MotionReply reply = mDriver->sendMotionCtrl(0, 0,
        msgs::motion_ctrl::MotionControlCmds::CHECK_QUEUE_CNT);

    return reply.result == msgs::motion_reply::MotionReplyResults::SUCCESS &&
        reply.subcode == 0;
}

void WriterTask::readGPIO()
{
    GPIOs gpios;
    if(gpios_addresses.size() <= 0)
        return;
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

    bool checked = checkInitialStatus();

    switch(state())
    {
        case NOT_CHECKED:
            if (checked)
                state(STATUS_CHECKED);
            break;
        case STATUS_CHECKED:
            if (readNewTrajectory())
                state(CHECK_MOTION_READY);
            break;
        case CHECK_MOTION_READY:
            startTrajectoryMode();
            if (checkMotionReady())
                state(TRAJECTORY_EXECUTION);
            break;
        case TRAJECTORY_EXECUTION:
            if (executeTrajectory())
            {
                // Need to wait for the queue count to be updated
                usleep(1000);
                state(CHECK_TRAJECTORY_END);
            }
            break;
        case CHECK_TRAJECTORY_END:
            if (checkTrajectoryEnd())
            {
                state(TRAJECTORY_END);
                state(STATUS_CHECKED);
            }
            break;
        default:
            if (stopTrajectory())
                state(NOT_CHECKED);
            else
                exception(TRAJECTORY_END_ERROR);
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
