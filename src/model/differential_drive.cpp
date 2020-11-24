#include "model/differential_drive.h"

namespace model
{
    void Performance::reset()
    {
        velErrData.clear();
        cteData.clear();
        ethetaData.clear();

        translationalEL.clear();
        rotationalEL.clear();

        costs.clear();
    }
    
    DifferentialDrive::DifferentialDrive()
    {
    }

    void DifferentialDrive::setSampleTime(double sampletime)
    {
        m_sampleTime = sampletime;
    }

    void DifferentialDrive::step(double speed, double omega)
    {
        m_state.x += m_state.linVel * cos(m_state.theta) * m_sampleTime;
        m_state.y += m_state.linVel * sin(m_state.theta) * m_sampleTime;

        m_state.theta += omega * m_sampleTime;
        m_state.throttle = (speed - m_state.linVel) / m_sampleTime;
        m_state.linVel = speed;
        m_state.angVel = omega;
    }

    State DifferentialDrive::getState() const
    {
        return m_state;
    }

    void DifferentialDrive::setInitState(const State &state)
    {
        m_state = state;
        m_initState = state;
    }

    void DifferentialDrive::reset()
    {
        m_state = m_initState;
    }
} // namespace model