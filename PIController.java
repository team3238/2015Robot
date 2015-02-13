package org.usfirst.frc.team3238.robot;

/**
 * This is a generic proportional integral controller.
 * 
 * @author Nick Papadakis
 */
public class PIController
{
    double m_cummulativeError = 0;
    double m_oldTime = 0;
    double m_pConstant;
    double m_iConstant;

    PIController(double pConstant, double iConstant)
    {
        m_pConstant = pConstant;
        m_iConstant = iConstant;
    }

    /**
     * Takes the current setpoint and sensorValue and calculates the necessary
     * response using the PI loop
     *
     * @param setpoint
     *            The desired sensorValue
     * @param sensorValue
     *            The current value of the sensor
     * @return m_returnPower The motor power calculated by the PI loop to get
     *         the sensorValue closer to the setpoint
     */
    double getMotorValue(double setpoint, double sensorValue)
    {
        double error = setpoint - sensorValue;
        m_cummulativeError += error;
        double time = System.currentTimeMillis();
        // If this is the first loop, set timeDifference to 0
        double timeDifference;
        if(m_oldTime == 0)
        {
            timeDifference = 0;
        } 
        else
        {
            timeDifference = time - m_oldTime;
        }
        double returnPower;
        if(error != 0)
        {
            returnPower = error * m_pConstant + m_cummulativeError
                    * m_iConstant * timeDifference;
        }
        else
        {
            returnPower = 0;
        }
        // Set up the "old" values for the next loop
        m_oldTime = time;
        return returnPower;
    }

    /**
     * Resets the cummulativeError and oldTime variables
     */
    void reinit()
    {
        m_cummulativeError = 0;
        m_oldTime = 0;
    }
    
    void inputConstants(double pConstant, double iConstant)
    {
        m_pConstant = pConstant;
        m_iConstant = iConstant;
    }
}
