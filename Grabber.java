package org.usfirst.frc.team3238.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.AnalogInput;

/**
 * This is the class that controls the vertical and horizontal collecter.
 *
 * @author Aaron Jenson
 */

public class Grabber
{
    CANTalon verticalTalon, horizontalTalon;
    AnalogInput verticalPot, horizontalPot, sonar; 
    PIController verticalPI, horizontalPI;
    
    //Store sensor values
    double m_horizontalPotDistance;
    double m_verticalPotDistance;
    double m_sonarDistance;
    double m_verticalPotSetpoint;
    double m_horizontalPotSetpoint;

    //Constants
    double m_retractedPotVal;
    double m_fullyRetractedPotVal;
    double m_threshold;
    double m_canHeight;
    double m_toteHeight;
    double m_canCollectingHeight;
    double m_toteCollectingHeight;
    double m_potValDifference;
    //TODO set constants

    double m_toteExtendHeight;
    double m_toteGrabHeight;
    double m_canExtendHeight;
    double m_canGrabHeight;
    double m_stepCanExtendHeight;
    double m_stepCanGrabHeight;
    double m_extendHeight;
    double m_grabHeight;

    //Switch state variables
    String m_horizontalState;
    String m_verticalState;

    //Status booleans
    boolean haveObject;
    boolean retracted;

    Grabber(int verticalTalonChannel, int horizontalTalonChannel, 
            int verticalPotPort, int horizontalPotPort, AnalogInput sonarSensor,
            double verticalPConstant, double verticalIConstant, 
            double horizontalPConstant, double horizontalIConstant)
    {
        verticalTalon = new CANTalon(verticalTalonChannel);
        horizontalTalon = new CANTalon(horizontalTalonChannel);
        verticalPot = new AnalogInput(verticalPotPort);
        horizontalPot = new AnalogInput(horizontalPotPort);
        sonar = sonarSensor;
        verticalPI = new PIController(verticalPConstant, verticalIConstant);
        horizontalPI = new PIController(horizontalPConstant,
                horizontalIConstant);
    }
   
    /**
     * Raises the grabber to the correct potentiometer value using the
     * PIController class.
     *
     * @param height The height at which the grabber should raise to, set
     * using the stored variables above.
     */
    boolean goToHeight(double height)
    {
        boolean heightReached = false;
        verticalTalon.set(verticalPI.getAdjustedRotationValue(height,
                    verticalPot.getVoltage()));
        if(Math.abs(m_vPotValue - height) <= m_threshold)
        {
            heightReached = true;
        }
        return heightReached;
    }

    /**
     * Gets whether or not the arm has retracted.
     * @return whether or not the arm has retracted so the lifter can work.
     */
    boolean getRetracted()
    {
        return retracted;
    }

    /**
     * Gets whether or not the can has been collected.
     * @return whether or not the can has been collected.
     */
    boolean getCanCollected()
    {
        return canCollected;
    }

    void grabCan()
    {

    }

    void grabStepCan()
    {

    }

    void grabTote()
    {

    }

    /**
     * Handles the state machine for the motion of the vertical and horizonal
     * motors, must be run every loop in order for the Grabber to operate
     */
    void idle()
    {
        // TODO Add mapping for potentiometers
        m_horizontalPotDistance = horizontalPot.getVoltage();
        m_verticalPotDistance = verticalPot.getVoltage();
        // TODO Check mapping of ultrasonic sensor with RoboRIO
        m_sonarDistance = 2.678677012 * sonar.getVoltage() +
            0.0204464172;       
        
        switch(m_horizontalState)
        {
            case "waitForCommand":
                horizontalTalon.set(0);
                break;

            case "extending":
                if(Math.abs(m_sonarDistance - m_horizontalPotDistance) 
                        > m_horizontalThreshold)
                {
                    horizonatalTalon.set(horizontalPI.getMotorValue(
                            m_sonarDistance, m_horizontalPotDistance));
                }
                else
                {
                    m_horizontalState = "waitForHook";
                }
                break;

            case "waitForHook":
                if(m_hooked)
                {
                    m_horizontalState = "retracting";
                }
                else
                {
                    horizontalTalon.set(0);
                }
                break;

            case "retracting":
                if(Math.abs(m_retractedLocation - m_horizontalPotDistance)
                        > m_horizontalThreshold)
                {
                    horizontalTalon.set(horizontalPI.getMotorValue(
                            m_retractedLocation, m_horizontalPotDistance));
                }
                else
                {
                    m_horizontalState = "waitForCommand";
                }
                break;
        }

        switch(m_verticalState)
        {
            case "waitForCommand":
                verticalTalon.set(0);
                break;

            case "prepareForToteGrab":
                m_extendHeight = m_toteExtendHeight;
                m_grabHeight = m_toteGrabHeight;
                m_verticalState = "goToExtendHeight";
                break;
                
            case "prepareForCanGrab":
                m_extendHeight = m_canExtendHeight;
                m_grabHeigth = m_canGrabHeight;
                m_verticalState = "goToExtendHeight";
                break;

            case "prepareForStepCanGrab":
                m_extendHeight = m_stepCanExtendHeight;
                m_grabHeight = m_stepCanGrabHeight;
                m_verticalState = "gotToExtendHeight";
                break;

            case "goToExtendHeight":
                if(Math.abs(m_extendHeight - m_verticalPotDistance)
                        > m_verticalThreshold)
                {
                    verticalTalon.set(verticalPI.getMotorValue(
                            m_retractedLocation, m_verticalPotDistance));
                }
                else
                {
                    m_verticalState = "waitForHorizontal";
                }
                break;

            case "waitForHorizontal":
                if(m_horizontalExtended)
                {
                    m_verticalState = "grab";
                }
                else
                {
                   verticalTalon.set(0);
                }
                break;

            case "grab":
                
                

        switch(m_stateMode)
        {
            case "grabTote":
            switch(m_stateIndex)
            {
                case "adjustingHeight":
                    if(GoToHeight(m_toteHeight))
                    {
                        m_stateIndex = "adjustingLength";
                        retracted = false;
                    }
                    break;
                
                case "adjustingLength":
                    init = false;
                    horizontalTalon.set(horizontalPI.
                            getAdjustedRotationValue(m_desiredPotValue,
                                m_sonarValue));
                    if(Math.abs(m_hPotValue - m_desiredPotValue) <= m_threshold)
                    {
                        m_stateIndex = "readjustingHeight";
                    }
                    break;
                
                case "readjustingHeight":
                    init = false;
                    if(GoToHeight(m_toteCollectingHeight))
                    {
                        m_stateIndex = "retracting";
                    }
                    break;
                
                case "retracting":
                    init = false;
                    horizontalTalon.set(horizontalPI.
                            getAdjustedRotationValue(m_retractedPotVal,
                                m_hPotValue));
                    if(Math.abs(m_hPotValue - m_retractedPotVal) <= m_threshold)
                    {
                        m_stateIndex = "reLoweringHeight";
                        retracted = true;
                    }
                    break;
                
                case "reLoweringHeight":
                    if(GoToHeight(m_toteHeight))
                    {
                        m_stateIndex = "retractingMore";
                    }
                    break;
                
                case "retractingMore":
                    horizontalTalon.set(horizontalPI.
                            getAdjustedRotationValue(m_fullyRetractedPotVal,
                                m_hPotValue));
                    if(Math.abs(m_hPotValue - m_fullyRetractedPotVal) <=
                            m_threshold)
                    {
                        m_stateIndex= "default";
                    }
                    break;
                
                default:
                    break;
            }
            break;

            case "grabCan":
            switch(m_stateIndex)
            {
                case "adjustingHeight":
                    if(GoToHeight(m_canHeight))
                    {
                        m_stateIndex = "adjustingLength";
                        canCollected = false;
                    }
                    break;

                case "adjustingLength":
                    horizontalTalon.set(horizontalPI.
                            getAdjustedRotationValue((m_desiredPotValue + 
                                    m_potValDifference), m_hPotValue));
                    if(Math.abs(m_hPotValue - (m_desiredPotValue + 
                                    m_potValDifference)) <= m_threshold)
                    {
                        m_stateIndex = "readjustingHeight";
                    }
                    break;

                case "readjustingHeight":
                    if(GoToHeight(m_canCollectingHeight))
                    {
                        m_stateIndex = "default";
                        canCollected = true;
                    }
                    break;

                default:
                    break;
            }           
        }
    } 
}
