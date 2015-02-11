package org.usfirst.frc.team3238.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;

/**
 * This is the class that controls the vertical and horizontal collector.
 *
 * @author Aaron Jenson
 */

public class Grabber
{
    CANTalon verticalTalon, horizontalTalon;
    AnalogInput verticalPot, horizontalPot, sonar;
    PIController verticalPI, horizontalPI;

    // Store sensor values
    double m_horizontalPotDistance;
    double m_verticalPotDistance;
    double m_sonarDistance;
    double m_verticalPotSetpoint;
    double m_horizontalPotSetpoint;

    // Constants
    double m_retractedLocation;
    double m_horizontalThreshold;
    double m_verticalThreshold;
    double m_toteExtendHeight;
    double m_toteGrabHeight;
    double m_canExtendHeight;
    double m_canGrabHeight;
    double m_stepCanExtendHeight;
    double m_stepCanGrabHeight;
    double m_extendHeight;
    double m_grabHeight;

    // Switch state variables
    String m_horizontalState;
    String m_verticalState;

    // Status booleans
    boolean m_doneCollecting;
    boolean m_hooked;
    boolean m_horizontalExtended;

    Grabber(CANTalon verticalCANTalon, CANTalon horizontalCANTalon,
    		AnalogInput  grabberVerticalPot, AnalogInput  grabberHorizontalPot,
            AnalogInput sonarSensor,
            double verticalPConstant, double verticalIConstant,
            double horizontalPConstant, double horizontalIConstant)
    {
        verticalTalon = verticalCANTalon;
        horizontalTalon = horizontalCANTalon;
        verticalPot = grabberVerticalPot;
        horizontalPot = grabberHorizontalPot;
        sonar = sonarSensor;
        verticalPI = new PIController(verticalPConstant, verticalIConstant);
        horizontalPI =
                new PIController(horizontalPConstant, horizontalIConstant);
    }

    /**
     * Gets whether or not the arm has retracted.
     * 
     * @return whether or not the arm has retracted so the lifter can work.
     */
    boolean doneCollecting()
    {
        return m_doneCollecting;
    }
    
    void inputPIConstants(double verticalPConstant, double verticalIConstant, 
            double horizontalPConstant, double horizontalIConstant)
    {
        verticalPI.inputConstants(verticalPConstant, verticalIConstant);
        horizontalPI.inputConstants(horizontalPConstant, verticalPConstant);
    }

    void grabCan()
    {
        m_verticalState = "prepareForCanGrab";
        m_horizontalState = "extending";
    }

    void grabStepCan()
    {
        m_verticalState = "prepareForStepCanGrab";
        m_horizontalState = "extending";
    }

    void grabTote()
    {
        m_verticalState = "prepareForToteGrab";
        m_horizontalState = "extending";
    }
    
    void goToHeight(double height)
    {
        m_verticalPotDistance = -0.4106463365 * (verticalPot.getVoltage()) +
                1.67119633;
        verticalTalon.set(verticalPI.getMotorValue(
                height, m_verticalPotDistance));
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
        m_sonarDistance = 2.678677012 * sonar.getVoltage() + 0.0204464172;

        switch(m_horizontalState)
        {
            case "waitForCommand":
                horizontalTalon.set(0);
                break;

            case "extending":
                m_doneCollecting = false;
                if(Math.abs(m_sonarDistance - m_horizontalPotDistance) 
                        > m_horizontalThreshold)
                {
                    horizontalTalon.set(horizontalPI.getMotorValue(
                            m_sonarDistance, m_horizontalPotDistance));
                } else
                {
                    m_horizontalState = "waitForHook";
                }
                break;

            case "waitForHook":
                if(m_hooked)
                {
                    m_horizontalState = "retracting";
                } else
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
                } else
                {
                    m_horizontalState = "waitForCommand";
                    m_doneCollecting = true;
                }
                break;

            default:
                System.out.println(
                        "Grabber m_horizontalState is in default state!");
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
                m_grabHeight = m_canGrabHeight;
                m_verticalState = "goToExtendHeight";
                break;

            case "prepareForStepCanGrab":
                m_extendHeight = m_stepCanExtendHeight;
                m_grabHeight = m_stepCanGrabHeight;
                m_verticalState = "gotToExtendHeight";
                break;

            case "goToExtendHeight":
                m_hooked = false;
                if(Math.abs(m_extendHeight - m_verticalPotDistance) 
                        > m_verticalThreshold)
                {
                    verticalTalon.set(verticalPI.getMotorValue(
                            m_extendHeight, m_verticalPotDistance));
                } else
                {
                    m_verticalState = "waitForHorizontal";
                }
                break;

            case "waitForHorizontal":
                if(m_horizontalExtended)
                {
                    m_verticalState = "grab";
                } else
                {
                    verticalTalon.set(0);
                }
                break;

            case "grab":
                if(Math.abs(m_grabHeight - m_verticalPotDistance) 
                        > m_verticalThreshold)
                {
                    verticalTalon.set(verticalPI.getMotorValue(m_grabHeight,
                            m_verticalPotDistance));
                } else
                {
                    m_verticalState = "waitForCommand";
                    m_hooked = true;
                }
                break;

            default:
                System.out.println(
                        "Grabber m_verticalState is in default state!");
                break;
        }
    }
}
