package org.usfirst.frc.team3238.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;

/**
 * This is the class that controls the vertical and horizontal collector.
 *
 * @author Aaron Jenson and Nick Papadakis
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
    double m_pauseDistanceFromObject;

    double m_horizontalThreshold;
    double m_verticalThreshold;
    
    double m_toteExtendHeight;
    double m_toteGrabHeight;
    double m_canExtendHeight;
    double m_canGrabHeight;
    double m_stepCanExtendHeight;
    double m_stepCanGrabHeight;
    
    double m_horizontalSetpoint = 0;
    
    double m_extendHeight;
    double m_grabHeight;

    // Switch state variables
    String m_horizontalState;
    String m_verticalState;

    // Status booleans
    boolean m_doneCollecting = false;
    boolean m_hooked = false;
    boolean m_horizontalExtended = false;
    boolean m_verticalDone = false;

    Grabber(CANTalon verticalCANTalon, CANTalon horizontalCANTalon,
    		AnalogInput  grabberVerticalPot, AnalogInput  grabberHorizontalPot,
            AnalogInput sonarSensor,
            double verticalPConstant, double verticalIConstant,
            double horizontalPConstant, double horizontalIConstant,
            double horizontalThreshold, double verticalThreshold,
            double canExtendHeight, double canGrabHeight, 
            double toteExtendHeight, double toteGrabHeight, 
            double stepCanExtendHeight, double stepCanGrabHeight,
            double retractedLocation, double pauseDistanceFromObject)
    {
        verticalTalon = verticalCANTalon;
        horizontalTalon = horizontalCANTalon;
        verticalPot = grabberVerticalPot;
        horizontalPot = grabberHorizontalPot;
        sonar = sonarSensor;
        verticalPI = new PIController(verticalPConstant, verticalIConstant);
        horizontalPI =
                new PIController(horizontalPConstant, horizontalIConstant);
        m_canExtendHeight = canExtendHeight;
        m_canGrabHeight = canGrabHeight;
        m_toteExtendHeight = toteExtendHeight;
        m_toteGrabHeight = toteGrabHeight;
        m_stepCanExtendHeight = stepCanExtendHeight;
        m_stepCanGrabHeight = stepCanGrabHeight;
        m_retractedLocation = retractedLocation;
        m_pauseDistanceFromObject = pauseDistanceFromObject;
        m_horizontalState = "waitForCommand";
        m_verticalState = "waitForCommand";
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
    
    void reset()
    {
        m_verticalState = "waitForCommand";
        m_horizontalState = "waitForCommand";
    }
    
    void goToHeight(double height)
    {
        m_verticalPotDistance = -0.2608156852* verticalPot.getVoltage()
                + 1.421847684;
        verticalTalon.set(-verticalPI.getMotorValue(
                height, m_verticalPotDistance));
        System.out.println(m_verticalPotDistance);
    }
    
    void goToLength(double length)
    {
        m_horizontalPotDistance = -0.4364133427* horizontalPot.getVoltage()
                + 1.78356027;
        horizontalTalon.set(-horizontalPI.getMotorValue
                (length, m_horizontalPotDistance));
        System.out.println(m_horizontalPotDistance);
    }

    /**
     * Handles the state machine for the motion of the vertical and horizonal
     * motors, must be run every loop in order for the Grabber to operate
     */
    void idle()
    {
        m_horizontalPotDistance = -0.4364133427* horizontalPot.getVoltage()
                + 1.78356027;
        m_verticalPotDistance = -0.2608156852* verticalPot.getVoltage()
                + 1.421847684;
        m_sonarDistance = 2.2121617347 * sonar.getVoltage() + 0.0467578232;
        System.out.println("HorizontalState: " + m_horizontalState);
        System.out.println("VerticalState: " + m_verticalState);

        switch(m_horizontalState)
        {
            case "waitForCommand":
                horizontalTalon.set(0);
                m_horizontalSetpoint = m_sonarDistance;
                break;

            case "extending":
                m_doneCollecting = false;
                m_horizontalExtended = false;
                if(Math.abs(m_horizontalSetpoint - m_pauseDistanceFromObject - 
                        m_horizontalPotDistance) > m_horizontalThreshold)
                {
                    horizontalTalon.set(-horizontalPI.getMotorValue(
                            m_horizontalSetpoint, m_horizontalPotDistance));
                }
                else
                {
                    m_horizontalState = "waitForVertical";
                }
                break;

            case "waitForVetical":
                if(m_verticalDone)
                {
                    m_horizontalState = "finishExtending";
                }
                break;
                
            case "finishExtending":
                if(Math.abs(m_horizontalSetpoint - m_horizontalPotDistance) 
                        > m_horizontalThreshold)
                {
                    horizontalTalon.set(-horizontalPI.getMotorValue(
                            m_horizontalSetpoint, m_horizontalPotDistance));
                }
                else
                {
                    m_horizontalState = "waitForHook";
                    m_horizontalExtended = true;
                }
             
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
                    horizontalTalon.set(-horizontalPI.getMotorValue(
                            m_retractedLocation, m_horizontalPotDistance));
                }
                else
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
                m_verticalDone = false;
                if(Math.abs(m_extendHeight - m_verticalPotDistance) 
                        > m_verticalThreshold)
                {
                    verticalTalon.set(-verticalPI.getMotorValue(
                            m_extendHeight, m_verticalPotDistance));
                }
                else
                {
                    m_verticalState = "waitForHorizontal";
                    m_verticalDone = true;
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
                if(Math.abs(m_grabHeight - m_verticalPotDistance) 
                        > m_verticalThreshold)
                {
                    verticalTalon.set(-verticalPI.getMotorValue(m_grabHeight,
                            m_verticalPotDistance));
                }
                else
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
