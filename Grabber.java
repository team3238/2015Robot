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
    
    double m_horizontalHome;
    double m_verticalHome;
    
    double m_toteExtendHeight;
    double m_toteGrabHeight;
    double m_canExtendHeight;
    double m_canGrabHeight;
    double m_stepCanExtendHeight;
    double m_stepCanGrabHeight;
    
    double m_toteHorizontalSetpoint = 0;
    double m_canHorizontalSetpoint = 0;
    
    double m_extendHeight;
    double m_grabHeight;
    
    double m_horizontalP;
    double m_horizontalI;
    double m_gentleHorizontalP;
    double m_gentleHorizontalI;
    
    double m_slowDownRetractThreshold;

    double m_horizontalYIntercept;
    double m_verticalYIntercept;
    
    long m_timestamp;
    // Switch state variables
    String m_toteHorizontalState;
    String m_canHorizontalState;
    String m_verticalState;

    // Status booleans
    boolean m_doneCollecting = false;
    boolean m_hooked = false;
    boolean m_horizontalExtended = false;
    boolean m_verticalDone = false;
    boolean m_foundReadPosition = false;
    boolean m_horizontalFoundHome = false;

    Grabber(CANTalon verticalCANTalon, CANTalon horizontalCANTalon,
    		AnalogInput  grabberVerticalPot, AnalogInput  grabberHorizontalPot,
            AnalogInput sonarSensor,
            double verticalPConstant, double verticalIConstant,
            double horizontalPConstant, double horizontalIConstant,
            double gentleHorizontalP, double gentleHorizontalI,
            double horizontalThreshold, double verticalThreshold,
            double canExtendHeight, double canGrabHeight, 
            double toteExtendHeight, double toteGrabHeight, 
            double stepCanExtendHeight, double stepCanGrabHeight,
            double retractedLocation, double pauseDistanceFromObject, 
            double horizontalHome, double verticalHome, 
            double slowDownRetractThreshold)
    {
        verticalTalon = verticalCANTalon;
        horizontalTalon = horizontalCANTalon;
        verticalPot = grabberVerticalPot;
        horizontalPot = grabberHorizontalPot;
        sonar = sonarSensor;
        verticalPI = new PIController(verticalPConstant, verticalIConstant);
        horizontalPI =
                new PIController(horizontalPConstant, horizontalIConstant);
        
        verticalPI.setThrottle(1.0);
        horizontalPI.setThrottle(0.7);
        
        m_gentleHorizontalP = gentleHorizontalP;
        m_gentleHorizontalI = gentleHorizontalI;
        m_canExtendHeight = canExtendHeight;
        m_canGrabHeight = canGrabHeight;
        m_toteExtendHeight = toteExtendHeight;
        m_toteGrabHeight = toteGrabHeight;
        m_stepCanExtendHeight = stepCanExtendHeight;
        m_stepCanGrabHeight = stepCanGrabHeight;
        m_pauseDistanceFromObject = pauseDistanceFromObject;
        m_toteHorizontalState = "waitForCommand";
        m_canHorizontalState = "waitForCommand";
        m_verticalState = "waitForCommand";
        m_horizontalThreshold = horizontalThreshold;
        m_verticalThreshold = verticalThreshold;
        m_horizontalHome = horizontalHome;
        m_verticalHome = verticalHome;
        m_slowDownRetractThreshold = slowDownRetractThreshold;
        m_retractedLocation = retractedLocation;
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
            double horizontalPConstant, double horizontalIConstant, 
            double gentleHorizontalP, double gentleHorizontalI)
    {
        verticalPI.inputConstants(verticalPConstant, verticalIConstant);
        horizontalPI.inputConstants(horizontalPConstant, horizontalIConstant);
        m_horizontalP = horizontalPConstant;
        m_horizontalI = horizontalIConstant;
        m_gentleHorizontalP = gentleHorizontalP;
        m_gentleHorizontalI = gentleHorizontalI;
    }

    void grabCan()
    {
        m_verticalState = "prepareForCanGrab";
        m_canHorizontalState = "waitForVertical";
    }

    void grabStepCan()
    {
        m_verticalState = "prepareForStepCanGrab";
        m_canHorizontalState = "extending";
    }

    void grabTote()
    {
        m_verticalState = "prepareForToteGrab";
        m_toteHorizontalState = "extending";
    }
    
    void reset()
    {
        m_verticalState = "waitForCommand";
        m_toteHorizontalState = "waitForCommand";
        m_canHorizontalState = "waitForCommand";
        m_verticalDone = false;
        m_horizontalExtended = false;
    }
    
    void goToHeight(double height)
    {
        mapSensors();
        verticalTalon.set(-verticalPI.getMotorValue(
                height, m_verticalPotDistance));
        System.out.println(m_verticalPotDistance);
    }
    
    void goToLength(double length)
    {
        mapSensors();
        horizontalTalon.set(-horizontalPI.getMotorValue
                (length, m_horizontalPotDistance));
        System.out.println(m_horizontalPotDistance);
    }
    
    void mapSensors()
    {
        m_sonarDistance = 2.2121617347 * sonar.getVoltage() 
                + 0.09;
        m_horizontalPotDistance = -0.4364133427 * 
                horizontalPot.getAverageVoltage() + m_horizontalYIntercept;
        m_verticalPotDistance = -0.2608156852 * verticalPot.getAverageVoltage()
                + m_verticalYIntercept;
    }

    void zeroPots()
    {
        m_horizontalYIntercept = 
                m_horizontalHome 
                - (-0.4364133427 * horizontalPot.getAverageVoltage());
        m_verticalYIntercept = 
                m_verticalHome 
                - (-0.2608156852 * verticalPot.getAverageVoltage());
    }
    
    void goHome()
    {
        m_toteHorizontalState = "goHome";
        m_verticalState = "goHome";
        m_horizontalFoundHome = false;
    }
    
    double limitPIOutput(double motorPower)
    {
        double returnPower;
        if(motorPower > 1.0)
        {
            returnPower = 1.0;
        }
        else if(motorPower < -1.0)
        {
            returnPower = -1.0;
        }
        else
        {
            returnPower = motorPower;
        }
        return returnPower;
    }
    
    /**
     * Handles the state machine for the motion of the vertical and horizonal
     * motors, must be run every loop in order for the Grabber to operate
     */
    void idle()
    {
        mapSensors();
        System.out.println(""+m_verticalDone);

        switch(m_toteHorizontalState)
        {
            case "waitForCommand":
                m_toteHorizontalSetpoint = m_sonarDistance;
                horizontalPI.reinit();
                break;
                
            case "goHome":
                if(Math.abs(m_horizontalHome - m_horizontalPotDistance + 0.02) 
                        > m_horizontalThreshold)
                {
                    horizontalTalon.set(-horizontalPI.getMotorValue(
                            m_horizontalHome, m_horizontalPotDistance));
                }
                else
                {
                    m_toteHorizontalState = "waitForCommand";
                    m_horizontalFoundHome = true;
                    horizontalTalon.set(0);
                }
                break;

            case "extending":
                m_doneCollecting = false;
                m_horizontalExtended = false;
                if(Math.abs(m_toteHorizontalSetpoint - m_pauseDistanceFromObject 
                        - m_horizontalPotDistance) > m_horizontalThreshold)
                {
                    horizontalTalon.set(-horizontalPI.getMotorValue(
                            m_toteHorizontalSetpoint, m_horizontalPotDistance));
                }
                else
                {
                    m_toteHorizontalState = "waitForVertical";
                    horizontalPI.reinit();
                }
                break;

            case "waitForVertical":
                if(m_verticalDone)
                {
                    m_toteHorizontalState = "finishExtending";
                    horizontalPI.inputConstants
                        (m_gentleHorizontalP, m_gentleHorizontalI);
                    horizontalPI.reinit();
                }
                horizontalTalon.set(0);
                break;
                
            case "finishExtending":
                if(Math.abs(m_toteHorizontalSetpoint - m_horizontalPotDistance) 
                        > m_horizontalThreshold)
                {
                    horizontalTalon.set(-horizontalPI.getMotorValue(
                            m_toteHorizontalSetpoint, m_horizontalPotDistance));
                }
                else
                {
                    m_toteHorizontalState = "waitForHook";
                    m_horizontalExtended = true;
                }
                break;
             
            case "waitForHook":
                if(m_hooked)
                {
                    m_toteHorizontalState = "retracting";
                    horizontalPI.inputConstants(m_horizontalP, m_horizontalI);
                    horizontalPI.reinit();
                }
                else
                {
                    horizontalTalon.set(0);
                }
                break;

            case "retracting":
                if((Math.abs(m_retractedLocation - m_horizontalPotDistance) 
                        > m_horizontalThreshold) && m_horizontalPotDistance
                        > m_slowDownRetractThreshold)
                {
                    horizontalTalon.set(-horizontalPI.getMotorValue(
                            m_retractedLocation, m_horizontalPotDistance));
                }
                else if(Math.abs(m_retractedLocation - m_horizontalPotDistance) 
                        > m_horizontalThreshold)
                {
                    
                    horizontalTalon.set(
                            1 * limitPIOutput(horizontalPI.getMotorValue(
                            m_retractedLocation, m_horizontalPotDistance)));
                }
                else
                {
                    m_toteHorizontalState = "waitForCommand";
                    horizontalTalon.set(0);
                    m_doneCollecting = true;
                }
                break;

            default:
                System.out.println(
                        "Grabber m_toteHorizontalState is in default state!");
                break;
        }
        
        switch(m_canHorizontalState)
        {
            case "waitForCommand":
                horizontalPI.reinit();
                break;

            case "waitForVertical":
                if(m_verticalDone)
                {
                    System.out.println("Setpoint: " + m_canHorizontalSetpoint);
                    m_canHorizontalState = "waitForSensing";
                    m_timestamp = System.currentTimeMillis();
                    horizontalPI.inputConstants
                        (m_horizontalP, m_horizontalI);
                    horizontalPI.reinit();
                }
                horizontalTalon.set(0);
                break;
                
            case "waitForSensing":
                if(System.currentTimeMillis() - m_timestamp > 100)
                {
                    m_canHorizontalState = "extending";
                }
                //m_canHorizontalSetpoint = m_sonarDistance;
                break;
                
            case "extending":
                m_doneCollecting = false;
                m_horizontalExtended = false;
                System.out.println(Math.abs(m_canHorizontalSetpoint 
                        - m_pauseDistanceFromObject 
                        - m_horizontalPotDistance));
                if(Math.abs(m_canHorizontalSetpoint - m_pauseDistanceFromObject 
                        - m_horizontalPotDistance) > m_horizontalThreshold)
                {
                    horizontalTalon.set(-horizontalPI.getMotorValue(
                            m_canHorizontalSetpoint, m_horizontalPotDistance));
                }
                else
                {
                    m_canHorizontalState = "finishExtending";
                    horizontalPI.reinit();
                    horizontalPI.inputConstants
                            (m_gentleHorizontalP, m_gentleHorizontalI);
                }
                break;
                
            case "finishExtending":
                if(Math.abs(m_canHorizontalSetpoint - m_horizontalPotDistance) 
                        > m_horizontalThreshold)
                {
                    horizontalTalon.set(-horizontalPI.getMotorValue(
                            m_canHorizontalSetpoint, m_horizontalPotDistance));
                }
                else
                {
                    m_canHorizontalState = "waitForHook";
                    m_horizontalExtended = true;
                }
                break;
             
            case "waitForHook":
                if(m_hooked)
                {
                    m_canHorizontalState = "retracting";
                    horizontalPI.inputConstants(m_horizontalP, m_horizontalI);
                    horizontalPI.reinit();
                }
                else
                {
                    horizontalTalon.set(0);
                }
                break;

            case "retracting":
                if((Math.abs(m_retractedLocation - m_horizontalPotDistance) 
                        > m_horizontalThreshold) && m_horizontalPotDistance
                        > m_slowDownRetractThreshold)
                {
                    horizontalTalon.set(-horizontalPI.getMotorValue(
                            m_retractedLocation, m_horizontalPotDistance));
                }
                else if(Math.abs(m_retractedLocation - m_horizontalPotDistance) 
                        > m_horizontalThreshold)
                {
                    
                    horizontalTalon.set(
                            -0.5 * limitPIOutput(horizontalPI.getMotorValue(
                            m_retractedLocation, m_horizontalPotDistance)));
                }
                else
                {
                    m_canHorizontalState = "waitForCommand";
                    horizontalTalon.set(0);
                    m_doneCollecting = true;
                }
                break;

            default:
                System.out.println(
                        "Grabber m_canHorizontalState is in default state!");
                break;
        }

        switch(m_verticalState)
        {
            case "waitForCommand":
                verticalTalon.set(0);
                verticalPI.reinit();
                break;
                
            case "goHome":
                if(m_horizontalFoundHome)
                {
                    if(Math.abs(m_verticalHome - m_verticalPotDistance) 
                            > m_verticalThreshold)
                    {
                        verticalTalon.set(-verticalPI.getMotorValue(
                                m_verticalHome, m_verticalPotDistance));
                    }
                    else
                    {
                        m_verticalState = "waitForCommand";
                        m_horizontalFoundHome = false;
                        verticalTalon.set(0);
                    }
                }
                    
                break;

            case "prepareForToteGrab":
                m_extendHeight = m_toteExtendHeight;
                m_grabHeight = m_toteGrabHeight;
                m_verticalState = "goToExtendHeight";
                break;

            case "prepareForCanGrab":
                m_extendHeight = 0.5;
                m_grabHeight = m_canGrabHeight;
                m_verticalState = "goToReadHeight";
                break;

            case "prepareForStepCanGrab":
                m_extendHeight = m_stepCanExtendHeight;
                m_grabHeight = m_stepCanGrabHeight;
                m_verticalState = "goToExtendHeight";
                break;
                
            case "goToReadHeight":
                m_extendHeight = 0.6;
                if(Math.abs(m_extendHeight - m_verticalPotDistance) 
                        > m_verticalThreshold && !m_foundReadPosition)
                {
                    verticalTalon.set(-verticalPI.getMotorValue(
                            m_extendHeight, m_verticalPotDistance));
                }
                else
                {
                    if(m_foundReadPosition == false)
                    {
                        m_timestamp = System.currentTimeMillis();
                        m_foundReadPosition = true;
                    }
                    mapSensors();
                    verticalTalon.set(0);
                    System.out.println("Waiting for a second..." + (System.currentTimeMillis() - m_timestamp)+"\n\n\n\n\n\n\n\n\n\n\n\n");
                    if(System.currentTimeMillis() - m_timestamp > 100)
                    {
                        m_canHorizontalSetpoint = m_sonarDistance;
                        System.out.println("NEW SETPOInT = " +m_canHorizontalSetpoint);
                        m_extendHeight = m_canExtendHeight;
                        m_grabHeight = m_canGrabHeight;
                        m_verticalState = "goToExtendHeight";
                        m_foundReadPosition = false;
                        //System.out.println("Waiting for a second..." + (System.currentTimeMillis() - m_timestamp)+"\n\n\n\n\n\n\n\n\n\n\n\n");
                    }
                }
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
                    verticalPI.reinit();
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
