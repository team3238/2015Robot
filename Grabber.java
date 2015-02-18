package org.usfirst.frc.team3238.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;

/**
 * This is the class that controls the vertical and horizontal collector.
 *
 * @author Aaron Jenson and Nick Papadakis and Anders Sjoboen
 */

public class Grabber
{
    CANTalon verticalTalon, horizontalTalon;
    AnalogInput verticalPot, horizontalPot, sonar, irSensor;
    PIController verticalPI, horizontalPI;
    UltraFilter ultrasonicFilter;

    // Store sensor values
    double m_horizontalPotDistance;
    double m_verticalPotDistance;
    double m_sonarDistance;
    double m_infraredDistance;
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
    double m_readCanHeight;
    double m_readCanStepHeight;
    double m_readHeight;
    
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
    String m_canAutoHorizontalState;

    // Status booleans
    boolean m_doneCollecting = false;
    boolean m_hooked = false;
    boolean m_horizontalExtended = false;
    boolean m_verticalDone = false;
    boolean m_foundReadPosition = false;
    boolean m_horizontalFoundHome = false;
    boolean m_finishedAutoGrab = false;

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
            double slowDownRetractThreshold, AnalogInput infraredSensor)
    {
        verticalTalon = verticalCANTalon;
        horizontalTalon = horizontalCANTalon;
        verticalPot = grabberVerticalPot;
        horizontalPot = grabberHorizontalPot;
        sonar = sonarSensor;
        verticalPI = new PIController(verticalPConstant, verticalIConstant);
        horizontalPI =
                new PIController(horizontalPConstant, horizontalIConstant);
        
        ultrasonicFilter = new UltraFilter();
        
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
        m_canAutoHorizontalState = "waitForCommand";
        m_horizontalThreshold = horizontalThreshold;
        m_verticalThreshold = verticalThreshold;
        m_horizontalHome = horizontalHome;
        m_verticalHome = verticalHome;
        m_slowDownRetractThreshold = slowDownRetractThreshold;
        m_retractedLocation = retractedLocation;
        irSensor = infraredSensor;
    }
    
    void inputConstants(double verticalPConstant, double verticalIConstant,
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
    	m_gentleHorizontalP = gentleHorizontalP;
        m_gentleHorizontalI = gentleHorizontalI;
        m_canExtendHeight = canExtendHeight;
        m_canGrabHeight = canGrabHeight;
        m_toteExtendHeight = toteExtendHeight;
        m_toteGrabHeight = toteGrabHeight;
        m_stepCanExtendHeight = stepCanExtendHeight;
        m_stepCanGrabHeight = stepCanGrabHeight;
        m_pauseDistanceFromObject = pauseDistanceFromObject;
        m_horizontalThreshold = horizontalThreshold;
        m_verticalThreshold = verticalThreshold;
        m_horizontalHome = horizontalHome;
        m_verticalHome = verticalHome;
        m_slowDownRetractThreshold = slowDownRetractThreshold;
        m_retractedLocation = retractedLocation;
        verticalPI.inputConstants(verticalPConstant, verticalIConstant);
        horizontalPI.inputConstants(horizontalPConstant, horizontalIConstant);
        m_horizontalP = horizontalPConstant;
        m_horizontalI = horizontalIConstant;
        m_readCanHeight = 0.6;
        m_readCanStepHeight = 0.7;
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
        reset();
        m_verticalState = "prepareForCanGrab";
        m_canHorizontalState = "waitForVertical";
        m_readCanHeight = 0.6;
    }

    void grabStepCan()
    {
        reset();
        m_verticalState = "prepareForStepCanGrab";
        m_canHorizontalState = "waitForVertical";
        m_readCanHeight = 0.55;
    }
    
    void grabStepCanAuto()
    {
        reset();
        m_verticalState = "prepareForStepCanGrab";
        m_canAutoHorizontalState = "waitForVertical";
        m_readCanHeight = 0.55;
    }

    void grabTote()
    {
        reset();
        m_verticalState = "prepareForToteGrab";
        m_toteHorizontalState = "goHome";
    }
    
    void reset()
    {
        m_verticalState = "waitForCommand";
        m_toteHorizontalState = "waitForCommand";
        m_canHorizontalState = "waitForCommand";
        m_canAutoHorizontalState = "waitForCommand";
        m_verticalDone = false;
        m_horizontalExtended = false;
        m_foundReadPosition = false;
        m_hooked = false;
        m_finishedAutoGrab = false;
        horizontalPI.reinit();
        verticalPI.reinit();
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
        ultrasonicFilter.addData(2.2121617347 * sonar.getVoltage() + 0.09);
        m_sonarDistance = ultrasonicFilter.getMedian();
        m_horizontalPotDistance = -0.4364133427 * 
                horizontalPot.getAverageVoltage() + m_horizontalYIntercept;
        m_verticalPotDistance = -0.2608156852 * verticalPot.getAverageVoltage()
                + m_verticalYIntercept;
        m_infraredDistance = 0.2680762026*Math.pow(irSensor.getAverageVoltage(), -1.130124285);
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
        reset();
    	m_horizontalFoundHome = false;
        m_toteHorizontalState = "goHome";
        m_verticalState = "goHome";
        horizontalPI.reinit();
    }
    
    void goToAvoidTotePosition()
    {
        reset();
        m_verticalState = "waitToAvoidTotePosition";
        m_toteHorizontalState = "goHome";
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
        System.out.println(m_canAutoHorizontalState);
        switch(m_canAutoHorizontalState)
        {
            case "waitForCommand":
                horizontalPI.reinit();
                break;

            case "waitForVertical":
                if(m_verticalDone)
                {
                    m_canAutoHorizontalState = "waitForSensing";
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
                    m_canAutoHorizontalState = "extending";
                }
                //m_canHorizontalSetpoint = m_sonarDistance;
                break;
                
            case "extending":
                horizontalPI.setThrottle(1.0);
                m_doneCollecting = false;
                m_horizontalExtended = false;
//                System.out.println(Math.abs(m_canHorizontalSetpoint 
//                        - m_pauseDistanceFromObject 
//                        - m_horizontalPotDistance));
                if(Math.abs(m_canHorizontalSetpoint - m_pauseDistanceFromObject 
                        - m_horizontalPotDistance) > m_horizontalThreshold)
                {
                    horizontalPI.inputConstants(9999, 1);
                    horizontalTalon.set(-horizontalPI.getMotorValue(
                            m_canHorizontalSetpoint, m_horizontalPotDistance));
                }
                else
                {
                    m_canAutoHorizontalState = "finishExtending";
                    horizontalPI.reinit();
                    horizontalPI.setThrottle(1.0);
//                    horizontalPI.inputConstants
//                            (m_gentleHorizontalP, m_gentleHorizontalI);
                }
                break;
                
            case "finishExtending":
                //Math.abs(
                if(((m_canHorizontalSetpoint+.40) - m_horizontalPotDistance) 
                        <= m_horizontalThreshold)
                {
                    //horizontalTalon.set(-horizontalPI.getMotorValue(
                     //       m_canHorizontalSetpoint, m_horizontalPotDistance));
                    horizontalTalon.set(0.75);
                }
                else
                {
                    m_canAutoHorizontalState = "waitForHook";
                    m_horizontalExtended = true;
                }
                break;
             
            case "waitForHook":
                if(m_hooked)
                {
                    m_canAutoHorizontalState = "retracting";
                    horizontalPI.inputConstants(m_horizontalP, m_horizontalI);
                    horizontalPI.reinit();
                }
                else
                {
                    horizontalTalon.set(0);
                }
                break;

            case "retracting":
                if((Math.abs(0.5 - m_horizontalPotDistance) 
                        > m_horizontalThreshold) && m_horizontalPotDistance
                        > m_slowDownRetractThreshold)
                {
                    horizontalPI.setThrottle(1.0);
                    horizontalTalon.set(-horizontalPI.getMotorValue(
                            0.5, m_horizontalPotDistance));
                }
                else if(horizontalTalon.getOutputCurrent() < 18)
                {
                    horizontalPI.setThrottle(0.8);
                    horizontalTalon.set(-horizontalPI.getMotorValue(
                            0.5, m_horizontalPotDistance));
                }
                else
                {
                    m_canAutoHorizontalState = "waitForCommand";
                    m_verticalState = "lowerABit";
                    horizontalTalon.set(0);
                    m_doneCollecting = true;
                }
                break;
                

            default:
                System.out.println(
                        "Grabber m_canAutoHorizontalState is in default state!");
                break;
        }
        switch(m_toteHorizontalState)
        {
            case "waitForCommand":
                m_toteHorizontalSetpoint = m_sonarDistance;
                horizontalPI.reinit();
                break;
                
            case "stopMovingHorizontal":
                horizontalTalon.set(0);
                m_toteHorizontalState = "waitForCommand";
                break;
                
            case "goHome":
                if(horizontalTalon.getOutputCurrent() < 17)
                {
                    horizontalTalon.set(0.65);
                }
                else
                {
                    m_toteHorizontalState = "waitForCommand";
                    m_horizontalFoundHome = true;
                    horizontalTalon.set(0);
                    verticalPI.reinit();
                }
                break;
                
            case "extendToToteSlowdown":
                m_doneCollecting = false;
                m_horizontalExtended = false;
                if(Math.abs(m_toteHorizontalSetpoint - m_pauseDistanceFromObject 
                        - m_horizontalPotDistance) > m_horizontalThreshold)
                {
                    horizontalPI.setThrottle(0.75);
                    horizontalTalon.set(-horizontalPI.getMotorValue(
                            m_toteHorizontalSetpoint, m_horizontalPotDistance));
                }
                else
                {
                    m_toteHorizontalState = "finishExtendingTote";
                    horizontalPI.reinit();
                }
                break;
                
            case "finishExtendingTote":
                if(Math.abs(m_toteHorizontalSetpoint - m_horizontalPotDistance) 
                        > m_horizontalThreshold)
                {
                    horizontalPI.setThrottle(0.5);
                    horizontalTalon.set(-horizontalPI.getMotorValue(
                            m_toteHorizontalSetpoint, m_horizontalPotDistance));
                }
                else
                {
                    m_toteHorizontalState = "waitForHook";
                    m_horizontalExtended = true;
                    horizontalPI.setThrottle(1.0);
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
                    //horizontalPI.inputConstants
                      //  (m_gentleHorizontalP, m_gentleHorizontalI);
                    horizontalPI.reinit();
                }
                horizontalTalon.set(0);
                break;
                
            case "finishExtending":
                if(Math.abs(m_toteHorizontalSetpoint - m_horizontalPotDistance) 
                        > m_horizontalThreshold)
                {
                    horizontalPI.setThrottle(1);
                    horizontalTalon.set(-horizontalPI.getMotorValue(
                            m_toteHorizontalSetpoint, m_horizontalPotDistance));
                }
                else
                {
                    m_toteHorizontalState = "waitForHook";
                    m_horizontalExtended = true;
                    horizontalPI.setThrottle(1.0);
                }
                break;
             
            case "waitForHook":
                if(m_hooked)
                {
                    m_toteHorizontalState = "retracting";
                    //horizontalPI.inputConstants(m_horizontalP, m_horizontalI);
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
        
        //System.out.println(m_canHorizontalState);
        switch(m_canHorizontalState)
        {
            case "waitForCommand":
                horizontalPI.reinit();
                break;

            case "waitForVertical":
                if(m_verticalDone)
                {
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
                horizontalPI.setThrottle(1.0);
                m_doneCollecting = false;
                m_horizontalExtended = false;
//                System.out.println(Math.abs(m_canHorizontalSetpoint 
//                        - m_pauseDistanceFromObject 
//                        - m_horizontalPotDistance));
                if(Math.abs(m_canHorizontalSetpoint - m_pauseDistanceFromObject 
                        - m_horizontalPotDistance) > m_horizontalThreshold)
                {
                    horizontalPI.inputConstants(9999, 1);
                    horizontalTalon.set(-horizontalPI.getMotorValue(
                            m_canHorizontalSetpoint, m_horizontalPotDistance));
                }
                else
                {
                    m_canHorizontalState = "finishExtending";
                    horizontalPI.reinit();
                    horizontalPI.setThrottle(0.7);
//                    horizontalPI.inputConstants
//                            (m_gentleHorizontalP, m_gentleHorizontalI);
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
                    horizontalPI.setThrottle(1.0);
                    horizontalTalon.set(-horizontalPI.getMotorValue(
                            m_retractedLocation, m_horizontalPotDistance));
                }
                else if(horizontalTalon.getOutputCurrent() < 18)
                {
                    horizontalPI.setThrottle(0.8);
                    horizontalTalon.set(-horizontalPI.getMotorValue(
                            m_retractedLocation, m_horizontalPotDistance));
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
                
            case "lowerABit":
                if(Math.abs(0.55 - m_verticalPotDistance) 
                        > m_verticalThreshold)
                {
                    verticalTalon.set(-verticalPI.getMotorValue(
                            0.55, m_verticalPotDistance));
                }
                else
                {
                    goHome();
                }
                break;
                
            case "waitToAvoidTotePosition":
                if(m_horizontalFoundHome == true)
                {
                    m_verticalState = "avoidTotePosition";
                }
                verticalTalon.set(0);
                break;
                
            case "avoidTotePosition":
                if(Math.abs(0.4 - m_verticalPotDistance) 
                        > m_verticalThreshold)
                {
                    verticalTalon.set(-verticalPI.getMotorValue(
                            0.4, m_verticalPotDistance));
                }
                else
                {
                    m_verticalState = "waitForCommand";
                }
                break;
                
            case "goHome":
                if(m_horizontalFoundHome)
                {
                    if(verticalTalon.getOutputCurrent() < 6)
                    {
                        verticalTalon.set(1);
                        //System.out.println(verticalTalon.getOutputCurrent());
                    }
                    else
                    {
                        m_verticalState = "waitForCommand";
                        m_horizontalFoundHome = false;
                        verticalTalon.set(0);
                        zeroPots();
                        reset();
                        m_finishedAutoGrab = true;
                        System.out.println("////////////////////////////////////////////");
                    }
                }
                else
                {
                	verticalTalon.set(0);
                }
                    
                break;
                
            case "goHomeToteGrab":
                if(m_horizontalFoundHome)
                {
                    if(verticalTalon.getOutputCurrent() < 6)
                    {
                        verticalTalon.set(1);
                        //System.out.println(verticalTalon.getOutputCurrent());
                    }
                    else
                    {
                        m_verticalState = "waitForHorizontal";
                        m_toteHorizontalState = "extendToToteSlowdown";
                        m_horizontalFoundHome = false;
                        verticalTalon.set(0);
                        zeroPots();
                        //reset();
                    }
                }
                else
                {
                    verticalTalon.set(0);
                }
                    
                break;

            case "prepareForToteGrab":
                m_extendHeight = m_toteExtendHeight;
                m_grabHeight = m_toteGrabHeight;
                m_verticalState = "goHomeToteGrab";
                break;

            case "prepareForCanGrab":
                m_extendHeight = m_canExtendHeight;
                m_grabHeight = m_canGrabHeight;
                m_verticalState = "goToReadHeight";
                m_verticalDone = false;
                m_readHeight = m_readCanHeight;
                break;

            case "prepareForStepCanGrab":
                m_extendHeight = m_stepCanExtendHeight;
                System.out.println("Extend Height = "+m_extendHeight);
                m_grabHeight = m_stepCanGrabHeight;
                System.out.println("Grab Height = "+m_grabHeight);
                m_verticalState = "goToReadHeight";
                m_readHeight = m_readCanStepHeight;
                break;
                
            case "goToReadHeight":
                //m_extendHeight = 0.6;//read height is 0.6
                if(Math.abs(m_readHeight - m_verticalPotDistance) 
                        > m_verticalThreshold && !m_foundReadPosition)
                {
                    verticalTalon.set(-verticalPI.getMotorValue(
                            m_readHeight, m_verticalPotDistance));
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
                    if(System.currentTimeMillis() - m_timestamp > 100)
                    {
                        m_canHorizontalSetpoint = m_sonarDistance;
                        //m_extendHeight = m_canExtendHeight;
                        //m_grabHeight = m_canGrabHeight;
                        m_verticalState = "goToExtendHeight";
                        m_foundReadPosition = false;
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
