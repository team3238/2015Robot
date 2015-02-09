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
    CANTalon verticalTalon;
    CANTalon horizontalTalon;
    AnalogInput verticalPot;
    AnalogInput horizontalPot;
    AnalogInput sonar; 
    PIController verticalPI;
    PIController horizontalPI;
    
    //Store sensor values
    double m_hPotValue;
    double m_vPotValue;
    double m_sonarValue;

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

    //Switch control
    String m_stateMode;
    String m_stateIndex;

    //Status booleans
    boolean canCollected;
    boolean retracted;

    Grabber(int verticalTalonChannel, int horizontalTalonChannel, 
            int verticalPotPort, int horizontalPotPort, int sonarPort,
            double verticalPConstant, double verticalIConstant, 
            double horizontalPConstant, double horizontalIConstant)
    {
        verticalTalon = new CANTalon(verticalTalonChannel);
        horizontalTalon = new CANTalon(horizontalTalonChannel);
        verticalPot = new AnalogInput(verticalPotPort);
        horizontalPot = new AnalogInput(horizontalPotPort);
        sonar = new AnalogInput(sonarPort);
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

    boolean GoToHeight(double height)
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

    /**
     * Allows the robot to run
     */

    void idle()
    {
        m_hPotValue = horizontalPot.getVoltage();
        m_sonarValue = 2.678677012 * sonar.getVoltage() +
            0.0204464172;       
        double m_desiredPotValue = /*Fancy math stuff from sonar val*/;

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
