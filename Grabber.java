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
    
    /**
     * @param m_hPotValue Variable to store the horizontal potentiometer
     * value.
     * @param m_vPotValue Variable to store the vertical potentiometer
     * value.
     * @param m_sonarValue Variable to store the Ultrasonic value.
     * @param m_retractedPotVal The length when a tote is all the way
     * against the chassis.
     * @param m_fullyRetractedPotVal The length when the arm is all the way
     * retracted into the robot.
     * @param m_threshold The stored level of accuracy required by the PI
     * controller.
     * @param m_canHeight The height at which the grabber must be as it
     * extends to the can.
     * @param m_toteHeight The height at which the grabber must be as it
     * extends to the tote.
     * @param m_canCollectingHeight The height that the grabber must raise
     * to so it actually engages with the can.
     * @param m_toteCollectingHeight The height that the grabber must raise
     * to so it actually engages with the can.
     * @param m_stateMode Variable to control the switch statement in
     * GrabTote.
     * @param m_stateIndex Variable to control the switch statement in
     * GrabCan.
     */

    double m_hPotValue;
    double m_vPotValue;
    double m_sonarValue;
    double m_retractedPotVal;
    double m_fullyRetractedPotVal;
    double m_threshold;
    double m_canHeight;
    double m_toteHeight;
    double m_canCollectingHeight;
    double m_toteCollectingHeight;
    String m_stateMode;
    String m_stateIndex;

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
     * Collects a tote and retracts back to the robot.
     */

    boolean GrabTote()
    {
        if(init)
        {
            m_stateMode = "adjustingHeight";
        }
        boolean retracted = false;
        boolean toteCollected = false;
        boolean init = true;
        
        m_hPotValue = horizontalPot.getVoltage();
        m_sonarValue = 2.678677012 * sonar.getVoltage() +
            0.0204464172;       
        double m_desiredPotValue = /*Fancy math stuff from sonar val*/;
        
        
        
        switch(m_stateMode)
        {
            case "adjustingHeight":
                if(GoToHeight(m_toteHeight))
                {
                    m_stateMode = "adjustingLength";
                }
                break;
                
            case "adjustingLength":
                horizontalTalon.set(horizontalPI.
                        getAdjustedRotationValue(m_desiredPotValue,
                            m_sonarValue));
                if(Math.abs(m_hPotValue - m_desiredPotValue) <= m_threshold)
                {
                    m_stateMode = "readjustingHeight";
                }
                break;
                
            case "readjustingHeight":
                if(GoToHeight(m_toteCollectingHeight))
                {
                    m_stateMode = "retracting";
                }
                break;
                
            case "retracting":
                horizontalTalon.set(horizontalPI.
                        getAdjustedRotationValue(m_retractedPotVal,
                            m_hPotValue));
                if(Math.abs(m_hPotValue - m_retractedPotVal) <= m_threshold)
                {
                    retracted = true;
                    m_stateMode = "reLoweringHeight";
                }
                break;
                
            case "reLoweringHeight":
                if(GoToHeight(m_toteHeight))
                {
                    m_stateMode = "retractingMore";
                }
                break;
                
            case "retractingMore":
                horizontalTalon.set(horizontalPI.
                        getAdjustedRotationValue(m_fullyRetractedPotVal,
                            m_hPotValue));
                if(Math.abs(m_hPotValue - m_fullyRetractedPotVal) <=
                        m_threshold)
                {
                    m_stateMode = "default";
                    init = false;
                }
                break;
                
            default:
                init = false;
                break;


                
        }

    }

    /**
     * This method collects a can, but does not retract it. It uses the PI 
     * controller class.
     */

    boolean GrabCan()
    {
        if(init)
        {
            m_stateIndex = "adjustingHeight";
        }
        boolean canCollected = false;
        boolean init = true;
        
        m_hPotValue = horizontalPot.getVoltage();
        m_sonarValue = 2.678677012 * sonar.getVoltage() +
            0.0204464172;       
        double m_desiredPotValue = /*Fancy math stuff from sonar val*/;
      

        switch(m_stateIndex)
        {
            case "adjustingHeight":
                if(GoToHeight(m_canHeight))
                {
                    m_stateIndex = "adjustingLength";
                }
                break;

            case "adjustingLength":
                horizontalTalon.set(horizontalPI.
                        getadjustedRotationValue(m_desiredPotValue,
                            m_hPotValue);
                if(Math.abs(m_hPotValue - m_desiredPotValue) <= m_threshold)
                {
                    m_stateIndex = "readjustingHeight";
                }
                break;

            case "readjustingHeight":
                if(GoToHeight(m_canCollectingHeight))
                {
                    m_stateIndex = "default";
                    cancollected = true;
                    init = false;
                }
                break;

            default:
                init = false;
                break;
        }
    }
        
}
