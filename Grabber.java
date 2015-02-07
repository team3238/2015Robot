package org.usfirst.frc.team3238.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.AnalogInput;

public class Grabber
{
    CANTalon verticalTalon;
    CANTalon horizontalTalon;
    AnalogInput verticalPot;
    AnalogInput horizontalPot;
    AnalogInput sonar; 
    PIController verticalPI;
    PIController horizontalPI;

    double m_hPotValue;
    double m_vPotValue;
    double m_sonarValue;
    double m_retractedPotVal;
    double m_fullyRetractedPotVal;
    double m_desiredPotValue;
    double m_threshold;
    double m_canHeight;
    double m_toteHeight;
    double m_canCollectingHeight;
    double m_toteCollectingHeight;
    String m_stateMode;

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
        horizontalPI = new PIController(horizontalPConstant, horizontalIConstant);
    }
    
    boolean GoToHeight(double height)
    {
        boolean heightReached = false;
        verticalTalon.set(verticalPI.getAdjustedRotationValue(height, verticalPot.getVoltage()));
        if(Math.abs(m_vPotValue - height) <= m_threshold)
        {
            heightReached = true;
        }
        return heightReached;
    }

    boolean GrabTote()
    {
        boolean retracted = false;
        boolean toteCollected = false;
        
        m_hPotValue = horizontalPot.getVoltage();
        m_sonarValue = 2.678677012 * sonar.getVoltage() +
            0.0204464172;       
        m_desiredPotValue = /*Fancy math stuff from sonar val*/;
        
        
        
        
        
        switch(m_stateMode)
        {
            case "adjustingHeight":
                if(GoToHeight(m_toteHeight))
                {
                    m_stateMode = "adjustingLength";
                }
                break;
                
            case "adjustingLength":
                PIController.getAdjustedRotationValue()
                horizontalTalon.set(horizontalPI.getAdjustedRotationValue(m_desiredPotValue, m_sonarValue));
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
                horizontalTalon.set(horizontalPI.getAdjustedRotationValue(m_retractedPotVal, m_hPotValue));
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
                horizontalTalon.set(horizontalPI.getAdjustedRotationValue(m_fullyRetractedPotVal, m_hPotValue));
                if(Math.abs(m_hPotValue - m_fullyRetractedPotVal) <= m_threshold)
                {
                    m_stateMode = "default";
                }
                break;
                
            default:
                break;
                
        }
        
    }
}
