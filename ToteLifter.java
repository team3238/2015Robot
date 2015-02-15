package org.usfirst.frc.team3238.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Servo;

/**
 * Class for controlling the lifting and storing of totes
 * 
 * @author James Campbell, Aaron Jenson, Anders Sjoboen, and Nick Papadakis
 */
public class ToteLifter
{
    CANTalon leftTalon, rightTalon;
    Servo leftLifterServo, rightLifterServo;
    AnalogInput leftPot, rightPot;
    PIController piControllerLeft;
    PIController piControllerRight;

    final int m_leftOpenServoPosition = 0;
    final int m_leftCloseServoPosition = 170;
    final int m_rightOpenServoPosition = 180;
    final int m_rightCloseServoPosition = 10;
    
    double m_collectLiftPosition;
    double m_openDogsLiftPosition;
    double m_closeDogsLiftPosition;
    double m_waitLiftPosition;
    
    String m_stateMode;
    String m_addSubstate;
    String m_dropSubstate;
    
    double m_threshold;
    boolean totesDropped;
    double m_homeHeight;
    
    double m_leftHeight;
    double m_rightHeight;

    double m_leftPotYIntercept;
    double m_rightPotYIntercept;
    
    long m_timeStamp;
    
    ToteLifter(CANTalon leftLift, CANTalon rightLift, Servo leftServo, 
            Servo rightServo, AnalogInput potentiometerLeft, 
            AnalogInput potentiometerRight, double leftP, double leftI, 
            double rightP, double rightI, double accuracyThreshold, 
            double homeHeight, double waitLiftPosition, 
            double openDogsLiftPosition, double closeDogsLiftPosition)
    {
        leftTalon = leftLift;
        rightTalon = rightLift;
        leftLifterServo = leftServo;
        rightLifterServo = rightServo;
        leftPot = potentiometerLeft;
        rightPot = potentiometerRight;
        piControllerLeft = new PIController(leftP, leftI);
        piControllerRight = new PIController(rightP, rightI);
        m_threshold = accuracyThreshold;
        m_homeHeight = homeHeight;
        m_collectLiftPosition = homeHeight - 0.0586;
        m_waitLiftPosition = waitLiftPosition + 0.06;
        m_openDogsLiftPosition = openDogsLiftPosition;
        m_closeDogsLiftPosition = closeDogsLiftPosition + 0.025;
        m_stateMode = "AddTote";
        m_addSubstate = "GoToWaitLiftPosition";
        m_dropSubstate = "WaitForCommand";
        totesDropped = false;
    }

    /**
     * Initializes member variables for the tote lifter
     */
    void reinit()
    {
        m_addSubstate = "GoToWaitLiftPosition";
        m_dropSubstate = "WaitForCommand";
        totesDropped = false;
    }
    
    void disable()
    {
        m_stateMode = "WaitForCommand";
        m_addSubstate = "WaitForCommand";
        m_dropSubstate = "WaitForCommand";
    }
    
    void zeroPots()
    {
        m_leftPotYIntercept = m_homeHeight - (0.132 * 
                leftPot.getAverageVoltage());
        m_rightPotYIntercept = m_homeHeight - (-0.132 * 
                rightPot.getAverageVoltage());
    }
    
    void mapPots()
    {
        m_leftHeight =  0.132 * leftPot.getAverageVoltage() + 
                m_leftPotYIntercept - 0.015;
        m_rightHeight = -0.132 * rightPot.getAverageVoltage() + 
                m_rightPotYIntercept;
    }

    /**
     * Opens the dogs
     */
    void openDogs()
    {
        leftLifterServo.setAngle(m_leftOpenServoPosition);
        rightLifterServo.setAngle(m_rightOpenServoPosition);
    }

    /**
     * Closes the dogs
     */
    void closeDogs()
    {
        leftLifterServo.setAngle(m_leftCloseServoPosition);
        rightLifterServo.setAngle(m_rightCloseServoPosition);
    }
    
    void inputPIConstants(double leftPConstant, double leftIConstant,
            double rightPConstant, double rightIConstant)
    {
        piControllerLeft.inputConstants(leftPConstant, leftIConstant);
        piControllerRight.inputConstants(rightPConstant, rightIConstant);
    }

    /**
     * Moves the tote lifter to a desired position
     * 
     * @param setpoint The desired position
     * @return Whether or not you have reached the desired position
     */
    boolean goToHeight(double setpoint)
    {
        boolean leftDone = false;
        boolean rightDone = false;
        boolean positionReached = false;
        mapPots();
        
        if(Math.abs(setpoint - m_leftHeight) > m_threshold)
        {
            leftTalon.set(-piControllerLeft.getMotorValue
                    (setpoint, m_leftHeight));
        }
        else
        {
            leftTalon.set(0);
            leftDone = true;
        }
        if(Math.abs(setpoint - m_rightHeight) > m_threshold)
        {
            rightTalon.set(piControllerRight.getMotorValue
                    (setpoint, m_rightHeight));
        }
        else
        {
            rightTalon.set(0);
            rightDone = true;
        }
        if(leftDone && rightDone)
        {
            positionReached = true;
        }
        return positionReached;
    }

    /**
     * Sets the states to start the dropping phases
     */
    void dropTotes()
    {
        m_dropSubstate = "GoToOpenDogsPosition";
        m_stateMode = "DropTotes";
        piControllerLeft.reinit();
        piControllerRight.reinit();
    }

    /**
     * Sets the states to start the storing phases
     */
    void addTote()
    {
        m_addSubstate = "GoToLiftPosition";
        m_stateMode = "AddTote";
        piControllerLeft.reinit();
        piControllerRight.reinit();
    }

    boolean getTotesDropped()
    {
        return totesDropped;
    }
    
    void reinitPIControllers()
    {
        piControllerLeft.reinit();
        piControllerRight.reinit();
    }

    /**
     * Controls the phases that the tote lifter goes through when it runs, this
     * must be called every loop for the tote lifter to operate
     */
    //TODO All states need comments
    void idle()
    {
        totesDropped = false;
        switch(m_stateMode)
        {
        	case "WaitForCommand":
        		leftTalon.set(0);
        		rightTalon.set(0);
        	    break;
        		
            case "AddTote":
                switch(m_addSubstate)
                {
                    case "WaitForCommand":
                        leftTalon.set(0);
                        rightTalon.set(0);
                        break;

                    case "GoToLiftPosition":
                        if(goToHeight(m_collectLiftPosition))
                        {
                            m_addSubstate = "GoToOpenDogsPosition";
                            piControllerLeft.reinit();
                            piControllerRight.reinit();
                        }
                        break;

                    case "GoToOpenDogsPosition":
                        if(goToHeight(m_openDogsLiftPosition))
                        {
                            m_addSubstate = "OpenDogs";
                            piControllerLeft.reinit();
                            piControllerRight.reinit();
                        }
                        break;

                    case "OpenDogs":
                        openDogs();
                        m_addSubstate = "GoToCloseDogsPosition";
                        break;

                    case "GoToCloseDogsPosition":
                        if(goToHeight(m_closeDogsLiftPosition))
                        {
                            m_addSubstate = "CloseDogs";
                            piControllerLeft.reinit();
                            piControllerRight.reinit();
                            m_timeStamp = System.currentTimeMillis();
                        }
                        break;

                    case "CloseDogs":
                        closeDogs();
                        if(System.currentTimeMillis() - m_timeStamp > 500)
                        {
                            m_addSubstate = "GoToWaitLiftPosition";
                        }
                        break;

                    case "GoToWaitLiftPosition":
                        if(goToHeight(m_waitLiftPosition))
                        {
                            m_addSubstate = "WaitForCommand";
                        }
                        break;

                    default:
                        break;
                }
                break;

            case "DropTotes":
                switch(m_dropSubstate)
                {
                    case "WaitForCommand":
                        leftTalon.set(0);
                        rightTalon.set(0);
                        break;

                    case "GoToOpenDogsPosition":
                        if(goToHeight(m_openDogsLiftPosition))
                        {
                            m_dropSubstate = "OpenDogs";
                            piControllerLeft.reinit();
                            piControllerRight.reinit();
                        }
                        break;

                    case "OpenDogs":
                        openDogs();
                        m_dropSubstate = "GoToLiftPosition";
                        break;

                    case "GoToLiftPosition":
                        if(goToHeight(m_collectLiftPosition))
                        {
                            m_dropSubstate = "WaitForCommand";
                            totesDropped = true;
                        }
                        break;

                    default:
                        break;
                }
                break;

            default:
                break;
        }
    }
}
