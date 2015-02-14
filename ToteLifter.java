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
    CANTalon liftTalonLeft, liftTalonRight;
    Servo leftLifterServo, rightLifterServo;
    AnalogInput leftPot, rightPot;
    PIController piControllerLeft;
    PIController piControllerRight;

    double m_collectLiftPosition;
    double m_openDogsLiftPosition;
    double m_closeDogsLiftPosition;
    double m_waitLiftPosition;
    double m_openServoPosition;
    double m_closeServoPosition;
    String m_stateIndex;
    String m_stateMode;
    double m_threshold;
    boolean totesDropped;

    ToteLifter(CANTalon leftLift, CANTalon rightLift, Servo leftServo, 
            Servo rightServo, AnalogInput potentiometerLeft, 
            AnalogInput potentiometerRight, PIController piContLeft, 
            PIController piContRight, double accuracyThreshold, 
            double openServoPosition, double closeServoPosition)
    {
        m_threshold = accuracyThreshold;
        m_openServoPosition = openServoPosition;
        m_closeServoPosition = closeServoPosition;
        liftTalonLeft = leftLift;
        liftTalonRight = rightLift;
        leftLifterServo = leftServo;
        rightLifterServo = rightServo;
        leftPot = potentiometerLeft;
        rightPot = potentiometerRight;
        piControllerLeft = piContLeft;
        piControllerRight = piContRight;
    }

    /**
     * Initializes member variables for the tote lifter
     */
    void init()
    {
        m_stateIndex = "Nothing";
        m_stateMode = "Nothing";
        totesDropped = false;
    }

    /**
     * Opens the dogs
     */
    void openDogs()
    {
        leftLifterServo.set(m_openServoPosition);
        rightLifterServo.set(m_openServoPosition);
    }

    /**
     * Closes the dogs
     */
    void closeDogs()
    {
        leftLifterServo.set(m_closeServoPosition);
        rightLifterServo.set(m_closeServoPosition);
    }
    
    void inputPIConstants(double leftPConstant, double leftIConstant,
            double rightPConstant, double rightIConstant)
    {
        piControllerLeft.inputConstants(leftPConstant, leftIConstant);
        piControllerRight.inputConstants(rightPConstant, rightIConstant);
    }
    
    void rightLifterGoToHeight(double height)
    {
        double currentHeight = -0.132 * rightPot.getVoltage() + 0.8485283019;
        if(Math.abs(height - currentHeight) > m_threshold)
            {
            liftTalonRight.set(piControllerRight.getMotorValue
                (height, currentHeight));
            }
        else
        {
            liftTalonRight.set(0);
        }
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
        double leftHeight = 0.132 * leftPot.getVoltage() + 0.2025183198;
        double rightHeight = -0.132 * rightPot.getVoltage() + 0.8485283019;
        
        if(Math.abs(setpoint - leftHeight) > m_threshold)
        {
            liftTalonLeft.set(-piControllerLeft.getMotorValue
                    (setpoint, leftHeight));
        }
        else
        {
            liftTalonLeft.set(0);
        }
        if(Math.abs(setpoint - rightHeight) > m_threshold)
        {
            liftTalonRight.set(piControllerRight.getMotorValue
                    (setpoint, rightHeight));
        }
        else
        {
            liftTalonRight.set(0);
        }
        if(leftDone && rightDone)
        {
            positionReached = true;
        }
        return positionReached;
    }

    /**
     * Sets the states to start the droping phases
     */
    void dropTotes()
    {
        m_stateIndex = "GoToOpenDogsPosition";
        m_stateMode = "DropTotes";
        piControllerLeft.reinit();
        piControllerRight.reinit();
    }

    /**
     * Sets the states to start the storing phases
     */
    void storeTote()
    {
        m_stateIndex = "GoToLiftPosition";
        m_stateMode = "AddTote";
        piControllerLeft.reinit();
        piControllerRight.reinit();
    }

    boolean getTotesDropped()
    {
        return totesDropped;
    }

    /**
     * Controls the phases that the tote lifter goes through when it runs, this
     * must be called every loop for the tote lifter to operate
     */
    //TO-DO All states need comments
    void idle()
    {
        totesDropped = false;
        switch(m_stateMode)
        {
        	case "Nothing":
        		break;
        		
            case "AddTote":
                switch(m_stateIndex)
                {
                    //TO-DO: Why is this state here?
                	//This state is here to be more efficient 
                    case "Nothing":
                        break;

                    case "GoToLiftPosition":
                        if(goToHeight(m_collectLiftPosition))
                        {
                            m_stateIndex = "GoToOpenDogsPosition";
                            piControllerLeft.reinit();
                            piControllerRight.reinit();
                        }
                        break;

                    case "GoToOpenDogsPosition":
                        if(goToHeight(m_openDogsLiftPosition))
                        {
                            m_stateIndex = "OpenDogs";
                            piControllerLeft.reinit();
                            piControllerRight.reinit();
                        }
                        break;

                    case "OpenDogs":
                        openDogs();
                        m_stateIndex = "GoToCloseDogsPosition";
                        break;

                    case "GoToCloseDogsPosition":
                        if(goToHeight(m_closeDogsLiftPosition))
                        {
                            m_stateIndex = "CloseDogs";
                            piControllerLeft.reinit();
                            piControllerRight.reinit();
                        }
                        break;

                    case "CloseDogs":
                        closeDogs();
                        m_stateIndex = "GoToWaitLiftPosition";
                        break;

                    case "GoToWaitLiftPosition":
                        if(goToHeight(m_waitLiftPosition))
                        {
                            m_stateIndex = "Nothing";

                        }
                        break;

                    default:
                        break;
                }
                break;

            case "DropTotes":
                switch(m_stateIndex)
                {
                    case "Nothing":
                        break;

                    case "GoToOpenDogsPosition":
                        if(goToHeight(m_openDogsLiftPosition))
                        {
                            m_stateIndex = "OpenDogs";
                            piControllerLeft.reinit();
                            piControllerRight.reinit();
                        }
                        break;

                    case "OpenDogs":
                        openDogs();
                        m_stateIndex = "GoToLiftPosition";
                        break;

                    case "GoToLiftPosition":
                        if(goToHeight(m_collectLiftPosition))
                        {
                            m_stateIndex = "Nothing";
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
