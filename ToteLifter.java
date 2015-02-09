package org.usfirst.frc.team3238.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Servo;

/**
 * Class for controlling the lifting and storing of totes
 * 
 * @author James Campbell, Aaron Jenson, Anders Sjoboen
 */
public class ToteLifter
{
    CANTalon liftMotorTalonLeft;
    CANTalon liftMotorTalonRight;
    Servo dogOneServo;
    Servo dogTwoServo;
    AnalogInput liftPotentiometerLeft;
    AnalogInput liftPotentiometerRight;
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

    ToteLifter(int liftMotorTalonPort, int servoOnePort, int servoTwoPort,
            int potentiometerPortLeft, int potentiometerPortRight, PIController piContLeft,
            PIController piContRight, double accuracyThreshold, double openServoPosition,
            double closeServoPosition)
    {
        m_threshold = accuracyThreshold;
        m_openServoPosition = openServoPosition;
        m_closeServoPosition = closeServoPosition;
        liftMotorTalonLeft = new CANTalon(liftMotorTalonPort);
        liftMotorTalonRight = new CANTalon(liftMotorTalonPort);
        dogOneServo = new Servo(servoOnePort);
        dogTwoServo = new Servo(servoTwoPort);
        liftPotentiometerLeft = new AnalogInput(potentiometerPortLeft);
        liftPotentiometerRight = new AnalogInput(potentiometerPortRight);
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
        dogOneServo.set(m_openServoPosition);
        dogTwoServo.set(m_openServoPosition);
    }

    /**
     * Closes the dogs
     */
    void closeDogs()
    {
        dogOneServo.set(m_closeServoPosition);
        dogTwoServo.set(m_closeServoPosition);
    }

    /**
     * Moves the tote lifter towards a desired position
     * 
     * @param setpoint The desired position
     * @return Whether or not you have reached the desired position
     */
    boolean goToPosition(double setpoint)
    {
        boolean positionReached = false;
        double outputValueLeft;
        double outputValueRight;
        double sensorValueLeft;
        double sensorValueRight;
        sensorValueLeft = liftPotentiometerLeft.getValue();
        sensorValueRight = liftPotentiometerRight.getValue();
        
        outputValueLeft = piControllerLeft.getAdjustedRotationValue(setpoint, sensorValueLeft);
        outputValueRight = piControllerRight.getAdjustedRotationValue(setpoint, sensorValueRight);
        // Go to desired vertical position

        liftMotorTalonLeft.set(outputValueLeft);
        liftMotorTalonRight.set(outputValueRight);

        if(Math.abs(((sensorValueLeft+sensorValueRight)*0.5) - setpoint) <= m_threshold)
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
                        if(goToPosition(m_collectLiftPosition))
                        {
                            m_stateIndex = "GoToOpenDogsPosition";
                            piControllerLeft.reinit();
                            piControllerRight.reinit();
                        }
                        break;

                    case "GoToOpenDogsPosition":
                        if(goToPosition(m_openDogsLiftPosition))
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
                        if(goToPosition(m_closeDogsLiftPosition))
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
                        if(goToPosition(m_waitLiftPosition))
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
                        if(goToPosition(m_openDogsLiftPosition))
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
                        if(goToPosition(m_collectLiftPosition))
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
