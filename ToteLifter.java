package org.usfirst.frc.team3238.robot;

import java.util.ArrayList;

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
    CANTalon liftMotorTalon;
    Servo dogOneServo;
    Servo dogTwoServo;
    AnalogInput liftPotentiometer;
    ArrayList<String> m_fileContents;
    PIController piControllerLeft;
    PIController piControllerRight;

    double m_collectLiftPosition;
    double m_openDogsLiftPosition;
    double m_closeDogsLiftPosition;
    double m_waitLiftPosition;
    double m_openServoPosition;
    double m_closeServoPosition;
    int m_potValue;
    int m_stateIndex;
    String m_stateMode;
    double m_threshold;
    double m_pConstant;
    double m_iConstant;
    boolean totesDropped;

    ToteLifter(int liftMotorTalonPort, int servoOnePort, int servoTwoPort,
            int potentiometerPort, PIController piContLeft,
            PIController piContRight)
    {
        //TO-DO: File handling should be done at the Robot class level
        m_fileContents = FileReader.readFile("RobotConstants.txt");
        m_pConstant = Double.parseDouble(m_fileContents.get(2));
        m_iConstant = Double.parseDouble(m_fileContents.get(2));
        m_threshold = Double.parseDouble(m_fileContents.get(2));
        m_openServoPosition = Double.parseDouble(m_fileContents.get(2));
        m_closeServoPosition = Double.parseDouble(m_fileContents.get(2));
        liftMotorTalon = new CANTalon(liftMotorTalonPort);
        dogOneServo = new Servo(servoOnePort);
        dogTwoServo = new Servo(servoTwoPort);
        liftPotentiometer = new AnalogInput(potentiometerPort);
        m_potValue = liftPotentiometer.getValue();
        piControllerLeft = piContLeft;
        piControllerRight = piContRight;

    }

    /**
     * Initializes member variables for the tote lifter
     */
    void init()
    {
        m_stateIndex = -1;
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
    //TO-DO: This method needs to use the PI controller
    boolean goToPosition(double setpoint)
    {
        boolean positionReached = false;
        // Go to desired vertical position
        m_potValue = liftPotentiometer.getValue();

        if(m_potValue > setpoint)
        {
            liftMotorTalon.set(-outputValue);
        } else if(m_potValue < setpoint)
        {
            liftMotorTalon.set(outputValue);
        }

        if(Math.abs(m_potValue - setpoint) <= m_threshold)
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
        m_stateIndex = 0;
        m_stateMode = "DropTotes";
        piControllerLeft.reinit();
        piControllerRight.reinit();
    }

    /**
     * Sets the states to start the storing phases
     */
    void storeTote()
    {
        m_stateIndex = 0;
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
            case "AddTote":
            {
                switch(m_stateIndex)
                {
                    //TO-DO: Why is this state here?
                    //TO-DO: All of these states need names instead of numbers
                    case -1:
                        break;

                    case 0:
                        if(goToPosition(m_collectLiftPosition))
                        {
                            m_stateIndex = 1;
                            piControllerLeft.reinit();
                            piControllerRight.reinit();
                        }
                        break;

                    case 1:
                        if(goToPosition(m_openDogsLiftPosition))
                        {
                            m_stateIndex = 2;
                            piControllerLeft.reinit();
                            piControllerRight.reinit();
                        }
                        break;

                    case 2:
                        openDogs();
                        m_stateIndex = 3;
                        break;

                    case 3:
                        if(goToPosition(m_closeDogsLiftPosition))
                        {
                            m_stateIndex = 4;
                            piControllerLeft.reinit();
                            piControllerRight.reinit();
                        }
                        break;

                    case 4:
                        closeDogs();
                        m_stateIndex = 5;
                        break;

                    case 5:
                        if(goToPosition(m_waitLiftPosition))
                        {
                            m_stateIndex = -1;

                        }
                        break;

                    default:
                        break;
                }
                break;
            }

            case "DropTotes":
            {
                switch(m_stateIndex)
                {
                    //TO-DO: Why is this state here?
                    //TO-DO: All of these states need names instead of numbers
                    case -1:
                        break;

                    case 0:
                        if(goToPosition(m_openDogsLiftPosition))
                        {
                            m_stateIndex = 1;
                            piControllerLeft.reinit();
                            piControllerRight.reinit();
                        }
                        break;

                    case 1:
                        openDogs();
                        m_stateIndex = 2;
                        break;

                    case 2:
                        if(goToPosition(m_collectLiftPosition))
                        {
                            m_stateIndex = -1;
                            totesDropped = true;
                        }
                        break;

                    default:
                        break;
                }
                break;
            }

            default:
                break;
        }
    }
}
