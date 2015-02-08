package org.usfirst.frc.team3238.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Servo;

/**
 * Class for handling the tote lifter and storer
 * 
 * @author James Campbell, Aaron Jenson, Anders Sjoboen
 */
public class ToteLifter
{

    CANTalon liftMotorTalon;
    Servo dogOneServo;
    Servo dogTwoServo;
    AnalogInput liftPotentiometer;

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

    ToteLifter(int liftMotorTalonPort, int servoOnePort, int servoTwoPort,
            int potentiometerPort)
    {
        liftMotorTalon = new CANTalon(liftMotorTalonPort);
        dogOneServo = new Servo(servoOnePort);
        dogTwoServo = new Servo(servoTwoPort);
        liftPotentiometer = new AnalogInput(potentiometerPort);
        m_potValue = liftPotentiometer.getValue();
        m_threshold = 0.05;

    }

    /**
     * initializes member variables for the tote lifter
     */
    void init()
    {
        m_stateIndex = -1;
        m_stateMode = "Nothing";
    }

    /**
     * open the dogs
     */
    void openDogs()
    {
        dogOneServo.set(m_openServoPosition);
        dogTwoServo.set(m_openServoPosition);
    }

    /**
     * closes the dogs
     */
    void closeDogs()
    {
        dogOneServo.set(m_closeServoPosition);
        dogTwoServo.set(m_closeServoPosition);
    }

    /**
     * This method moves the liftMotorTalon towards the position you want
     * 
     * @param wantedPosition
     * @return whether or not you have reached the desired position
     */
    boolean goToPosition(double wantedPosition)
    {
        boolean positionReached = false;
        // Go to desired vertical position
        m_potValue = liftPotentiometer.getValue();

        if(m_potValue > wantedPosition)
        {
            liftMotorTalon.set(-outputValue);
        } else if(m_potValue < wantedPosition)
        {
            liftMotorTalon.set(outputValue);
        }

        if(Math.abs(m_potValue - wantedPosition) <= m_threshold)
        {
            positionReached = true;
        }

        return positionReached;
    }

    /**
     * sets the states to start the dropping phases
     */
    void dropTotes()
    {
        m_stateIndex = 0;
        m_stateMode = "DropTotes";
    }

    /**
     * sets the states to start the storing phases
     */
    void storeTote()
    {
        m_stateIndex = 0;
        m_stateMode = "AddTote";
    }

    /**
     * controls the phases that the tote lifter goes through when it runs. This
     * must be called periodically for it to work
     */
    void idle()
    {
        switch(m_stateMode)
        {
            case "AddTote":
            {
                switch(m_stateIndex)
                {
                    case -1:
                        break;

                    case 0:
                        if(goToPosition(m_collectLiftPosition))
                        {
                            m_stateIndex = 1;
                        }
                        break;

                    case 1:
                        if(goToPosition(m_openDogsLiftPosition))
                        {
                            m_stateIndex = 2;
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
                    case -1:
                        break;

                    case 0:
                        if(goToPosition(m_openDogsLiftPosition))
                        {
                            m_stateIndex = 1;
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
