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
    AnalogInput leftPot, rightPot, irSensor;
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
    String m_dropFullState;
    
    double m_threshold;
    boolean totesDropped;
    double m_homeHeight;
    
    double m_leftHeight;
    double m_rightHeight;

    double m_leftPotYIntercept;
    double m_rightPotYIntercept;
    
    long m_timeStamp;
    
    boolean m_leftFoundHome;
    boolean m_rightFoundHome;
    
    double m_infraredDistance;
    
    boolean m_manuelControl;
    double m_manuelPosition;
    
    boolean m_goAllTheWayDownOnDrop;
    
    ToteLifter(CANTalon leftLift, CANTalon rightLift, Servo leftServo, 
            Servo rightServo, AnalogInput potentiometerLeft, 
            AnalogInput potentiometerRight, double leftP, double leftI, 
            double rightP, double rightI, double accuracyThreshold, 
            double homeHeight, double waitLiftPosition, 
            double openDogsLiftPosition, double closeDogsLiftPosition,
            double homeOffset, AnalogInput infraredSensor)
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
        m_collectLiftPosition = homeHeight + homeOffset;
        m_waitLiftPosition = waitLiftPosition;
        m_openDogsLiftPosition = openDogsLiftPosition;
        m_closeDogsLiftPosition = closeDogsLiftPosition;
        m_stateMode = "AddTote";
        m_addSubstate = "GoToWaitLiftPosition";
        m_dropSubstate = "WaitForCommand";
        m_dropFullState = "WaitForCommand";
        totesDropped = false;
        irSensor = infraredSensor;
        m_manuelControl = false;
        m_manuelPosition = 0.5;
        m_goAllTheWayDownOnDrop = true;
    }

    void inputConstants(double leftP, double leftI, 
        double rightP, double rightI, double accuracyThreshold, 
        double homeHeight, double waitLiftPosition, 
        double openDogsLiftPosition, double closeDogsLiftPosition, 
        double homeOffset)
    {
    	piControllerLeft.inputConstants(leftP, leftI);
    	piControllerRight.inputConstants(rightP, rightI);
    	m_threshold = accuracyThreshold;
        m_homeHeight = homeHeight;
        m_collectLiftPosition = homeHeight + homeOffset;
        m_waitLiftPosition = waitLiftPosition;
        m_openDogsLiftPosition = openDogsLiftPosition;
        m_closeDogsLiftPosition = closeDogsLiftPosition;
    }
    
    /**
     * Initializes member variables for the tote lifter
     */
    void reinit()
    {
        m_stateMode = "AddTote";
        m_addSubstate = "GoToWaitLiftPosition";
        m_dropSubstate = "WaitForCommand";
        m_dropFullState = "WaitForCommand";
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
        m_leftPotYIntercept = m_homeHeight - (0.121 * 
                leftPot.getAverageVoltage());
        m_rightPotYIntercept = m_homeHeight - (-0.121 * 
                rightPot.getAverageVoltage());
    }
    
    void mapPots()
    {
        m_leftHeight =  0.121 * leftPot.getAverageVoltage() + 
                m_leftPotYIntercept - 0.007;
        m_rightHeight = -0.121 * rightPot.getAverageVoltage() + 
                m_rightPotYIntercept - 0.007;
    }
    
    void mapDistance()
    {
        m_infraredDistance = 0.2680762026*Math.pow(irSensor.getAverageVoltage(),
                -1.130124285);
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
        double adjust = 0.0;
        mapPots();
        adjust = (m_leftHeight-m_rightHeight)*10;
        
        if(Math.abs(setpoint - m_leftHeight) > m_threshold)
        {
            leftTalon.set(-piControllerLeft.getMotorValue
                    (setpoint, m_leftHeight)+adjust);
        }
        else
        {
            leftTalon.set(0);
            leftDone = true;
        }
        
        if(Math.abs(setpoint - m_rightHeight) > m_threshold)
        {
            rightTalon.set(piControllerRight.getMotorValue
                    (setpoint, m_rightHeight)+adjust);
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
        
        //if(leftTalon.getOutputCurrent() > 16 || rightTalon.getOutputCurrent() > 16)
        //{
            //positionReached = true;
            //zeroPots();
        //}
        return positionReached;
    }

    /**
     * Sets the states to start the dropping phases
     */
    void dropTotes()
    {
        piControllerLeft.setThrottle(1.0);
        piControllerRight.setThrottle(1.0);
        m_dropSubstate = "GoToOpenDogsPosition";
        m_stateMode = "DropTotes";
        piControllerLeft.reinit();
        piControllerRight.reinit();
        mapDistance();
        if(m_infraredDistance < 0.15)
        {
        	dropAllTotes();
        }
    }
    
    void dropFullTotes()
    {
        m_dropFullState = "GoToLiftPosition";
        m_stateMode = "DropFullTotes";
        piControllerLeft.reinit();
        piControllerRight.reinit();
    }
    
    void dropAllTotes()
    {
        piControllerLeft.setThrottle(1.0);
        piControllerRight.setThrottle(1.0);
        m_dropFullState = "GoToPointFour";
        m_stateMode = "DropFullTotes";
        piControllerLeft.reinit();
        piControllerRight.reinit();
    }

    /**
     * Sets the states to start the storing phases
     */
    void addTote()
    {
        piControllerLeft.setThrottle(1.0);
        piControllerRight.setThrottle(1.0);
        m_addSubstate = "GoToLiftPosition";
        m_stateMode = "AddTote";
        piControllerLeft.reinit();
        piControllerRight.reinit();
    }

    boolean getTotesDropped()
    {
        return totesDropped;
    }
    
    void approachHome()
    {
        m_stateMode = "GoHome";
    }
    
    void goToHomeOnly()
    {
        m_stateMode = "GoHomeOnly";
    }
    
    
    boolean goToHome()
    {
        piControllerLeft.setThrottle(1.0);
        piControllerRight.setThrottle(1.0);
        if(leftTalon.getOutputCurrent() < 14 && !m_leftFoundHome)
        {
            leftTalon.set(1);
        }
        else
        {
            leftTalon.set(0);
            m_leftFoundHome = true;
        }
        
        if(rightTalon.getOutputCurrent() < 14 && !m_rightFoundHome)
        {
            rightTalon.set(-1);
        }
        else
        {
            rightTalon.set(0);
            m_rightFoundHome = true;
        }
        
        if(m_leftFoundHome && m_rightFoundHome)
        {
            m_leftFoundHome = false;
            m_rightFoundHome = false;
            zeroPots();
            return true;
        }
        else
        {
            return false;
        }
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
    void idle()
    {
        totesDropped = false;
        if(!m_manuelControl)
        {
            mapPots();
            m_manuelPosition = m_leftHeight;
            switch(m_stateMode)
            {
            	case "WaitForCommand":
            		leftTalon.set(0);
            		rightTalon.set(0);
            	    break;
            	    
            	case "GoHomeOnly":
            	    if(goToHome())
            	    {
            	        m_stateMode = "WaitForCommand";
                        piControllerLeft.reinit();
                        piControllerRight.reinit();
            	    }
            	    break;
            	    
            	case "GoHome":
            	    if(goToHome())
            	    {
            	        m_stateMode = "WaitHome";
            	        piControllerLeft.reinit();
                        piControllerRight.reinit();
            	    }
            	    break;
            	    
            	case "WaitHome":
            	    if(goToHeight(m_waitLiftPosition))
                    {
                        m_stateMode = "WaitForCommand";
                    }
            	    break;
            	    
            	case "DropFullTotes":
            	    switch(m_dropFullState)
            	    {
            	        case"WaitForCommand":
            	            
            	            break;
            	            
            	        case "GoToLiftPosition":
                            if(goToHome())
                            {
                                m_dropFullState = "GoToFullPosition";
                                piControllerLeft.reinit();
                                piControllerRight.reinit();
                            }
                            break;
            	            
            	        case "GoToFullPosition":
            	            //System.out.println("Left Height: "+ m_leftHeight + "             Right Height: "+m_rightHeight);
                            if(goToHeight(0.30))
                            {
                                m_dropFullState = "WaitForCommand";
                                piControllerLeft.reinit();
                                piControllerRight.reinit();
                                m_timeStamp = System.currentTimeMillis();
                                leftTalon.set(0);
                                rightTalon.set(0);
                            }
                            break;
                            
            	        case "GoToPointFour":
            	            if(goToHeight(0.4))
                            {
                                m_dropFullState = "OpenDogs";
                                piControllerLeft.reinit();
                                piControllerRight.reinit();
                                m_timeStamp = System.currentTimeMillis();
                            }
            	            break;
            	            
            	        case "OpenDogs":
                            openDogs();
                            leftTalon.set(0);
                            rightTalon.set(0);
                            if(System.currentTimeMillis() - m_timeStamp > 500)
                            {
                                m_dropFullState = "GoToAlmostHome";
                                if(m_goAllTheWayDownOnDrop)
                                {
                                    m_dropFullState = "GoToLastLiftPosition";
                                }
                            }
                            break;
                            
            	        case "GoToAlmostHome":
                            if(goToHeight(0.25))
                            {
                                m_dropFullState = "WaitForCommand";
                                totesDropped = true;
                                m_stateMode = "WaitForCommand";
                            }
                            break;
                            
            	        case "GoToLastLiftPosition":
                            if(goToHome())
                            {
                                m_dropFullState = "WaitForCommand";
                                totesDropped = true;
                                m_stateMode = "WaitForCommand";
                            }
                            break;
                            
                        default:
                            break;
            	    }
            	    break;
            		
                case "AddTote":
                    switch(m_addSubstate)
                    {
                        case "WaitForCommand":
                            leftTalon.set(0);
                            rightTalon.set(0);
                            break;
    
                        case "GoToLiftPosition":
                            if(goToHome())
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
                            m_timeStamp = System.currentTimeMillis();
                            break;
    
                        case "OpenDogs":
                            openDogs();
                            leftTalon.set(0);
                            rightTalon.set(0);
                            if(System.currentTimeMillis() - m_timeStamp > 250)
                            {
                                m_addSubstate = "GoToCloseDogsPosition";
                            }
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
                            leftTalon.set(0);
                            rightTalon.set(0);
                            if(System.currentTimeMillis() - m_timeStamp > 500)
                            {
                                m_addSubstate = "GoToWaitLiftPosition";
                                m_timeStamp = System.currentTimeMillis();
                                piControllerLeft.setThrottle(0.25);
                                piControllerRight.setThrottle(0.25);
                            }
                            break;
    
                        case "GoToWaitLiftPosition":
                            if(System.currentTimeMillis()- m_timeStamp > 1000)
                            {
                                piControllerLeft.setThrottle(1.0);
                                piControllerRight.setThrottle(1.0);
                            }
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
                            if(goToHeight(m_closeDogsLiftPosition))
                            {
                                m_dropSubstate = "OpenDogs";
                                m_timeStamp = System.currentTimeMillis();
                                piControllerLeft.reinit();
                                piControllerRight.reinit();
                            }
                            break;
    
                        case "OpenDogs":
                            openDogs();
                            if(System.currentTimeMillis() - m_timeStamp > 500)
                            {
                                m_dropSubstate = "GoToAlmostHome";
                                if(m_goAllTheWayDownOnDrop)
                                {
                                    m_dropSubstate = "GoToLastLiftPosition";
                                }
                            }
                            break;
                            
                        case "GoToAlmostHome":
                            if(goToHeight(0.25))
                            {
                                m_dropSubstate = "WaitForCommand";
                                totesDropped = true;
                            }
                            break;
    
                        case "GoToLiftPosition":
                            if(goToHome())
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
        else
        {
            System.out.println("pos = "+m_manuelPosition);
            if(m_manuelPosition > 0.6)
            {
                m_manuelPosition = 0.6;
            }
            goToHeight(m_manuelPosition);
        }
    }
}
