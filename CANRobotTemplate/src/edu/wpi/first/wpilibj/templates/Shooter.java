package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Timer;

/**
 *
 * @author Mech Cadets
 */
public class Shooter
{
    Jaguar winch1, winch2;
    DoubleSolenoid hook;
    DigitalInput limitSwitch;
    Encoder winchEncoder;
    
    private int hookUpdateState = Constants.HOOK_READY;
    private double hookUpdateStartTime;
    private int encoderCounter;
    private DoubleSolenoid.Value pistonPrevState;
    
    double superLong_BeltLength;
    double long_BeltLength;
    double mediumLong_BeltLength;
    double mediumShort_BeltLength;
    double short_BeltLength;
    
    private static DoubleSolenoid.Value latched;
    private static DoubleSolenoid.Value unlatched;
    
    Timer timer;
    
    public Shooter()
    {
        winch1 = new Jaguar(5);
        winch2 = new Jaguar(6);
        limitSwitch = new DigitalInput(3);//true = open; false = close
        
        winchEncoder = new Encoder(1, 2, false, CounterBase.EncodingType.k1X);
        winchEncoder.setDistancePerPulse(1.0/750);//This pulse rate is for the competition robot
        
        hook = new DoubleSolenoid(1,2);  //Assigns Solenoid to Control Shooter Hook
        latched = DoubleSolenoid.Value.kReverse; //Assigns variable to Shooter Hook Solenoid to unhook
        unlatched = DoubleSolenoid.Value.kForward; //Assigns variable to Shooter Hook Solenoid to hook
        hookUpdateStartTime = 0.0;
        
        pistonPrevState = DoubleSolenoid.Value.kOff; //Assigns Variable to Piston to Default state
        winchEncoder.start();
        
        //init a new timer
        timer = new Timer();
    }
    
    //Sets the winches to a given value
    public void setWinches(double winchValue)
    {
        winch1.set(winchValue);
        winch2.set(winchValue);
    }
    
    //Returns the winch distance
    public double getWinchDistance()
    {
        return winchEncoder.getDistance();
    }
    
    //This opens the limit switch
    public boolean getLimitSwitch()
    {
        return limitSwitch.get();
    }
    
    //This shoots the ball
    public void shoot(RobotTemplate robot)
    {
        robot.loader.setPiston(DoubleSolenoid.Value.kForward);  
        
        //TODO stop chassis (DriveTrain object) here
        robot.driveTrain.drive(0.0,0.0,0.0); //Turn off Mecanum Drive
        
        Timer.delay(0.70);
        hook.set(unlatched);
        Timer.delay(0.5);
        this.reload(robot);
    }
    
    //This releases the winch to a specified distance
    public void releaseWinch(double distanceOfWinch)
    {   
        //Use the encoder values from testing
        
        if(winchEncoder.getDistance() < distanceOfWinch)
        {
            distanceOfWinch -= Constants.WINCH_OVERSHOOT;

            while(winchEncoder.getDistance() < distanceOfWinch)
            {
                this.setWinches(-1.0);
            }
        }
        else
        {
            distanceOfWinch+= Constants.WINCH_OVERSHOOT;

            while(winchEncoder.getDistance() > distanceOfWinch)
            {
                this.setWinches(1.0);
            }
        }
        this.setWinches(0.0);
    }
    
    public void stopWinch()
    {
        this.setWinches(0.0);
    }
    
    public void unwindWinchAuto(RobotTemplate robot)
    {
        //unwind belt to value -7.0 for autonomous
        winchEncoder.reset();  //Used to make sure encoder is set to 0 at the start
        
        //Unwind winch to belt length necessary to shot from starting position
        while(winchEncoder.getDistance() > -7.0 && robot.isAutonomous()) 
        {
            this.setWinches(1.0);   
        }
    }
    
    //autonomous shooting
    public void shootAuto(Loader loader)
    {
        loader.setPiston(DoubleSolenoid.Value.kForward);
        Timer.delay(0.5);
        hook.set(unlatched); //The mechanism will launch ball after getting to desired encoder value
        Timer.delay(0.5);
    }
    
    public void reload(RobotTemplate robot)
    {
        //make sure the latch is open
        hook.set(unlatched);
        winchEncoder.start();
        
        //while the limit switch is pressed, push the winch negative
        while(limitSwitch.get()  && (robot.isAutonomous() || robot.isOperatorControl()))
        {
            this.setWinches(-1.0);
        }
        //Set winch to 0
        this.setWinches(0.0);
        hook.set(latched);
        robot.loader.setPiston(DoubleSolenoid.Value.kReverse);  
        Timer.delay(0.8);
        winchEncoder.reset();       
    }
    
    //function to start hooking the latch
    //Not being used right now
    public void startHookLatch()
    {
        if (hookUpdateState == Constants.HOOK_READY)
        {
            hook.set(latched);
        }
    }
    
    //function to start the hook release process
    //Not being used right now
    public void startHookRelease(Loader loader)
    {
        if (hookUpdateState !=Constants.HOOK_READY)
        {
            return;
        }
        loader.setPiston(DoubleSolenoid.Value.kForward);  
        hookUpdateStartTime = timer.get();
        hookUpdateState = Constants.HOOK_WAIT_1;
    }
    
    //update the hook status
    //Not being used right now
    public void hookUpdate(RobotTemplate robot)
    {
        if (hookUpdateState == Constants.HOOK_READY)
        {
            //Do Nothing...
        }
        else if ((hookUpdateState == Constants.HOOK_WAIT_1) && ((timer.get() - hookUpdateStartTime) > 0.2))
        {
            hook.set(unlatched);
            hookUpdateState = Constants.HOOK_WAIT_2;
        }
        else if ((hookUpdateState == Constants.HOOK_WAIT_2) && ((timer.get() - hookUpdateStartTime) >1.2))
        {
            this.reload(robot);
            hookUpdateState = Constants.HOOK_READY;
        }
    }
}