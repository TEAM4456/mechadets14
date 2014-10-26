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
    static Jaguar winch1, winch2;
    static DoubleSolenoid hook;
    static DigitalInput limitSwitch;
    static Encoder winchEncoder;
    
    private static int hookUpdateState = Constants.HOOK_READY;
    private static double hookUpdateStartTime = 0.0;
    private static int encoderCounter;
    private static DoubleSolenoid.Value pistonPrevState;
    
    static double superLong_BeltLength;
    static double long_BeltLength;
    static double mediumLong_BeltLength;
    static double mediumShort_BeltLength;
    static double short_BeltLength;
    
    private static DoubleSolenoid.Value latched;
    private static DoubleSolenoid.Value unlatched;
    
    public static void init()
    {
        winch1 = new Jaguar(5);
        winch2= new Jaguar(6);
        limitSwitch = new DigitalInput(3);//true = open; false = close
        
        winchEncoder = new Encoder(1, 2, false, CounterBase.EncodingType.k1X);
        winchEncoder.setDistancePerPulse(1.0/750);//This pulse rate is for the competition robot
        
        hook = new DoubleSolenoid(1,2);  //Assigns Solenoid to Control Shooter Hook
        latched = DoubleSolenoid.Value.kReverse; //Assigns variable to Shooter Hook Solenoid to unhook
        unlatched = DoubleSolenoid.Value.kForward; //Assigns variable to Shooter Hook Solenoid to hook
        
        pistonPrevState = DoubleSolenoid.Value.kOff; //Assigns Variable to Piston to Default state
        winchEncoder.start();
    }
    
    //Sets the winches to a given value
    public static void setWinches(double winchValue)
    {
        winch1.set(winchValue);
        winch2.set(winchValue);
    }
    
    //Returns the winch distance
    public static double getWinchDistance()
    {
        return winchEncoder.getDistance();
    }
    
    //This opens the limit switch
    public static void openLimitSwitch()
    {
        limitSwitch = true;
    }
    
    //This closes the limit switch
    public static void closeLimitSwitch()
    {
        limitSwitch = false;
    }
    
    //This shoots the ball
    public static void shoot()
    {
        Loader.setPiston(DoubleSolenoid.Value.kForward);  
        chassis.mecanumDrive_Polar(0.0,0.0,0.0); //Turn off Mecanum Driv
        Timer.delay(0.70);
        hook.set(unlatched);
        Timer.delay(0.5);
        Shooter.reload();
    }
    
    //This releases the winch to a specified distance
    public static void releaseWinch(double distanceOfWinch)
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
    
    public static void stopWinch()
    {
        this.setWinches(0.0);
    }
    
    public static void unwindWinchAuto()
    {
        //unwind belt to value -7.0 for autonomous
        winchEncoder.reset();  //Used to make sure encoder is set to 0 at the start
        
        //Unwind winch to belt length necessary to shot from starting position
        while(winchEncoder.getDistance() > -7.0 && RobotTemplate.isAutonomous()) 
        {
            this.setWinches(1.0);   
        }
    }
    
    public static void shootAuto()
    {
        Loader.setPiston(DoubleSolenoid.Value.kForward);  
        RobotTemplate.timer.delay(0.5);
        hook.set(unlatched); //The mechanism will launch ball after getting to desired encoder value
        RobotTemplate.timer.delay(0.5);
    }
    
    public static void reload()
    {
        //make sure the latch is open
        hook.set(unlatched);
        winchEncoder.start();
        //while the limit switch is pressed, push the winch negative
        while(limitSwitch.get()  && (isAutonomous() || isOperatorControl()))
        {
            this.setWinches(-1.0);
        }
        //Set winch to 0
        this.setWinches(0.0);
        hook.set(latched);
        Loader.setPiston(DoubleSolenoid.Value.kReverse);  
        Timer.delay(0.8);
        winchEncoder.reset();       
    }
    
    //function to start hooking the latch
    //Not being used right now
    public static void startHookLatch()
    {
        if (hookUpdateState == Constants.HOOK_READY)
        {
            hook.set(latched);
        }
    }
    
    //function to start the hook release process
    //Not being used right now
    public static void startHookRelease()
    {
        if (hookUpdateState !=Constants.HOOK_READY)
        {
            return;
        }
        Loader.setPiston(DoubleSolenoid.Value.kForward);  
        hookUpdateStartTime = RobotTemplate.timer.get();
        hookUpdateState = Constants.HOOK_WAIT_1;
    }
    
    //update the hook status
    //Not being used right now
    public static void hookUpdate()
    {
        if (hookUpdateState == Constants.HOOK_READY)
        {
            //Do Nothing...
        }
        else if ((hookUpdateState == Constants.HOOK_WAIT_1) && ((RobotTemplate.timer.get() - hookUpdateStartTime) > 0.2))
        {
            hook.set(unlatched);
            hookUpdateState = Constants.HOOK_WAIT_2;
        }
        else if ((hookUpdateState == Constants.HOOK_WAIT_2) && ((RobotTemplate.timer.get() - hookUpdateStartTime) >1.2))
        {
            this.reload();
            hookUpdateState = Constants.HOOK_READY;
        }
    }
}