/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.CounterBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Timer;

/**
 *
 * @author samega15
 */
public class Shooter
{
    Jaguar winch1, winch2;
    DoubleSolenoid hook;
    DigitalInput limitSwitch;
    Encoder winchEncoder;
    
    private int hookUpdateState = Constants.HOOK_READY;
    private double hookUpdateStartTime = 0.0;
    private int encoderCounter;
    private DoubleSolenoid.Value pistonPrevState;
    
    double superLong_BeltLength;
    double long_BeltLength;
    double mediumLong_BeltLength;
    double mediumShort_BeltLength;
    double short_BeltLength;
    
    private DoubleSolenoid.Value latched;
    private DoubleSolenoid.Value unlatched;
    
    public void init()
    {
        winch1 = new Jaguar(5);
        winch2= new Jaguar(6);
        limitSwitch = new DigitalInput(3);//true = open; false = close
        
        winchEncoder = new Encoder(1, 2, false, CounterBase.EncodingType.k1X);
        winchEncoder.setDistancePerPulse(1.0/750);//this pulse rate is for the competition robot. devin
        
        hook = new DoubleSolenoid(1,2);  //Assigns Solenoid to Control Shooter Hook
        latched = DoubleSolenoid.Value.kReverse; //Assigns variable to Shooter Hook Solenoid to unhook
        unlatched = DoubleSolenoid.Value.kForward; //Assigns variable to Shooter Hook Solenoid to hook
        
        pistonPrevState = DoubleSolenoid.Value.kOff; //Assigns Variable to Piston to Default state
        winchEncoder.start();
    }
    
    public void setWinches(double winchValue)
    {
        winch1.set(winchValue);
        winch2.set(winchValue);
    }
    
    public static double getWinchDistance()
    {
        return winchEncoder.getDistance();
    }
    
    //This opens the limit switch
    public void openLimitSwitch()
    {
        limitSwitch = true;
    }
    
    //This closes the limit switch
    public void closeLimitSwitch()
    {
        limitSwitch = false;
    }
    
    //This shoots the ball
    public void shoot()
    {
        Loader.setPiston(DoubleSolenoid.Value.kForward);  
        chassis.mecanumDrive_Polar(0.0,0.0,0.0); //Turn off Mecanum Driv
        Timer.delay(0.70);
        hook.set(unlatched);
        Timer.delay(0.5);
        Shooter.reload();
    }
    
    //This releases the winch to a specified distance
    public void releaseWinch(double distanceOfWinch)
    {   
        //use the encoder values from testing. Devin
        
        
        if(winchEncoder.getDistance() < distanceOfWinch)
        {
            distanceOfWinch -= Constants.WINCH_OVERSHOOT;

            while(winchEncoder.getDistance() < distanceOfWinch)
            {
                winch1.set(-1.0);
                winch2.set(-1.0);
            }
        }
        else
        {
            distanceOfWinch+= Constants.WINCH_OVERSHOOT;

            while(winchEncoder.getDistance() > distanceOfWinch)
            {
                winch1.set(1.0);
                winch2.set(1.0);
            }
        }
        winch1.set(0.0);
        winch2.set(0.0);
    }
    
    public void stopWinch()
    {
        winch1.set(0.0);
        winch2.set(0.0);
    }
    
    public void unwindWinchAuto()
    {
        //unwind belt to value -7.0 for autonomous
        winchEncoder.reset();  //Used to make sure encoder is set to 0 at the start
        
        //Unwind winch to belt length necessary to shot from starting position
        while(winchEncoder.getDistance() > -7.0 && RobotTemplate.isAutonomous()) 
        {
            winch1.set(1.0);
            winch2.set(1.0);     
        }
    }
    
    public void shootAuto()
    {
        Loader.setPiston(DoubleSolenoid.Value.kForward);  
        RobotTemplate.timer.delay(0.5);
        hook.set(unlatched); //The mechanism will launch ball after getting to desired encoder value
        RobotTemplate.timer.delay(0.5);
    }
    
    public void reload()
    {
        //make sure the latch is open
        hook.set(unlatched);
        winchEncoder.start();
        // *while the limit switch is pressed, push the winch negative
        while(limitSwitch.get()  && (isAutonomous() || isOperatorControl()))
        {
            winch1.set(-1.0);
            winch2.set(-1.0);
        }
        //*set winch to 0
        winch1.set(0);
        winch2.set(0);
        hook.set(latched);
        Loader.setPiston(DoubleSolenoid.Value.kReverse);  
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
    public void startHookRelease()
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
    public void hookUpdate()
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