package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Jaguar;

/**
 *
 * @author Mech Cadets
 */
public class Loader
{
    private Jaguar loaderArm1, loaderArm2;
    private DoubleSolenoid loaderPiston;
    
    public Loader()
    {
        // Assign Pick Up Arm Motors
        loaderArm1 = new Jaguar(7);
        loaderArm2 = new Jaguar(8);
        
        //Assigns Solenoid to Control Pick Up Arm
        loaderPiston = new DoubleSolenoid(3,4);
    }
    
    //This sets the piston to a specified value
    public void setPiston(Value value)
    {
        loaderPiston.set(value);  
    }
    
    //This takes a number and sets the two arms to that number.
    //The first arm is negative
    public void setArms(double magnitude)
    {
        loaderArm1.set(-magnitude);
        loaderArm2.set(magnitude);
    }
    
    //This will pick up the ball
    public void pickUpBall()
    {
        this.setPiston(DoubleSolenoid.Value.kForward);
        this.setArms(.75);
    }
    
    //This will move the arms to their default position
    public void resetArms()
    {
        this.setPiston(DoubleSolenoid.Value.kReverse);
        this.setArms(0);
    }   
}