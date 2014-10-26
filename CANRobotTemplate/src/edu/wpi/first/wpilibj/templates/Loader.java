/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Jaguar;

/**
 *
 * @author samega15
 */
public class Loader
{
    private static Jaguar loaderArm1, loaderArm2;
    private static DoubleSolenoid loaderPiston;
    
    public void init()
    {
        // Assign Pick Up Arm Motors
        loaderArm1 = new Jaguar(7);
        loaderArm2 = new Jaguar(8);
        
        //Assigns Solenoid to Control Pick Up Arm
        loaderPiston = new DoubleSolenoid(3,4);
    }
    
    public void setPiston(Value value)
    {
        loaderPiston.set(value);  
    }
    
    public void setArms(double magnitude)
    {
        loaderArm1.set(-magnitude);
        loaderArm2.set(magnitude);
    }
    
    public void pickUpBall()
    {
        this.setPiston(DoubleSolenoid.Value.kForward);
        this.setArms(.75);
    }
    
    public static void resetArms()
    {
        this.setPiston(DoubleSolenoid.Value.kReverse);
        this.setArms(0);
    }
    
}
