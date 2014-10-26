package edu.wpi.first.wpilibj.templates;

import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 * @author Mech Cadets
 */
public class Vision
{
    AxisCamera camera;
    private double visionDistance;
    private double VisionCounter;
    
    public void distanceCalculate() 
    {
        ColorImage image = null;
        BinaryImage thresholdRGBImage = null;
        BinaryImage thresholdHSIImage = null;
        BinaryImage bigObjectsImage= null;
        BinaryImage convexHullImage=null;
        BinaryImage filteredImage = null;
        
        try 
        {
            image = camera.getImage();
            camera.writeBrightness(50);
            image.write("originalImage.jpg");
            thresholdRGBImage = image.thresholdRGB(0, 45, 175, 255, 0, 47);
            
            thresholdRGBImage.write("thresholdRGBImage.bmp");
            thresholdHSIImage = image.thresholdHSI(0, 255, 0, 255, 200, 255);
            thresholdHSIImage.write("thresholdHSIImage.bmp");
            bigObjectsImage = thresholdHSIImage.removeSmallObjects(false, 2);
            bigObjectsImage.write("bigObjectsImage.bmp");
            convexHullImage = bigObjectsImage.convexHull(false);
            convexHullImage.write("convexHullImage.bmp");
            filteredImage = convexHullImage.particleFilter(cc);
            filteredImage.write("filteredImage.bmp");
            ParticleAnalysisReport[] reports = filteredImage.getOrderedParticleAnalysisReports();
            
            String output;

            if (reports.length > 0) 
            {
                double pixelsHeight = reports[0].boundingRectHeight;

                double angle = pixelsHeight/Constants.pixelsV*(Constants.vertAngle*Math.PI/180);
                double Vdistance = Constants.targetHeight/angle;
                visionDistance = Vdistance;

                SmartDashboard.putNumber("Distance: ", Vdistance);
            } 
            else 
            {
                
            }
        }
        catch (Exception ex) 
        {
            ex.printStackTrace();
        }
        finally
        {
            
        }
        
        try 
        {
            filteredImage.free();
            convexHullImage.free();
            bigObjectsImage.free();
            thresholdHSIImage.free();
            image.free();
        } 
        catch (NIVisionException ex) 
        {
            ex.printStackTrace();
        }
    }   
}