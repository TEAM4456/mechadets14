/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package edu.wpi.first.wpilibj.templates;
import edu.wpi.first.wpilibj.camera.AxisCamera;
import edu.wpi.first.wpilibj.image.BinaryImage;
import edu.wpi.first.wpilibj.image.ColorImage;
import edu.wpi.first.wpilibj.image.CriteriaCollection;
import edu.wpi.first.wpilibj.image.NIVision;
import edu.wpi.first.wpilibj.image.NIVisionException;
import edu.wpi.first.wpilibj.image.ParticleAnalysisReport;

/**
 *
 * @author brian
 */
public class distanceCalculator {
    AxisCamera camera;
    CriteriaCollection cc;
    double pixelsV;
    double vertAngle;
    double targetHeight;
    
    public distanceCalculator(AxisCamera cam, CriteriaCollection cc, double pixelsV, double vertAngle, double targetHeight) {
        this.camera = cam;
        this.cc = cc;
        this.pixelsV = pixelsV;
        this.vertAngle = vertAngle;
        this.targetHeight = targetHeight;
    }
    
    public double getDistance() {
        ColorImage image = null;
        BinaryImage thresholdRGBImage = null;
        BinaryImage thresholdHSIImage = null;
        BinaryImage bigObjectsImage= null;
        BinaryImage convexHullImage=null;
        BinaryImage filteredImage = null;
        double visionDistance = 0.0;
        
        try {
            image = this.camera.getImage();
            this.camera.writeBrightness(50);
            image.write("originalImage.jpg");
            thresholdRGBImage = image.thresholdRGB(0, 45, 175, 255, 0, 47);
            
            thresholdRGBImage.write("thresholdRGBImage.bmp");
            thresholdHSIImage = image.thresholdHSI(0, 255, 0, 255, 200, 255);
            thresholdHSIImage.write("thresholdHSIImage.bmp");
            bigObjectsImage = thresholdHSIImage.removeSmallObjects(false, 2);
            bigObjectsImage.write("bigObjectsImage.bmp");
            convexHullImage = bigObjectsImage.convexHull(false);
            convexHullImage.write("convexHullImage.bmp");
            filteredImage = convexHullImage.particleFilter(this.cc);
            filteredImage.write("filteredImage.bmp");
            ParticleAnalysisReport[] reports = filteredImage.getOrderedParticleAnalysisReports();
            
            String output;
//            for(int i = 0; i<reports.length+1; i++) {
//                robot.println(DriverStationLCD.Line.kUser6, 1, ""+reports[i].center_mass_x);
//                System.out.println(reports[i].center_mass_x);
//            }
            if (reports.length > 0) {
                double pixelsHeight = reports[0].boundingRectHeight;
//                double centerX = reports[0].center_mass_x_normalized;
//                double centerY = reports[0].center_mass_y_normalized;
//                double targetAngle = 47*(centerX);
                double angle = pixelsHeight/this.pixelsV*(this.vertAngle*Math.PI/180);
                double Vdistance = this.targetHeight/angle;
                visionDistance = Vdistance;

                //SmartDashboard.putNumber("Distance: ", Vdistance);
            } else {
//                robot.println(DriverStationLCD.Line.kUser6, 1, "no targets. :(");
            }
        }catch (Exception ex) {
            ex.printStackTrace();
        }finally{
        }
        try {
            filteredImage.free();
            convexHullImage.free();
            bigObjectsImage.free();
            //thresholdRGBImage.free();
            thresholdHSIImage.free();
            image.free();
        } catch (NIVisionException ex) {
            ex.printStackTrace();
        }
        return visionDistance;
    }
    
}
