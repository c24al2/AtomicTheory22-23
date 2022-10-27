package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
// purple is zone 1, green is zone 2, orange is zone 3
public class blueConePipeline extends OpenCvPipeline{
    ArrayList<MatOfPoint> contours = new ArrayList<>();
    Point point;
    int contourIndex;
    Rect Orangerect;
    Rect Purplerect;
    Rect Greenrect;
    public double OrangeRectArea = Orangerect.area();
    public double GreenRectArea = Greenrect.area();
    public double PurpleRectArea = Purplerect.area();

    public enum ParkingPosition {
        ZONE1,
        ZONE2,
        ZONE3
    }
    private volatile ParkingPosition position = ParkingPosition.ZONE1;

    @Override
    public Mat processFrame(Mat input) {
        Mat purple = new Mat();
        Imgproc.cvtColor(input,purple,Imgproc.COLOR_RGB2HSV);
        Imgproc.GaussianBlur(purple,purple,new Size(5,5),0);
        Scalar upperPurple = new Scalar(55,0,95);
        Scalar lowerPurple = new Scalar(65,5,105);
        Core.inRange(purple,lowerPurple,upperPurple,purple);
        Imgproc.morphologyEx(purple,purple,Imgproc.MORPH_OPEN,Mat.ones(new Size(3,3), CvType.CV_32F));
        contours.clear();
        MatOfPoint Pcontour = null;
        Imgproc.findContours(purple,contours, new Mat(), Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_TC89_KCOS);
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint newContour = contours.get(i);
            if(Imgproc.boundingRect(newContour).width >10 && Imgproc.boundingRect(newContour).height > 10){
                if(Pcontour == null){
                    Pcontour = newContour;
                    contourIndex=i;
                }else if(Imgproc.contourArea(newContour)> Imgproc.contourArea(Pcontour)){
                    Pcontour = newContour;
                    contourIndex=i;
                }
            }
        }
        try {
            Purplerect = Imgproc.boundingRect(contours.get(contourIndex));
            Imgproc.drawContours(input,contours,contourIndex,new Scalar(0,255,255));
        }catch(Exception ignored){}
        if(Purplerect !=null) {
            Imgproc.rectangle(input, Purplerect, new Scalar(255, 0, 0));
            point = new Point(Purplerect.x + Purplerect.width / 2.0, Purplerect.y + Purplerect.height / 2.0);
        }else {
            point = new Point(0, 0);
        }
        purple.release();


        Mat green = new Mat();
        Imgproc.cvtColor(input,green,Imgproc.COLOR_RGB2HSV);
        Imgproc.GaussianBlur(green,green,new Size(5,5),0);
        Scalar upperGreen = new Scalar(16,195,221);
        Scalar lowerGreen = new Scalar(16,195,221);
        Core.inRange(green,lowerGreen,upperGreen,green);
        Imgproc.morphologyEx(green,green,Imgproc.MORPH_OPEN,Mat.ones(new Size(3,3), CvType.CV_32F));
        contours.clear();
        MatOfPoint Gcontour = null;
        Imgproc.findContours(green,contours, new Mat(), Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_TC89_KCOS);
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint newContour = contours.get(i);
            if(Imgproc.boundingRect(newContour).width >10 && Imgproc.boundingRect(newContour).height > 10){
                if(Gcontour == null){
                    Gcontour = newContour;
                    contourIndex=i;
                }else if(Imgproc.contourArea(newContour)> Imgproc.contourArea(Gcontour)){
                    Gcontour = newContour;
                    contourIndex=i;
                }
            }
        }
        try {
            Greenrect = Imgproc.boundingRect(contours.get(contourIndex));
            Imgproc.drawContours(input,contours,contourIndex,new Scalar(0,255,255));
        }catch(Exception ignored){}
        if(Greenrect !=null) {
            Imgproc.rectangle(input, Greenrect, new Scalar(255, 0, 0));
            point = new Point(Greenrect.x + Greenrect.width / 2.0, Greenrect.y + Greenrect.height / 2.0);
        }else {
            point = new Point(0, 0);
        }
        green.release();



        Mat orange = new Mat();
        Imgproc.cvtColor(input,orange,Imgproc.COLOR_RGB2HSV);
        Imgproc.GaussianBlur(orange,orange,new Size(5,5),0);
        Scalar lowerOrange = new Scalar(13,187,215);
        Scalar upperOrange = new Scalar(16,195,221);
        Core.inRange(orange,lowerOrange,upperOrange,orange);
        Imgproc.morphologyEx(orange,orange,Imgproc.MORPH_OPEN,Mat.ones(new Size(3,3), CvType.CV_32F));
        contours.clear();
        MatOfPoint Ocontour = null;

        Imgproc.findContours(orange,contours, new Mat(), Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_TC89_KCOS);
        for (int i = 0; i < contours.size(); i++) {
            MatOfPoint newContour = contours.get(i);
            if(Imgproc.boundingRect(newContour).width >10 && Imgproc.boundingRect(newContour).height > 10){
                if(Ocontour == null){
                    Ocontour = newContour;
                    contourIndex=i;
                }else if(Imgproc.contourArea(newContour)> Imgproc.contourArea(Ocontour)){
                    Ocontour = newContour;
                    contourIndex=i;
                }
            }
        }
        try {
            Orangerect = Imgproc.boundingRect(contours.get(contourIndex));
            Imgproc.drawContours(input,contours,contourIndex,new Scalar(0,255,255));
        }catch(Exception ignored){}
        if(Orangerect !=null) {
            Imgproc.rectangle(input, Orangerect, new Scalar(255, 0, 0));
            point = new Point(Orangerect.x + Orangerect.width / 2.0, Orangerect.y + Orangerect.height / 2.0);
        }else {
            point = new Point(0, 0);
        }
        if (PurpleRectArea > OrangeRectArea && PurpleRectArea > GreenRectArea){
            position = ParkingPosition.ZONE1;
        }
        else if (GreenRectArea > OrangeRectArea && GreenRectArea > PurpleRectArea){
            position = ParkingPosition.ZONE2;
        }
        else if (OrangeRectArea > PurpleRectArea && OrangeRectArea > GreenRectArea){
            position = ParkingPosition.ZONE3;
        }
        else {
            position = ParkingPosition.ZONE1;
        }
        orange.release();
        return input;
    }

    public ParkingPosition getPosition() {
        return position;
    }
}
