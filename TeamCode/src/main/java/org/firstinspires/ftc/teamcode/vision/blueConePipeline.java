package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class blueConePipeline extends OpenCvPipeline {
    int contourIndex;
    double OrangeRectArea;
    double GreenRectArea;
    double PurpleRectArea;

    public boolean ParkingPositionPurple = false;
    public boolean ParkingPositionGreen = false;
    public boolean ParkingPositionOrange = false;

    @Override
    public Mat processFrame(Mat input) {
        Mat processedImage = new Mat();
        Imgproc.cvtColor(input,processedImage,Imgproc.COLOR_RGB2HSV);
        Imgproc.GaussianBlur(processedImage,processedImage,new Size(3,3),0);

        // Orange
        Mat orangeMat = new Mat();
        Scalar orangeLower = new Scalar(8,30,30);
        Scalar orangeUpper = new Scalar(10,255,255);
        Core.inRange(processedImage,orangeLower,orangeUpper,orangeMat);
//        Imgproc.morphologyEx(orangeMat,orangeMat,Imgproc.MORPH_OPEN,Mat.ones(new Size(3,3), CvType.CV_32F));
        ArrayList<MatOfPoint> orangeContours = new ArrayList<>();
        MatOfPoint orangeContour = null;
        Imgproc.findContours(orangeMat,orangeContours, new Mat(), Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_TC89_KCOS);
        for (int i = 0; i < orangeContours.size(); i++) {
            MatOfPoint newContour = orangeContours.get(i);
            if(Imgproc.boundingRect(newContour).width >10 && Imgproc.boundingRect(newContour).height > 10){
                if(orangeContour == null){
                    orangeContour = newContour;
                    contourIndex=i;
                }else if(Imgproc.contourArea(newContour)> Imgproc.contourArea(orangeContour)){
                    orangeContour = newContour;
                    contourIndex=i;
                }
            }
        }
        orangeMat.release();
        Rect orangerect = null;
        try {
            orangerect = Imgproc.boundingRect(orangeContours.get(contourIndex));
            Imgproc.drawContours(input,orangeContours,contourIndex,new Scalar(255, 150, 0));
            if(orangerect.area()<200){
                orangerect = null;
            }
        }catch(Exception ignored){}
        if(orangerect !=null) {
            Imgproc.rectangle(input, orangerect, new Scalar(255, 150, 0));
            OrangeRectArea = orangerect.area();
        }else {
            OrangeRectArea = 0;
        }

        // Green
        Mat greenMat = new Mat();
        Scalar greenUpper = new Scalar(60,255,255);
        Scalar greenLower = new Scalar(42,0,0);
        Core.inRange(processedImage,greenLower,greenUpper,greenMat);
//        Imgproc.morphologyEx(greenMat,greenMat,Imgproc.MORPH_OPEN,Mat.ones(new Size(3,3), CvType.CV_32F));
        ArrayList<MatOfPoint> greenContours = new ArrayList<>();
        MatOfPoint greenContour = null;
        Imgproc.findContours(greenMat,greenContours, new Mat(), Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_TC89_KCOS);
        for (int i = 0; i < greenContours.size(); i++) {
            MatOfPoint newContour = greenContours.get(i);
            if(Imgproc.boundingRect(newContour).width >10 && Imgproc.boundingRect(newContour).height > 10){
                if(greenContour == null){
                    greenContour = newContour;
                    contourIndex=i;
                }else if(Imgproc.contourArea(newContour)> Imgproc.contourArea(greenContour)){
                    greenContour = newContour;
                    contourIndex=i;
                }
            }
        }
        greenMat.release();
        Rect greenrect = null;
        try {
            greenrect = Imgproc.boundingRect(greenContours.get(contourIndex));
            Imgproc.drawContours(input,greenContours,contourIndex,new Scalar(0,255,255));
            if(greenrect.area()<200){
                greenrect = null;
            }
        }catch(Exception ignored){}
        if(greenrect !=null) {
            Imgproc.rectangle(input, greenrect, new Scalar(0, 255, 0));
            GreenRectArea = greenrect.area();
        }else {
            GreenRectArea = 0;
        }

        // Purple
        Mat purpleMat = new Mat();
        Scalar purpleLower = new Scalar(158,0,0);
        Scalar purpleUpper = new Scalar(174,255,255);
        Core.inRange(processedImage,purpleLower,purpleUpper,purpleMat);
//        Imgproc.morphologyEx(purpleMat,purpleMat,Imgproc.MORPH_OPEN,Mat.ones(new Size(3,3), CvType.CV_32F));
        ArrayList<MatOfPoint> purpleContours = new ArrayList<>();
        MatOfPoint purpleContour = null;
        Imgproc.findContours(purpleMat,purpleContours, new Mat(), Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_TC89_KCOS);
        for (int i = 0; i < purpleContours.size(); i++) {
            MatOfPoint newContour = purpleContours.get(i);
            if(Imgproc.boundingRect(newContour).width >10 && Imgproc.boundingRect(newContour).height > 10){
                if(purpleContour == null){
                    purpleContour = newContour;
                    contourIndex=i;
                }else if(Imgproc.contourArea(newContour)> Imgproc.contourArea(purpleContour)){
                    purpleContour = newContour;
                    contourIndex=i;
                }
            }
        }
        purpleMat.release();
        Rect purplerect = null;
        try {
            purplerect = Imgproc.boundingRect(purpleContours.get(contourIndex));
            Imgproc.drawContours(input,purpleContours,contourIndex,new Scalar(0,255,255));
            if(purplerect.area()<200){
                purplerect = null;
            }
        }catch(Exception ignored){}
        if(purplerect !=null) {
            Imgproc.rectangle(input, purplerect, new Scalar(255, 0, 0));
            PurpleRectArea = purplerect.area();
        }else {
            PurpleRectArea = 0;
        }

        if (PurpleRectArea > OrangeRectArea && PurpleRectArea > GreenRectArea){
            ParkingPositionPurple = true;
            ParkingPositionGreen = false;
            ParkingPositionOrange = false;
        }
        else if (GreenRectArea > OrangeRectArea && GreenRectArea > PurpleRectArea){
            ParkingPositionPurple = false;
            ParkingPositionGreen = true;
            ParkingPositionOrange = false;
        }
        else if (OrangeRectArea > PurpleRectArea && OrangeRectArea > GreenRectArea){
            ParkingPositionPurple = false;
            ParkingPositionGreen = false;
            ParkingPositionOrange = true;
        }
        else {
            ParkingPositionPurple = false;
            ParkingPositionGreen = false;
            ParkingPositionOrange = false;
        }

        processedImage.release();
//        Imgproc.cvtColor(processedImage,input,Imgproc.COLOR_HSV2RGB);


        return input;
    }
}