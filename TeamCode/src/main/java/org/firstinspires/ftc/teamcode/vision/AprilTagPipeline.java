package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

@Config
public class AprilTagPipeline extends OpenCvPipeline {
    // Stores that LAST SEEN AprilTag representing a parking position
    private ParkingPosition parkingPosition = ParkingPosition.NO_TAGS_SEEN;

    public static double TAG_SIZE = 0.034;  // Units are meters
    public static String TAG_FAMILY = "tag25h9";  // Chose 25h9 tags because they have the largest data bits but still have a relatively high hemming distance
    public static float DECIMATION = 1.0f;  // Increasing decimation speeds up tag finding, but might reduce accuracy/reliability
    public static int THREADS = 1;  // Number of threads to use to scan for the AprilTag
    public static double FX = 1.39747644e+03;  // The camera's horizontal focal length in pixels (don't change unless using new camera)
    public static double FY = 1.39191699e+03 ;  // The camera's vertical focal length in pixels (don't change unless using new camera)
    public static double CX = 9.64795834e+02;  // The camera's horizontal focal center in pixels (don't change unless using new camera)
    public static double CY = 5.09429377e+02;  // The camera's vertical focal center in pixels (don't change unless using new camera)

    // Very technical (in short, stores pointer to underlying C object), so ignore if possible :) Shouldn't have to be touched
    private long nativeApriltagPtr;

    Mat grayscaleImage = new Mat();

    // Constructor - called when the pipeline is created
    public AprilTagPipeline() {
        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(TAG_FAMILY, DECIMATION, THREADS);
    }

    // Destructor - called when the pipeline is done being used
    @Override
    protected void finalize() {
        // Might be null if createApriltagDetector() threw an exception
        if(nativeApriltagPtr != 0) {
            // Delete the native context we created in the constructor
            AprilTagDetectorJNI.releaseApriltagDetector(nativeApriltagPtr);
            nativeApriltagPtr = 0;
        } else {
            System.out.println("AprilTagPipeline.finalize(): nativeApriltagPtr was NULL");
        }
    }

    @Override
    public Mat processFrame(Mat image) {
        // Convert croppedImage to grayscale
        Imgproc.cvtColor(image, grayscaleImage, Imgproc.COLOR_RGBA2GRAY);

        // Run AprilTag detection on the grayscale, cropped image
        ArrayList<AprilTagDetection> tags = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, grayscaleImage, TAG_SIZE, FX, FY, CX, CY);

        // TODO: What if multiple AprilTags are found?
        for (AprilTagDetection tag : tags) {
            if(tag.id == 1) {
                parkingPosition = ParkingPosition.ZONE1;
            } else if (tag.id == 2) {
                parkingPosition = ParkingPosition.ZONE2;
            } else if (tag.id == 3) {
                parkingPosition = ParkingPosition.ZONE3;
            }
        }

        return grayscaleImage;
    }

    public ParkingPosition getParkingPosition() {
        return parkingPosition;
    }
}