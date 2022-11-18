package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class AprilTagPipeline extends OpenCvPipeline {
    private long nativeApriltagPtr;  // Set in the constructor. Very technical, so please ignore if possible :)

    // Configuration variables (can be changed)
    private final double tagSize = 0.16;  // Units are meters
    private final String tagFamily = "tag16h5";  // Chose 16h5 tags because they have the largest data bits
    private final float decimation = 3.0f;  // Increasing decimation speeds up tag finding, but might reduce accuracy/reliability
    private final int threads = 3;  // Number of threads to use to scan for the AprilTag

    // Calibration Values for Logitech C920 (don't change unless using new camera)
    private final double fx;  // The camera's horizontal focal length (in pixels)
    private final double fy;  // The camera's vertical focal length (in pixels)
    private final double cx;  // The camera's horizontal focal center (in pixels)
    private final double cy;  // The camera's vertical focal center (in pixels)

    // parkingPlace is only UNKNOWN until it sees an AprilTag. At that point, parkingPlace
    // stores that last seen AprilTag that wasn't unknown
    public ParkingPosition parkingPosition = ParkingPosition.UNKNOWN;

    // Constuctor - called when the pipeline is created
    public AprilTagPipeline() {
        // Allocate a native context object. See the corresponding deletion in the finalizer
        nativeApriltagPtr = AprilTagDetectorJNI.createApriltagDetector(tagFamily, decimation, threads);
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
    public Mat processFrame(Mat input) {
        // Convert to grayscale
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGBA2GRAY);

        // Run AprilTag detection on the grayscale image
        ArrayList<AprilTagDetection> tags = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, input, tagSize, fx, fy, cx, cy);

        // TODO: What if multiple AprilTags are found?
        for(AprilTagDetection tag : tags) {
            if(tag.id == 1) {
                parkingPosition = ParkingPosition.ZONE1;
            } else if (tag.id == 2) {
                parkingPosition = ParkingPosition.ZONE2;
            } else if (tag.id == 3) {
                parkingPosition = ParkingPosition.ZONE3;
            }
        }

        return input;
    }
}