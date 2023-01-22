package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class ParkingPositionPipeline extends OpenCvPipeline {
    enum ParkingPosition {
        UNKNOWN,
        ZONE1,
        ZONE2,
        ZONE3,
    }

    public static final double TAG_SIZE = 0.038;  // Units are meters
    public static final String TAG_FAMILY = "tag25h9";  // Chose 25h9 tags because they have the largest data bits but still have a relatively high hemming distance
    public static final float DECIMATION = 1.0f;  // Increasing decimation speeds up tag finding, but might reduce accuracy/reliability
    public static final int THREADS = 2;  // Number of threads to use to scan for the AprilTag
    public static final double FX = 1.39747644e+03;  // The camera's horizontal focal length in pixels (don't change unless using new camera)
    public static final double FY = 1.39191699e+03 ;  // The camera's vertical focal length in pixels (don't change unless using new camera)
    public static final double CX = 9.64795834e+02;  // The camera's horizontal focal center in pixels (don't change unless using new camera)
    public static final double CY = 5.09429377e+02;  // The camera's vertical focal center in pixels (don't change unless using new camera)

    // parkingPlace is only UNKNOWN until it sees an AprilTag. At that point, parkingPlace
    // stores that last seen AprilTag representing a parking position
    public ParkingPosition parkingPosition;

    // Set in the constructor. Very technical with how it works, so please ignore if possible :)
    private long nativeApriltagPtr;

    // Constuctor - called when the pipeline is created
    public ParkingPositionPipeline() {
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
    public Mat processFrame(Mat input) {
        int x = (int) (0.2*input.width());
        int y = (int) (0.6*input.height());
        int width = (int) (0.6*input.width());
        int height = (int) (0.4*input.height());
        Rect cropArea = new Rect(x, y, width, height);

        Mat cropped = input.submat(cropArea);

        // Convert croppedImage to grayscale
        Imgproc.cvtColor(cropped, cropped, Imgproc.COLOR_RGBA2GRAY);

        // Run AprilTag detection on the grayscale, cropped image
        ArrayList<AprilTagDetection> tags = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeApriltagPtr, cropped, TAG_SIZE, FX, FY, CX, CY);

        // TODO: What if multiple AprilTags are found?
        for(AprilTagDetection tag : tags) {
            System.out.println(tag.id);
            if(tag.id == 1) {
                parkingPosition = ParkingPosition.ZONE1;
            } else if (tag.id == 2) {
                parkingPosition = ParkingPosition.ZONE2;
            } else if (tag.id == 3) {
                parkingPosition = ParkingPosition.ZONE3;
            }
        }

        cropped.release();  // Stops memory leak :)

        Imgproc.rectangle(input, cropArea, new Scalar(255, 0, 0));
        return input;
    }
}