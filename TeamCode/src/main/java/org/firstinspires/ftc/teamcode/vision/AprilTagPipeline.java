package org.firstinspires.ftc.teamcode.vision;

import static org.firstinspires.ftc.teamcode.Constants.CX;
import static org.firstinspires.ftc.teamcode.Constants.CY;
import static org.firstinspires.ftc.teamcode.Constants.DECIMATION;
import static org.firstinspires.ftc.teamcode.Constants.FX;
import static org.firstinspires.ftc.teamcode.Constants.FY;
import static org.firstinspires.ftc.teamcode.Constants.TAG_FAMILY;
import static org.firstinspires.ftc.teamcode.Constants.TAG_SIZE;
import static org.firstinspires.ftc.teamcode.Constants.THREADS;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;

public class AprilTagPipeline extends OpenCvPipeline {
    // Set in the constructor. Very technical with how it works, so please ignore if possible :)
    private long nativeApriltagPtr;

    // parkingPlace is only UNKNOWN until it sees an AprilTag. At that point, parkingPlace
    // stores that last seen AprilTag representing a parking position
    public ParkingPosition parkingPosition = ParkingPosition.UNKNOWN;

    // Constuctor - called when the pipeline is created
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