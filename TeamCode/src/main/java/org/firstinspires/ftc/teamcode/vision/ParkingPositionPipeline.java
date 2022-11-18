package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.objdetect.QRCodeDetector;
import org.openftc.easyopencv.OpenCvPipeline;

public class ParkingPositionPipeline extends OpenCvPipeline {
    QRCodeDetector qrCodeDetector = new QRCodeDetector();
    public ParkingPosition parkingPosition;

    @Override
    public Mat processFrame(Mat input) {
        int x = (int) (0.2*input.width());
        int y = (int) (0.6*input.height());
        int width = (int) (0.6*input.width());
        int height = (int) (0.4*input.height());
        Rect cropArea = new Rect (x, y, width, height);
        Mat croppedProcessedImage = new Mat(input, cropArea);
        Imgproc.cvtColor(croppedProcessedImage,croppedProcessedImage,Imgproc.COLOR_RGB2GRAY);
        Imgproc.GaussianBlur(croppedProcessedImage,croppedProcessedImage,new Size(3,3),0);
        String zone = qrCodeDetector.detectAndDecodeCurved(croppedProcessedImage);

        if (zone.equals("1")) {
            parkingPosition = ParkingPosition.ZONE1;
        } else if (zone.equals("2")) {
            parkingPosition = ParkingPosition.ZONE2;
        } else if (zone.equals("3")) {
            parkingPosition = ParkingPosition.ZONE3;
        } else {
            parkingPosition = ParkingPosition.UNKNOWN;
        }

        if (zone.length() > 0) {

            System.out.println(zone);
            System.out.println(parkingPosition);
            System.out.println();
        }

        return croppedProcessedImage;
    }
}