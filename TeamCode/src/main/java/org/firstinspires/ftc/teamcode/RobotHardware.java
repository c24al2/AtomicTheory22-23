/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.vision.blueConePipeline;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotHardware {
    /* Public OpMode members. */
    public DcMotor W1;
    public DcMotor W2;
    public DcMotor W3;
    public DcMotor intake;
    private BNO055IMU imu = null;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    private OpenCvWebcam webcam;
    private blueConePipeline BlueConePipeline;
    public int parkingPlace;


    /* Constructor */

    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        // Define and initialize motors
        W1 = hardwareMap.get(DcMotor.class, "fl");
        W2 = hardwareMap.get(DcMotor.class, "fr");
        W3 = hardwareMap.get(DcMotor.class, "bl");
        intake = hardwareMap.get(DcMotor.class, "intake");
        W1.setDirection(DcMotor.Direction.FORWARD);
        W2.setDirection(DcMotor.Direction.FORWARD);
        W3.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        //set zero power behaviors for each motor
        W1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        W2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        W3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set all motors to zero power
        W1.setPower(0);
        W2.setPower(0);
        W3.setPower(0);
        intake.setPower(0);

        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        W1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        W2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        W3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);


        telemetry.addData("Camera status:", "waiting");
        telemetry.update();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        BlueConePipeline = new blueConePipeline();
        webcam.setPipeline(BlueConePipeline);
        webcam.setMillisecondsPermissionTimeout(2500);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(Constants.CAM_WIDTH, Constants.CAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                //telemetry.addData("Camera status:", "initialized");
                telemetry.update();
            }

            @Override
            public void onError(int errorCode) {
                // This will be called if the camera could not be opened
            }
        });
        while(BlueConePipeline.ParkingPositionPurple == false &&
        BlueConePipeline.ParkingPositionGreen == false &&
        BlueConePipeline.ParkingPositionOrange == false){
            telemetry.addData("camera ready?", "true");
            telemetry.addData("pipeline chosen", "Cone");
            telemetry.update();
        }
    }

    public void driveByAngleEncoder(double angle, double distance, double power, double timeout) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double newAngle = Math.toRadians(angle + 90);
        //reset encoders
        W1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        double angleX = Math.cos(newAngle);
        double angleY = Math.sin(newAngle);
        double startAngle = imu.getAngularOrientation().firstAngle;
        double LockAngleX = (Math.cos(startAngle) * angleX + Math.sin(startAngle) * angleY);
        double LockAngleY = (-Math.sin(startAngle) * angleX + Math.cos(startAngle) * angleY);

        double W1Target = -.33 * LockAngleX + .58 * LockAngleY ;
        double W2Target = -.33 * LockAngleX - .58 * LockAngleY;
        double W3Target = .67 * LockAngleX;
        double W1Encoder = W1Target * distance;
        double W2Encoder = W2Target * distance;
        double W3Encoder = W3Target * distance;
        int intW1Encoder = (int)Math.round(W1Encoder);
        int intW2Encoder = (int)Math.round(W2Encoder);
        int intW3Encoder = (int)Math.round(W3Encoder);

        W1.setTargetPosition(intW1Encoder);
        W2.setTargetPosition(intW2Encoder);
        W3.setTargetPosition(intW3Encoder);
        W1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        W2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        W3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double proportion = Math.max(Math.max(Math.abs(W1Encoder), Math.abs(W2Encoder)), Math.max(Math.abs(W2Encoder), Math.abs(W3Encoder)));
        W1.setPower(power * W1Encoder/ proportion);
        W2.setPower(power * W2Encoder/ proportion);
        W3.setPower(power * W3Encoder/ proportion);
        //wait until the motors finish or time expires
        //noinspection StatementWithEmptyBody
        while ((W1.isBusy() || W2.isBusy() || W3.isBusy()) && timer.milliseconds() < timeout) {
        }
        stopDrive();
    }
    public void lift(double Distance, double timeout, double power){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int distance = (int)Math.round(Distance);
        intake.setTargetPosition(distance);
        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setPower(power);
        while (intake.isBusy() && timer.milliseconds() < timeout) {
        }
        stopDrive();
    }



    public void driveRightSide(double distance, double timeout, double power){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        //reset encoders
        W1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double W1Target = -.33;
        double W2Target = -.33;
        double W3Target = .67;
        double W1Encoder = W1Target * distance;
        double W2Encoder = W2Target * distance;
        double W3Encoder = W3Target * distance;
        int intW1Encoder = (int)Math.round(W1Encoder);
        int intW2Encoder = (int)Math.round(W2Encoder);
        int intW3Encoder = (int)Math.round(W3Encoder);

        W1.setTargetPosition(intW1Encoder);
        W2.setTargetPosition(intW2Encoder);
        W3.setTargetPosition(intW3Encoder);
        W1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        W2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        W3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double proportion = Math.max(Math.max(Math.abs(W1Encoder), Math.abs(W2Encoder)), Math.max(Math.abs(W2Encoder), Math.abs(W3Encoder)));
        W1.setPower(power * W1Encoder/ proportion);
        W2.setPower(power * W2Encoder/ proportion);
        W3.setPower(power * W3Encoder/ proportion);
        //wait until the motors finish or time expires
        //noinspection StatementWithEmptyBody
        while ((W1.isBusy() || W2.isBusy() || W3.isBusy()) && timer.milliseconds() < timeout) {
        }
        stopDrive();
        }

    public void rotate (double targetRotation, double timeout, double power){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        double newAngle = Math.toRadians(targetRotation + 90);
        double rotationInEncoderCounts = (newAngle / (2 * Math.PI)) * 1250;
        int intRotationInEncoderCounts = (int)Math.round(rotationInEncoderCounts);
        W1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W1.setTargetPosition(intRotationInEncoderCounts);
        W2.setTargetPosition(intRotationInEncoderCounts);
        W3.setTargetPosition(intRotationInEncoderCounts);
        W1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        W2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        W3.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        W1.setPower(power);
        W2.setPower(power);
        W3.setPower(power);
        while ((W1.isBusy() || W2.isBusy() || W3.isBusy()) && timer.milliseconds() < timeout) {
        }
        stopDrive();
    }

    public void driveFront(double distance, double timeout, double power){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        //reset encoders
        W1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double W1Target = .58;
        double W2Target = - .58;
        double W3Target = 0;
        double W1Encoder = W1Target * distance;
        double W2Encoder = W2Target * distance;
        double W3Encoder = W3Target * distance;
        int intW1Encoder = (int)Math.round(W1Encoder);
        int intW2Encoder = (int)Math.round(W2Encoder);
        int intW3Encoder = (int)Math.round(W3Encoder);

        W1.setTargetPosition(intW1Encoder);
        W2.setTargetPosition(intW2Encoder);
        W3.setTargetPosition(intW3Encoder);
        W1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        W2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        W3.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double proportion = Math.max(Math.max(Math.abs(W1Target), Math.abs(W2Target)), Math.max(Math.abs(W2Target), Math.abs(W3Target)));
        W1.setPower(power * W1Encoder/ proportion);
        W2.setPower(power * W2Encoder/ proportion);
        W3.setPower(power * W3Encoder/ proportion);
        //wait until the motors finish or time expires
        //noinspection StatementWithEmptyBody
        while ((W1.isBusy() || W2.isBusy() || W3.isBusy()) && timer.milliseconds() < timeout) {
        }
        stopDrive();
    }

    public void stopDrive(){
        W1.setPower(0);
        W2.setPower(0);
        W3.setPower(0);
        intake.setPower(0);
    }

    public int getParkingPlace(){
        if (BlueConePipeline.ParkingPositionPurple){
            return 1;
        }
        else if (BlueConePipeline.ParkingPositionGreen){
            return 2;
        }
        else if (BlueConePipeline.ParkingPositionOrange){
            return 3;
        }
        // in case of camera failure returns 0
        else {
            return 0;
        }
    }



}