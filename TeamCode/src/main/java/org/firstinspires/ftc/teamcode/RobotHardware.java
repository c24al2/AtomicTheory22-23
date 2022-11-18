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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.vision.ParkingPosition;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.robotcore.hardware.Servo;

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
    private AprilTagPipeline aprilTagPipeline;
    public int parkingPlace;
    private Servo clawClose;


    /* Constructor */

    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        // Define and initialize motors
        W1 = hardwareMap.get(DcMotor.class, "fl");
        W2 = hardwareMap.get(DcMotor.class, "fr");
        W3 = hardwareMap.get(DcMotor.class, "bl");
        intake = hardwareMap.get(DcMotor.class, "intake");
        clawClose = hardwareMap.get(Servo.class, "clawClose");
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
        aprilTagPipeline = new AprilTagPipeline();
        webcam.setPipeline(aprilTagPipeline);
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
        while(aprilTagPipeline.parkingPosition == ParkingPosition.UNKNOWN){
            telemetry.addData("camera ready?", "true");
            telemetry.addData("pipeline chosen", "April Tag Pipeline");
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
    public void servoSetZero(){
        clawClose.setPosition(.77);
    }

    public void setQuadraticTrajectory(double quadraticA, double quadraticB, double rangeStart, double rangeEnd, double timeout, double power){
        double CurrentXEval = rangeStart;
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < timeout) {
            double range = rangeEnd - rangeStart;
            double sensitivity = range * Constants.EPSILON/timeout;
            while (CurrentXEval < rangeEnd){
                ElapsedTime EpsilonTime = new ElapsedTime();
                EpsilonTime.reset();
                // build a quadratic out of quadraticA and quadraticB and evaluate the derivative at CurrentXEval
                while(EpsilonTime.milliseconds() < Constants.EPSILON) {
                    double CurrentDerivativeSlopeEval = 2 * quadraticA * CurrentXEval + quadraticB;
                    double CurrentAngleFromSlope = Math.atan(CurrentDerivativeSlopeEval);
                    double CurrentXofVector = Math.cos(CurrentAngleFromSlope);
                    double CurrentYofVector = Math.sin(CurrentAngleFromSlope);
                    double w = 0;
                    double W1Power = -.33 * CurrentXofVector + .58 * CurrentYofVector + .33 * w;
                    double W2Power = -.33 * CurrentXofVector - .58 * CurrentYofVector + .33 * w;
                    double W3Power = .67 * CurrentXofVector + 0.33 * w;
                    //keep the powers proportional and within a range of -1 to 1
                    double motorMax = Math.max(Math.max(Math.abs(W1Power), Math.abs(W2Power)), Math.abs(W3Power));
                    double proportion = Math.min(1, motorMax);
                    W1.setPower(W1Power * power / proportion);
                    W2.setPower(W2Power * power / proportion);
                    W3.setPower(W3Power * power / proportion);
                }
                //take the slope of the tangent line (z_1) and rewrite as fractional form --> (z_1)/1
                //z_1 is the Y value and 1 is the X value
                // plug into the motor power calculation matrix
                //set new motor powers
                CurrentXEval = sensitivity + CurrentXEval;
            }
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

        double W1Target = -.5;
        double W2Target = -.5;
        double W3Target = 1;
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

    public void driveRightSideMillimeters(double distanceInMillimeters, double timeout, double power){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        double distance = distanceInMillimeters * 1.78288404788;
        //reset encoders
        W1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double W1Target = -.5;
        double W2Target = -.5;
        double W3Target = 1;
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
        double W2Target = -.58;
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

    public void driveFrontByMillimeters(double distanceInMillimeters, double timeout, double power){
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        //reset encoders
        double distance = distanceInMillimeters * 2.058575804;

        W1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        double W1Target = 1;
        double W2Target =-1;
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

    public ParkingPosition getParkingPosition(){
        return aprilTagPipeline.parkingPosition;
    }




}