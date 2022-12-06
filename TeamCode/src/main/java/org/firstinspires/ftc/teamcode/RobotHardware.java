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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pid.MotorPID;
import org.firstinspires.ftc.teamcode.pid.PIDConstants;
import org.firstinspires.ftc.teamcode.vision.ParkingPosition;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    /* Public OpMode members. */
    public DcMotorEx W1;
    public DcMotorEx W2;
    public DcMotorEx W3;
    public DcMotorEx intake;
    private BNO055IMU imu;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    private OpenCvWebcam webcam;
    private AprilTagPipeline aprilTagPipeline;
    private Servo clawClose;


    /* Constructor */

    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        // Define and initialize motors
        W1 = hardwareMap.get(DcMotorEx.class, "fl");
        W2 = hardwareMap.get(DcMotorEx.class, "fr");
        W3 = hardwareMap.get(DcMotorEx.class, "bl");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
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

    public void PIDQuadraticTrajectoryController(double trajectoryA, double trajectoryB, double trajectoryC, double timeout){
        PIDConstants positionPIDConstants = new PIDConstants(1, 0, 0);
        PIDConstants velocityPIDConstants = new PIDConstants(1, 0, 0);

        MotorPID W1PID = new MotorPID(W1, positionPIDConstants, velocityPIDConstants);
        MotorPID W2PID = new MotorPID(W2, positionPIDConstants, velocityPIDConstants);
        MotorPID W3PID = new MotorPID(W3, positionPIDConstants, velocityPIDConstants);

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while (timer.milliseconds() < timeout){
            double evaluationT = timer.seconds();

            double parametrizedLookAheadX = evaluationT + (trajectoryB * -1)/(2 * trajectoryA);
            double parametrizedLookAheadY = trajectoryA * evaluationT * evaluationT + (4*trajectoryA*trajectoryC - trajectoryB*trajectoryB)/(4*trajectoryA);

            double W1IdealEncoderPosition = -1.0/3 * parametrizedLookAheadX + .58 * parametrizedLookAheadY;
            double W2IdealEncoderPosition = -1.0/3 * parametrizedLookAheadX - .58 * parametrizedLookAheadY;
            double W3IdealEncoderPosition = 2.0/3 * parametrizedLookAheadX;

            double W1IdealEncoderVelocity = -1.0/3 + .58 * 2 * trajectoryA * evaluationT;
            double W2IdealEncoderVelocity = -1.0/3 - .58 * 2 * trajectoryA * evaluationT;
            double W3IdealEncoderVelocity = 2.0/3;

            W1PID.step(evaluationT, W1IdealEncoderPosition, W1IdealEncoderVelocity);
            W2PID.step(evaluationT, W2IdealEncoderPosition, W2IdealEncoderVelocity);
            W3PID.step(evaluationT, W3IdealEncoderPosition, W3IdealEncoderVelocity);
        }
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