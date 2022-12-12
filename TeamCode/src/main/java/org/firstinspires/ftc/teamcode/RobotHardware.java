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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pid.MotorPID;
import org.firstinspires.ftc.teamcode.vision.ParkingPosition;
import org.firstinspires.ftc.teamcode.vision.AprilTagPipeline;
import java.util.ArrayList;

public class RobotHardware {
    /* Public OpMode members. */
    public DcMotorEx W1;
    public DcMotorEx W2;
    public DcMotorEx W3;
    private BNO055IMU imu;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    private AprilTagPipeline aprilTagPipeline;


    /* Constructor */

    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        // Define and initialize motors
        W1 = hardwareMap.get(DcMotorEx.class, "fr");
        W2 = hardwareMap.get(DcMotorEx.class, "fl");
        W3 = hardwareMap.get(DcMotorEx.class, "b");
        W1.setDirection(DcMotor.Direction.FORWARD);
        W2.setDirection(DcMotor.Direction.FORWARD);
        W3.setDirection(DcMotor.Direction.FORWARD);
        //set zero power behaviors for each motor
        W1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        W2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        W3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        W1.setPower(0);
        W2.setPower(0);
        W3.setPower(0);

        W1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        W2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        W3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        W3.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);


//        telemetry.addData("Camera status:", "waiting");
//        telemetry.update();
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
//        aprilTagPipeline = new AprilTagPipeline();
//        webcam.setPipeline(aprilTagPipeline);
//        webcam.setMillisecondsPermissionTimeout(2500);
//        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//            @Override
//            public void onOpened() {
//                webcam.startStreaming(Constants.CAM_WIDTH, Constants.CAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
//                //telemetry.addData("Camera status:", "initialized");
//                telemetry.update();
//            }
//
//            @Override
//            public void onError(int errorCode) {
//                // This will be called if the camera could not be opened
//            }
//        });
//        while(aprilTagPipeline.parkingPosition == ParkingPosition.UNKNOWN){
//            telemetry.addData("camera ready?", "true");
//            telemetry.addData("pipeline chosen", "April Tag Pipeline");
//            telemetry.update();
//        }
    }

//    public void lift(double Distance, double timeout, double power){
//        ElapsedTime timer = new ElapsedTime();
//        timer.reset();
//        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        int distance = (int)Math.round(Distance);
//        intake.setTargetPosition(distance);
//        intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        intake.setPower(power);
//        while (intake.isBusy() && timer.milliseconds() < timeout) {
//        }
//        stopDrive();
//    }
//
//    public void servoSetZero(){
//        clawClose.setPosition(.77);


    // Generate the table describing the trajectory to follow


    // at every step in your PID loop, you advance one time index in your traj[] array.

    // to get x,y from traj, first find the index in the list where you are (given by the time), and then x = traj[1][i], y = traj[2][i]


    public void PIDQuadraticTrajectoryController(double trajectoryA, double trajectoryB, double trajectoryC, double timeout, double x_final){
        // create three arrays (2d) that form one large array
        //take 5 seconds (time the trajectory should take) cut that into 100 intervals
        //populate the first array with a list of times in those intervals
        //generate your x-list take your final x position and subtract your initial x poition and divide by the number of steps you want to have


        PIDCoefficients positionPIDConstants = new PIDCoefficients(1, 0, 0);
        PIDCoefficients velocityPIDConstants = new PIDCoefficients(1, 0, 0);

        MotorPID W1PID = new MotorPID(W1, positionPIDConstants, velocityPIDConstants, telemetry);
        MotorPID W2PID = new MotorPID(W2, positionPIDConstants, velocityPIDConstants, telemetry);
        MotorPID W3PID = new MotorPID(W3, positionPIDConstants, velocityPIDConstants, telemetry);

        //create ArrayLists to hold the variables we want to keep track of
        ArrayList<Double> times = new ArrayList<>();
        ArrayList<Double> xcoords = new ArrayList<>();
        ArrayList<Double> ycoords = new ArrayList<>();
        ArrayList<Double> yVelocities = new ArrayList<>();
        ArrayList<Double> xVelocities = new ArrayList<>();

        double tf = timeout;
        double xf = x_final;
        // calculate the number of steps we should have based on the timeout and the runtime of the loop (usually under .15)
        double doublenumSteps = tf/0.15;
        int  numSteps = (int) Math.round (doublenumSteps);
        for (int i = 0; i <= numSteps; i++) {
            double time = tf/numSteps*i;
            times.add(time); // time generation
            double xValue = (xf)/numSteps*i;
            xcoords.add(xValue); // assume constant velocity
            double yValue = (trajectoryA*xValue*xValue + trajectoryB*xValue + trajectoryC);
            ycoords.add(yValue);
            // add telemetry so we know what trajectory the robot is trying to follow
            if (i % (numSteps/10) == 0) {
                telemetry.addData("Point", i);
                telemetry.addData("times", times.get(i));
                telemetry.addData("xcoords", xcoords.get(i));
                telemetry.addData("ycoords", ycoords.get(i));
                telemetry.addData("", "");
            }
        }


        for (int i = 0; i < numSteps; i++) {
            // take the lookahead and subtract the current, divide by time to find the magnitude of the velocity vector, named here simply as velocity
            double currentX = xcoords.get(i);
            double currentY = ycoords.get(i);
            double lookaheadX = xcoords.get(i+1);
            double lookaheadY = ycoords.get(i+1);
            double yVelocity = (lookaheadY - currentY)/(tf/numSteps);
            double xVelocity = (lookaheadX - currentX)/(tf/numSteps);
            yVelocities.add(yVelocity);
            xVelocities.add(xVelocity);
            if (i % (numSteps/10) == 0) {
                telemetry.addData("yVelocity", yVelocities.get(i));
                telemetry.addData("xVelocity", xVelocities.get(i));
            }
        }
        yVelocities.add(0.0);
        xVelocities.add(0.0);

        telemetry.update();

        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        while(timer.seconds() < 8){}

        ElapsedTime internaltimer = new ElapsedTime();
        double evaluationTime = 0;
        // track what index you're looking at
        int loopRuns = 0;
        timer.reset();
        while (timer.seconds() < timeout){
            internaltimer.reset();
            evaluationTime = evaluationTime + tf/numSteps*loopRuns;

            //xcoords and ycoords are in encoder counts
            double parametrizedLookAheadX = xcoords.get(loopRuns);
            double parametrizedLookAheadY = ycoords.get(loopRuns);

            // plug in generated x and y into the matrix for encoder positions
            double W1IdealEncoderPosition = -1.0/3 * parametrizedLookAheadX + .58 * parametrizedLookAheadY;
            double W2IdealEncoderPosition = -1.0/3 * parametrizedLookAheadX - .58 * parametrizedLookAheadY;
            double W3IdealEncoderPosition = 2.0/3 * parametrizedLookAheadX;

            telemetry.addData("W1IdealEncoderPosition", W1IdealEncoderPosition);
            telemetry.addData("W2IdealEncoderPosition", W2IdealEncoderPosition);
            telemetry.addData("W3IdealEncoderPosition", W3IdealEncoderPosition);

            // plug in velocity values for velocity of each motor given by the matrix
            double W1IdealEncoderVelocity = -1.0/3 * xVelocities.get(loopRuns) + .58 * yVelocities.get(loopRuns);
            double W2IdealEncoderVelocity = -1.0/3 * xVelocities.get(loopRuns) - .58 * yVelocities.get(loopRuns);
            double W3IdealEncoderVelocity = 2.0/3 * xVelocities.get(loopRuns);

            telemetry.addData("W1IdealEncoderVelocity", W1IdealEncoderVelocity);
            telemetry.addData("W2IdealEncoderVelocity", W2IdealEncoderVelocity);
            telemetry.addData("W3IdealEncoderVelocity", W3IdealEncoderVelocity);

            W1PID.step(evaluationTime, W1IdealEncoderPosition, W1IdealEncoderVelocity);
            W2PID.step(evaluationTime, W2IdealEncoderPosition, W2IdealEncoderVelocity);
            W3PID.step(evaluationTime, W3IdealEncoderPosition, W3IdealEncoderVelocity);

            loopRuns = loopRuns + 1;

            while(internaltimer.seconds() < 0.15){}
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
    }

    public ParkingPosition getParkingPosition(){
        return aprilTagPipeline.parkingPosition;
    }




}