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
    public DcMotor slides;
    public Servo intakeTurn;
    public BNO055IMU imu = null;
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
        W3 = hardwareMap.get(DcMotor.class, "br");
        slides = hardwareMap.get(DcMotor.class, "slides");
        W1.setDirection(DcMotor.Direction.FORWARD);
        W2.setDirection(DcMotor.Direction.FORWARD);
        W3.setDirection(DcMotor.Direction.FORWARD);
        slides.setDirection(DcMotor.Direction.FORWARD);
        //set zero power behaviors for each motor
        W1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        W2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        W3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set all motors to zero power
        W1.setPower(0);
        W2.setPower(0);
        W3.setPower(0);
        slides.setPower(0);
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
                webcam.startStreaming(Constants.CAM_WIDTH, Constants.CAM_HEIGHT, OpenCvCameraRotation.SIDEWAYS_LEFT);
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
            telemetry.addData("camera ready?", "false");
            telemetry.addData("pipeline chosen", "Shipping");
            telemetry.update();
        }
    }

    public void driveRightSide(double time){
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.milliseconds()<time){
            W1.setPower(-.495);
            W2.setPower(-.495);
            W3.setPower(1);
        }
    }
    public void driveLeftSide(double time){
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.milliseconds()<time){
            W1.setPower(.495);
            W2.setPower(.495);
            W3.setPower(-1);
        }
    }
    public void driveFront(double time){
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.milliseconds()<time){
            double w = 0;
            double LockStickY = 1;
            double LockStickX = 0;
            double W1Power = -.33 * LockStickX + .58 * LockStickY + .33 * w;
            double W2Power = -.33 * LockStickX - .58 * LockStickY + .33 * w;
            double W3Power = .67 * LockStickX + 0.33 * w;
            W1.setPower(W1Power);
            W2.setPower(W2Power);
            W3.setPower(W3Power);
        }
    }
    public void driveBack(double time){
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.milliseconds()<time){
            W1.setPower(-1);
            W2.setPower(-1);
            W3.setPower(0);
        }
    }
    public void stopDrive(){
        W1.setPower(0);
        W2.setPower(0);
        W3.setPower(0);
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