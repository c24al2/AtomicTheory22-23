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

public class RobotHardware {
    /* Public OpMode members. */
    public DcMotor W1;
    public DcMotor W2;
    public DcMotor W3;
    public BNO055IMU imu = null;
    HardwareMap hardwareMap;
    Telemetry telemetry;


    /* Constructor */

    public RobotHardware(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;

        // Define and initialize motors
        W1 = hardwareMap.get(DcMotor.class, "fl");
        W2 = hardwareMap.get(DcMotor.class, "fr");
        W3 = hardwareMap.get(DcMotor.class, "br");
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
            W1.setPower(1);
            W2.setPower(1);
            W3.setPower(0);
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

//    public void driveByTime(double angle, double time, double targetRotation, double power, double timeout) {
//        //timer
//        ElapsedTime timer = new ElapsedTime();
//        timer.reset();
//        //get movement direction in rads
//        double newAngle = Math.toRadians(angle + 90);
//        //reset encoders
//        //get start angle
//        double startAngle = imu.getAngularOrientation().firstAngle;
//        //get components of original vector
//        double xComponent = Math.cos(newAngle);
//        double yComponent = Math.sin(newAngle);
//        //rotate vector 45 degrees
//        double LockStickX = (xComponent * Math.cos(startAngle)) - (yComponent * Math.sin(startAngle));
//        double LockStickY = (yComponent * Math.cos(startAngle)) + (xComponent * Math.sin(startAngle));
//        //get rotation angle in rads
//        double targetRotationRad = Math.toRadians(targetRotation);
//        //get needed rotation
//        double rotation = targetRotationRad - startAngle;
//        //get rotation between -360 and 360 degrees
//        while (rotation > Math.PI * 2) {
//            rotation -= Math.PI * 2;
//        }
//        while (rotation < -Math.PI * 2) {
//            rotation += Math.PI * 2;
//        }
//        //make sure turn direction is correct
//        if (rotation < -Math.PI) {
//            rotation = Math.PI * 2 + rotation;
//        }
//        if (rotation > Math.PI) {
//            rotation = -Math.PI * 2 + rotation;
//        }
//        //get the number of encoder counts for the target rotation
//        double rotationInEncoderCounts = (rotation / (2 * Math.PI)) * Constants.FULL_SPIN;
//        //setup target positions for each wheel
//        double w = rotation;
//        double W1Power = -.33 * LockStickX + .58 * LockStickY + .33 * w;
//        double W2Power = -.33 * LockStickX - .58 * LockStickY + .33 * w;
//        double W3Power = .67 * LockStickX + 0.33 * w;
//        //set target positions
//        //make powers less than 1
//        double proportion = Math.max(Math.max(Math.abs(W1Power), Math.abs(W2Power)), (W3Power));
//        //set the powers
//        frontLeft.setPower(power *  W1Power/ proportion);
//        backLeft.setPower(power * W2Power / proportion);
//        backRight.setPower(power * W3Power / proportion);
//        //wait until the motors finish or time expires
//        //noinspection StatementWithEmptyBody
//        //end the path
//        //stopDrive();
//    }

}