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
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Constants;

public class Chassis {
    /* Public OpMode members. */
    protected DcMotor W1 = null;
    protected DcMotor W2 = null;
    protected DcMotor W3 = null;
    protected BNO055IMU imu = null;

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap hardwareMap) {
        // Define and initialize motors
        W1 = hardwareMap.get(DcMotor.class, "fl");
        W2 = hardwareMap.get(DcMotor.class, "fr");
        W3 = hardwareMap.get(DcMotor.class, "bl");

        // Set direction of the motors
        W1.setDirection(DcMotor.Direction.FORWARD);
        W2.setDirection(DcMotor.Direction.FORWARD);
        W3.setDirection(DcMotor.Direction.FORWARD);

        // Set zero power behaviors for each motor
        W1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        W2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        W3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set all motors to zero power
        W1.setPower(0);
        W2.setPower(0);
        W3.setPower(0);

        // Initialize the IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
    }

    public void driveRightSide(double msDuration){
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();
        while (runtime.milliseconds()<msDuration){
            double W1Power = -.33 * 1;
            double W2Power = -.33 * 1;
            double W3Power = .67 * 1;
            double motorMax = Math.max(Math.max(Math.abs(W1Power), Math.abs(W2Power)), Math.abs(W3Power));
            double proportion = Math.min(1, motorMax);
            W1.setPower(W1Power/ proportion);
            W2.setPower(W2Power/ proportion);
            W3.setPower(W3Power/ proportion);
        }
        W1.setPower(0);
        W2.setPower(0);
        W3.setPower(0);
    }

    public void run(Gamepad gamepad) {
        //Get the positions of the left stick in terms of x and y
        //Invert y because of the input from the controller
        double StickX = Math.abs(gamepad.left_stick_x) < Constants.STICK_THRESH ? 0 : gamepad.left_stick_x;
        double StickY = Math.abs(gamepad.left_stick_y) < Constants.STICK_THRESH ? 0 : -gamepad.left_stick_y;
        double rotation = gamepad.left_trigger * Constants.ROTATION_SENSITIVITY - gamepad.right_trigger * Constants.ROTATION_SENSITIVITY;
        double angle = imu.getAngularOrientation().firstAngle;
        double LockStickX = (Math.cos(angle) * StickX + Math.sin(angle) * StickY);
        double LockStickY = (-Math.sin(angle) * StickX + Math.cos(angle) * StickY);
        double StickPowerScalar = Math.sqrt(StickY * StickY + StickX * StickX);
        boolean areTriggersDown = Math.abs(rotation) > Constants.STICK_THRESH;
        boolean areSticksMoved = Math.sqrt((StickX * StickX) + (StickY * StickY)) > Constants.STICK_THRESH;
        if (areSticksMoved) {
            // create the speed vector
            double w = rotation;
            //motor power based on inverted matrix DO NOT CHANGE THE HARDCODED NUMBERS
            double W1Power = -.33 * LockStickX + .58 * LockStickY + .33 * w;
            double W2Power = -.33 * LockStickX - .58 * LockStickY + .33 * w;
            double W3Power = .67 * LockStickX + 0.33 * w;
            //keep the powers proportional and within a range of -1 to 1
            double motorMax = Math.max(Math.max(Math.abs(W1Power), Math.abs(W2Power)), Math.abs(W3Power));
            double proportion = Math.min(1, motorMax);
            W1.setPower(W1Power * StickPowerScalar / proportion);
            W2.setPower(W2Power * StickPowerScalar / proportion);
            W3.setPower(W3Power * StickPowerScalar / proportion);
        }
        else if (areTriggersDown){
            double w = rotation;
            //motor power based on inverted matrix DO NOT CHANGE THE HARDCODED NUMBERS
            double W1Power = -.33 * LockStickX + .58 * LockStickY + .33 * w;
            double W2Power = -.33 * LockStickX - .58 * LockStickY + .33 * w;
            double W3Power = .67 * LockStickX + 0.33 * w;
            //keep the powers proportional and within a range of -1 to 1
            double motorMax = Math.max(Math.max(Math.abs(W1Power), Math.abs(W2Power)), Math.abs(W3Power));
            double proportion = Math.min(1, motorMax);
            StickPowerScalar = .5;
            W1.setPower(W1Power * StickPowerScalar / proportion);
            W2.setPower(W2Power * StickPowerScalar / proportion);
            W3.setPower(W3Power * StickPowerScalar / proportion);
        }
        else {
            W1.setPower(0);
            W2.setPower(0);
            W3.setPower(0);
        }
    }
}
