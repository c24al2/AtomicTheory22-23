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

import static java.lang.Math.PI;
import static java.lang.Math.sqrt;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that rus in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Robot V1!", group = "Iterative Opmode")

public class Iterative_Opmode_V_2 extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private DcMotor W1 = null;
    private DcMotor W2 = null;
    private DcMotor W3 = null;
    private DcMotor intake = null;
    private BNO055IMU imu = null;
    private Servo clawClose = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        W1 = hardwareMap.get(DcMotor.class, "fl");
        W2 = hardwareMap.get(DcMotor.class, "fr");
        W3 = hardwareMap.get(DcMotor.class, "br");
        intake = hardwareMap.get(DcMotor.class, "intake");
        clawClose = hardwareMap.get(Servo.class, "clawClose");

        //initialize the imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
        // Reverse the motor that runs backwards when connected directly to the battery
        W1.setDirection(DcMotor.Direction.FORWARD);
        W2.setDirection(DcMotor.Direction.FORWARD);
        W3.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);
        //set zero behaviors
        W1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        W2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        W3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status", "Initialized");
        //Quality-of-life changes here
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //Get the positions of the left stick in terms of x and y
        //Invert y because of the input from the controller
            double StickX = Math.abs(gamepad1.left_stick_x) < Constants.STICK_THRESH ? 0 : gamepad1.left_stick_x;
            double StickY = Math.abs(gamepad1.left_stick_y) < Constants.STICK_THRESH ? 0 : -gamepad1.left_stick_y;
            double rotation = gamepad1.left_trigger * Constants.ROTATION_SENSITIVITY - gamepad1.right_trigger * Constants.ROTATION_SENSITIVITY;
            double gunnerStickY = Math.abs(gamepad2.left_stick_y) < Constants.STICK_THRESH ? 0 : -gamepad2.left_stick_y;
            double angle = imu.getAngularOrientation().firstAngle;
            double LockStickX = (Math.cos(angle) * StickX + Math.sin(angle) * StickY);
            double LockStickY = (-Math.sin(angle) * StickX + Math.cos(angle) * StickY);
            double StickPowerScalar = Math.sqrt(StickY * StickY + StickX * StickX);
            boolean areTriggersDown = Math.abs(rotation) > Constants.STICK_THRESH;
            boolean areSticksMoved = Math.sqrt((StickX * StickX) + (StickY * StickY)) > Constants.STICK_THRESH;
            boolean isGunnerStickMoved = Math.abs(gunnerStickY) > Constants.STICK_THRESH;
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
                telemetry.addData("Status:","Not Moving");
            }
        if (isGunnerStickMoved){
            intake.setPower(gunnerStickY);
        }
        else {
            intake.setPower(0);
        }
        boolean driversKnowEndgame = false;
        telemetry.addData("Runtime", getRuntime());
        if (runtime.milliseconds()>83000 && driversKnowEndgame == false){
            gamepad1.rumble(1000);
            gamepad2.rumble(1000);
            if (runtime.milliseconds()>85000){
                gamepad1.stopRumble();
                gamepad2.stopRumble();
            }
            driversKnowEndgame = true;
            telemetry.addData("Endgame:","Yes");
        }
        else {
            telemetry.addData("Endgame:", "No");
        }

        if (gamepad2.left_bumper){
            clawClose.setPosition(Servo.MAX_POSITION);
        }
        //close the claw
        if (gamepad2.right_bumper){
            clawClose.setPosition(Servo.MIN_POSITION);
        }
        //open the claw
    }


    @Override
    public void stop() {
    }
}
