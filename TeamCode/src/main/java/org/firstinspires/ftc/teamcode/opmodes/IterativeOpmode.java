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

package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.Chassis;

@TeleOp(name = "Robot V1!", group = "Iterative Opmode")
public class IterativeOpmode extends OpMode {
    // Declare OpMode members.
    private final ElapsedTime runtime = new ElapsedTime();
    private final Chassis chassis = new Chassis();
    private DcMotor intake = null;
    private BNO055IMU imu = null;
    private Servo intakeServo = null;
    private boolean driversKnowEndgame = false;
    private boolean useEncoders = true;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        chassis.init(hardwareMap);
        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeServo = hardwareMap.get(Servo.class, "intakeServo");
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
//        intake = hardwareMap.get(DcMotor.class, "intake");
//        clawClose = hardwareMap.get(Servo.class, "clawClose");

        //initialize the imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);
        // Reverse the motor that runs backwards when connected directly to the battery
//        intake.setDirection(DcMotor.Direction.REVERSE);
//        //set zero behaviors
//        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addData("Status: ", "Initialized");
        //Quality-of-life changes here
//        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {}

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
        chassis.run(gamepad1);
        double gunnerStickY = Math.abs(gamepad2.left_stick_y) < Constants.STICK_THRESH ? 0 : -gamepad2.left_stick_y;
        boolean isGunnerStickMoved = Math.abs(gunnerStickY) > Constants.STICK_THRESH;

        telemetry.addData("Runtime: ", getRuntime());
        boolean isEndGame = runtime.milliseconds() > 83000;
        telemetry.addData("Endgame:", isEndGame);
        if (isEndGame && !driversKnowEndgame) {
            gamepad1.rumble(1000);
            gamepad2.rumble(1000);
            driversKnowEndgame = true;
        }

        // Servo positions
        if (gamepad2.right_stick_button) {
            intakeServo.setPosition(1);
        }

        if (gamepad2.y) {
           intakeServo.setPosition(0.25);
        }

        if (gamepad2.x) {
            intakeServo.setPosition(0.50);
        }

        if (gamepad2.y) {
            intakeServo.setPosition(.75);
        }
        if (gamepad1.x) {
            intakeServo.setPosition(0.0);
        }
//
//        // Lift
        if (isGunnerStickMoved) {
            useEncoders = false;
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setPower(-gamepad2.left_stick_y * 0.7);
        } else {
            intake.setPower(0);
        }
    }


    @Override
    public void stop() {
    }
}