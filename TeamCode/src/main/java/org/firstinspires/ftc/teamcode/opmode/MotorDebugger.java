package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.drive.SampleOmniDrive;

/**
 * This is a simple teleop routine for debugging your motor configuration.
 * Pressing each of the buttons will power its respective motor.
 *
 * Button Mappings:
 *
 * Xbox/PS4 Button - Motor
 *   X / ▢         - Left Motor
 *   A / X         - Back Motor
 *   B / O         - Right Motor
 *                   / ______ \
 *     ------------.-'   _  '-..+
 *              /   _  ( Y )  _  \
 *             |  ( X )  _  ( B ) |
 *        ___  '.      ( A )     /|
 *      .'    '.    '-._____.-'  .'
 *     |       |                 |
 *      '.___.' '.               |
 *               '.             /
 *                \.          .'
 *                  \________/
 *
 */
@Config
@TeleOp
public class MotorDebugger extends LinearOpMode {
    public static double MOTOR_POWER = 1;

    @Override
    public void runOpMode() throws InterruptedException {
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        SampleOmniDrive drive = new SampleOmniDrive(hardwareMap);
        Intake intake = new Intake(hardwareMap);

        telemetry.addLine("Press play to begin the debugging opmode");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            telemetry.addLine("Press each button to turn on its respective motor");

            if (gamepad1.x) {
                drive.setMotorPowers(MOTOR_POWER, 0, 0);
                telemetry.addLine("Running Motor: Left");
            } else if (gamepad1.b) {
                drive.setMotorPowers(0, 0, MOTOR_POWER);
                telemetry.addLine("Running Motor: Right");
            } else if (gamepad1.a) {
                drive.setMotorPowers(0, MOTOR_POWER, 0);
                telemetry.addLine("Running Motor: Back");
            } else if (gamepad1.y) {
                intake.setPower(0.1);
                telemetry.addLine("Running Motor: Intake");
            } else {
                drive.setMotorPowers(0, 0, 0);
                telemetry.addLine("Running Motor: None");
            }

            telemetry.addLine();
            telemetry.addData("Left Motor Position: ", drive.leftMotor.getCurrentPosition());
            telemetry.addData("Back Motor Position: ", drive.backMotor.getCurrentPosition());
            telemetry.addData("Right Motor Position: ", drive.rightMotor.getCurrentPosition());
            telemetry.addData("Intake Motor Position: ", intake.intake.getCurrentPosition());
            telemetry.addLine();
            telemetry.addData("Left Motor Velocity: ", drive.leftMotor.getVelocity());
            telemetry.addData("Back Motor Velocity: ", drive.backMotor.getVelocity());
            telemetry.addData("Right Motor Velocity: ", drive.rightMotor.getVelocity());
            telemetry.addData("Intake Motor Velocity: ", intake.intake.getVelocity());

            telemetry.update();
        }
    }
}
