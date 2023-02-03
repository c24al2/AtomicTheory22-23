package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.intake.Intake;
import org.firstinspires.ftc.teamcode.drive.SampleOmniDrive;

@Config
@TeleOp
public class MotorDebugger extends OpMode {
    public static double MOTOR_POWER = 1;

    public SampleOmniDrive drive;
    public Intake intake;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        drive = new SampleOmniDrive(hardwareMap);
        intake = new Intake(hardwareMap);

        telemetry.addLine("Press play to begin the motor debugging opmode");
    }

    @Override
    public void loop() {
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
    }
}
