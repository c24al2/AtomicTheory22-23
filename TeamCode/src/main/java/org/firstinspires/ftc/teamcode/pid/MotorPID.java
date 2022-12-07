package org.firstinspires.ftc.teamcode.pid;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MotorPID {
    PIDConstants positionPIDConstants;
    PIDConstants velocityPIDConstants;
    DcMotorEx motor;
    Telemetry telemetry;

    double previousError = 0;

    //set the proportional, integral and derivate constants for both position and velocity
    public MotorPID(DcMotorEx motor, PIDConstants positionPIDConstants, PIDConstants velocityPIDConstants, Telemetry telemetry) {
        this.positionPIDConstants = positionPIDConstants;
        this.velocityPIDConstants = velocityPIDConstants;

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.motor = motor;

        this.telemetry = telemetry;
    }

    //here is where one does the PID "step", where we actually assign the new target position and velocity values
    public void step(double evaluationT, double idealEncoderPosition, double idealEncoderVelocity) {
        double currentPosition = this.motor.getCurrentPosition();
        int intTargetEncoderPosition = calculateFinalEncoderValue(this.positionPIDConstants, evaluationT, currentPosition, idealEncoderPosition);
        this.motor.setTargetPosition(intTargetEncoderPosition);
        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    // for explanation on getVelocity compatibility as used here, please see:
    // https://docs.revrobotics.com/duo-control/programming/hello-robot-autonomous-robot/robot-nav-onbot-java/autonomous-navigation-onbot
        double currentVelocity = this.motor.getVelocity();
        int intTargetEncoderVelocity = calculateFinalEncoderValue(this.velocityPIDConstants, evaluationT, currentVelocity, idealEncoderVelocity);
        this.motor.setVelocity(intTargetEncoderVelocity);
    }


    private int calculateFinalEncoderValue(PIDConstants pidConstants, double evaluationT, double currentEncoderValue, double idealEncoderValue) {
        double error = idealEncoderValue - currentEncoderValue;
        double integralSum = previousError + error;
        double targetEncoderValue = idealEncoderValue + pidConstants.Kp*error + pidConstants.Ki*integralSum+ pidConstants.Kd*(previousError - error);
        int intTargetEncoderValue = (int) Math.round(targetEncoderValue);
        previousError = error;
        return intTargetEncoderValue;
    }
}
