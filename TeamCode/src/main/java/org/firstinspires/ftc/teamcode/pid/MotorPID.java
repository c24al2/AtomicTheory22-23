package org.firstinspires.ftc.teamcode.pid;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MotorPID {
    PIDConstants positionPIDConstants;
    PIDConstants velocityPIDConstants;
    DcMotorEx motor;

    double previousError = 0;

    public MotorPID(DcMotorEx motor, PIDConstants positionPIDConstants, PIDConstants velocityPIDConstants) {
        this.positionPIDConstants = positionPIDConstants;
        this.velocityPIDConstants = velocityPIDConstants;

        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        this.motor = motor;
    }

    public void step(double evaluationT, double idealEncoderPosition, double idealEncoderVelocity) {
        double currentPosition = this.motor.getCurrentPosition();
        int intTargetEncoderPosition = calculateFinalEncoderValue(this.positionPIDConstants, evaluationT, currentPosition, idealEncoderPosition);
        this.motor.setTargetPosition(intTargetEncoderPosition);

        this.motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double currentVelocity = this.motor.getVelocity();
        int intTargetEncoderVelocity = calculateFinalEncoderValue(this.velocityPIDConstants, evaluationT, currentVelocity, idealEncoderVelocity);
        this.motor.setVelocity(intTargetEncoderVelocity);
    }

    private int calculateFinalEncoderValue(PIDConstants pidConstants, double evaluationT, double currentEncoderValue, double idealEncoderValue) {
        double error = idealEncoderValue - currentEncoderValue;
        double targetEncoderValue = idealEncoderValue + pidConstants.Kp*error + pidConstants.Ki*error*evaluationT + pidConstants.Kd*(previousError - error);
        int intTargetEncoderValue = (int) Math.round(targetEncoderValue);
        previousError = error;
        return intTargetEncoderValue;
    }
}
