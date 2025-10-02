package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftSubsystem extends SubsystemBase {
    private static final double LIFT_DISTANCE_PER_TICK = 1 / 537.7;
    private static final double LIFT_PID_P = 1;

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;

    public LiftSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        leftMotor = hardwareMap.get(DcMotorEx.class, "liftL");
        rightMotor = hardwareMap.get(DcMotorEx.class, "liftR");
//        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        leftMotor.setTargetPosition(0);
//        rightMotor.setTargetPosition(0);
//        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        leftMotor.setPositionPIDFCoefficients(LIFT_PID_P);
//        rightMotor.setPositionPIDFCoefficients(LIFT_PID_P);
    }

    /**
     * sets left right motor target position to given distance
     *
     * @param pos the distance in m
     */
    private void liftGoTo(double pos) {
        int _pos = (int) (pos / LIFT_DISTANCE_PER_TICK);
        leftMotor.setTargetPosition(_pos);
        rightMotor.setTargetPosition(_pos);
    }

    /**
     * Directly sets the power of the lift motors.
     *
     * @param power The power in the range [-1, 1]
     */
    public void setPower(double power) {
        leftMotor.setPower(power);
        rightMotor.setPower(power);
    }
}


