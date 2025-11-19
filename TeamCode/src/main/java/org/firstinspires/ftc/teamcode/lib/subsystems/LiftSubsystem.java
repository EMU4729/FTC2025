package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
        rightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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


