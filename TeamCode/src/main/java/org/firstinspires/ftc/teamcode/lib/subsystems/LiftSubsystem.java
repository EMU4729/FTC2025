package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftSubsystem extends SubsystemBase {
    /**
     * The distance the lift extends in one encoder count, in meters.
     * <p>
     * The formula to calculate this involves:
     *
     * <ul>
     * <li>The number of encoder counts per rotor revolution (excluding the gearbox)</li>
     * <li>The gear ratio of the motor's gearbox</li>
     * <li>The gear ratios of any other gears in the mechanism</li>
     * <li>The circumference of the pulley used for the actual lift</li>
     * </ul>
     * <p>
     * You got this!
     */
    private static final double ENCODER_RATIO = 1; // TODO: Change this!!!

    /**
     * The error gain used in the motor's proportional controller.
     */
    private static final double CONTROLLER_KP = 0; // TODO: Tune this!!!

    /**
     * Preset heights for the lift, in meters.
     */
    public static class Heights {
        public static final double FLOOR = 0.0;
        // TODO: Add more preset heights
    }

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private final Telemetry telemetry;

    public LiftSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightLift");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftMotor.setPositionPIDFCoefficients(CONTROLLER_KP);
        rightMotor.setPositionPIDFCoefficients(CONTROLLER_KP);

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        updateTelemetry();
    }

    /**
     * Sets the target height of the lift.
     *
     * @param heightMeters The target height, in meters. It's recommended to use the preset heights in {@link Heights}.
     */
    public void setHeight(double heightMeters) {
        int counts = (int) (heightMeters / ENCODER_RATIO);
        leftMotor.setTargetPosition(counts);
        rightMotor.setTargetPosition(counts);
    }

    /**
     * @return The target height of the lift, in meters.
     */
    public double getTargetHeight() {
        return leftMotor.getTargetPosition() * ENCODER_RATIO;
    }

    private void updateTelemetry() {
        telemetry.addData("Lift Target Height (m)", getTargetHeight());
        telemetry.addData("Lift Left Motor Current Distance", leftMotor.getCurrentPosition() * ENCODER_RATIO);
        telemetry.addData("Lift Left Motor Current Distance", leftMotor.getCurrentPosition() * ENCODER_RATIO);
    }
}
