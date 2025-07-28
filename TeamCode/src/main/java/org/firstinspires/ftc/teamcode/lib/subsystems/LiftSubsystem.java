package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftSubsystem extends SubsystemBase {
    /**
     * The maximum distance extension of the lift
     */
    private static final int MAX = 1700;

    /**
     * The error gain used in the motor's proportional controller.
     */
    private static final double CONTROLLER_KP = 0.005;
    /**
     * The feedforward used to counteract gravity
     */
    private static final double CONTROLLER_FF = 0.1;

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private final Telemetry telemetry;

    private int targetPosition = 0;

    public LiftSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightLift");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset encoders upon initialization
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        int position = (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;
        int error = targetPosition - position;
        double power = error * CONTROLLER_KP + CONTROLLER_FF;
        leftMotor.setPower(power);
        rightMotor.setPower(power);

        updateTelemetry();
    }

    /**
     * Sets the target height of the lift.
     *
     * @param percent The target height percentage.
     */
    public void setTargetPosition(double percent) {
        targetPosition = (int) (percent * MAX);
    }

    private void updateTelemetry() {
        telemetry.addData("Lift Target Position", targetPosition);
        telemetry.addData("Lift Left Position", leftMotor.getCurrentPosition());
        telemetry.addData("Lift Right Position", rightMotor.getCurrentPosition());
    }
}
