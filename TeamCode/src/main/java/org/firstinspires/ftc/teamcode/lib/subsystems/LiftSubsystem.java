package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftSubsystem extends SubsystemBase {
    private static final int MAX = 1700;

    /**
     * The error gain used in the motor's proportional controller.
     */
    private static final double CONTROLLER_KP = 0.1; // TODO: Tune this!!!

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;
    private final Telemetry telemetry;

    public LiftSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        leftMotor = hardwareMap.get(DcMotorEx.class, "leftLift");
        rightMotor = hardwareMap.get(DcMotorEx.class, "rightLift");

        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setTargetPosition(0);
        rightMotor.setTargetPosition(0);

//        leftMotor.setVelocityPIDFCoefficients(0.1, 0, 0, 0);
//        rightMotor.setVelocityPIDFCoefficients(0.1, 0, 0, 0);
//        leftMotor.setPositionPIDFCoefficients(CONTROLLER_KP);
//        rightMotor.setPositionPIDFCoefficients(CONTROLLER_KP);

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        updateTelemetry();
    }

    /**
     * Sets the target height of the lift.
     *
     * @param percent The target height percentage.
     */
    public void setTargetPosition(double percent) {
        int target = (int) (percent * MAX);
        leftMotor.setTargetPosition(target);
        rightMotor.setTargetPosition(target);
    }

    private void updateTelemetry() {
        telemetry.addData("Lift Target Position", leftMotor.getTargetPosition());
        telemetry.addData("Lift Left Position", leftMotor.getCurrentPosition());
        telemetry.addData("Lift Right Position", rightMotor.getCurrentPosition());
    }
}
