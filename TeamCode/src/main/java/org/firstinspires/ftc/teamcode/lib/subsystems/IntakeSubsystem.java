package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class IntakeSubsystem extends SubsystemBase {
    private final DcMotor spinMotor;
    private final Telemetry telemetry;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        spinMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        this.telemetry = telemetry;
    }

    /**
     * Spin the intake motor for intake
     * @param power Motor power between [-1, 1]
     */
    public void setPower(double power) {
        spinMotor.setPower(power);
    }
}
