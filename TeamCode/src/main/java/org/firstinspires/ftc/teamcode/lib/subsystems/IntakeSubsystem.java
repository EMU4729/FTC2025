package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeSubsystem extends SubsystemBase {
    private static final double INTAKE_CURRENT_THRESHOLD = 1; // TODO: tune
    private final DcMotorEx spinMotor;
    private final Telemetry telemetry;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        spinMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        spinMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        telemetry.addData("Intake Motor Current (A)", spinMotor.getCurrent(CurrentUnit.AMPS));
    }

    /**
     * Spin the intake motor for intake
     *
     * @param power Motor power between [-1, 1]
     */
    public void setPower(double power) {
        spinMotor.setPower(power);
    }

    /**
     * @return true if the current spikes, indicating a ball is being intaken
     */
    public boolean ballIntaken() {
        return spinMotor.getCurrent(CurrentUnit.AMPS) > INTAKE_CURRENT_THRESHOLD;
    }
}
