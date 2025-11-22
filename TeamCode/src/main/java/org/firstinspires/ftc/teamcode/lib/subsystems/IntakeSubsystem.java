package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.ftccommon.configuration.EditServoListActivity;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeSubsystem extends SubsystemBase {
    private static final double INTAKE_CURRENT_THRESHOLD = 3.5;
    private final DcMotorEx spinMotor;
    private final Telemetry telemetry;
    private double maxCurrent = 0;
    private final ElapsedTime timer = new ElapsedTime();
    private double startTime = -1;

    public IntakeSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        spinMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        spinMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.telemetry = telemetry;
    }

    @Override
    public void periodic() {
        double motorCurrent = spinMotor.getCurrent(CurrentUnit.AMPS);
        maxCurrent = Math.max(maxCurrent, motorCurrent);
        telemetry.addData("Intake Motor Current (A)", motorCurrent);
        telemetry.addData("Intake Max Current (A)", maxCurrent);
        telemetry.addData("Intake Detected Ball", ballIntaken());
    }

    /**
     * Spin the intake motor for intake
     *
     * @param power Motor power between [-1, 1]
     */
    public void setPower(double power) {
        if (spinMotor.getPower() == 0 && power != 0) {
            startTime = timer.time();
        } else if (power == 0) {
            startTime = -1;
        }

        spinMotor.setPower(power);
    }

    /**
     * @return true if the current spikes, indicating a ball is being intaken
     */
    public boolean ballIntaken() {
        if (startTime == -1) return false;
        return timer.time() - startTime >= 0.2 && spinMotor.getCurrent(CurrentUnit.AMPS) > INTAKE_CURRENT_THRESHOLD;
    }
}
