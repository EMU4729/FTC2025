package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ConveyorSubsystem extends SubsystemBase {
    private final DcMotor beltMotor;
    private final Telemetry telemetry;

    public ConveyorSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        beltMotor = hardwareMap.get(DcMotor.class, "conveyorMotor");
        beltMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        this.telemetry = telemetry;
    }

    /**
     * Run the belt motor for the conveyor
     *
     * @param power Motor power between [-1, 1]
     */
    public void setPower(double power) {
        beltMotor.setPower(power);
    }
}
