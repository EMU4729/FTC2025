package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.ShooterSubsystem;

public class Robot {
    /**
     * Code version number - make sure to update this before you deploy!
     */
    private static final int VERSION_NUMBER = 0;

    public final DriveSubsystem drive;
    public final IndexSubsystem index;
    public final IntakeSubsystem intake;

    public final ShooterSubsystem shooter;

    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        index = new IndexSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);

        shooter = new ShooterSubsystem(hardwareMap, telemetry);

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;
    }

    /**
     * This method should be called once every robot loop, regardless of opmode type.
     */
    public void periodic() {
        drive.periodic();
        index.periodic();
        intake.periodic();

        shooter.periodic();
        telemetry.addData("Code Version", VERSION_NUMBER);
    }
}
