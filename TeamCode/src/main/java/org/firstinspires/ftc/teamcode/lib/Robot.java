package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.subsystems.DriveSubsystem;

public class Robot {
    /**
     * Code version number - make sure to update this before you deploy!
     */
    private static final int VERSION_NUMBER = 0;

    public final DriveSubsystem drive;
    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        drive = new DriveSubsystem(hardwareMap, telemetry);
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
        telemetry.addData("Code Version", VERSION_NUMBER);
    }

    /**
     * This method should be called only once when a teleop opmode starts.
     */
    public void teleopInit() {
        // If we ever need any initialization code, this is where to keep it.
    }

    /**
     * This method should be called once every robot loop in teleop opmodes only.
     * <p>
     * This is a good place to keep all gamepad button bindings.
     */
    public void teleopPeriodic() {
        // drive bindings
        drive.driveRobotRelative(-gamepad2.left_stick_y, gamepad2.left_stick_x, -gamepad2.right_stick_x);
    }

    /**
     * This method should be called regularly in the teleop testing opmode.
     * <p>
     * This is a good place to keep any testing code that should not be used in competition.
     */
    public void teleopTestPeriodic() {
    }
}
