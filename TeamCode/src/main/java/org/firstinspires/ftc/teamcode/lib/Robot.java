package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.LiftSubsystem;

public class Robot {
    private final DriveSubsystem drive;
    private final ExtensionSubsystem extension;
    private final IntakeSubsystem intake;
    private final LiftSubsystem lift;
    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;
    private final Gamepad gamepad;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        extension = new ExtensionSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        lift = new LiftSubsystem(hardwareMap, telemetry);

        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        this.gamepad = gamepad;
    }

    /**
     * This method should be called once every robot loop, regardless of opmode type.
     */
    public void periodic() {
        drive.periodic();
        intake.periodic();
        lift.periodic();
        extension.periodic();
    }

    /**
     * This method should be called only once when a teleop opmode starts.
     */
    public void teleopInit() {
        // TODO: Add any teleop init code here
    }

    /**
     * This method should be called once every robot loop in teleop opmodes only.
     */
    public void teleopPeriodic() {
        // TODO: Add any teleop loop code here (this would be a great place to put button bindings!)
        // For example, here's code to get the robot driving field-relative:
        drive.driveFieldRelative(gamepad.left_stick_x, -gamepad.left_stick_y, gamepad.right_stick_x);
    }
}
