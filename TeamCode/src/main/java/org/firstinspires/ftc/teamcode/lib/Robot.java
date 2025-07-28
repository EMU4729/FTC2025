package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.lib.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.ExtensionSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.LiftSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.OuttakeSubsystem;

public class Robot {
    private final DriveSubsystem drive;
    private final ExtensionSubsystem extension;
    private final IntakeSubsystem intake;
    private final LiftSubsystem lift;
    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;
    private final OuttakeSubsystem outtake;
    private final Gamepad gamepad;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad) {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        extension = new ExtensionSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        lift = new LiftSubsystem(hardwareMap, telemetry);
        outtake = new OuttakeSubsystem(hardwareMap, telemetry);

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
        // If we ever need any initialization code, this is where to keep it.
    }

    /**
     * This method should be called once every robot loop in teleop opmodes only.
     * <p>
     * This is a good place to keep all gamepad button bindings.
     */
    public void teleopPeriodic() {
        // extension bindings
        if (gamepad.left_bumper) {
            extension.set(0);
        }
        if (gamepad.right_bumper) {
            extension.set(1);
        }

        // lift bindings
        if (gamepad.dpad_down) {
            lift.setTargetHeight(0);
        } else if (gamepad.dpad_up) {
            lift.setTargetHeight(0.8);
        } else if (gamepad.dpad_left) {
            lift.setTargetHeight(0.65);
        }

        // outtake binds
        if (gamepad.left_trigger > 0.2) {
            outtake.grab();
        } else if (gamepad.right_trigger > 0.2) {
            outtake.score();
        }

        // intake binds
        if (gamepad.a) {
            intake.applyState(IntakeSubsystem.State.HOVER);
        } else if (gamepad.b) {
            intake.applyState(IntakeSubsystem.State.GROUND);
        } else if (gamepad.x) {
            intake.applyState(IntakeSubsystem.State.CLOUD);
        }


        // drive bindings
        drive.driveRobotRelative(-gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x);
    }

    /**
     * This method should be called regularly in the teleop testing opmode.
     * <p>
     * This is a good place to keep any testing code that should not be used in competition.
     */
    public void teleopTestPeriodic() {
    }
}
