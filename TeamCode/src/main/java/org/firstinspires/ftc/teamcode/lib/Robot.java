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
    /**
     * Code version number - make sure to update this before you deploy!
     */
    private static final int VERSION_NUMBER = 0;

    public final DriveSubsystem drive;
    public final ExtensionSubsystem extension;
    public final IntakeSubsystem intake;
    public final LiftSubsystem lift;
    private final Telemetry telemetry;
    private final HardwareMap hardwareMap;
    private final OuttakeSubsystem outtake;
    private final Gamepad gamepad1;
    private final Gamepad gamepad2;

    public Robot(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        drive = new DriveSubsystem(hardwareMap, telemetry);
        extension = new ExtensionSubsystem(hardwareMap, telemetry);
        intake = new IntakeSubsystem(hardwareMap, telemetry);
        lift = new LiftSubsystem(hardwareMap, telemetry);
        outtake = new OuttakeSubsystem(hardwareMap, telemetry);

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
        intake.periodic();
        lift.periodic();
        extension.periodic();
        telemetry.addData("Code Version", VERSION_NUMBER);
    }

    /**
     * This method should be called only once when a teleop opmode starts.
     */
    public void teleopInit() {
        // If we ever need any initialization code, this is where to keep it.
        outtake.init();
    }

    /**
     * This method should be called once every robot loop in teleop opmodes only.
     * <p>
     * This is a good place to keep all gamepad button bindings.
     */
    public void teleopPeriodic() {
        // extension bindings
        if (gamepad1.left_bumper) {
            extension.set(0);
        }
        if (gamepad1.right_bumper) {
            extension.set(1);
        }

        // lift bindings
        if (gamepad2.dpad_down) {
            lift.setTargetHeight(0);
        } else if (gamepad2.dpad_up) {
            lift.setTargetHeight(0.8);
        } else if (gamepad2.dpad_left) {
            lift.setTargetHeight(0.65);
        }

        // outtake binds
        if (gamepad2.left_trigger > 0.2) {
            intake.applyState(IntakeSubsystem.State.GRAB);
            outtake.grab();
        } else if (gamepad2.right_trigger > 0.2) {
            outtake.score();
        }

        // intake binds
        if (gamepad1.a) {
            intake.applyState(IntakeSubsystem.State.HOVER);
        } else if (gamepad1.b) {
            intake.applyState(IntakeSubsystem.State.GROUND);
        } else if (gamepad1.x) {
            intake.applyState(IntakeSubsystem.State.CLOUD);
        }

        if (gamepad2.left_stick_button) {
            outtake.close();
        }

        if (gamepad2.right_stick_button) {
            outtake.open();
        }

        if (gamepad2.dpad_right) {
            intake.applyState(IntakeSubsystem.State.GRAB);
        }


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
