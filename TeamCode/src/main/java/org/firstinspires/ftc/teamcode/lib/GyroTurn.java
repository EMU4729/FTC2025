package org.firstinspires.ftc.teamcode.lib;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.OTOSLocalisationSubsystem;

public class GyroTurn {
    private final DriveSubsystem drive;
    private final OTOSLocalisationSubsystem localisation;
    private final Telemetry telemetry;
    private double targetYawDelta;
    private double startYaw;

    public GyroTurn(double targetYawDelta, DriveSubsystem drive, OTOSLocalisationSubsystem localisation, Telemetry telemetry) {
        this.targetYawDelta = targetYawDelta;
        this.drive = drive;
        this.localisation = localisation;
        this.telemetry = telemetry;
    }

    public void setTarget(double targetYawDelta) {
        this.targetYawDelta = targetYawDelta;
    }

    private double getYaw() {
        return localisation.getIMUAngles().getYaw(AngleUnit.RADIANS);
    }

    public void start() {
        startYaw = getYaw();
    }

    public void execute() {
        double targetYaw = startYaw + targetYawDelta;
        double error = IndexSubsystem.wrappedSignedAngleBetween(getYaw(), targetYaw);
        telemetry.addData("Gyro Turn Error", error);
        telemetry.update();
        drive.driveRobotRelative(0, 0, -1.5 * error);
    }

    public boolean isFinished() {
        double targetYaw = startYaw + targetYawDelta;
        double error = IndexSubsystem.wrappedSignedAngleBetween(getYaw(), targetYaw);
        return Math.abs(error) < 0.1;
    }

    public void end() {
        drive.stop();
    }
}
