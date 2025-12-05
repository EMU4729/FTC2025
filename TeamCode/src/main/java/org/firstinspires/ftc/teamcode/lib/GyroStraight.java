package org.firstinspires.ftc.teamcode.lib;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.IndexSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.OTOSLocalisationSubsystem;

public class GyroStraight {
    private final DriveSubsystem drive;
    private final OTOSLocalisationSubsystem localisation;
    private final Telemetry telemetry;
    private double x;
    private double y;
    private double targetYaw;

    public GyroStraight(double x, double y, DriveSubsystem drive, OTOSLocalisationSubsystem localisation, Telemetry telemetry) {
        this.x = x;
        this.y = y;
        this.drive = drive;
        this.localisation = localisation;
        this.telemetry = telemetry;
    }

    public void setDirection(double x, double y) {
        this.x = x;
        this.y = y;
    }

    private double getYaw() {
        return localisation.getIMUAngles().getYaw(AngleUnit.RADIANS);
    }

    public void start() {
        targetYaw = getYaw();
    }

    public void execute() {
        double error = IndexSubsystem.wrappedSignedAngleBetween(getYaw(), targetYaw);
        telemetry.addData("Gyro Straight Error", error);
        telemetry.update();
        drive.driveRobotRelative(x, y, -2 * error);
    }

    public void end() {
        drive.stop();
    }
}