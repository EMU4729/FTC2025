package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.lib.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.lib.subsystems.LocalisationSubsystem;

public class DriveGoTo {
    private static final double TRANSLATION_P = 0.3; // TODO: tune
    private static final double TRANSLATION_FF = 0.2; // TODO: tune
    private static final double ROTATION_P = 0.2; // TODO: tune
    private static final double ROTATION_FF = 0.1; // TODO: tune
    private static final double TRANSLATION_THRESH = 0.1; // TODO: tune
    private static final double ROTATION_THRESH = 0.1; // TODO: tune

    private final DriveSubsystem drive;
    private final LocalisationSubsystem localisation;
    private final SparkFunOTOS.Pose2D target;

    public DriveGoTo(DriveSubsystem drive, LocalisationSubsystem localisation, SparkFunOTOS.Pose2D target) {
        this.drive = drive;
        this.localisation = localisation;
        this.target = target;
    }

    private SparkFunOTOS.Pose2D getError(SparkFunOTOS.Pose2D robotPose) {
        return new SparkFunOTOS.Pose2D(
                target.x - robotPose.x,
                target.y - robotPose.y,
                target.h - robotPose.h);
    }

    public void execute() {
        SparkFunOTOS.Pose2D robotPose = localisation.getPose();
        SparkFunOTOS.Pose2D error = getError(robotPose);

        double outputX = TRANSLATION_P * error.x;
        outputX += Math.copySign(outputX, TRANSLATION_FF);
        double outputY = TRANSLATION_P * error.y;
        outputY += Math.copySign(outputY, TRANSLATION_FF);
        double outputR = ROTATION_P * error.h;
        outputR += Math.copySign(outputR, ROTATION_FF);

        drive.driveFieldRelative(outputX, outputY, outputR, robotPose.h);
    }

    public boolean atTarget() {
        SparkFunOTOS.Pose2D robotPose = localisation.getPose();
        SparkFunOTOS.Pose2D error = getError(robotPose);
        return Math.abs(error.x) < TRANSLATION_THRESH && Math.abs(error.y) < TRANSLATION_THRESH && Math.abs(error.h) < ROTATION_THRESH;
    }
}
