package org.firstinspires.ftc.teamcode.lib.subsystems;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Optional;

public class IMULocalisationSubsystem {
    private static final Position CAMERA_POSITION = new Position(
            DistanceUnit.METER, -0.13, -0.16, 0.045, 0);
    private static final YawPitchRollAngles CAMERA_ORIENTATION = new YawPitchRollAngles(AngleUnit.DEGREES,
            180, -45, 0, 0);
    private static final IndexSubsystem.Ball[][] OBELISK_PATTERNS = {
            {IndexSubsystem.Ball.GREEN, IndexSubsystem.Ball.PURPLE, IndexSubsystem.Ball.PURPLE},
            {IndexSubsystem.Ball.GREEN, IndexSubsystem.Ball.PURPLE, IndexSubsystem.Ball.PURPLE},
            {IndexSubsystem.Ball.GREEN, IndexSubsystem.Ball.PURPLE, IndexSubsystem.Ball.PURPLE},
    };
    private final Telemetry telemetry;
    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;
    private SparkFunOTOS.Pose2D robotPose = new SparkFunOTOS.Pose2D();
    private final IMU imu;
    private int obeliskId = -1;
    private boolean initialised = false;

    public IMULocalisationSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // setup vision
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(CAMERA_POSITION, CAMERA_ORIENTATION)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new Size(1280, 800))
                .addProcessor(aprilTag)
                .build();

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )));
    }

    /**
     * @return The yaw, pitch and roll angles, as reported by the IMU.
     */
    public YawPitchRollAngles getIMUAngles() {
        return imu.getRobotYawPitchRollAngles();
    }

    /**
     * @return true if the robot's pose has been initialised from a detected AprilTag, false if not
     */
    public boolean isInitialised() {
        return initialised;
    }

    /**
     * @return The current pose of the robot.
     */
    public SparkFunOTOS.Pose2D getPose() {
        return robotPose;
    }

    /**
     * @return A 3-length array of {@link IndexSubsystem.Ball} indicating the motif for the game, or an empty optional if the obelisk has not yet been detected.
     */
    public Optional<IndexSubsystem.Ball[]> getMotif() {
        if (obeliskId == -1) return Optional.empty();
        return Optional.of(OBELISK_PATTERNS[obeliskId - 21]);
    }

    private void updateTelemetry() {
        telemetry.addData("AprilTag Positioning Complete", initialised);
        telemetry.addData("Robot Pose", robotPose.toString());
    }

    public void periodic() {
        updateTelemetry();

        List<AprilTagDetection> freshDetections = aprilTag.getFreshDetections();
        if (freshDetections == null || freshDetections.isEmpty()) return;

        SparkFunOTOS.Pose2D newPose = new SparkFunOTOS.Pose2D();

        for (AprilTagDetection detection : freshDetections) {
            // handle obelisk tags
            if (detection.metadata.id >= 21 && detection.metadata.id <= 23) {
                if (obeliskId == -1) obeliskId = detection.metadata.id;
                continue;
            }

            // handle localisation initialisation
            newPose.x += detection.robotPose.getPosition().x;
            newPose.y += detection.robotPose.getPosition().y;
            newPose.h += detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS);
            initialised = true;
        }

        newPose.x /= freshDetections.size();
        newPose.y /= freshDetections.size();
        newPose.h /= freshDetections.size();

        robotPose = newPose;
    }
}
