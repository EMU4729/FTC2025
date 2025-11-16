package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

public class LocalisationSubsystem {
    private static final double OTOS_LINEAR_SCALAR = 1.0; // todo: find exact value, need robot physically
    private static final double OTOS_ANGULAR_SCALAR = 1.0; // todo: find exact value, need robot physically
    private static final SparkFunOTOS.Pose2D OTOS_OFFSET = new SparkFunOTOS.Pose2D(0, 0, 0); // todo: find exact values, need robot physically
    private static final Position CAMERA_POSITION = new Position(DistanceUnit.METER,
            0, 0, 0, 0);

    // pitch was init'd as -90 in demo, but set to 0 here for now (-90 made no sense)
    private static final YawPitchRollAngles CAMERA_ORIENTATION = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);
    private static final IndexSubsystem.Ball[][] OBELISK_PATTERNS = {
            {IndexSubsystem.Ball.GREEN, IndexSubsystem.Ball.PURPLE, IndexSubsystem.Ball.PURPLE},
            {IndexSubsystem.Ball.GREEN, IndexSubsystem.Ball.PURPLE, IndexSubsystem.Ball.PURPLE},
            {IndexSubsystem.Ball.GREEN, IndexSubsystem.Ball.PURPLE, IndexSubsystem.Ball.PURPLE},
    };
    private final HardwareMap hardwareMap;
    private final Telemetry telemetry;
    private final SparkFunOTOS otosSensor;
    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;
    private SparkFunOTOS.Pose2D robotPose = new SparkFunOTOS.Pose2D();
    private boolean hasPositionedFromCamera = false;
    private int obeliskId = -1;

    public LocalisationSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        this.hardwareMap = hardwareMap;

        // setup vision
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(CAMERA_POSITION, CAMERA_ORIENTATION)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "webcam"))
                .addProcessor(aprilTag)
                .build();

        // setup otos
        otosSensor = hardwareMap.get(SparkFunOTOS.class, "otos");
        otosSensor.setLinearUnit(DistanceUnit.METER);
        otosSensor.setAngularUnit(AngleUnit.DEGREES);
        otosSensor.setOffset(OTOS_OFFSET);
        otosSensor.setLinearScalar(OTOS_LINEAR_SCALAR);
        otosSensor.setAngularScalar(OTOS_ANGULAR_SCALAR);
        otosSensor.calibrateImu();
        otosSensor.resetTracking();

        // starting position, will be updated by april tag positioning.
        otosSensor.setPosition(robotPose);
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
        telemetry.addData("AprilTag Positioning Complete", hasPositionedFromCamera);
        telemetry.addData("Robot X", robotPose.x);
        telemetry.addData("Robot Y", robotPose.y);
        telemetry.addData("Robot Heading (deg)", Math.toDegrees(robotPose.h));
    }

    public void periodic() {
        updateTelemetry();

        if (!hasPositionedFromCamera) {
            List<AprilTagDetection> freshDetections = aprilTag.getFreshDetections();

            if (freshDetections.isEmpty()) return;

            for (AprilTagDetection detection : freshDetections) {
                // handle obelisk tags
                if (detection.metadata.id >= 21 && detection.metadata.id <= 23) {
                    if (obeliskId == -1) obeliskId = detection.metadata.id;
                    continue;
                }

                robotPose = new SparkFunOTOS.Pose2D(
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)
                );
                hasPositionedFromCamera = true;
            }

            // only update otos position if a non-obelisk apriltag was detected
            if (hasPositionedFromCamera) otosSensor.setPosition(robotPose);
        }

        robotPose = otosSensor.getPosition();
    }
}
