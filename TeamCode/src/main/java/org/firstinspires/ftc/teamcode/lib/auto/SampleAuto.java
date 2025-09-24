package org.firstinspires.ftc.teamcode.lib.auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.lib.subsystems.VisionSystem;
import org.firstinspires.ftc.teamcode.lib.*;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

import processors.SpikeMarkProcessor;

@Autonomous
public class SampleAuto extends LinearOpMode {

    private VisionSystem visionSystem;
    private SpikeMarkProcessor.SpikeLocation spikeLocation;

    @Override
    public void runOpMode() throws InterruptedException {

        visionSystem = new VisionSystem(hardwareMap);

        // Wait for the driver to press start
        while (!isStarted() && !isStopRequested()) {
            // During init, get the detected spike location
            spikeLocation = visionSystem.getSpikeLocation();
            telemetry.addData("Detected Spike Mark", spikeLocation);
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;


        switch (spikeLocation) {
            case LEFT:
                // Run code to score on the left spike mark
                break;
            case CENTER:
                // Run code to score on the center spike mark
                break;
            case RIGHT:
                // Run code to score on the right spike mark
                break;
        }


        // Example: Find a specific tag
        List<AprilTagDetection> detections = visionSystem.getDetections();
        for (AprilTagDetection detection : detections) {
            if (detection.id == 2) {
                // We found our target tag!
                // Use detection.ftcPose.range, .bearing, .yaw to navigate
                telemetry.addLine("Found target AprilTag!");
                telemetry.update();
            }
        }

        // At the very end, make sure to clean up
        visionSystem.close();
    }
}
