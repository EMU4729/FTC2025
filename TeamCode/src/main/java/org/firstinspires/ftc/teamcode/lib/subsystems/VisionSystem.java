package org.firstinspires.ftc.teamcode.lib.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

import processors.SpikeMarkProcessor;

public class VisionSystem {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private SpikeMarkProcessor spikeMarkProcessor;

    // The constructor takes the hardwareMap from the OpMode
    public VisionSystem(HardwareMap hardwareMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder().build();
        spikeMarkProcessor = new SpikeMarkProcessor(); // Our custom processor

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(spikeMarkProcessor, aprilTagProcessor) // Add both processors
                .build();
    }

    // A method to get the latest AprilTag detections
    public List<AprilTagDetection> getDetections() {
        return aprilTagProcessor.getDetections();
    }

    // A method to get the detected spike mark location
    public SpikeMarkProcessor.SpikeLocation getSpikeLocation() {
        return spikeMarkProcessor.getSpikeLocation();
    }

    // A method to close the vision portal
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
