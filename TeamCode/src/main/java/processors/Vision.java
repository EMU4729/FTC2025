package processors;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Autonomous()
public class Vision extends OpMode {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;


    //public Vision(AprilTagProcessor aprilTag, VisionProcessor visionProcessor) {
    //    AprilTag = aprilTag;
//
      //   this.visionProcessor = visionProcessor;
    //}

    @Override
    public void init(){
        WebcamName webCamName = hardwareMap.get(WebcamName.class, "WebCam 1");
        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = visionPortal.easyCreateWithDefaults(
                webCamName,aprilTagProcessor
        );
    }
    @Override
    public void init_loop(){
        ArrayList<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        StringBuilder idsFound = new StringBuilder();
        for (AprilTagDetection aprilTagDetection : currentDetections){
            idsFound.append(aprilTagDetection.id);
            idsFound.append(' ');
        }
    }

    @Override
    public void start(){
        visionPortal.stopStreaming();
    }

    @Override
    public void loop(){

    }


}
