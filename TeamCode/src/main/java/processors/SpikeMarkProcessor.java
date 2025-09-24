package processors;


import android.graphics.Canvas;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class SpikeMarkProcessor implements VisionProcessor {

    // An enum to represent the spike mark location
    public enum SpikeLocation {
        LEFT,
        CENTER,
        RIGHT
    }

    // This volatile variable is crucial for thread safety
    private volatile SpikeLocation selectedSpike;

    // Define the Regions of Interest (ROIs) for the three spike marks
    // Adjust these rectangles to fit your camera's view
    public static Rect ROI_LEFT = new Rect(0, 200, 213, 279);
    public static Rect ROI_CENTER = new Rect(214, 200, 213, 279);
    public static Rect ROI_RIGHT = new Rect(427, 200, 213, 279);

    // Color thresholds for your alliance color (e.g., RED) in HSV color space
    // TUNE THESE VALUES
    public Scalar lower = new Scalar(0, 100, 100);  // Lower bound for Red
    public Scalar upper = new Scalar(10, 255, 255); // Upper bound for Red

    private Mat hsvMat = new Mat();
    private Mat binaryMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // This is called when the processor is initialized
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Convert the input frame from RGB to HSV color space
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        // Apply the color threshold to find pixels within the specified range
        Core.inRange(hsvMat, lower, upper, binaryMat);

        // Create sub-matrices for each ROI
        Mat leftRegion = binaryMat.submat(ROI_LEFT);
        Mat centerRegion = binaryMat.submat(ROI_CENTER);
        Mat rightRegion = binaryMat.submat(ROI_RIGHT);

        // Calculate the percentage of non-zero pixels (the ones that are your color)
        double leftPercentage = Core.sumElems(leftRegion).val[0] / ROI_LEFT.area() / 255;
        double centerPercentage = Core.sumElems(centerRegion).val[0] / ROI_CENTER.area() / 255;
        double rightPercentage = Core.sumElems(rightRegion).val[0] / ROI_RIGHT.area() / 255;

        // Release the sub-matrices to avoid memory leaks
        leftRegion.release();
        centerRegion.release();
        rightRegion.release();

        // Determine which region has the highest percentage of the target color
        if (leftPercentage > centerPercentage && leftPercentage > rightPercentage) {
            selectedSpike = SpikeLocation.LEFT;
        } else if (centerPercentage > leftPercentage && centerPercentage > rightPercentage) {
            selectedSpike = SpikeLocation.CENTER;
        } else {
            selectedSpike = SpikeLocation.RIGHT;
        }

        return null; // No context object is returned
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // This method allows you to draw on the camera stream preview
        // We can draw the ROIs to help with alignment
        // This is optional but very useful for debugging
    }

    // A public method for the OpMode to call to get the detection result
    public SpikeLocation getSpikeLocation() {
        return selectedSpike;
    }
}