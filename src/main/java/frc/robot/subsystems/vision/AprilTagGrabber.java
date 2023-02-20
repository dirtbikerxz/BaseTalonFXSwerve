package frc.robot.subsystems.vision;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.apriltag.AprilTagPoseEstimator.Config;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Background tag that will initialize the vision system and try
 * to recognize AprilTags using the camera.
 */
public class AprilTagGrabber implements Runnable {

    // network table name
    public static final String TABLE = "April Tags";
    
    // image size
    public static final int WIDTH = 640;
    public static final int HEIGHT = 480;

    // colors for annotations
    public static final Scalar OUTLINE = new Scalar(0, 255, 0);
    public static final Scalar CROSS = new Scalar(0, 0, 255);
    public static final Scalar TAGID = new Scalar(0, 0, 255);

    // camera params
    public static final Config LIFECAM = new Config(
        0.1524, 
        699.3778103158814, 
        677.7161226393544, 
        345.6059345433618, 
        207.12741326228522);

    private final AprilTagDetector detector;
    private final AprilTagPoseEstimator estimator;
    private final CvSink input;
    private final CvSource output;
    private final Mat colorMat;
    private final Mat grayMat;
    private final NetworkTable table;
    private final AtomicReference<AprilTag> current;
    private final AtomicInteger frames;

    public AprilTagGrabber() {

        // initialize detector
        detector = new AprilTagDetector();
        detector.addFamily("tag16h5", 0);
        estimator = new AprilTagPoseEstimator(LIFECAM);

        // initialize the input/output streams
        CameraServer.startAutomaticCapture().setResolution(WIDTH, HEIGHT);
        input = CameraServer.getVideo();
        output = CameraServer.putVideo("Detected", 640, 480);

        // allocate memory buffers for frames
        colorMat = new Mat();
        grayMat = new Mat();

        // create network table & handoff
        table = NetworkTableInstance.getDefault().getTable("apriltags");
        current = new AtomicReference<>();
        frames = new AtomicInteger();
    }

    public int getFrameCount() {
        return frames.get();
    }

    public AprilTag getCurrentTag() {
        return current.get();
    }

    @Override
    public void run() {

        while (true) {

            if (!grabNextImage()) {
                output.notifyError(input.getError());
                continue;
            }

            AprilTagDetection [] detections = detector.detect(grayMat);
            for (AprilTagDetection d : detections) {
                outlineTag(d);
                crossTag(d);
                identifyTag(d);
            }

            postAndSelectTag(detections);
            frames.incrementAndGet();
        }
    }
    
    // grab the next image and grayscale it; returns true if it was successful.
    // sends error output if there was a problem.
    private boolean grabNextImage() {
        if (input.grabFrame(colorMat) == 0) {
            return false;
        }
        Imgproc.cvtColor(colorMat, grayMat, Imgproc.COLOR_RGB2GRAY);
        return true;
    }

    private void outlineTag(AprilTagDetection detection) {
        for (int i=0; i<3; i++) {
            int j = (i + 1) % 4;
            Point p1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
            Point p2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
            Imgproc.line(colorMat, p1, p2, OUTLINE, 2);
        }
    }

    private void crossTag(AprilTagDetection detection) {
        double cx = detection.getCenterX();
        double cy = detection.getCenterY();
        double ll = 10;
        Imgproc.line(colorMat, new Point(cx - ll, cy), new Point(cx + ll, cy), CROSS, 2);
        Imgproc.line(colorMat, new Point(cx, cy - ll), new Point(cx, cy + ll), CROSS, 2);
    }

    private void identifyTag(AprilTagDetection detection) {
        double cx = detection.getCenterX();
        double cy = detection.getCenterY();
        double ll = 10;
        Point p = new Point(cx + ll, cy);
        Imgproc.putText(
              colorMat, Integer.toString(detection.getId()), p,
              Imgproc.FONT_HERSHEY_SIMPLEX, 1, CROSS, 3);
    }

    private void postAndSelectTag(AprilTagDetection [] detections) {

        Transform3d bestPose = null;
        AprilTagDetection bestDetection = null;
        double bestDistance = Double.MAX_VALUE;

        for (AprilTagDetection d : detections) {

            Transform3d thisPose = estimator.estimate(d);
            table
                .getEntry("tag-"+d.getId())
                .setDoubleArray(new double[]{ thisPose.getX(), thisPose.getY(), thisPose.getZ() }); 

            double thisDistance = thisPose.getTranslation().getNorm();
            if (thisDistance < bestDistance) {
                bestPose = thisPose;
                bestDistance = thisDistance;
                bestDetection = d;
            }
        }

        current.set(new AprilTag(bestDetection.getId(), bestPose));
    }

    public void start() {
        Thread thread = new Thread(this, "AprilTag Detector");
        thread.setDaemon(true);
        thread.start();
    }
}
