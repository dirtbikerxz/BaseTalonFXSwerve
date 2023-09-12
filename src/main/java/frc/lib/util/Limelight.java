package frc.lib.util;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private NetworkTable table;
    private String name;
    private final double ntDefaultDouble = 0.0;
    private final String ntDefaultString = "";
    private final double[] ntDefaultArray = {};

    public Limelight(String name) {
        this.name = name;
        this.table = NetworkTableInstance.getDefault().getTable(name);
    }

    /**
     * get a double from network tables with the specified key
     * @param key the key to get the value from
     * @return the value of the key, or ntDefaultDouble if the key does not exist or has some other error
     */
    private double getDoubleNT(String key) {
        return table.getEntry(key).getDouble(ntDefaultDouble);
    }

    /**
     * get a boolean from network tables with the specified key
     * @param key the key to get the value from
     * @return the value of the key, or ntDefaultDouble if the key does not exist or has some other error
     */
    private int getIntNT(String key) {
        return (int) table.getEntry(key).getInteger((long) ntDefaultDouble);
    }

    /**
     * get a String from network tables with the specified key
     * @param key the key to get the value from
     * @return the value of the key, or ntDefaultString if the key does not exist or has some other error
     */
    private String getStringNT(String key) {
        return table.getEntry(key).getString(ntDefaultString);
    }

    /**
     * get a double array from network tables with the specified key
     * @param key the key to get the value from
     * @return the value of the key, or ntDefaultArray if the key does not exist or has some other error
     */
    private double[] getArrayNT(String key) {
        return table.getEntry(key).getDoubleArray(ntDefaultArray);
    }

    /**
     * set a double in network tables with the specified key
     * @param key the key to set the value of
     * @param value the value to set the key to (can be int or double)
     */
    private void setNumNT(String key, Number value) {
        table.getEntry(key).setNumber(value);
    }

    /**
     * set a String in network tables with the specified key
     * @param key the key to set the value of
     * @param value the value to set the key to
     */
    private void setArrayNT(String key, double[] value) {
        table.getEntry(key).setDoubleArray(value);
    }

    /**
     * @return Whether the limelight has any valid targets
     */
    public boolean hasTarget() {
        return getDoubleNT("tv") == 1.0;
    }

    //TODO: add limelight 3 fov
    /**
     * Horizontal Offset From Crosshair To Target
     * @return (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
     */
    public double getTargetX() {
        return getDoubleNT("tx");
    }

    //TODO: add limelight 3 fov
    /**
     * Vertical Offset From Crosshair To Target
     * @return (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
     */
    public double getTargetY() {
        return getDoubleNT("ty");
    }

    /**
     * @return Target Area (0% of image to 100% of image)
     */
    public double getTargetArea() {
        return getDoubleNT("ta");
    }

    /**
     * @return The pipeline’s latency contribution (ms)
     */
    public double getPipelineLatency() {
        return getDoubleNT("tl");
    }

    /**
     * @return Capture pipeline latency (ms). Time between the end of the exposure of the middle row of the sensor to the beginning of the tracking pipeline.
     */
    public double getCaptureLatency() {
        return getDoubleNT("cl");
    }

    /**
     * @return the total latency of the limelight (ms). This is the sum of the pipeline and capture latency.
     */
    public double getTotalLatency() {
        return getPipelineLatency() + getCaptureLatency();
    }

    /**
     * @return Sidelength of shortest side of the fitted bounding box (pixels)
     */
    public double getTShort() {
        return getDoubleNT("tshort");
    }

    /**
     * @return Sidelength of longest side of the fitted bounding box (pixels)
     */
    public double getTLong() {
        return getDoubleNT("tlong");
    }

    /**
     * @return Horizontal sidelength of the rough bounding box (0 - 320 pixels)
     */
    public double getTHor() {
        return getDoubleNT("thor");
    }

    /**
     * @return Vertical sidelength of the rough bounding box (0 - 320 pixels)
     */
    public double getTVert() {
        return getDoubleNT("tvert");
    }

    /**
     * @return True active pipeline index of the camera (0 .. 9)
     */
    public int getPipeline() {
        return getIntNT("getpipe");
    }

    /**
     * @return Full JSON dump of targeting results
     */
    public String getTargetJSON() {
        return getStringNT("json");
    }

    /**
     * @return Class ID of primary neural detector result or neural classifier result
     */
    public String getClassID() {
        return getStringNT("tclass");
    }

    /**
     * @return Get the average HSV color underneath the crosshair region as a NumberArray
     */
    public double[] getAverageHSV() {
        return getArrayNT("tc");
    }

    /**
     * Convert an array of 7 doubles to a Pose4d
     * @param ntValues array of 7 doubles containing translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
     * @return a new Pose4d object with the values from the array
     */
    private Pose4d toPose4d(double[] ntValues) {
        return new Pose4d(new Translation3d(ntValues[0], ntValues[1], ntValues[2]), new Rotation3d(ntValues[3], ntValues[4], ntValues[5]), ntValues[6]);
    }

    /**
     * Convert an array of 6 doubles to a Pose3d
     * @param ntValues array of 6 doubles containing translation (X,Y,Z) Rotation(Roll,Pitch,Yaw)
     * @return a new Pose3d object with the values from the array
     */
    private Pose3d toPose3d(double[] ntValues) {
        return new Pose3d(new Translation3d(ntValues[0], ntValues[1], ntValues[2]), new Rotation3d(ntValues[3], ntValues[4], ntValues[5]));
    }

    /**
     * Robot transform in field-space
     * @return Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
     */
    public Pose4d getBotPose() {
        return toPose4d(getArrayNT("botpose"));
    }

    /**
     * Robot transform in field-space (blue driverstation WPILIB origin)
     * @return Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
     */
    private Pose4d getBotPoseBlue() {
        return toPose4d(getArrayNT("botpose_wpiblue"));
    }

    /**
     * Robot transform in field-space (red driverstation WPILIB origin)
     * @return Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw), total latency (cl+tl)
     */
    private Pose4d getBotPoseRed() {
        return toPose4d(getArrayNT("botpose_wpired"));
    }


    /**
     * @return 3D transform of the camera in the coordinate system of the primary in-view AprilTag
     */
    private Pose3d getCamPoseTargetSpace() {
        return toPose3d(getArrayNT("camerapose_targetspace"));
    }

    /**
     * @return 3D transform of the camera in the coordinate system of the Robot
     */
    private Pose3d getCamPoseRobotSpace() {
        return toPose3d(getArrayNT("camerapose_robotspace"));
    }

    /**
     * @return 3D transform of the primary in-view AprilTag in the coordinate system of the Camera
     */
    private Pose3d getTargetPoseCameraSpace() {
        return toPose3d(getArrayNT("targetpose_cameraspace"));
    }

    /**
     * @return 3D transform of the primary in-view AprilTag in the coordinate system of the Robot
     */
    private Pose3d getTargetPoseRobotSpace() {
        return toPose3d(getArrayNT("targetpose_robotspace"));
    }

    /**
     * @return ID of the primary in-view apriltag
     */
    public int getAprilTagID() {
        return getIntNT("tid");
    }

    public enum LEDMode {
        PIPELINE(0),
        OFF(1),
        BLINK(2),
        ON(3);

        private int value;

        LEDMode(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    /**
     * Sets limelight’s LED state
     * PIPELINE: use the LED Mode set in the current pipeline
     * OFF: force off
     * BLINK: force blink
     * ON: force on
     * @param mode LED Mode
     */
    public void setLEDMode(LEDMode mode) {
        setNumNT("ledMode", mode.getValue());
    }

    public enum CamMode {
        VISION(0),
        DRIVER(1);

        private int value;

        CamMode(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    /**
     * Sets limelight’s operation mode
     * VISION: use for vision processing
     * DRIVER: Driver Camera (Increases exposure, disables vision processing)
     * @param mode Cam Mode
     */
    public void setCamMode(CamMode mode) {
        setNumNT("camMode", mode.getValue());
    }

    public enum StreamMode {
        STANDARD(0),
        PIP_MAIN(1),
        PIP_SECONDARY(2);

        private int value;

        StreamMode(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }

    /**
     * Sets limelight’s streaming mode
     * STANDARD: Side-by-side streams if a webcam is attached to Limelight
     * PIP_MAIN: The secondary camera stream is placed in the lower-right corner of the primary camera stream
     * PIP_SECONDARY: The primary camera stream is placed in the lower-right corner of the secondary camera stream
     * @param mode Stream Mode
     */
    public void setStreamMode(StreamMode mode) {
        setNumNT("stream", mode.getValue());
    }

    /**
     * Sets limelight’s current pipeline
     */
    public void setPipeline(int pipeline) {
        setNumNT("pipeline", pipeline);
    }

    /**
     * Take exactly one snapshot with the current limelight settings
     */
    public void takeSnapshot() {
        setNumNT("snapshot", 0);
        setNumNT("snapshot", 1);
    }

    public void setCropSize(double xMin, double yMin, double xMax, double yMax) {
        setArrayNT("crop", new double[] {xMin, xMax, yMin, yMax});
    }
}
