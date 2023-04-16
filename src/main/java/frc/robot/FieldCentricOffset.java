package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FieldCentricOffset {

    private static FieldCentricOffset instance = null;

    private double m_offset = 180.0;

    /**
     * <h3>setOffset</h3>
     * 
     * <p>Sets an offset so that drive controls are oriented correctly.</p>
     * @param offset - desired offset in degrees.
     */
    public void setOffset(double offset) {
        m_offset = offset;
        SmartDashboard.putNumber("debug/Offset", m_offset);
    }
   
    /**
     * <h3>setOffsetByPose</h3>
     * 
     * <p>Finds and sets needed offset of robot through given pose.</p>
     * @param pose - pose to zero from
     */
    public void setOffsetByPose(Pose2d pose) {
        setOffset(pose.getRotation().getDegrees());
    }

    /**
     * <h3>getInstance</h3>
     * 
     * <p>Provides the pre-existing instance of FieldCentricOffset, if there is one, so values don't get changed.</p>
     * @return - the existing instance, if there is one, otherwise, creates a new one
     */
    public static FieldCentricOffset getInstance() {
        if (instance == null) {
            instance = new FieldCentricOffset();
            SmartDashboard.putNumber(instance.getClass().getSimpleName() + "/Offset", instance.m_offset);
        }
        
        return instance;
    }

    /**
     * <h3>getOffset</h3>
     * 
     * <p>Gets the current offset</p>
     * @return - the current offset
     */
    public double getOffset() {
        return m_offset;
    }

}
