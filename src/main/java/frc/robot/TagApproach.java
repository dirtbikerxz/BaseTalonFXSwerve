package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class TagApproach {
    enum gameTarget {
        Amp,
        Speaker,
        Source, 
        Stage
    }
    
    private int _fiduciaryNumber;
    private double _offset_X;
    private double _offset_y;
    private Alliance _alliance;
    private gameTarget _targetType;
    private Pose2d _desiredPose;
    
    public TagApproach(int id, double tagOffsetX, double tagOffsetY, Alliance alliance, gameTarget targetType, 
        Pose2d desiredPose) {
        _fiduciaryNumber = id;
        _offset_X = tagOffsetX;
        _offset_y = tagOffsetY;
        _alliance = alliance;
        _targetType = targetType;
        _desiredPose = desiredPose;
    }

    public int FiduciaryNumber(){
        return _fiduciaryNumber;
    }

    public double Offset_X(){
        return _offset_X;
    }

    public double Offset_Y(){
        return _offset_y;
    }

    public Alliance TagAlliance(){
        return _alliance;
    }

    public gameTarget GameTarget(){
        return _targetType;
    }

    public String GameTargetName() {
        return String.format("%s - %s Alliance",_targetType.name(), _alliance.name());
    }
    
    public Pose2d DesiredPos(){
        return _desiredPose;
    }
}
