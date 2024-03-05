package frc.robot;

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
    
    public TagApproach(int id, double x, double y, Alliance alliance, gameTarget targetType) {
        _fiduciaryNumber = id;
        _offset_X = x;
        _offset_y = y;
        _alliance = alliance;
        _targetType = targetType;
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
}
