package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.TagApproach;
import frc.robot.TagApproach.gameTarget;

public class TagApproaches {
    private TagApproach[] tagArray;

    // distance in meters from camera lens to front edge of bumper.
    public final double cameraOffset=15;
    
    public TagApproaches() {
        tagArray = new TagApproach[16];
        tagArray[0] = new TagApproach(1, 0, 0.64, Alliance.Blue, gameTarget.Source);
        tagArray[1] = new TagApproach(2, 0, -0.64, Alliance.Blue, gameTarget.Source);
        tagArray[2] = new TagApproach(3, 0, 0, Alliance.Red, gameTarget.Speaker);
        tagArray[3] = new TagApproach(4, 0, 0, Alliance.Red, gameTarget.Speaker);
        tagArray[4] = new TagApproach(5, 0, 0, Alliance.Red, gameTarget.Amp);
        tagArray[5] = new TagApproach(6, 0, 0, Alliance.Blue, gameTarget.Amp);
        tagArray[6] = new TagApproach(7, 0.92, 0, Alliance.Blue, gameTarget.Speaker);
        tagArray[7] = new TagApproach(8, 0.92, 0.56, Alliance.Blue, gameTarget.Speaker);
        tagArray[8] = new TagApproach(9, 0, -0.64, Alliance.Red, gameTarget.Source);
        tagArray[9] = new TagApproach(10, 0, 0.64, Alliance.Red, gameTarget.Source);
        tagArray[10] = new TagApproach(11, 0, 0, Alliance.Red, gameTarget.Stage);
        tagArray[11] = new TagApproach(12, 0, 0, Alliance.Red, gameTarget.Stage);
        tagArray[12] = new TagApproach(13, 0, 0, Alliance.Red, gameTarget.Stage);
        tagArray[13] = new TagApproach(14, 0, 0, Alliance.Blue, gameTarget.Stage);
        tagArray[14] = new TagApproach(15, 0, 0, Alliance.Blue, gameTarget.Stage);
        tagArray[15] = new TagApproach(16, 0, 0, Alliance.Blue, gameTarget.Stage);
    }

    public int FiduciaryNumber(int tagID){
        return tagArray[tagID - 1].FiduciaryNumber();
    }

    public double Offset_X(int tagID){
        return tagArray[tagID - 1].Offset_X() ;
    }

    public double Offset_Y(int tagID){
        return tagArray[tagID - 1].Offset_Y() ;
    }

    public Alliance TagAlliance(int tagID){
        return tagArray[tagID - 1].TagAlliance();
    }

    public gameTarget GameTarget(int tagID){
        return tagArray[tagID - 1].GameTarget() ;
    }

    public String GameTargetName(int tagID) {
        return tagArray[tagID - 1].GameTargetName() ;
    }
}
