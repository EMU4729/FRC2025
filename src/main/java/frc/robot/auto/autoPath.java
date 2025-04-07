package frc.robot.auto;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems;
import frc.robot.constants.DriveConstants;

public class autoPath extends Command {
  private final Translation2d reefCentre = new Translation2d(13.1, 4);
  private final double reefRadius = 1.4; 
  private Pose2d targetLocation;
  private double leftRightOffset = 0;

  private Command pathCommand;
  public autoPath(double leftRightOffset){
    this.leftRightOffset = leftRightOffset;
  }
  
  
  @Override
  public void initialize() {
    targetLocation = closestReefSide();
    pathCommand = AutoBuilder.pathfindToPose(targetLocation, DriveConstants.PATH_CONSTRAINTS);
    pathCommand.schedule();
  }

  @Override
  public boolean isFinished() {
    return !pathCommand.isScheduled();
  }

  @Override
  public void end(boolean interrupted) {
    if(interrupted){pathCommand.cancel();}
  }



  private Pose2d closestReefSide(){
    Translation2d curLoc = Subsystems.nav.getPose().getTranslation();

    Translation2d minLoc = new Translation2d(100000, 100000);
    double minDist = 1000000000;
    Rotation2d minAngle = new Rotation2d();
    for(int i = 0; i < 360; i+=60){
      Rotation2d sideAngle = Rotation2d.fromDegrees(i);
      Translation2d sideLoc = new Translation2d(reefRadius, leftRightOffset).rotateBy(sideAngle).plus(reefCentre);
      double curDist = sideLoc.getDistance(curLoc);
      if (curDist < minDist){
        minDist = curDist;
        minLoc = sideLoc;
        minAngle = sideAngle;
      }
    }

    return new Pose2d(minLoc, minAngle.minus(Rotation2d.k180deg));
  }
}
