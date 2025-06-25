package frc.robot.CoralOutake;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Subsystems;
import frc.robot.constants.DriveConstants;

public class CoralOutakeDriveToPosition extends Command {
  private final Translation2d reefCentre = new Translation2d(13.1, 4);
  private final double reefRadius = 1.4; 
  private Pose2d targetLocation;
  private double leftRightOffset = 0;

  private Command pathCommand;
  public CoralOutakeDriveToPosition(double leftRightOffset){
    this.leftRightOffset = leftRightOffset;
}

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

  public void DriveToPosition(double x, double y, double angle){
    ChassisSpeeds speeds = new ChassisSpeeds(x,y,angle);
    Subsystems.drive.drive(speeds, isScheduled());

  }

  public void CoordinateMagicNearest(){
    Translation2d curLoc = Subsystems.nav.getPose().getTranslation();

    
  }

  



}