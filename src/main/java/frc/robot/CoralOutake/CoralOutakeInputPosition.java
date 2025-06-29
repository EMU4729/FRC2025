package frc.robot.CoralOutake;

import java.security.Key;
import java.util.Collection;
import java.util.Comparator;
import java.util.Dictionary;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.SystemTests.SystemTest;
import frc.robot.constants.DriveConstants;

public class CoralOutakeInputPosition extends Command{
  
  private AprilTagFieldLayout loadedAprilTagFieldLayout;

    // A functional interface to get the robot's current pose.
    // This makes the class modular, as it doesn't directly depend on a specific
    // odometry implementation.
    private final Pose2d robotPoseSupplier;

  public static HashMap<Integer, Pose2d> Coordinate_Positions = new HashMap<>();

  public CoralOutakeInputPosition(Pose2d robotPoseSupplier) {
    this.robotPoseSupplier = robotPoseSupplier;
    // Load the field layout once when the class is instantiated
    try {
        loadedAprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    } catch (Exception e) {
        System.err.println("Failed to load AprilTag field layout: " + e.getMessage());
        // Handle error, perhaps load a fallback or disable AprilTag features
        loadedAprilTagFieldLayout = null; 
    }
    SetCoordinateValues(); // Populate the AprilTag coordinates upon construction
}



  public void SetCoordinateValues(){
    AprilTagFieldLayout loadedAprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent()){
      var alliance_colour = alliance.get();

      if (alliance_colour == DriverStation.Alliance.Red){
        for (int i = 1; i <= 8; i++) {
          final int tagId = i; // Create an effectively final copy
          loadedAprilTagFieldLayout.getTagPose(tagId).ifPresent(pose3d ->
                  Coordinate_Positions.put( tagId, pose3d.toPose2d()));
          //
          //System.out.println(Coordinate_Positions);
      
      }
      } else{
        for (int i = 9; i <= 16; i++) {
          final int tagId = i; // Create an effectively final copy
          loadedAprilTagFieldLayout.getTagPose(tagId).ifPresent(pose3d ->
                  Coordinate_Positions.put( tagId, pose3d.toPose2d()));
      }
        }
    }
   }


public Pose2d calculateTargetPoseFromClosestAprilTag(Pose2d robotPose) {
  if (Coordinate_Positions.isEmpty()) {
    return null;
}

Map.Entry<Integer, Pose2d> closestEntry = null;
double minDistance = Double.MAX_VALUE;
for (Map.Entry<Integer, Pose2d> currentEntry : Coordinate_Positions.entrySet()) {
  // Calculate the distance from the robot to the current AprilTag.
  double distance = currentEntry.getValue().getTranslation().getDistance(robotPose.getTranslation());
  //
  System.err.println(distance);
  // If this tag is closer than any we've seen so far, it's our new candidate.
  if (distance < minDistance) {
      minDistance = distance;
      closestEntry = currentEntry;
  }
}

Translation2d offset = DriveConstants.APRILTAG_TO_TARGET_OFFSETS.get(closestEntry.getKey());
// Get the pose of the winning AprilTag.
Pose2d aprilTagPose = closestEntry.getValue();
System.out.println(aprilTagPose);
 
    // 1. ROTATE THE OFFSET: Apply the offset relative to the tag's orientation.
    // This ensures that an offset of (X=-0.5, Y=0) is always "in front" of the tag.
    Translation2d finalTargetTranslation = aprilTagPose.getTranslation().plus(offset.rotateBy(aprilTagPose.getRotation()));

    // 2. CALCULATE THE CORRECT ROTATION: Calculate the angle from the robot to the target.
    // This makes the robot TURN TO FACE the target.
    Translation2d vectorToTarget = finalTargetTranslation.minus(robotPose.getTranslation());
    Rotation2d finalTargetRotation = vectorToTarget.getAngle();

    // For scoring, you might want a fixed 180-degree rotation. If so, use this instead:
    // Rotation2d finalTargetRotation = Rotation2d.fromDegrees(180);

    // 3. CREATE THE FINAL TARGET POSE with the corrected translation and rotation.
    Pose2d targetPose = new Pose2d(finalTargetTranslation, finalTargetRotation);

    System.out.println("Final Target: " + targetPose);
    return targetPose;

}



  
}
