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
import frc.robot.constants.DriveConstants;

public class CoralOutakeInputPosition extends Command{
  
  private AprilTagFieldLayout loadedAprilTagFieldLayout;

    // A functional interface to get the robot's current pose.
    // This makes the class modular, as it doesn't directly depend on a specific
    // odometry implementation.
    private final java.util.function.Supplier<Pose2d> robotPoseSupplier;

  public static HashMap<String, Pose2d> Coordinate_Positions = new HashMap<>();

  public CoralOutakeInputPosition(java.util.function.Supplier<Pose2d> robotPoseSupplier) {
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
                  Coordinate_Positions.put("ID " + tagId, pose3d.toPose2d()));
      
      }
      } else{
        for (int i = 9; i <= 16; i++) {
          final int tagId = i; // Create an effectively final copy
          loadedAprilTagFieldLayout.getTagPose(tagId).ifPresent(pose3d ->
                  Coordinate_Positions.put("ID " + tagId, pose3d.toPose2d()));
      }
        }
    }
   }


public Pose2d calculateTargetPoseFromClosestAprilTag(Pose2d robotPose) {
    return Coordinate_Positions.entrySet().stream()
            // Filter to only include tags we have an offset for
            .filter(entry -> DriveConstants.APRILTAG_TO_TARGET_OFFSETS.containsKey(entry.getKey()))
            // Find the one with the minimum distance
            .min(Comparator.comparingDouble(entry ->
                    entry.getValue().getTranslation().getDistance(robotPose.getTranslation())
            ))
            // If a tag was found, map it to the final target pose
            .map(winner -> {
                Translation2d offset = DriveConstants.APRILTAG_TO_TARGET_OFFSETS.get(winner.getKey());
                return new Pose2d(winner.getValue().getTranslation().plus(offset), winner.getValue().getRotation());
            })
            // Otherwise, return null
            .orElse(null);
}



  
}
