package frc.robot.CoralOutake;

import java.util.Dictionary;
import java.util.HashMap;
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

   /**
 * Calculates the target pose based on the closest known AprilTag.
 * @return An Optional containing the calculated Pose2d of the target, or empty if no suitable tag is found.
 */
public Optional<Pose2d> GetOffsetPositionFromAprilTag() {
  Pose2d robotPose = robotPoseSupplier.get();

  // Find the closest AprilTag from our populated list
  Optional<Pose2d> closestTagPose = findClosestAprilTag(robotPose);

  if (closestTagPose.isEmpty()) {
      return Optional.empty(); // No tag found to calculate from
  }

  // Find the ID of the closest tag
  // This is a bit inefficient; the 'better implementation' below solves this.
  int closestTagId = -1;
  for (var entry : Coordinate_Positions.entrySet()) {
      if (entry.getValue().equals(closestTagPose.get())) {
          // Extract the integer ID from the string "ID X"
          closestTagId = Integer.parseInt(entry.getKey().replace("ID ", ""));
          break;
      }
  }

  if (closestTagId == -1 || !DriveConstants.APRILTAG_TO_TARGET_OFFSETS.containsKey(closestTagId)) {
      // We found a tag, but we don't have a tuned offset for it
      return Optional.empty();
  }

  // Get the manually-tuned offset for this specific tag
  Translation2d offset = DriveConstants.APRILTAG_TO_TARGET_OFFSETS.get(closestTagId);

  // Calculate the final target pose: Start with the tag's pose and apply the offset
  Pose2d targetPose = new Pose2d(
      closestTagPose.get().getTranslation().plus(offset),
      closestTagPose.get().getRotation() // Assuming target has same orientation as tag
  );

  return Optional.of(targetPose);
}
/**
 * Helper function to find the closest AprilTag to the robot.
 * @param robotPose The current pose of the robot.
 * @return An Optional containing the Pose2d of the closest tag.
 */
private Optional<Pose2d> findClosestAprilTag(Pose2d robotPose) {
  return Coordinate_Positions.values().stream()
          .min((pose1, pose2) -> Double.compare(
                  pose1.getTranslation().getDistance(robotPose.getTranslation()),
                  pose2.getTranslation().getDistance(robotPose.getTranslation())));
}


  
}
