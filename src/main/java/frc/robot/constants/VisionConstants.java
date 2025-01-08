package frc.robot.constants;

import edu.wpi.first.math.geometry.Transform3d;

public class VisionConstants {
  public static final String PHOTON_CAMERA_NAME = "photonvision";

  // SIMULATION CONSTANTS
  public static final Transform3d ROBOT_TO_CAMERA = new Transform3d();
  // Here is what the PhotonVision docs had:
  // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot
  // pose, (Robot pose is considered the center of rotation at the floor level, or
  // Z = 0) and pitched 15 degrees up.
  // public static final Transform3d ROBOT_TO_CAMERA_A = new Transform3d(
  // new Translation3d(0.1, 0, 0.5),
  // new Rotation3d(0, Math.toRadians(-15), 0));

  // our camera's specs:
  // https://support.logi.com/hc/en-us/articles/17368538634519-QuickCam-Pro-9000-Technical-Specifications
  // https://support.logi.com/hc/en-us/articles/360023465073-QuickCam-Pro-9000-Technical-Specifications
  // (not exactly sure why there are 2 links for the same camera but i've put both
  // here just in case)
  public static final double CAM_DIAG_FOV = 75; // degrees - assume wide-angle camera
  public static final double MAX_LED_RANGE = 5; // meters
  public static final int CAM_RES_WIDTH = 640; // pixels
  public static final int CAM_RES_HEIGHT = 480; // pixels
  public static final double MIN_TARGET_AREA = 10; // square pixels
}
