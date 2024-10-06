package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class PhotonVisionSubsystem extends SubsystemBase {
  private AprilTagFieldLayout m_fieldLayout;
  private PhotonPoseEstimator m_photonPoseEstimator;

  public PhotonVisionSubsystem() {
    try {
      m_fieldLayout =
          new AprilTagFieldLayout(
              Path.of(
                  Filesystem.getDeployDirectory().getAbsolutePath(), "vision", "tinyfield.json"));
    } catch (IOException e) {
      // Swallowing exceptions because I really don't feel like dealing with exception handling
      // everywhere, sorry kids we'll learn about this some other time :-)
      e.printStackTrace();
      throw new RuntimeException(e);
    }

    // Forward Camera
    PhotonCamera cam = new PhotonCamera("forwardcam");

    // Cam mounted facing forward, 7 cm forward of center, 3 cm above center.
    Transform3d robotToCam =
        new Transform3d(new Translation3d(0.0, 0.07, 0.03), new Rotation3d(0, 0, 0));

    // Construct PhotonPoseEstimator
    m_photonPoseEstimator =
        new PhotonPoseEstimator(
            m_fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, cam, robotToCam);
  }

  // Determine where we think the robot is right now based on AprilTags we can see.
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    return m_photonPoseEstimator.update();
  }
}
