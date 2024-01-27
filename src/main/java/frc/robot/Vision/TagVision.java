package frc.robot.Vision;

import java.io.IOException;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants.LimeLightConstants;

public class TagVision {
    Pose2d prevPose;
    PhotonCamera camera = new PhotonCamera(LimeLightConstants.cameraName);
    Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout
            .loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
    PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout,
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, robotToCam);

    public TagVision() throws IOException {

    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        photonPoseEstimator.setReferencePose(prevPose);
        System.out.println(photonPoseEstimator.update());
        return photonPoseEstimator.update();
    }
}