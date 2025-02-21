package frc.robot.utils;

import static frc.robot.Constants.VisionConstants.fieldLayout;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.VisionConstants.Photonvision.PhotonConfig;
import lib.vision.PhotonVisionSource;

public final class Visionhelpers {

    private Visionhelpers() {}

    public static final PhotonVisionSource buildPVSourceFromConfig(PhotonConfig config,Supplier<Rotation2d> headingSupplier) {
        var est = new PhotonPoseEstimator(
            fieldLayout, 
            config.multiTagPoseStrategy,
            config.offset
        );
        est.setMultiTagFallbackStrategy(config.singleTagPoseStrategy);
        var cam = new PhotonCamera(config.name);
        return new PhotonVisionSource(
            cam, est, config.defaultSingleTagStdDevs, config.defaultMultiTagStdDevs, headingSupplier
        );
    }
}
