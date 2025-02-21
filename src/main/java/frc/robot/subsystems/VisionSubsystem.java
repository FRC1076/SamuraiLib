// Copyright (c) FRC 1076 PiHi Samurai
// You may use, distribute, and modify this software under the terms of
// the license found in the root directory of this project

package frc.robot.subsystems;

import static frc.robot.Constants.VisionConstants.fieldLayout;
import static frc.robot.Constants.VisionConstants.Photonvision.kDefaultMultiTagStdDevs;
import static frc.robot.Constants.VisionConstants.Photonvision.kDefaultSingleTagStdDevs;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.SystemConstants;
import frc.robot.Constants.VisionConstants.Photonvision.PhotonConfig;
import frc.robot.utils.VirtualSubsystem;
import lib.functional.TriConsumer;
import lib.vision.PhotonVisionLocalizer;
import lib.vision.VisionLocalizationSystem;

public class VisionSubsystem extends VirtualSubsystem {
    private final VisionLocalizationSystem m_localizationSystem;

    private final PhotonVisionLocalizer LeftElevatorCameraLocalizer;
    private final PhotonVisionLocalizer RightElevatorCameraLocalizer;

    private final VisionSystemSim sim;

    private final Supplier<Pose2d> poseSupplier;

    public VisionSubsystem(Supplier<Pose2d> poseSupplier) {

        m_localizationSystem = new VisionLocalizationSystem();
        this.poseSupplier = poseSupplier;
        
        var lcam = new PhotonCamera(PhotonConfig.ELEVATOR_LEFT_CAM.name);
        var lest = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            PhotonConfig.ELEVATOR_LEFT_CAM.offset
        );
        lest.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
        LeftElevatorCameraLocalizer = new PhotonVisionLocalizer(
            lcam,
            lest,
            kDefaultSingleTagStdDevs, 
            kDefaultMultiTagStdDevs
        );
        var rcam = new PhotonCamera(PhotonConfig.ELEVATOR_LEFT_CAM.name);
        var rest = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            PhotonConfig.ELEVATOR_LEFT_CAM.offset
        );
        rest.setMultiTagFallbackStrategy(PoseStrategy.AVERAGE_BEST_TARGETS);
        RightElevatorCameraLocalizer = new PhotonVisionLocalizer(
            rcam,
            rest,
            kDefaultSingleTagStdDevs, 
            kDefaultMultiTagStdDevs
        );
        if (SystemConstants.currentMode == 1) {
            var lcamsim = new PhotonCameraSim(lcam);
            var rcamsim = new PhotonCameraSim(rcam);
            sim.addCamera(lcamsim,PhotonConfig.ELEVATOR_LEFT_CAM.offset);
            sim.addCamera(rcamsim,PhotonConfig.ELEVATOR_RIGHT_CAM.offset);
        } else {
            sim = null;
        }

        LeftElevatorCameraLocalizer.addHeadingSupplier(() -> poseSupplier.get().getRotation());
        RightElevatorCameraLocalizer.addHeadingSupplier(() -> poseSupplier.get().getRotation());

        m_localizationSystem.addCamera(LeftElevatorCameraLocalizer);
        m_localizationSystem.addCamera(RightElevatorCameraLocalizer);

    }

    public VisionSubsystem withLocalizationConsumer(TriConsumer<Pose2d,Double,Matrix<N3,N1>> consumer) {
        m_localizationSystem.registerMeasurementConsumer(consumer);
        return this;
    }

    @Override
    public void periodic() {
        if (SystemConstants.currentMode == 1) {
            sim.update(poseSupplier.get());
        }
        m_localizationSystem.update();
    }

    /** Enables trig PNP solving algorithm on the elevator
     * @param enabled whether or not trig estimation should be enabled in single-tag mode
     */
    public void enableElevatorTrigPNP(boolean enabled) {
        LeftElevatorCameraLocalizer.setFallbackPoseStrategy(
            enabled 
                ? PoseStrategy.PNP_DISTANCE_TRIG_SOLVE 
                : PoseStrategy.AVERAGE_BEST_TARGETS
        );
    }
}
