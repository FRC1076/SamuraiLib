package org.pihisamurai.frc2025.robot.utils;

import java.util.ArrayList;
import java.util.List;

import org.pihisamurai.frc2025.robot.Constants.FieldConstants.PoseOfInterest;
import org.pihisamurai.frc2025.robot.Constants.FieldConstants.ReefFace;
import org.pihisamurai.lib.control.LQRHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;

/**
 * A class containing utility functions for localization
 * 
 * @author Tejas Gupta
 * @author David Loh
 * @author Jesse Kane
 */
public final class Localization {

    /**
     * @author Jesse Kane
     * @param robotPose
     * @return the closest Reef Face
     */
    
    public static ReefFace getClosestReefFace(Pose2d robotPose){
        double closestDistance = Double.MAX_VALUE; // Distance away from april tag
        ReefFace closestFace = null;
        for (ReefFace face: ReefFace.values()){
            double distance = robotPose.getTranslation().getDistance(face.AprilTag.getTranslation());
            if (distance < closestDistance) {
                closestDistance = distance;
                closestFace = face;
            }
        }

        return closestFace;
    }

    /** 
     * Returns the pose of the nearest coral station 
     * 
     * @author Jesse Kane
     */
    public static Pose2d getClosestCoralStation(Pose2d robotPose) {
        return robotPose.nearest(getCoralStationPoses());
    }

    public static Pose2d getClosestBlueCoralStation(Pose2d robotPose) {
        return robotPose.nearest(getBlueCoralStationPoses());
    }

    public static Pose2d getClosestRedCoralStation(Pose2d robotPose) {
        return robotPose.nearest(getRedCoralStationPoses());
    }

    /** Returns a list of all coral stations */
    public static List<Pose2d> getCoralStationPoses() {
        List<Pose2d> poseList = new ArrayList<>();
        poseList.add(PoseOfInterest.BLU_CORAL_STATION_OPPOSITE.pose);
        poseList.add(PoseOfInterest.BLU_CORAL_STATION_PROCESSOR.pose);
        poseList.add(PoseOfInterest.RED_CORAL_STATION_PROCESSOR.pose);
        poseList.add(PoseOfInterest.RED_CORAL_STATION_OPPOSITE.pose);
        return poseList;
    }
    /** Returns a list of red coral stations */
    public static List<Pose2d> getRedCoralStationPoses() {
        List<Pose2d> poseList = new ArrayList<>();
        poseList.add(PoseOfInterest.RED_CORAL_STATION_PROCESSOR.pose);
        poseList.add(PoseOfInterest.RED_CORAL_STATION_OPPOSITE.pose);
        return poseList;
    }

    /** Returns a list of blue coral stations */
    public static List<Pose2d> getBlueCoralStationPoses() {
        List<Pose2d> poseList = new ArrayList<>();
        poseList.add(PoseOfInterest.BLU_CORAL_STATION_OPPOSITE.pose);
        poseList.add(PoseOfInterest.BLU_CORAL_STATION_PROCESSOR.pose);
        return poseList;
    }

    private static ReefFace getReefFromAprilTagID(int AprilTagID){
        for (ReefFace face: ReefFace.values()) {
            if (face.AprilTagID == AprilTagID) {
                return face;
            }
        }
        return null;
    }
}
