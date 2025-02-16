package frc.robot.utils;


import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.GameConstants;
import frc.robot.Constants.GameConstants.StartPositions;
import frc.robot.Constants.GameConstants.TeamColors;
import frc.robot.Constants.SuperstructureConstants.GrabberPossession;
import frc.robot.Constants.SuperstructureConstants.IndexPossession;

public final class Elastic {

    public static void putNumber(String key, double value) {
        SmartDashboard.putNumber(key, value);
    }

    public static void putBoolean(String key, boolean value) {
        SmartDashboard.putBoolean(key, value);
    }

    public static void putString(String key, String value) {
        SmartDashboard.putString(key, value);
    }

    public static void putIndexPossession(IndexPossession indexPossession) {
        // System.out.println("indexPossession: " + indexPossession.name);
        SmartDashboard.putString("indexPossession", indexPossession.name);
    }

    public static void putGrabberPossession(GrabberPossession grabberPossession) {
        // System.out.println("grabberPossession: " + grabberPossession.name);
        SmartDashboard.putString("grabberPossession", grabberPossession.name);
    }

    public static void putData(Sendable value) {
        SmartDashboard.putData(value);
    }

    public static ShuffleboardTab getTab(String title) {
        return Shuffleboard.getTab(title);
    }

}