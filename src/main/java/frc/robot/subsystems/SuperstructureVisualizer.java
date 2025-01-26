package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;

/*
 * This class is used to create mechanism visualizations and pass them into IOSim objects
 */
public class SuperstructureVisualizer {
    private final Mechanism2d superstructure;
    private final MechanismRoot2d elevatorRoot;
    private final MechanismLigament2d elevator;
    private final MechanismLigament2d wrist;

    public SuperstructureVisualizer(){
        superstructure = new Mechanism2d(0.762, 2);
        elevatorRoot = superstructure.getRoot("Elevator Root", 0.381, 0);
        elevator = elevatorRoot.append(
            new MechanismLigament2d("Elevator", 0, 90)
        );
        wrist = elevator.append(
            new MechanismLigament2d("Wrist", 0.3048, 0)
        );
        SmartDashboard.putData("Superstructure Visualization", superstructure);
    }

    public MechanismLigament2d getElevatorLigament(){
        return elevator;
    }

    public MechanismLigament2d getWristLigament(){
        return wrist;
    }

}
