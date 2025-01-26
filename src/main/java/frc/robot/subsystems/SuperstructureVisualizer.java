package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color8Bit;

/*
 * This class is used to create mechanism visualizations and pass them into IOSim objects
 */
public class SuperstructureVisualizer {
    private final Mechanism2d superstructure;
    private final MechanismRoot2d elevatorRoot;
    private final MechanismLigament2d elevator;
    private final MechanismLigament2d fixed;
    private final MechanismLigament2d wrist;

    public SuperstructureVisualizer(){
        superstructure = new Mechanism2d(0.762, 3);
        elevatorRoot = superstructure.getRoot("Elevator Root", 0.381 + 0.127, 0.23114);
        elevator = elevatorRoot.append(
            new MechanismLigament2d("Elevator", 0, 90, 10, new Color8Bit("#000000"))
        );
        fixed = elevator.append(
            new MechanismLigament2d("Fixed", 0.127, -90, 10, new Color8Bit("#000000"))
        );
        wrist = fixed.append(
            new MechanismLigament2d("Wrist", 0.3048, 0, 10, new Color8Bit("#770085"))
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
