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
        // Create the canvas for mechanisms
        // Width is side frame perimeter (30 inches), height is max elevator height + some buffer
        superstructure = new Mechanism2d(0.762, 3);
        //0.381 is center of frame, 0.127 shifts elevator 5 inches to side like with the real robot
        elevatorRoot = superstructure.getRoot("Elevator Root", 0.381 + 0.127, 0.23114);
        elevator = elevatorRoot.append(
            new MechanismLigament2d("Elevator", 0, 90, 10, new Color8Bit("#000000"))
        );
        //The fixed part of the wrist that has the pivot on the end for the grabber
        fixed = elevator.append(
            new MechanismLigament2d("Fixed", 0.127, -90, 10, new Color8Bit("#000000"))
        );
        //Technically visualizes the grabber
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
