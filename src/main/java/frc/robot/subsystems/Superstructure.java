package frc.robot.subsystems;

import frc.robot.Constants.ElevatorSimConstants;
import frc.robot.Constants.SuperstructureConstants.CoralScore;
import frc.robot.Constants.SuperstructureConstants.GrabberPosition;
import frc.robot.Constants.SuperstructureConstants.GrabberPossession;
import frc.robot.Constants.SuperstructureConstants.GrabberState;
import frc.robot.Constants.SuperstructureConstants.IndexPossession;
import frc.robot.Constants.SuperstructureConstants.IndexState;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.grabber.GrabberSubsystem;
import frc.robot.subsystems.index.IndexSubsystem;
import frc.robot.subsystems.wrist.WristSubsystem;
import frc.robot.commands.elevator.SetElevatorPositionCommand;
import frc.robot.commands.wrist.SetWristAngleCommand;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Set;
import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.IndexConstants.kIndexVoltage;

public class Superstructure {

    // A mutable class representing the Superstructure's state
    public static class SuperState {

        private GrabberState grabberState;
        private GrabberPosition grabberPosition;
        private IndexState indexState;
        private GrabberPossession grabberPossession;
        private IndexPossession indexPossession;

        public SuperState(GrabberState grabberState, GrabberPosition grabberPosition, IndexState indexState){
            this.grabberState = grabberState;
            this.grabberPosition = grabberPosition;
            this.indexState = indexState;
            grabberPossession = GrabberPossession.EMPTY;
            indexPossession = IndexPossession.EMPTY;
            //calculatePossession();
        }

        public SuperState() {}

        //Calculates game piece possession from beambreaks
        public void calculatePossession(boolean indexBeamBreak, boolean transferBeamBreak, boolean grabberBeamBreak){
            indexPossession = indexBeamBreak 
                ? IndexPossession.CORAL 
                : IndexPossession.EMPTY;
            grabberPossession = grabberBeamBreak 
                ? (transferBeamBreak 
                    ? GrabberPossession.CORAL 
                    : GrabberPossession.ALGAE) 
                : GrabberPossession.EMPTY;
        }

        public void setGrabberPosition(GrabberPosition position) {
            this.grabberPosition = position;
        }

        public void setGrabberState(GrabberState state) {
            this.grabberState = state;
        }

        public void setIndexerState(IndexState state) {
            this.indexState = state;
        }

        public GrabberState getGrabberState() {
            return grabberState;
        }

        public GrabberPosition getGrabberPosition() {
            return grabberPosition;
        }

        public IndexState getIndexerState() {
            return indexState;
        }

        public GrabberPossession getGrabberPossession(){
            return grabberPossession;
        }

        public IndexPossession getIndexerPossession(){
            return indexPossession;
        }
    }

    private final ElevatorSubsystem m_elevator;
    private final GrabberSubsystem m_grabber;
    private final IndexSubsystem m_index;
    private final WristSubsystem m_wrist;

    public final SuperstructureCommandFactory CommandBuilder;

    //Super State
    private final SuperState superState = new SuperState();

    public Superstructure(
        ElevatorSubsystem elevator,
        GrabberSubsystem grabber,
        IndexSubsystem index,
        WristSubsystem wrist,
        BooleanSupplier indexBeamBreak, //returns true when beam broken
        BooleanSupplier transferBeamBreak, //returns true when beam broken
        BooleanSupplier grabberBeamBreak //returns true when beam broken
    ) {
        m_elevator = elevator;
        m_grabber = grabber;
        m_index = index;
        m_wrist = wrist;

        CommandBuilder = new SuperstructureCommandFactory(this, indexBeamBreak, transferBeamBreak, grabberBeamBreak);
    }

    public SuperState getSuperState() {
        return this.superState;
    }

    public Command applyGrabberPosition(GrabberPosition position) {
        Command wristPreMoveCommand;
        switch (superState.getGrabberPossession()) {
            case EMPTY:
            case CORAL:
                wristPreMoveCommand = new SetWristAngleCommand(Rotation2d.kCW_90deg, m_wrist);
                break;
            case ALGAE:
                wristPreMoveCommand = new SetWristAngleCommand(Rotation2d.fromDegrees(65), m_wrist);
                break;
            default:
                wristPreMoveCommand = new SetWristAngleCommand(Rotation2d.kCW_90deg, m_wrist);
                break;
        }
        return Commands.sequence (
            Commands.runOnce(() -> superState.setGrabberPosition(position)),
            wristPreMoveCommand,
            new SetElevatorPositionCommand(position.elevatorHeightMeters, m_elevator),
            new SetWristAngleCommand(position.wristAngle, m_wrist)
        );
    }

    public Command applyGrabberState(GrabberState state) {
        return Commands.sequence(
            Commands.runOnce(() -> superState.setGrabberState(state)),
            m_grabber.runOnce(() -> m_grabber.runVoltsDifferential(state.leftVoltage, state.rightVoltage)) //Can do a runOnce because runVolts is sticky
        );
    }

    public Command applyIndexState(IndexState state) {
        return Commands.sequence(
            Commands.runOnce(() -> superState.setIndexerState(state)),
            m_index.runOnce(() -> m_index.runVolts(kIndexVoltage)) //Can do a runOnce because runVolts is sticky
        );
    }

    public class SuperstructureCommandFactory { 
        private final Superstructure superstructure;
        private final BooleanSupplier m_indexBeamBreak;
        private final BooleanSupplier m_transferBeamBreak;
        private final BooleanSupplier m_grabberBeamBreak;
        private SuperstructureCommandFactory (
            Superstructure superstructure,
            BooleanSupplier indexBeamBreak,
            BooleanSupplier transferBeamBreak,
            BooleanSupplier grabberBeamBreak
        ) {
            this.superstructure = superstructure;
            m_indexBeamBreak = indexBeamBreak;
            m_transferBeamBreak = transferBeamBreak;
            m_grabberBeamBreak = grabberBeamBreak;
        }

        public Command scoreGamePiece() {
            switch (superstructure.getSuperState().getGrabberPosition()) {
                //May not work due to the sequential command group being generated being interrupted by the external trigger
                case L1:
                case L2:
                case L3:
                case L4:
                    return 
                        superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE);
                case NET:
                case PROCESSOR:
                    return
                        superstructure.applyGrabberState(GrabberState.ALGAE_OUTTAKE);
                default:
                    return Commands.none();
            }
        }

        public Command stopGrabber(){
            return superstructure.applyGrabberState(GrabberState.IDLE);
        }

        public Command retractMechanisms(){
            return superstructure.applyGrabberPosition(
                GrabberPosition.TRAVEL
            );
        }

        public Command stopAndRetract(){
            return Commands.sequence(
                stopGrabber(),
                retractMechanisms()
            );
        }

        public Command preL1(){
            return superstructure.applyGrabberPosition(
                GrabberPosition.L1
            );
        }

        public Command preL2(){
            return superstructure.applyGrabberPosition(
                GrabberPosition.L2
            );
        }

        public Command preL3(){
            return superstructure.applyGrabberPosition(
                GrabberPosition.L3
            );
        }

        public Command preL4(){
            return superstructure.applyGrabberPosition(
                GrabberPosition.L4
            );
        }

        public Command preNet(){
            return superstructure.applyGrabberPosition(
                GrabberPosition.NET
            );
        }

        public Command preProcessor(){
            return superstructure.applyGrabberPosition(
                GrabberPosition.PROCESSOR
            );
        }

        public Command groundAlgaeIntake(){
            return 
            Commands.sequence(
                superstructure.applyGrabberPosition(
                    GrabberPosition.GROUND_INTAKE
                ),
                superstructure.applyGrabberState(
                    GrabberState.ALGAE_INTAKE
                )
            );
            
        }

        public Command lowAlgaeIntake(){
            return 
            Commands.sequence(
                superstructure.applyGrabberPosition(
                    GrabberPosition.LOW_INTAKE
                ),
                superstructure.applyGrabberState(
                    GrabberState.ALGAE_INTAKE
                )
            );   
        }

        public Command highAlgaeIntake(){
            return 
            Commands.sequence(
                superstructure.applyGrabberPosition(
                    GrabberPosition.HIGH_INTAKE
                ),
                superstructure.applyGrabberState(
                    GrabberState.ALGAE_INTAKE
                )
            );
            
        }

        public Command calculatePossession(){
            return Commands.runOnce(
                () -> this.superstructure.superState.calculatePossession(
                    m_indexBeamBreak.getAsBoolean(),
                    m_transferBeamBreak.getAsBoolean(),
                    m_grabberBeamBreak.getAsBoolean()
                )
            );
        }
    }

}
