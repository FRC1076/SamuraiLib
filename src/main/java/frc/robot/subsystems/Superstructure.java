package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

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
import edu.wpi.first.wpilibj2.command.ConditionalCommand;


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

    private Command applyGrabberPosition(GrabberPosition position) {
        Command wristPreMoveCommand = Commands.either(
            new SetWristAngleCommand(Rotation2d.fromDegrees(65), m_wrist),
            new SetWristAngleCommand(Rotation2d.kCW_90deg, m_wrist),
            () -> superState.getGrabberPossession() == GrabberPossession.ALGAE
        );
        return Commands.sequence (
            Commands.runOnce(() -> superState.setGrabberPosition(position)),
            wristPreMoveCommand,
            new SetElevatorPositionCommand(position.elevatorHeightMeters, m_elevator),
            new SetWristAngleCommand(position.wristAngle, m_wrist)
        );
    }

    private Command applyGrabberState(GrabberState state) {
        return Commands.sequence(
            Commands.runOnce(() -> superState.setGrabberState(state)),
            m_grabber.applyDifferentialVolts(state.leftVoltage, state.rightVoltage) //Can do a runOnce because runVolts is sticky
        );
    }

    private Command applyIndexState(IndexState state) {
        return Commands.sequence(
            Commands.runOnce(() -> superState.setIndexerState(state)),
            m_index.applyVolts(kIndexVoltage).onlyIf(() -> state.running) //Can do a runOnce because runVolts is sticky
        );
    }

    public class SuperstructureCommandFactory { 
        private final Superstructure superstructure;
        private final BooleanSupplier m_indexBeamBreak;
        private final BooleanSupplier m_transferBeamBreak;
        private final BooleanSupplier m_grabberBeamBreak;
        private final Map<GrabberPosition, Command> scoringCommands = new HashMap<>();
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
            scoringCommands.put(GrabberPosition.L1, superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE));
            scoringCommands.put(GrabberPosition.L2, superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE));
            scoringCommands.put(GrabberPosition.L3, superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE));
            scoringCommands.put(GrabberPosition.L4, superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE));
            scoringCommands.put(GrabberPosition.NET, superstructure.applyGrabberState(GrabberState.ALGAE_OUTTAKE));
            scoringCommands.put(GrabberPosition.PROCESSOR, superstructure.applyGrabberState(GrabberState.ALGAE_OUTTAKE));
            scoringCommands.put(null, superstructure.applyGrabberState(GrabberState.CORAL_OUTTAKE)); //default if above isn't found

        }

        /*
         * Score game piece differently depending on robot state
         */
        public Command scoreGamePiece() {
            return Commands.select(
                scoringCommands,
                () -> {
                    final GrabberPosition grabberPosition = superstructure.getSuperState().grabberPosition;
                    return scoringCommands.containsKey(grabberPosition)
                        ? grabberPosition
                        : null;
                }
            );
        }

        /*
         * Stop grabber wheels from spinning
         */
        public Command stopGrabber(){
            return superstructure.applyGrabberState(GrabberState.IDLE);
        }

        /*
         * Retract mechanisms to travel state
         */
        public Command retractMechanisms(){
            return superstructure.applyGrabberPosition(GrabberPosition.TRAVEL);
        }

        /*
         * Call this on button releases, stops grabber movement and retracts mechanisms to travel state
         */
        public Command stopAndRetract(){
            return Commands.parallel(
                stopGrabber(),
                retractMechanisms()
            );
        }

        /*
         * Set elevator and wrist to L1 preset
         */
        public Command preL1(){
            return superstructure.applyGrabberPosition(GrabberPosition.L1);
        }

        /*
         * Set elevator and wrist to L2 preset
         */
        public Command preL2(){
            return superstructure.applyGrabberPosition(GrabberPosition.L2);
        }

        /*
         * Set elevator and wrist to L3 preset
         */
        public Command preL3(){
            return superstructure.applyGrabberPosition(GrabberPosition.L3);
        }

        /*
         * Set elevator and wrist to L4 preset
         */
        public Command preL4(){
            return superstructure.applyGrabberPosition(GrabberPosition.L4);
        }

        /*
         * Set elevator and wrist to net preset
         */
        public Command preNet(){
            return superstructure.applyGrabberPosition(GrabberPosition.NET);
        }

        /*
         * Set elevator and wrist to processor preset
         */
        public Command preProcessor(){
            return superstructure.applyGrabberPosition(GrabberPosition.PROCESSOR);
        }

        /*
         * Set elevator and wrist to ground algae preset while running grabber intake if open
         */
        public Command groundAlgaeIntake(){
            return 
            Commands.parallel(
                superstructure.applyGrabberPosition(GrabberPosition.GROUND_INTAKE),
                superstructure.applyGrabberState(GrabberState.ALGAE_INTAKE)
                    .unless(() -> superState.getGrabberPossession() == GrabberPossession.ALGAE) // Check if there is already an algae intaked
            );
        }

        /*
         * Set elevator and wrist to low algae preset while running grabber intake if open
         */
        public Command lowAlgaeIntake(){
            return 
            Commands.parallel(
                superstructure.applyGrabberPosition(GrabberPosition.LOW_INTAKE),
                superstructure.applyGrabberState(GrabberState.ALGAE_INTAKE)
                    .unless(() -> superState.getGrabberPossession() == GrabberPossession.ALGAE) // Check if there is already an algae intaked
            );   
        }

        /*
         * Set elevator and wrist to high algae preset while running grabber intake if open
         */
        public Command highAlgaeIntake(){
            return Commands.parallel(
                superstructure.applyGrabberPosition(GrabberPosition.HIGH_INTAKE),
                superstructure.applyGrabberState(GrabberState.ALGAE_INTAKE)
                    .unless(() -> superState.getGrabberPossession() == GrabberPossession.ALGAE) // Check if there is already an algae intaked
            );
            
        }

        /*
         * Used to calculate what the robot is possessing based on breambreaks
         */
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
