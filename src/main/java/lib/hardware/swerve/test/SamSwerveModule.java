package lib.hardware.swerve.test;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import lib.hardware.swerve.SwerveModuleBase;
import lib.hardware.swerve.test.SamSwerveModule.ModuleConstants.Common.Drive;
import lib.hardware.swerve.test.SamSwerveModule.ModuleConstants.Common.Turn;
import lib.hardware.swerve.test.SamSwerveModule.ModuleConstants.SamModuleConfig;


/** A class for testing SamuraiSwerve's high-level logic on Sam */
public class SamSwerveModule implements SwerveModuleBase {
    
    static class ModuleConstants {
        public static class Common {
            public static class Drive {
                public static final int CurrentLimit = 60;
                public static final double gearRatio = 6.75;
                public static final double VoltageCompensation = 12;
                public static final double MaxModuleSpeed = 14.0; // Maximum attainable module speed
                public static final double WheelRadius = Units.inchesToMeters(4); // Meters
                public static final double WheelCOF = 1.0; // Coefficient of friction
                public static final double PositionConversionFactor = 2 * WheelRadius * Math.PI / gearRatio; // Units:
                                                                                                             // Meters
                public static final double VelocityConversionFactor = PositionConversionFactor / 60; // Units:
                                                                                                     // Meters per
                                                                                                     // second

                // PID constants
                public static final double kP = 0.035;
                public static final double kI = 0.000;
                public static final double kD = 0.0012;

                // Feedforward constants
                public static final double kV = 2.78;
                public static final double kS = 0.0;
                public static final double kA = 0.0;
            }

            public static class Turn {
                public static final int CurrentLimit = 60;
                public static final double VoltageCompensation = 12;
                public static final double gearRatio = 12.8;
                public static final double PositionConversionFactor = 1 / gearRatio; // Units: Rotations
                public static final double VelocityConversionFactor = PositionConversionFactor; // Units: RPM

                // PID constants
                public static double kP = 0.75;
                public static final double kI = 0.0;
                public static final double kD = 0.0001;
            }
        }

        public static enum SamModuleConfig {

            FrontLeft(1, 11, 21, -0.441162109375 + 0.5),
            FrontRight(2, 12, 22, -0.3984375 + 0.5),
            RearLeft(3, 13, 23, -0.525146484375),
            RearRight(4, 14, 24, -0.931396484375);

            public final int DrivePort;
            public final int TurnPort;
            public final int EncoderPort;
            public final double EncoderOffsetRots;

            private SamModuleConfig(int DrivePort, int TurnPort, int EncoderPort, double EncoderOffsetRots) {
                this.DrivePort = DrivePort;
                this.TurnPort = TurnPort;
                this.EncoderPort = EncoderPort;
                this.EncoderOffsetRots = EncoderOffsetRots;
            }
        }
    }

    private final SparkMax m_turnMotor;
    private final SparkClosedLoopController TurnPID;
    private final RelativeEncoder TurnRelEncoder;

    private final SparkMax m_driveMotor;
    private final SparkClosedLoopController DrivePID;
    private final RelativeEncoder DriveRelEncoder;

    private final CANcoder m_turnEncoder;
    private final StatusSignal<Angle> turnAbsolutePosition;


    public SamSwerveModule(int index) {
        SamModuleConfig config;
        switch (index) {
            case 0:
                config = SamModuleConfig.FrontLeft;
                break;
            case 1:
                config = SamModuleConfig.FrontRight;
                break;
            case 2:
                config = SamModuleConfig.RearLeft;
                break;
            case 3:
                config = SamModuleConfig.RearRight;
                break;
            default:
                config = null;
        }
        
        m_turnMotor = new SparkMax(config.TurnPort,MotorType.kBrushless);
        TurnPID = m_turnMotor.getClosedLoopController();
        TurnRelEncoder = m_turnMotor.getEncoder();

        m_driveMotor = new SparkMax(config.DrivePort,MotorType.kBrushless);
        DrivePID = m_driveMotor.getClosedLoopController();
        DriveRelEncoder = m_driveMotor.getEncoder();
    
        m_turnEncoder = new CANcoder(config.EncoderPort);

        //Config turn absolute encoder here
        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.FutureProofConfigs = false;
        encoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1.0;
        encoderConfig.MagnetSensor.MagnetOffset = config.EncoderOffsetRots;
        m_turnEncoder.getConfigurator().apply(encoderConfig);
        turnAbsolutePosition = m_turnEncoder.getAbsolutePosition();

        SparkMaxConfig turnConfig = new SparkMaxConfig();
        turnConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Turn.CurrentLimit)
            .voltageCompensation(Turn.VoltageCompensation);
        turnConfig
            .encoder
            .positionConversionFactor(Turn.PositionConversionFactor)
            .velocityConversionFactor(Turn.VelocityConversionFactor);
        turnConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(Turn.kP)
            .i(Turn.kI)
            .d(Turn.kD)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0,1);
        turnConfig
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000/250))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        
        m_turnMotor.configure(turnConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        TurnRelEncoder.setPosition(turnAbsolutePosition.getValueAsDouble());
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        driveConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(Drive.CurrentLimit)
            .voltageCompensation(Drive.VoltageCompensation);
        driveConfig
            .encoder
            .positionConversionFactor(Drive.PositionConversionFactor)
            .velocityConversionFactor(Drive.VelocityConversionFactor);
        driveConfig
            .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(Drive.kP)
            .i(Drive.kI)
            .d(Drive.kD);
        driveConfig
            .signals
            .primaryEncoderPositionAlwaysOn(true)
            .primaryEncoderPositionPeriodMs((int) (1000/250))
            .primaryEncoderVelocityAlwaysOn(true)
            .primaryEncoderVelocityPeriodMs(20)
            .appliedOutputPeriodMs(20)
            .busVoltagePeriodMs(20)
            .outputCurrentPeriodMs(20);
        
        m_driveMotor.configure(driveConfig,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        DriveRelEncoder.setPosition(0.0);
    }

    @Override
    public synchronized SwerveModuleState getState() {
        return new SwerveModuleState(DriveRelEncoder.getVelocity(),Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble()));
    }

    @Override
    public synchronized SwerveModulePosition getPosition() {
        return new SwerveModulePosition(DriveRelEncoder.getPosition(),Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble()));
    }

    @Override
    public void setDesiredState(SwerveModuleState state) {
        state.optimize(Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble()));
        DrivePID.setReference(
            state.speedMetersPerSecond,
            ControlType.kVelocity
        );

        TurnPID.setReference(
            state.angle.getRotations(),
            ControlType.kPosition
        );
    }



}
