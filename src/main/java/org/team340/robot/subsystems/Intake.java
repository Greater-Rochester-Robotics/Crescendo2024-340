package org.team340.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.GRRSubsystem;
import org.team340.lib.util.Math2;
import org.team340.robot.Constants.IntakeConstants;
import org.team340.robot.Constants.RobotMap;

// TODO Motion profiling (?)

/**
 * This subsystem intakes notes from the floor and can score them in the amp, or pass them to the shooter.
 */
public class Intake extends GRRSubsystem {

    private final CANSparkFlex armLeftMotor;
    private final CANSparkFlex armRightMotor;
    private final CANSparkMax rollerUpperMotor;
    private final CANSparkMax rollerLowerMotor;
    private final SparkAbsoluteEncoder armEncoder;
    private final SparkPIDController armPID;
    private final DigitalInput noteDetector;

    private double maintainAngle = 0.0;
    private double target = 0.0;

    public Intake() {
        super("Intake");
        armLeftMotor = createSparkFlex("Arm Left Motor", RobotMap.INTAKE_ARM_LEFT_MOTOR, MotorType.kBrushless);
        armRightMotor = createSparkFlex("Arm Right Motor", RobotMap.INTAKE_ARM_RIGHT_MOTOR, MotorType.kBrushless);
        armEncoder = createSparkFlexAbsoluteEncoder("Arm Encoder", armLeftMotor, Type.kDutyCycle);
        armPID = armLeftMotor.getPIDController();
        armPID.setFeedbackDevice(armEncoder);

        rollerUpperMotor = createSparkMax("Roller Upper Motor", RobotMap.INTAKE_ROLLER_UPPER_MOTOR, MotorType.kBrushless);
        rollerLowerMotor = createSparkMax("Roller Lower Motor", RobotMap.INTAKE_ROLLER_LOWER_MOTOR, MotorType.kBrushless);

        noteDetector = createDigitalInput("Note Detector", RobotMap.INTAKE_NOTE_DETECTOR);

        IntakeConstants.ArmConfigs.LEFT_MOTOR.apply(armLeftMotor);
        IntakeConstants.ArmConfigs.RIGHT_MOTOR.apply(armRightMotor);
        IntakeConstants.RollerConfigs.MOTOR.apply(rollerUpperMotor);
        IntakeConstants.RollerConfigs.MOTOR.apply(rollerLowerMotor);
        IntakeConstants.ArmConfigs.ENCODER.apply(armLeftMotor, armEncoder);
        IntakeConstants.ArmConfigs.PID.apply(armLeftMotor, armPID);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);
        builder.addDoubleProperty("armTarget", () -> target, null);
        builder.addBooleanProperty("hasNote", this::hasNote, null);
    }

    /**
     * Returns {@code true} when the note detector is detecting a note.
     */
    public boolean hasNote() {
        return noteDetector.get();
    }

    /**
     * Sets the {@link #armPID} to go to the specified angle if it is valid
     * (within the intake {@link IntakeConstants#MINIMUM_ANGLE minimum} and
     * {@link IntakeConstants#MAXIMUM_ANGLE maximum} angles).
     * @param position The angle to set.
     */
    private void setValidPosition(double position) {
        if (position < IntakeConstants.MINIMUM_ANGLE || position > IntakeConstants.MAXIMUM_ANGLE) {
            DriverStation.reportWarning(
                "The angle " +
                position +
                " is not valid. is must be within " +
                IntakeConstants.MAXIMUM_ANGLE +
                " and " +
                IntakeConstants.MINIMUM_ANGLE +
                ".",
                true
            );
        } else {
            armPID.setReference(position, ControlType.kPosition);
            target = position;
        }
    }

    private Command useState(double angle, double upperSpeed, double lowerSpeed, boolean willFinish) {
        return commandBuilder()
            .onInitialize(() -> {
                rollerUpperMotor.set(upperSpeed);
                rollerLowerMotor.set(lowerSpeed);
            })
            .onExecute(() -> {
                setValidPosition(angle);
                maintainAngle = armEncoder.getPosition();
            })
            .isFinished(() -> willFinish && Math2.epsilonEquals(armEncoder.getPosition(), angle, IntakeConstants.CLOSED_LOOP_ERROR))
            .onEnd(interrupted -> {
                if (!interrupted || Math2.epsilonEquals(armEncoder.getPosition(), angle, IntakeConstants.CLOSED_LOOP_ERROR)) maintainAngle =
                    angle;
                rollerUpperMotor.stopMotor();
                rollerLowerMotor.stopMotor();
                armLeftMotor.stopMotor();
            });
    }

    /**
     * This moves the intake down to the intake position, but doesn't start the rollers.
     * @return This command.
     */
    public Command intakeDown() {
        return useState(IntakeConstants.DEPLOY_POSITION, 0, 0, true).withName("intake.intakeDown()");
    }

    /**
     * Deploys the intake and runs the roller motors to intake a note.
     */
    public Command intake() {
        return useState(
            IntakeConstants.DEPLOY_POSITION,
            IntakeConstants.INTAKE_ROLLER_SPEED,
            IntakeConstants.INTAKE_ROLLER_SPEED * .6,
            false
        )
            .withName("intake.intake()");
    }

    /**
     * This moves the intake arm to point straight up.
     * @return This command.
     */
    public Command retract() {
        return useState(IntakeConstants.STRAIGHT_UP_POSITION, 0, 0, true).withName("intake.retract()");
    }

    /**
     * Retracts the intake into the frame perimeter and stops the rollers.
     */
    public Command toSafePosition() {
        return useState(IntakeConstants.SAFE_POSITION, 0, 0, true).withName("intake.toSafePosition()");
    }

    /**
     * This command moves the intake to the {@link IntakeConstants#SCORE_AMP_POSITION SCORE_AMP_POSITION}
     * and ends once the position has been reached.
     * @return This command.
     */
    public Command scoreAmpPosition() {
        return useState(IntakeConstants.SCORE_AMP_POSITION, 0, 0, true).withName("intake.scoreAmpPosition()");
    }

    /**
     * This command moves the intake up, and then scores it with a delay. This doesn't bring it back down.
     * @return This command.
     */
    public Command scoreAmp() {
        return scoreAmpPosition()
            .andThen(
                useState(
                    IntakeConstants.SCORE_AMP_POSITION,
                    IntakeConstants.SCORE_AMP_ROLLER_SPEED,
                    IntakeConstants.SCORE_AMP_ROLLER_SPEED,
                    false
                )
            )
            .withName("intake.scoreAmp()");
    }

    /**
     * This command spits using the {@link IntakeConstants#FROM_SHOOTER_ROLLER_SPEED slow roller speed}.
     * @return This command.
     */
    public Command receiveFromShooter() {
        return useState(
            IntakeConstants.DEPLOY_POSITION,
            IntakeConstants.FROM_SHOOTER_ROLLER_SPEED,
            IntakeConstants.FROM_SHOOTER_ROLLER_SPEED,
            false
        )
            .withName("intake.receiveFromShooter()");
    }

    /**
     * This command maintains the position stored in {@link #maintainAngle} unless it's null.
     * It should only be null if the position hasn't been set yet.
     * @return This command.
     */
    public Command maintainPosition() {
        return commandBuilder("intake.maintainPosition()")
            .onInitialize(() -> {
                if (MathUtil.angleModulus(maintainAngle) < 0.0) maintainAngle = 0.0;
            })
            .onExecute(() -> {
                if (maintainAngle < IntakeConstants.MINIMUM_PID_ANGLE) {
                    armLeftMotor.stopMotor();
                } else {
                    setValidPosition(maintainAngle);
                }
            });
    }

    public Command onDisable() {
        return commandBuilder()
            .onInitialize(() -> {
                armLeftMotor.setIdleMode(IdleMode.kCoast);
                armRightMotor.setIdleMode(IdleMode.kCoast);
            })
            .onExecute(() -> maintainAngle = armEncoder.getPosition())
            .onEnd(() -> {
                armLeftMotor.setIdleMode(IdleMode.kBrake);
                armRightMotor.setIdleMode(IdleMode.kBrake);
            })
            .ignoringDisable(true)
            .withName("intake.onDisable()");
    }
}
