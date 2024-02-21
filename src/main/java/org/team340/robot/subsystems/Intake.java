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

/**
 * The intake subsystem. Intakes notes from the floor and scores
 * them in the amp, or pass them to the shooter.
 */
public class Intake extends GRRSubsystem {

    private final CANSparkFlex armLeftMotor;
    private final CANSparkFlex armRightMotor;
    private final CANSparkMax rollerUpperMotor;
    private final CANSparkMax rollerLowerMotor;
    private final SparkAbsoluteEncoder armEncoder;
    private final DigitalInput noteDetector;
    private final SparkPIDController armPID;

    private double armMaintain = 0.0;
    private double armTarget = 0.0;

    /**
     * Create the intake subsystem.
     */
    public Intake() {
        super("Intake");
        armLeftMotor = createSparkFlex("Arm Left Motor", RobotMap.INTAKE_ARM_LEFT_MOTOR, MotorType.kBrushless);
        armRightMotor = createSparkFlex("Arm Right Motor", RobotMap.INTAKE_ARM_RIGHT_MOTOR, MotorType.kBrushless);
        rollerUpperMotor = createSparkMax("Roller Upper Motor", RobotMap.INTAKE_ROLLER_UPPER_MOTOR, MotorType.kBrushless);
        rollerLowerMotor = createSparkMax("Roller Lower Motor", RobotMap.INTAKE_ROLLER_LOWER_MOTOR, MotorType.kBrushless);
        armEncoder = createSparkFlexAbsoluteEncoder("Arm Encoder", armLeftMotor, Type.kDutyCycle);
        noteDetector = createDigitalInput("Note Detector", RobotMap.INTAKE_NOTE_DETECTOR);
        armPID = armLeftMotor.getPIDController();
        armPID.setFeedbackDevice(armEncoder);

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
        builder.addDoubleProperty("armTarget", () -> armTarget, null);
        builder.addBooleanProperty("hasNote", this::hasNote, null);
    }

    /**
     * RReturns {@code true} when the beam break detects a note.
     */
    public boolean hasNote() {
        return noteDetector.get();
    }

    /**
     * Returns {@code true} if the intake arm is on target.
     * @param targetAngle
     * @return
     */
    private boolean armOnTarget(double targetAngle) {
        return Math2.epsilonEquals(MathUtil.angleModulus(armEncoder.getPosition()), targetAngle, IntakeConstants.CLOSED_LOOP_ERROR);
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
            armTarget = position;
        }
    }

    private Command useState(double angle, double upperSpeed, double lowerSpeed, boolean willFinish) {
        return commandBuilder()
            .onInitialize(() -> {
                rollerUpperMotor.set(upperSpeed);
                rollerLowerMotor.set(lowerSpeed);
            })
            .onExecute(() -> {
                if (
                    angle < IntakeConstants.MINIMUM_PID_ANGLE &&
                    MathUtil.angleModulus(armEncoder.getPosition()) < IntakeConstants.MINIMUM_PID_ANGLE
                ) {
                    armLeftMotor.stopMotor();
                } else {
                    setValidPosition(angle);
                }
                armMaintain = armEncoder.getPosition();
            })
            .isFinished(() -> willFinish && armOnTarget(angle))
            .onEnd(interrupted -> {
                if (!interrupted || armOnTarget(angle)) armMaintain = angle;
                rollerUpperMotor.stopMotor();
                rollerLowerMotor.stopMotor();
                armLeftMotor.stopMotor();
            });
    }

    /**
     * This moves the intake down to the intake position, but doesn't start the rollers.
     */
    public Command intakeDown() {
        return useState(IntakeConstants.DOWN_POSITION, 0, 0, true).withName("intake.intakeDown()");
    }

    /**
     * Deploys the intake and runs the roller motors to intake a note.
     */
    public Command intake() {
        return useState(IntakeConstants.DOWN_POSITION, IntakeConstants.INTAKE_ROLLER_SPEED, IntakeConstants.INTAKE_ROLLER_SPEED * .6, false)
            .withName("intake.intake()");
    }

    /**
     * This moves the intake arm to point straight up.
     */
    public Command retract() {
        return useState(IntakeConstants.RETRACT_POSITION, 0, 0, true).withName("intake.retract()");
    }

    public Command toUprightPosition() {
        return useState(IntakeConstants.UPRIGHT_POSITION, 0.0, 0.0, true);
    }

    /**
     * Retracts the intake into the frame perimeter and stops the rollers.
     */
    public Command toSafePosition() {
        return useState(IntakeConstants.SAFE_POSITION, 0, 0, true).withName("intake.toSafePosition()");
    }

    public Command toSpitPosition() {
        return useState(IntakeConstants.SPIT_POSITION, 0, 0, true).withName("intake.toSpitPosition()");
    }

    /**
     * This command moves the intake to the {@link IntakeConstants#SCORE_AMP_POSITION SCORE_AMP_POSITION}
     * and ends once the position has been reached.
     */
    public Command scoreAmpPosition() {
        return useState(IntakeConstants.SCORE_AMP_POSITION, 0, 0, true).withName("intake.scoreAmpPosition()");
    }

    /**
     * This command moves the intake up, and then scores it with a delay. This doesn't bring it back down.
     */
    public Command scoreAmp() {
        return scoreAmpPosition()
            .andThen(
                useState(
                    IntakeConstants.SCORE_AMP_POSITION,
                    IntakeConstants.SCORE_AMP_ROLLER_SPEED_UPPER,
                    IntakeConstants.SCORE_AMP_ROLLER_SPEED_LOWER,
                    false
                )
            )
            .withName("intake.scoreAmp()");
    }

    /**
     * This command spits using the {@link IntakeConstants#FROM_SHOOTER_ROLLER_SPEED slow roller speed}.
     */
    public Command receiveFromShooter() {
        return useState(
            IntakeConstants.DOWN_POSITION,
            IntakeConstants.FROM_SHOOTER_ROLLER_SPEED,
            IntakeConstants.FROM_SHOOTER_ROLLER_SPEED,
            false
        )
            .withName("intake.receiveFromShooter()");
    }

    /**
     * This spits the note out of the intake, this doesn't end of it's own accord.
     */
    public Command spit() {
        return useState(IntakeConstants.SPIT_POSITION, IntakeConstants.SPIT_ROLLER_SPEED, IntakeConstants.SPIT_ROLLER_SPEED, false)
            .withName("intake.spit()");
    }

    /**
     * This command maintains the position stored in {@link #armMaintain} unless it's null.
     * It should only be null if the position hasn't been set yet.
     */
    public Command maintainPosition() {
        return commandBuilder("intake.maintainPosition()")
            .onInitialize(() -> {
                if (armMaintain > Math.PI && armMaintain < 3 * Math2.HALF_PI) armMaintain = Math.PI; else if (
                    MathUtil.angleModulus(armMaintain) < 0.0
                ) armMaintain = 0.0;
            })
            .onExecute(() -> {
                if (armMaintain < IntakeConstants.MINIMUM_PID_ANGLE) {
                    armLeftMotor.stopMotor();
                } else {
                    setValidPosition(armMaintain);
                }
            });
    }

    /**
     * This command sets the pivot motors to coast mode, and then back to break mode after it ends,
     * it should be called when the robot is disabled.
     */
    public Command onDisable() {
        return commandBuilder()
            .onInitialize(() -> {
                armLeftMotor.setIdleMode(IdleMode.kCoast);
                armRightMotor.setIdleMode(IdleMode.kCoast);
            })
            .onExecute(() -> armMaintain = armEncoder.getPosition())
            .onEnd(() -> {
                armLeftMotor.setIdleMode(IdleMode.kBrake);
                armRightMotor.setIdleMode(IdleMode.kBrake);
            })
            .ignoringDisable(true)
            .withName("intake.onDisable()");
    }
}
