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
import org.team340.robot.Constants.PivotConstants;
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
        builder.addDoubleProperty("armMaintain", () -> armMaintain, null);
        builder.addBooleanProperty("hasNote", this::hasNote, null);
    }

    /**
     * Returns {@code true} when the beam break detects a note.
     */
    public boolean hasNote() {
        return noteDetector.get();
    }

    /**
     * Returns {@code true} if the arm is at the specified position.
     * @param position The position to check for in radians.
     */
    private boolean atPosition(double position) {
        return Math2.epsilonEquals(MathUtil.angleModulus(armEncoder.getPosition()), position, IntakeConstants.CLOSED_LOOP_ERROR);
    }

    /**
     * Sets the {@link #armPID} to go to the specified position if it is valid
     * (within the intake {@link IntakeConstants#MINIMUM_ANGLE minimum} and
     * {@link IntakeConstants#MAXIMUM_ANGLE maximum} angles).
     * @param position The position to set.
     */
    private void setValidPosition(double position) {
        if (position < IntakeConstants.MINIMUM_ANGLE || position > IntakeConstants.MAXIMUM_ANGLE) {
            DriverStation.reportWarning(
                "Invalid intake arm position. " +
                Math2.formatRadians(position) +
                " degrees is not between " +
                Math2.formatRadians(PivotConstants.MINIMUM_ANGLE) +
                " and " +
                Math2.formatRadians(PivotConstants.MAXIMUM_ANGLE),
                false
            );
        } else {
            armTarget = position;
            if (
                position < IntakeConstants.MINIMUM_PID_ANGLE &&
                MathUtil.angleModulus(armEncoder.getPosition()) < IntakeConstants.MINIMUM_PID_ANGLE
            ) {
                armLeftMotor.stopMotor();
            } else {
                armPID.setReference(position, ControlType.kPosition);
            }
        }
    }

    /**
     * Sets the state of the intake.
     * @param position The position for the arm to move to in radians.
     * @param upperSpeed The duty cycle of the upper roller.
     * @param lowerSpeed The duty cycle of the lower roller.
     * @param willFinish If {@code true}, the command will end after the arm reaches the specified angle.
     */
    private Command useState(double position, double upperSpeed, double lowerSpeed, boolean willFinish) {
        return commandBuilder(
            "intake.useState(" + Math2.formatRadians(position) + ", " + upperSpeed + ", " + lowerSpeed + ", " + willFinish + ")"
        )
            .onInitialize(() -> {
                rollerUpperMotor.set(upperSpeed);
                rollerLowerMotor.set(lowerSpeed);
            })
            .onExecute(() -> {
                setValidPosition(position);
                armMaintain = armEncoder.getPosition();
            })
            .isFinished(() -> willFinish && atPosition(position))
            .onEnd(interrupted -> {
                if (!interrupted || atPosition(position)) armMaintain = position;
                rollerUpperMotor.stopMotor();
                rollerLowerMotor.stopMotor();
                armLeftMotor.stopMotor();
            });
    }

    /**
     * Moves to the down position. Runs until the arm is at the position.
     */
    public Command downPosition() {
        return useState(IntakeConstants.DOWN_POSITION, 0, 0, true).withName("intake.downPosition()");
    }

    /**
     * Moves to the retract position. Runs until the arm is at the position.
     */
    public Command retractPosition() {
        return useState(IntakeConstants.RETRACT_POSITION, 0, 0, true).withName("intake.retractPosition()");
    }

    /**
     * Moves to the safe position. Runs until the arm is at the position.
     */
    public Command safePosition() {
        return useState(IntakeConstants.SAFE_POSITION, 0, 0, true).withName("intake.safePosition()");
    }

    /**
     * Moves to the upright position. Runs until the arm is at the position.
     */
    public Command uprightPosition() {
        return useState(IntakeConstants.UPRIGHT_POSITION, 0.0, 0.0, true).withName("intake.uprightPosition()");
    }

    /**
     * Moves to the spit position. Runs until the arm is at the position.
     */
    public Command spitPosition() {
        return useState(IntakeConstants.BARF_POSITION, 0, 0, true).withName("intake.spitPosition()");
    }

    /**
     * Moves to the amp position. Runs until the arm is at the position.
     */
    public Command ampPosition() {
        return useState(IntakeConstants.SCORE_AMP_POSITION, 0, 0, true).withName("intake.ampPosition()");
    }

    /**
     * Intakes from the ground. Does not end.
     */
    public Command intake() {
        return useState(IntakeConstants.DOWN_POSITION, IntakeConstants.INTAKE_ROLLER_SPEED, IntakeConstants.INTAKE_ROLLER_SPEED * .6, false)
            .withName("intake.intake()");
    }

    /**
     * Receives a note back from the shooter. Does not end.
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
     * Scores in the amp. Does not end.
     */
    public Command scoreAmp() {
        return ampPosition()
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
     * Spits the note out of the intake. Does not end.
     */
    public Command barf() {
        return useState(IntakeConstants.BARF_POSITION, IntakeConstants.BARF_ROLLER_SPEED, IntakeConstants.BARF_ROLLER_SPEED, false)
            .withName("intake.barf()");
    }

    /**
     * Maintains the last set position of the arm.
     */
    public Command maintainPosition() {
        return commandBuilder("intake.maintainPosition()")
            .onInitialize(() -> {
                if (armMaintain > Math.PI && armMaintain < 3 * Math2.HALF_PI) armMaintain = Math.PI; else if (
                    MathUtil.angleModulus(armMaintain) < 0.0
                ) armMaintain = 0.0;
            })
            .onExecute(() -> setValidPosition(armMaintain));
    }

    /**
     * Should be called when disabled, and cancelled when enabled.
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
