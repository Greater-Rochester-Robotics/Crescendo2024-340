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
import java.util.function.Supplier;
import org.team340.lib.GRRSubsystem;
import org.team340.lib.util.Math2;
import org.team340.robot.Constants;
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
        return Math2.epsilonEquals(MathUtil.angleModulus(armEncoder.getPosition()), position, IntakeConstants.CLOSED_LOOP_ERR);
    }

    /**
     * Sets the {@link #armPID arms' PIDs} to go to the specified position if it is valid
     * (within the intake {@link IntakeConstants#MIN_POS minimum} and
     * {@link IntakeConstants#MAX_POS maximum} angles).
     * @param position The position to set.
     */
    private void applyPosition(double position) {
        if (position < IntakeConstants.MIN_POS || position > IntakeConstants.MAX_POS) {
            DriverStation.reportWarning(
                "Invalid intake arm position. " +
                Math2.formatRadians(position) +
                " degrees is not between " +
                Math2.formatRadians(PivotConstants.MIN_POS) +
                " and " +
                Math2.formatRadians(PivotConstants.MAX_POS),
                false
            );
        } else {
            armTarget = position;
            if (
                position < IntakeConstants.PID_INACTIVE_POSITION &&
                MathUtil.angleModulus(armEncoder.getPosition()) < IntakeConstants.PID_INACTIVE_POSITION
            ) {
                armLeftMotor.stopMotor();
                armRightMotor.stopMotor();
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
                applyPosition(position);
                armMaintain = armEncoder.getPosition();
            })
            .isFinished(() -> willFinish && atPosition(position))
            .onEnd(interrupted -> {
                if (!interrupted || atPosition(position)) armMaintain = position;
                armLeftMotor.stopMotor();
                armRightMotor.stopMotor();
                rollerUpperMotor.stopMotor();
                rollerLowerMotor.stopMotor();
            });
    }

    /**
     * Moves to the down position. Runs until the arm is at the position.
     */
    public Command downPosition() {
        return useState(IntakeConstants.DOWN_POSITION, 0, 0, true).withName("intake.downPosition()");
    }

    /**
     * Moves to the safe position. Runs until the arm is at the position.
     */
    public Command safePosition() {
        return useState(IntakeConstants.SAFE_POSITION, 0, 0, true).withName("intake.safePosition()");
    }

    /**
     * Moves to the retract position. Runs until the arm is at the position.
     */
    public Command retractPosition() {
        return useState(IntakeConstants.RETRACT_POSITION, 0, 0, true).withName("intake.retractPosition()");
    }

    /**
     * Moves to the upright position. Runs until the arm is at the position.
     */
    public Command uprightPosition() {
        return useState(IntakeConstants.UPRIGHT_POSITION, 0.0, 0.0, true).withName("intake.uprightPosition()");
    }

    /**
     * Moves to the barf position. Runs until the arm is at the position.
     */
    public Command barfPosition() {
        return useState(IntakeConstants.BARF_POSITION, 0, 0, true).withName("intake.barfPosition()");
    }

    /**
     * Moves to the poop position. Runs until the arm is at the position.
     */
    public Command poopPosition() {
        return useState(IntakeConstants.POOP_POSITION, 0, 0, true).withName("intake.poopPosition()");
    }

    /**
     * Moves to the amp position. Runs until the arm is at the position.
     */
    public Command ampPosition() {
        return useState(IntakeConstants.AMP_POSITION, 0, 0, true).withName("intake.ampPosition()");
    }

    /**
     * Intakes from the ground. Does not end.
     */
    public Command intake() {
        return useState(IntakeConstants.DOWN_POSITION, IntakeConstants.INTAKE_SPEED, IntakeConstants.INTAKE_SPEED * .6, false)
            .withName("intake.intake()");
    }

    /**
     * Receives a note back from the shooter for amp scoring. Does not end.
     */
    public Command ampHandoff() {
        return useState(IntakeConstants.DOWN_POSITION, IntakeConstants.AMP_HANDOFF_SPEED, IntakeConstants.AMP_HANDOFF_SPEED, false)
            .withName("intake.ampHandoff()");
    }

    /**
     * Scores in the amp. Does not end.
     */
    public Command scoreAmp() {
        return ampPosition()
            .andThen(useState(IntakeConstants.AMP_POSITION, IntakeConstants.AMP_UPPER_SPEED, IntakeConstants.AMP_LOWER_SPEED, false))
            .withName("intake.scoreAmp()");
    }

    /**
     * Barfs the note out of the intake. Does not end.
     */
    public Command barf() {
        return useState(IntakeConstants.BARF_POSITION, IntakeConstants.BARF_SPEED, IntakeConstants.BARF_SPEED, false)
            .withName("intake.barf()");
    }

    /**
     * Poops the note out of the intake. Does not end.
     */
    public Command poop() {
        return useState(IntakeConstants.POOP_POSITION, IntakeConstants.POOP_SPEED, IntakeConstants.POOP_SPEED, false)
            .withName("intake.poop()");
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
            .onExecute(() -> applyPosition(armMaintain));
    }

    /**
     * Runs rollers at a set speed to intake manually.
     */
    public Command intakeOverride() {
        return commandBuilder("intake.intakeOverride()")
            .onExecute(() -> {
                rollerUpperMotor.set(IntakeConstants.OVERRIDE_INTAKE_SPEED);
                rollerLowerMotor.set(IntakeConstants.OVERRIDE_INTAKE_SPEED);
            })
            .onEnd(() -> {
                rollerUpperMotor.stopMotor();
                rollerLowerMotor.stopMotor();
            });
    }

    /**
     * Drives the arms manually. Will hold position.
     * @param speed The speed of the arms in radians/second.
     */
    public Command driveArmManual(Supplier<Double> speed) {
        return commandBuilder("intake.driveArmManual()")
            .onExecute(() -> {
                double diff = speed.get() * Constants.PERIOD;
                double armPos = armEncoder.getPosition();
                if (armPos < IntakeConstants.MIN_POS) {
                    diff = Math.max(diff, 0.0);
                } else if (armPos > IntakeConstants.MAX_POS) {
                    diff = Math.min(diff, 0.0);
                }

                armMaintain += diff;
                applyPosition(armMaintain);
            });
    }

    /**
     * Should be called when disabled, and cancelled when enabled.
     */
    public Command onDisable() {
        return commandBuilder()
            .onInitialize(() -> {
                armLeftMotor.setIdleMode(IdleMode.kCoast);
                armRightMotor.setIdleMode(IdleMode.kCoast);
                armLeftMotor.stopMotor();
                armRightMotor.stopMotor();
                rollerUpperMotor.stopMotor();
                rollerLowerMotor.stopMotor();
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
