package org.team340.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.GRRSubsystem;
import org.team340.lib.commands.CommandBuilder;
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

    private Double targetAngle = null;

    public Intake() {
        super("Intake");
        armLeftMotor = createSparkFlex("Arm Left Motor", RobotMap.INTAKE_ARM_LEFT_MOTOR, MotorType.kBrushless);
        armRightMotor = createSparkFlex("Arm Right Motor", RobotMap.INTAKE_ARM_RIGHT_MOTOR, MotorType.kBrushless);
        rollerUpperMotor = createSparkMax("Roller Upper Motor", RobotMap.INTAKE_ROLLER_UPPER_MOTOR, MotorType.kBrushless);
        rollerLowerMotor = createSparkMax("Roller Lower Motor", RobotMap.INTAKE_ROLLER_LOWER_MOTOR, MotorType.kBrushless);
        armEncoder = createSparkFlexAbsoluteEncoder("Arm Encoder", armLeftMotor, Type.kDutyCycle);
        armPID = armLeftMotor.getPIDController();
        noteDetector = new DigitalInput(RobotMap.INTAKE_NOTE_DETECTOR);

        IntakeConstants.ArmConfigs.LEFT_MOTOR.apply(armLeftMotor);
        IntakeConstants.ArmConfigs.RIGHT_MOTOR.apply(armRightMotor);
        IntakeConstants.RollerConfigs.UPPER_MOTOR.apply(rollerUpperMotor);
        IntakeConstants.RollerConfigs.LOWER_MOTOR.apply(rollerLowerMotor);
        IntakeConstants.ArmConfigs.ENCODER.apply(armLeftMotor, armEncoder);
        IntakeConstants.ArmConfigs.PID.apply(armLeftMotor, armPID);
    }

    /**
     * Returns {@code true} when the note detector is detecting a note.
     */
    public boolean getNoteDetector() {
        return !noteDetector.get();
    }

    /**
     * Set idle mode of pivot motor to brake or coast.
     * @param brakeOn If idle mode should be set to brake.
     */
    public void setBrakeMode(boolean brakeOn) {
        armLeftMotor.setIdleMode(brakeOn ? IdleMode.kBrake : IdleMode.kCoast);
        armRightMotor.setIdleMode(brakeOn ? IdleMode.kBrake : IdleMode.kCoast);
    }

    /**
     * Sets the {@link #armPID} to go to the specified angle if it is valid
     * (within the intake {@link IntakeConstants#MINIMUM_ANGLE minimum} and
     * {@link IntakeConstants#MAXIMUM_ANGLE maximum} angles).
     * @param position The angle to set.
     */
    private void setValidPosition(Double position) {
        if (position == null) {
            // This is to check if the position hasn't been set yet.
            return;
        } else if (position < IntakeConstants.MINIMUM_ANGLE || position > IntakeConstants.MAXIMUM_ANGLE) {
            DriverStation.reportWarning(
                "The angle " +
                position +
                " is not valid. is must be within " +
                IntakeConstants.MAXIMUM_ANGLE +
                " and " +
                IntakeConstants.MINIMUM_ANGLE +
                ".",
                null
            );
        } else {
            targetAngle = position;
            armPID.setReference(position, ControlType.kPosition);
        }
    }

    /**
     * This moves the intake down to the intake position, but doesn't start the rollers.
     * @return This command.
     */
    public Command intakeDown() {
        return commandBuilder("intake.intakeDown()")
            .onInitialize(() -> setValidPosition(IntakeConstants.DEPLOY_POSITION))
            .isFinished(() -> armEncoder.getPosition() < IntakeConstants.DEPLOY_POSITION + IntakeConstants.CLOSED_LOOP_ERROR)
            .onEnd(() -> armLeftMotor.stopMotor());
    }

    /**
     * Deploys the intake and runs the roller motors to intake a note.
     */
    public Command intake() {
        return commandBuilder("intake.intake()")
            .onInitialize(() -> rollerUpperMotor.set(IntakeConstants.INTAKE_ROLLER_SPEED))
            .onExecute(() -> {
                setValidPosition(IntakeConstants.DEPLOY_POSITION);
            })
            .onEnd(interrupted -> {
                rollerUpperMotor.stopMotor();
                armLeftMotor.stopMotor();
                if (interrupted) {
                    targetAngle = armEncoder.getPosition();
                }
            });
    }

    /**
     * This moves the intake arm to point straight up.
     * @return This command.
     */
    public Command retract() {
        return commandBuilder("intake.retract()")
            .onInitialize(() -> rollerUpperMotor.stopMotor())
            .onExecute(() -> setValidPosition(IntakeConstants.STRAIGHT_UP_POSITION))
            .onEnd(interrupted -> {
                if (interrupted) {
                    targetAngle = armEncoder.getPosition();
                }
            });
    }

    /**
     * Retracts the intake into the frame perimeter and stops the rollers.
     */
    public Command toSafePosition() {
        return commandBuilder("intake.toSafePosition()")
            .onInitialize(() -> rollerUpperMotor.stopMotor())
            .onExecute(() -> setValidPosition(IntakeConstants.SAFE_POSITION))
            .onEnd((Boolean interrupted) -> {
                if (interrupted) {
                    targetAngle = armEncoder.getPosition();
                }
            });
    }

    /**
     * This command moves the intake to the {@link IntakeConstants#SCORE_AMP_POSITION SCORE_AMP_POSITION}
     * and ends once the position has been reached.
     * @return This command.
     */
    public Command scoreAmpPosition() {
        return commandBuilder("intake.scoreAmpPosition()")
            .onExecute(() -> setValidPosition(IntakeConstants.SCORE_AMP_POSITION))
            .isFinished(() -> Math2.epsilonEquals(armEncoder.getPosition(), IntakeConstants.SCORE_AMP_POSITION))
            .onEnd(interrupted -> {
                if (interrupted) {
                    targetAngle = armEncoder.getPosition();
                }
            });
    }

    /**
     * This command moves the intake up, and then scores it with a delay. This doesn't bring it back down.
     * @return This command.
     */
    public Command scoreAmp() {
        return scoreAmpPosition()
            .andThen(
                commandBuilder("intake.scoreAmp().scoring")
                    .onInitialize(() -> rollerUpperMotor.set(IntakeConstants.SCORE_AMP_ROLLER_SPEED))
                    .onEnd(() -> rollerUpperMotor.stopMotor())
                    .withTimeout(IntakeConstants.AMP_SCORING_TIMEOUT)
            );
    }

    /**
     * This command maintains the position stored in {@link #targetAngle} unless it's null.
     * It should only be null if the position hasn't been set yet.
     * @return This command.
     */
    public CommandBuilder maintainPosition() {
        return commandBuilder("intake.maintainPosition()")
            .onExecute(() -> {
                if (targetAngle != null && targetAngle < IntakeConstants.MINIMUM_PID_ANGLE) {
                    armLeftMotor.stopMotor();
                } else {
                    setValidPosition(targetAngle);
                }
            });
    }

    /**
     * Spits the note out of the intake in case it is stuck.
     */
    public Command spit() {
        return maintainPosition()
            .onInitialize(() -> rollerUpperMotor.set(IntakeConstants.SPIT_ROLLER_SPEED))
            .onEnd(() -> rollerUpperMotor.stopMotor())
            .withName("intake.spit()");
    }

    /**
     * This command spits using the {@link IntakeConstants#SPIT_SLOW_ROLLER_SPEED slow roller speed}.
     * @return This command.
     */
    public Command spitSlow() {
        return maintainPosition()
            .onInitialize(() -> rollerUpperMotor.set(IntakeConstants.SPIT_SLOW_ROLLER_SPEED))
            .onEnd(() -> rollerUpperMotor.stopMotor())
            .withName("intake.spitSlow()");
    }
}
