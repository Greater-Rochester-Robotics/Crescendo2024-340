package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

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
import org.team340.lib.util.Math2;
import org.team340.robot.Constants.IntakeConstants;
import org.team340.robot.Constants.RobotMap;

// TODO Motion profiling (?)
// TODO Receive from feeder
// TODO Amp score

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
     * Deploys the intake and runs the roller motors to intake a note.
     */
    public Command deploy() {
        return commandBuilder("intake.deploy()")
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
     * Spits the note out of the intake in case it is stuck.
     */
    public Command spit() {
        return commandBuilder("intake.spit()")
            .onInitialize(() -> rollerUpperMotor.set(IntakeConstants.SPIT_ROLLER_SPEED))
            .onEnd(() -> rollerUpperMotor.stopMotor());
    }

    public Command spitSlow() {
        return commandBuilder("intake.spitSlow()")
            .onInitialize(() -> rollerUpperMotor.set(IntakeConstants.SPIT_SLOW_ROLLER_SPEED))
            .onEnd(() -> rollerUpperMotor.stopMotor());
    }

    /**
     * This command moves the intake up, and then scores it with a delay. This doesn't bring it back down.
     * @return This command.
     */
    public Command scoreAmp() {
        return commandBuilder("intake.scoreAmp()")
            .onExecute(() -> setValidPosition(IntakeConstants.SCORE_AMP_POSITION))
            .isFinished(() -> Math2.epsilonEquals(armEncoder.getPosition(), IntakeConstants.SCORE_AMP_POSITION))
            .onEnd((Boolean interrupted) -> {
                rollerUpperMotor.set(IntakeConstants.SCORE_AMP_ROLLER_SPEED);
                if (interrupted) {
                    targetAngle = armEncoder.getPosition();
                }
            })
            .andThen(waitSeconds(2), runOnce(() -> rollerUpperMotor.stopMotor()));
    }

    /**
     * This command maintains the position stored in {@link #targetAngle} unless it's null.
     * It should only be null if the position hasn't been set yet.
     * @return This command.
     */
    public Command maintainPosition() {
        return commandBuilder("intake.maintainPosition()")
            .onExecute(() -> {
                if (targetAngle != null && targetAngle < IntakeConstants.MINIMUM_PID_ANGLE) {
                    armLeftMotor.stopMotor();
                } else {
                    setValidPosition(targetAngle);
                }
            });
    }
}
