package org.team340.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.GRRSubsystem;
import org.team340.lib.util.Math2;
import org.team340.robot.Constants.IntakeConstants;
import org.team340.robot.Constants.RobotMap;

// TODO Motion profiling (?)
// TODO Note detector
// TODO Receive from feeder
// TODO Amp score
// TODO Coast mode on disable

// TODO Docs
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

    public Intake() {
        super("Intake");
        armLeftMotor = createSparkFlex("Arm Left Motor", RobotMap.INTAKE_ARM_LEFT_MOTOR, MotorType.kBrushless);
        armRightMotor = createSparkFlex("Arm Right Motor", RobotMap.INTAKE_ARM_RIGHT_MOTOR, MotorType.kBrushless);
        rollerUpperMotor = createSparkMax("Roller Upper Motor", RobotMap.INTAKE_ROLLER_UPPER_MOTOR, MotorType.kBrushless);
        rollerLowerMotor = createSparkMax("Roller Lower Motor", RobotMap.INTAKE_ROLLER_LOWER_MOTOR, MotorType.kBrushless);
        armEncoder = createSparkFlexAbsoluteEncoder("Arm Encoder", armLeftMotor, Type.kDutyCycle);
        armPID = armLeftMotor.getPIDController();

        IntakeConstants.ARM_LEFT_MOTOR_CONFIG.apply(armLeftMotor);
        IntakeConstants.ARM_RIGHT_MOTOR_CONFIG.apply(armRightMotor);
        IntakeConstants.ROLLER_UPPER_MOTOR_CONFIG.apply(rollerUpperMotor);
        IntakeConstants.ROLLER_LOWER_MOTOR_CONFIG.apply(rollerLowerMotor);
        IntakeConstants.ARM_ENCODER_CONFIG.apply(armLeftMotor, armEncoder);
        IntakeConstants.ARM_MOTOR_PID_CONFIG.apply(armLeftMotor, armPID);
    }

    /**
     * Sets the {@link #armPID} to go to the specified angle if it is valid
     * (within the intake {@link IntakeConstants#MINIMUM_ANGLE minimum} and
     * {@link IntakeConstants#MAXIMUM_ANGLE maximum} angles).
     * @param position The angle to set.
     */
    private void setValidPosition(double position) {
        if (position > IntakeConstants.MINIMUM_ANGLE && position < IntakeConstants.MAXIMUM_ANGLE) {
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
                armPID.setReference(IntakeConstants.DEPLOY_POSITION, ControlType.kPosition);
            })
            .onEnd(() -> {
                rollerUpperMotor.stopMotor();
                armLeftMotor.stopMotor();
            });
    }

    /**
     * Retracts the intake into the frame perimeter and stops the rollers.
     */
    public Command retract() {
        return commandBuilder("intake.retract()")
            .onInitialize(() -> rollerUpperMotor.stopMotor())
            .onExecute(() -> armPID.setReference(0, ControlType.kPosition, 0));
    }

    /**
     * Spits the note out of the intake in case it is stuck.
     */
    public Command spit() {
        return commandBuilder("intake.spit()")
            .onInitialize(() -> rollerUpperMotor.set(IntakeConstants.SPIT_ROLLER_SPEED))
            .onEnd(() -> rollerUpperMotor.stopMotor());
    }

    /**
     * This command moves the intake up, and then scores it with a delay. This doesn't bring it back down.
     * @return This command.
     */
    public Command scoreAmp() {
        return commandBuilder()
            .onExecute(() -> setValidPosition(IntakeConstants.INTAKE_AMP_SCORE_POSITION))
            .isFinished(() -> Math2.epsilonEquals(armEncoder.getPosition(), IntakeConstants.INTAKE_AMP_SCORE_POSITION))
            .onEnd(() -> rollerUpperMotor.set(IntakeConstants.ROLLER_SCORE_AMP_SPEED))
            .andThen(waitSeconds(2), runOnce(() -> rollerUpperMotor.stopMotor()));
    }
}
