package org.team340.robot.subsystems;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import org.team340.lib.GRRSubsystem;
import org.team340.robot.Constants.IntakeConstants;
import org.team340.robot.Constants.RobotMap;

// TODO Motion profiling (?)
// TODO Note detector
// TODO Receive from feeder
// TODO Amp score
// TODO Coast mode on disable

// TODO Docs
/**
 * The Intake subsystem.
 */
public class Intake extends GRRSubsystem {

    private final CANSparkMax armLeftMotor;
    private final CANSparkMax armRightMotor;
    private final CANSparkMax rollerUpperMotor;
    private final CANSparkMax rollerLowerMotor;
    private final SparkAbsoluteEncoder armEncoder;
    private final SparkPIDController armPID;

    public Intake() {
        super("Intake");
        armLeftMotor = createSparkMax("Arm Left Motor", RobotMap.INTAKE_ARM_LEFT_MOTOR, MotorType.kBrushless);
        armRightMotor = createSparkMax("Arm Right Motor", RobotMap.INTAKE_ARM_RIGHT_MOTOR, MotorType.kBrushless);
        rollerUpperMotor = createSparkMax("Roller Upper Motor", RobotMap.INTAKE_ROLLER_UPPER_MOTOR, MotorType.kBrushless);
        rollerLowerMotor = createSparkMax("Roller Lower Motor", RobotMap.INTAKE_ROLLER_LOWER_MOTOR, MotorType.kBrushless);
        armEncoder = createSparkMaxAbsoluteEncoder("Arm Encoder", armLeftMotor, Type.kDutyCycle);
        armPID = armLeftMotor.getPIDController();

        IntakeConstants.ARM_LEFT_MOTOR_CONFIG.apply(armLeftMotor);
        IntakeConstants.ARM_RIGHT_MOTOR_CONFIG.apply(armRightMotor);
        IntakeConstants.ROLLER_UPPER_MOTOR_CONFIG.apply(rollerUpperMotor);
        IntakeConstants.ROLLER_LOWER_MOTOR_CONFIG.apply(rollerLowerMotor);
        IntakeConstants.ARM_ENCODER_CONFIG.apply(armLeftMotor, armEncoder);
        IntakeConstants.ARM_MOTOR_PID_CONFIG.apply(armLeftMotor, armPID);
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
}
