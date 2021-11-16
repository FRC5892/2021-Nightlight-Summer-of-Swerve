// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.simulationWrappers.SparkMaxWrapper;

/** Add your docs here. */
public class SwerveModule {
	private CANSparkMax driveMotor;
	private CANSparkMax steerMotor;
	private CANPIDController drivePIDController;
	private CANPIDController steerPIDController;
	private CANEncoder driveEncoder;
	private CANEncoder steerEncoder;
	private DutyCycleEncoder steerLampreyEncoder;
	private SwerveModuleState swerveState = new SwerveModuleState(180, new Rotation2d(1));

	public CANSparkMax configuredCANSparkMax(int ID) {
		CANSparkMax sparkMax = new SparkMaxWrapper(ID, MotorType.kBrushless);
		sparkMax.restoreFactoryDefaults();
		sparkMax.setInverted(false);
		sparkMax.setIdleMode(IdleMode.kBrake);
		sparkMax.setSmartCurrentLimit(60);
		return sparkMax;
	}

	public SwerveModule(int driveMotorID, int turnMotorID, int lampreyID) {
		driveMotor = configuredCANSparkMax(driveMotorID);
		driveEncoder = driveMotor.getEncoder();
		// driveEncoder.setPositionConversionFactor(Constants.kSwerveDriveTrain.kDrive.kEncoderConversionFactor);
		// driveEncoder.setVelocityConversionFactor(Constants.kSwerveDriveTrain.kDrive.kEncoderConversionFactor / 60);

		drivePIDController = driveMotor.getPIDController();
		drivePIDController.setP(Constants.kSwerveDriveTrain.kDrive.kP);
		drivePIDController.setI(Constants.kSwerveDriveTrain.kDrive.kI);
		drivePIDController.setD(Constants.kSwerveDriveTrain.kDrive.kD);
		steerLampreyEncoder = new DutyCycleEncoder(lampreyID);
		steerMotor = configuredCANSparkMax(turnMotorID);
		steerEncoder = steerMotor.getEncoder();
		// steerEncoder.setPositionConversionFactor(Constants.kSwerveDriveTrain.kSteer.kEncoderConversionFactor);
		// steerEncoder.setVelocityConversionFactor(Constants.kSwerveDriveTrain.kSteer.kEncoderConversionFactor / 60);
		// steerEncoder.setPosition(steerLampreyEncoder.get()/Constants.kSwerveDriveTrain.kSteer.kEncoderConversionFactor);
		steerPIDController = steerMotor.getPIDController();
		steerPIDController.setP(Constants.kSwerveDriveTrain.kDrive.kP);
		steerPIDController.setI(Constants.kSwerveDriveTrain.kDrive.kI);
		steerPIDController.setD(Constants.kSwerveDriveTrain.kDrive.kD);
	}

	public double getLampreyPosition() {
		return steerLampreyEncoder.get()*-0.0027+1.8734;
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(driveMotor.getEncoder().getVelocity(), new Rotation2d(getSteerPosition()));
	}

	public double getDrivePosition() {
		return driveEncoder.getPosition()*Constants.kSwerveDriveTrain.kDrive.kEncoderConversionFactor;
	}

	public double getSteerPosition() {
		return (steerEncoder.getPosition()*Constants.kSwerveDriveTrain.kSteer.kEncoderConversionFactor);
	}

	public void setDesiredState(SwerveModuleState desiredState) {
		swerveState = desiredState;
		SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getSteerPosition()));
		drivePIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
		steerPIDController
				.setReference(state.angle.getDegrees(), ControlType.kVelocity, 0,
						new SimpleMotorFeedforward(Constants.kSwerveDriveTrain.kSteer.kS,
								Constants.kSwerveDriveTrain.kSteer.kV, Constants.kSwerveDriveTrain.kSteer.kA)
										.calculate(state.angle.getDegrees()));
	}

	public void setMotors(double speed) {
		driveMotor.set(speed);
		steerMotor.set(speed);
	}

	public void resetEncoders() {
		driveEncoder.setPosition(0);
		steerEncoder.setPosition(0);
	}

	public SwerveModuleState getDesiredSteerState() {
		return swerveState;
	}

	public void stop() {
		driveMotor.stopMotor();
		steerMotor.stopMotor();
	}
}
