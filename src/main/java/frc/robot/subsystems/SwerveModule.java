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
	private SwerveModuleState desiredSwerveState = new SwerveModuleState(0, new Rotation2d(0));
	private SwerveModuleState swerveState = new SwerveModuleState(0, new Rotation2d(0));
	private double lampreyOffset;

	public CANSparkMax configuredCANSparkMax(int ID) {
		CANSparkMax sparkMax = new SparkMaxWrapper(ID, MotorType.kBrushless);
		sparkMax.restoreFactoryDefaults();
		sparkMax.setInverted(false);
		sparkMax.setIdleMode(IdleMode.kBrake);
		sparkMax.setSmartCurrentLimit(60);
		return sparkMax;
	}

	public SwerveModule(int driveMotorID, int turnMotorID, int lampreyID, double lO) {
		lampreyOffset = lO;
		driveMotor = configuredCANSparkMax(driveMotorID);
		driveEncoder = driveMotor.getEncoder();
		drivePIDController = driveMotor.getPIDController();
		drivePIDController.setP(Constants.kSwerveDriveTrain.kDrive.kP);
		drivePIDController.setI(Constants.kSwerveDriveTrain.kDrive.kI);
		drivePIDController.setD(Constants.kSwerveDriveTrain.kDrive.kD);
		driveMotor.burnFlash();

		steerLampreyEncoder = new DutyCycleEncoder(lampreyID);

		steerMotor = configuredCANSparkMax(turnMotorID);
		steerEncoder = steerMotor.getEncoder();
		steerEncoder.setPosition(getLampreyPosition() / Constants.kSwerveDriveTrain.kSteer.kEncoderConversionFactor);
		steerPIDController = steerMotor.getPIDController();
		steerPIDController.setP(Constants.kSwerveDriveTrain.kSteer.kP);
		steerPIDController.setI(Constants.kSwerveDriveTrain.kSteer.kI);
		steerPIDController.setD(Constants.kSwerveDriveTrain.kSteer.kD);
		steerMotor.burnFlash();
		setPositionZero();

	}

	public double getLampreyPosition() {
		return (steerLampreyEncoder.get() * 125.664) - 6.28319 + lampreyOffset;
	}

	public SwerveModuleState getState() {
		return swerveState;
	}

	public SwerveModuleState getDesiredState() {
		return desiredSwerveState;
	}

	public double getDrivePosition() {
		return driveEncoder.getPosition() * Constants.kSwerveDriveTrain.kDrive.kEncoderConversionFactor;
	}

	public double getDriveVelocity() {
		return driveEncoder.getVelocity() * (Constants.kSwerveDriveTrain.kDrive.kEncoderConversionFactor / 60);
	}

	public double getSteerPosition() {
		return steerEncoder.getPosition() * Constants.kSwerveDriveTrain.kSteer.kEncoderConversionFactor;
	}

	public void setDesiredState(SwerveModuleState desiredState) {
		desiredSwerveState = desiredState;
		swerveState = SwerveModuleState.optimize(desiredState, new Rotation2d(getSteerPosition()));
		drivePIDController.setReference(
				swerveState.speedMetersPerSecond / (Constants.kSwerveDriveTrain.kDrive.kEncoderConversionFactor / 60),
				ControlType.kVelocity);
		// Conversion factor divided by 60 to convert from rotations per minute to meters per second
		steerPIDController.setReference(
				(swerveState.angle.getRadians() - new Rotation2d(getSteerPosition()).getRadians() + getSteerPosition())
						/ Constants.kSwerveDriveTrain.kSteer.kEncoderConversionFactor,
				ControlType.kPosition);
	}

	public void setMotors(double speed) {
		driveMotor.set(speed);
		steerMotor.set(speed);
	}

	public void resetEncoders() {
		driveEncoder.setPosition(0);
		steerEncoder.setPosition(0);
	}

	public void setPositionZero() {
		steerMotor.set(0);
	}

	public void stop() {
		driveMotor.stopMotor();
		steerMotor.stopMotor();
	}
}
