// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
	private DutyCycleEncoder steerLampreyEncoder;

	public CANSparkMax configuredCANSparkMax(int ID) {
		CANSparkMax sparkMax = new SparkMaxWrapper(ID, MotorType.kBrushless);
		sparkMax.restoreFactoryDefaults();
		sparkMax.setInverted(false);
		sparkMax.setIdleMode(IdleMode.kBrake);
		sparkMax.setSmartCurrentLimit(60);
		sparkMax.burnFlash();
		return sparkMax;
	}

	public SwerveModule(int driveMotorID, int turnMotorID, int lampreyID) {
		driveMotor = configuredCANSparkMax(driveMotorID);
		driveMotor.getEncoder()
				.setVelocityConversionFactor(Constants.kSwerveDriveTrain.kDrive.kEncoderConversionFactor);
		drivePIDController = driveMotor.getPIDController();
		drivePIDController.setP(Constants.kSwerveDriveTrain.kDrive.kP);
		drivePIDController.setI(Constants.kSwerveDriveTrain.kDrive.kI);
		drivePIDController.setD(Constants.kSwerveDriveTrain.kDrive.kD);
		steerLampreyEncoder = new DutyCycleEncoder(lampreyID);
		steerMotor = configuredCANSparkMax(turnMotorID);
		steerMotor.getEncoder()
				.setPositionConversionFactor(Constants.kSwerveDriveTrain.kSteer.kEncoderConversionFactor);
		// commented out until lampreys are hooked up
		// steerMotor.getEncoder()
		// .setPosition(steerLampreyEncoder.get());
		steerPIDController = steerMotor.getPIDController();
		steerPIDController.setP(Constants.kSwerveDriveTrain.kDrive.kP);
		steerPIDController.setI(Constants.kSwerveDriveTrain.kDrive.kI);
		steerPIDController.setD(Constants.kSwerveDriveTrain.kDrive.kD);
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(driveMotor.getEncoder().getVelocity(),
				new Rotation2d(steerMotor.getEncoder().getPosition()));
	}

	public void setDesiredState(SwerveModuleState desiredState) {
		SwerveModuleState state = SwerveModuleState.optimize(desiredState,
				new Rotation2d(steerMotor.getEncoder().getPosition()));
		drivePIDController.setReference(state.speedMetersPerSecond, ControlType.kVelocity);
		steerPIDController.setReference(state.angle.getRadians(), ControlType.kVelocity);

	}

	public void setMotors(double speed) {
		driveMotor.set(speed);
		steerMotor.set(speed);
	}

	public void resetEncoders() {
		driveMotor.getEncoder().setPosition(0);
		steerMotor.getEncoder().setPosition(0);
	}

	public void stop() {
		driveMotor.stopMotor();
		steerMotor.stopMotor();
	}
}
