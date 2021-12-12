// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveDriveTrain extends SubsystemBase {
	private Translation2d locationFL;
	private Translation2d locationFR;
	private Translation2d locationBL;
	private Translation2d locationBR;
	private ChassisSpeeds speeds;
	private SwerveDriveKinematics kinematics;
	private SwerveDriveOdometry odometry;
	private SwerveModule fLSwerveModule;
	private SwerveModule fRSwerveModule;
	private SwerveModule bLSwerveModule;
	private SwerveModule bRSwerveModule;
	private final AHRS gyro = new AHRS(SPI.Port.kMXP);

	public SwerveDriveTrain() {
		double wheelBase = 0.434;
		double trackWidth = 0.382;

		locationFR = new Translation2d(wheelBase / 2, trackWidth / 2);
		locationFL = new Translation2d(wheelBase / 2, -trackWidth / 2);
		locationBR = new Translation2d(-wheelBase / 2, trackWidth / 2);
		locationBL = new Translation2d(-wheelBase / 2, -trackWidth / 2);

		kinematics = new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);
		odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());

		fRSwerveModule = new SwerveModule(1, 2, 1, -0.86);
		fLSwerveModule = new SwerveModule(3, 4, 3, -2.44);
		bRSwerveModule = new SwerveModule(5, 6, 5, -5.43);
		bLSwerveModule = new SwerveModule(7, 8, 7, -4.29);

		resetEncoders();
	}

	public void drive(double forwardVelocity, double strafeVelocity, double rotationVelocity, boolean fieldRelative) {
		SwerveModuleState states[] = kinematics.toSwerveModuleStates(fieldRelative // wierd if else syntax below
				? ChassisSpeeds.fromFieldRelativeSpeeds(forwardVelocity, strafeVelocity, rotationVelocity,
						gyro.getRotation2d())
				: new ChassisSpeeds(forwardVelocity, strafeVelocity, rotationVelocity));
		SwerveDriveKinematics.normalizeWheelSpeeds(states, Constants.kSwerveDriveTrain.kMaxSpeedMetersPerSecond);
		fLSwerveModule.setDesiredState(states[0]);
		fRSwerveModule.setDesiredState(states[1]);
		bLSwerveModule.setDesiredState(states[2]);
		bRSwerveModule.setDesiredState(states[3]);
	}

	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	public void resetOdometry(Pose2d pose) {
		odometry.resetPosition(pose, gyro.getRotation2d());
	}

	public void resetEncoders() {
		fLSwerveModule.resetEncoders();
		fRSwerveModule.resetEncoders();
		bLSwerveModule.resetEncoders();
		bRSwerveModule.resetEncoders();
	}

	public void zeroHeading() {
		gyro.reset();
	}

	public double getHeading() {
		return gyro.getRotation2d().getDegrees();
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Constants.kSwerveDriveTrain.kMaxSpeedMetersPerSecond);
		fLSwerveModule.setDesiredState(desiredStates[0]);
		fRSwerveModule.setDesiredState(desiredStates[1]);
		bLSwerveModule.setDesiredState(desiredStates[2]);
		bRSwerveModule.setDesiredState(desiredStates[3]);
	}

	public double getTurnRate() {
		return gyro.getRate();
	}

	public void setMotors(double speed) {
		fLSwerveModule.setMotors(speed);
		fRSwerveModule.setMotors(speed);
		bLSwerveModule.setMotors(speed);
		bRSwerveModule.setMotors(speed);
	}

	public void stop() {
		fLSwerveModule.stop();
		fRSwerveModule.stop();
		bLSwerveModule.stop();
		bRSwerveModule.stop();
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		SmartDashboard.putNumber("FL Steer Position", fLSwerveModule.getSteerPosition());
		SmartDashboard.putNumber("FR Steer Position", fRSwerveModule.getSteerPosition());
		SmartDashboard.putNumber("BL Steer Position", bLSwerveModule.getSteerPosition());
		SmartDashboard.putNumber("BR Steer Position", bRSwerveModule.getSteerPosition());
		SmartDashboard.putNumber("FL Lamprey", fLSwerveModule.getLampreyPosition());
		SmartDashboard.putNumber("FR Lamprey", fRSwerveModule.getLampreyPosition());
		SmartDashboard.putNumber("BL Lamprey", bLSwerveModule.getLampreyPosition());
		SmartDashboard.putNumber("BR Lamprey", bRSwerveModule.getLampreyPosition());

		SmartDashboard.putNumber("FL Drive Positon", fLSwerveModule.getDrivePosition());
		SmartDashboard.putNumber("Steer State Position", fLSwerveModule.getState().angle.getRadians());
		SmartDashboard.putNumber("Steer State Desired Position", fLSwerveModule.getDesiredState().angle.getRadians());
		odometry.update(gyro.getRotation2d(), fLSwerveModule.getState(), bLSwerveModule.getState(),
				fRSwerveModule.getState(), bRSwerveModule.getState());
	}
}
