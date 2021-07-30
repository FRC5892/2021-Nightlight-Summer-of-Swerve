// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
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
	private final Gyro gyro = new ADXRS450_Gyro();

	public SwerveDriveTrain() {
		double wheelBase = 1;
		double trackWidth = 2;

		locationFL = new Translation2d(wheelBase / 2, trackWidth / 2);
		locationFR = new Translation2d(wheelBase / 2, -trackWidth / 2);
		locationBL = new Translation2d(-wheelBase / 2, trackWidth / 2);
		locationBR = new Translation2d(-wheelBase / 2, -trackWidth / 2);

		kinematics = new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);
		odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d());

		fLSwerveModule = new SwerveModule(1, 2);
		fRSwerveModule = new SwerveModule(3, 4);
		bLSwerveModule = new SwerveModule(5, 6);
		bRSwerveModule = new SwerveModule(7, 8);
	}

	public void drive(double forward, double strafe, double rotation) {
		speeds = new ChassisSpeeds(forward, strafe, rotation);
		SwerveModuleState states[] = kinematics.toSwerveModuleStates(speeds);

	}

	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	public void resetOdometry (Pose2d pose) {
		odometry.resetPosition(pose, gyro.getRotation2d());
	}

	public void resetEncoders() {
		fLSwerveModule.resetEncoders();
		fRSwerveModule.resetEncoders();
		bLSwerveModule.resetEncoders();
		fRSwerveModule.resetEncoders();
	}

	public void zeroHeading() {
		gyro.reset();
	}

	public double getHeading() {
		return gyro.getRotation2d().getDegrees();
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.normalizeWheelSpeeds(
		desiredStates, Constants.kSwerveDriveTrain.kMaxSpeedMetersPerSecond);
		fLSwerveModule.setDesiredState(desiredStates[0]);
		fRSwerveModule.setDesiredState(desiredStates[1]);
		bLSwerveModule.setDesiredState(desiredStates[2]);
		bRSwerveModule.setDesiredState(desiredStates[3]);
	  }

	public double getTurnRate() {
		return gyro.getRate();
	}

	public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
		var swerveModuleStates =
			kinematics(
				fieldRelative
					? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
					: new ChassisSpeeds(xSpeed, ySpeed, rot));
		kinematics.normalizeWheelSpeeds(
			swerveModuleStates, Constants.kSwerveDriveTrain.kMaxSpeedMetersPerSecond);
		fLSwerveModule.setDesiredState(swerveModuleStates[0]);
		fRSwerveModule.setDesiredState(swerveModuleStates[1]);
		bLSwerveModule.setDesiredState(swerveModuleStates[2]);
		bRSwerveModule.setDesiredState(swerveModuleStates[3]);
	  }

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		odometry.update(
			gyro.getRotation2d(), 
			fLSwerveModule.getState(),
			fRSwerveModule.getState(),
			bLSwerveModule.getState(),
			bRSwerveModule.getState()
			);
	}
}
