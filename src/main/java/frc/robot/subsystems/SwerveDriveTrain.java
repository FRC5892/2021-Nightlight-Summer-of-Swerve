// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDriveTrain extends SubsystemBase {
	private Translation2d locationFL;
	private Translation2d locationFR;
	private Translation2d locationBL;
	private Translation2d locationBR;
	private ChassisSpeeds speeds;
	private SwerveDriveKinematics kinematics;
	private SwerveDriveOdometry odometry;
	private SwerveModule FLSwerveModule;

	public SwerveDriveTrain() {
		double wheelBase = 1;
		double trackWidth = 2;

		locationFL = new Translation2d(wheelBase / 2, trackWidth / 2);
		locationFR = new Translation2d(wheelBase / 2, -trackWidth / 2);
		locationBL = new Translation2d(-wheelBase / 2, trackWidth / 2);
		locationBR = new Translation2d(-wheelBase / 2, -trackWidth / 2);

		kinematics = new SwerveDriveKinematics(locationFL, locationFR, locationBL, locationBR);
		odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(180));//replace with gyro

		FLSwerveModule = new SwerveModule(1, 2);
	}

	public void drive(double forward, double strafe, double rotation) {
		speeds = new ChassisSpeeds(forward, strafe, rotation);
		SwerveModuleState states[] = kinematics.toSwerveModuleStates(speeds);

	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		odometry.update(
			new Rotation2d(180), FLSwerveModule.getState()//replace with gyro
			);
	}
}
