// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.OperatorInput;
import frc.robot.subsystems.SwerveDriveTrain;

public class DriveWithJoysticks extends CommandBase {
	private final SwerveDriveTrain swerveDriveTrain;

	/** Creates a new DriveWithJoysticks. */
	public DriveWithJoysticks(SwerveDriveTrain sdt) {
		swerveDriveTrain = sdt;
		addRequirements(swerveDriveTrain);
		// Use addRequirements() here to declare subsystem dependencies.
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		swerveDriveTrain.drive(
				OperatorInput.driverJoystick.getRawAxis(1) * Constants.kSwerveDriveTrain.kMaxSpeedMetersPerSecond,
				OperatorInput.driverJoystick.getRawAxis(0) * Constants.kSwerveDriveTrain.kMaxSpeedMetersPerSecond,
				OperatorInput.driverJoystick.getRawAxis(4) * Constants.kSwerveDriveTrain.kMaxSpeedMetersPerSecond,
				false);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		swerveDriveTrain.stop();
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return false;
	}
}
