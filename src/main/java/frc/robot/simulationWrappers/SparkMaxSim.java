// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulationWrappers;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;

/** Add your docs here. */
public class SparkMaxSim {
	private final SimDouble simVelocity;
	private final SimDouble simPosition;
	private final SimDouble simAppliedOutput;

	public SparkMaxSim(int deviceID) {
		SimDeviceSim simSparkMax = new SimDeviceSim("SPARK MAX " + "[" + deviceID + "]");
		simVelocity = simSparkMax.getDouble("Velocity");
		simPosition = simSparkMax.getDouble("Position");
		simAppliedOutput = simSparkMax.getDouble("Applied Output");
	}

	public void setVelocity(double velocity) {
		simVelocity.set(velocity);
	}

	public void setPosition(double position) {
		simPosition.set(position);
	}

	public void setAppliedOutput(double appliedOutput) {
		simAppliedOutput.set(appliedOutput);
	}
}
