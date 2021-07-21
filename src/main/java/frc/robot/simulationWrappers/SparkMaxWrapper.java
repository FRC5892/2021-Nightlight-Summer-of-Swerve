// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.simulationWrappers;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class SparkMaxWrapper extends CANSparkMax {
	private SimDeviceSim simSparkMax;
	private SimDouble simAppliedOutput;

	public SparkMaxWrapper(int deviceID, MotorType type) {
		super(deviceID, type);
		if (RobotBase.isSimulation()) {
			simSparkMax = new SimDeviceSim("SPARK MAX " + "[" + deviceID + "]");
		}
		if (simSparkMax != null) {
			simAppliedOutput = simSparkMax.getDouble("Applied Output");
		}
	}

	@Override
	public void set(double speed) {
		super.set(speed);
		if (simSparkMax != null) {
			simAppliedOutput.set(speed * RobotController.getBatteryVoltage());
			SmartDashboard.putNumber("Applied Output", simAppliedOutput.get());
		}
	}
}
