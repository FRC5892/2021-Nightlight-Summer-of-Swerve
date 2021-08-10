package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OperatorInput {
	public static XboxController driverJoystick = new XboxController(0);
	static JoystickButton driveTrainTestButton = new JoystickButton(driverJoystick, XboxController.Button.kA.value);
}
