// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.commands.ArmToPose;
import frc.robot.util.Controller;
import frc.robot.subsystems.arm.Arm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {

  private final Controller controller = new Controller(0);
  private Arm arm = new Arm();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    //TODO make sure this is right for changing the setpoint
    controller.onPress(controller.A, new ArmToPose(arm, SmartDashboard.getEntry("setpoint").getDouble(Constants.Arm.kMinAngle)));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
