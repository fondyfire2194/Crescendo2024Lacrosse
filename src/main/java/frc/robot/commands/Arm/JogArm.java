// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Arm;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class JogArm extends Command {
  /** Creates a new JogArm. */
  private ArmSubsystem m_arm;
  private CommandXboxController m_controller;

  public JogArm(ArmSubsystem arm, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    m_controller = controller;
    addRequirements(m_arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_arm.disable();
    m_arm.enableArm=false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    boolean allowDown = m_arm.getAngleRadians() >= ArmConstants.armMinRadians
        || m_controller.getHID().getBackButton();

    boolean allowUp = m_arm.getAngleRadians() <= ArmConstants.armMaxRadians
        || m_controller.getHID().getBackButton();

    double yval = -m_controller.getLeftY() / 10;

    if (yval > 0 && allowUp || yval < 0 && allowDown) {
      m_arm.appliedVolts = yval * RobotController.getBatteryVoltage();
      SmartDashboard.putNumber("ArmVoltsJog", m_arm.appliedVolts);
      m_arm.armMotor.setVoltage(m_arm.appliedVolts);
    } else {
      m_arm.armMotor.setVoltage(0);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     m_arm.setGoal(m_arm.getAngleRadians());
     m_arm.enable();
     m_arm.enableArm=false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
