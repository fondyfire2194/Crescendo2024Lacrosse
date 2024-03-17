// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Transfer;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.TransferSubsystem;

public class JogTransfer extends Command {
  /** Creates a new JogShooter. */
  private TransferSubsystem m_transfer;
  private CommandXboxController m_controller;

  public JogTransfer(TransferSubsystem transfer, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_transfer = transfer;
    m_controller = controller;
    addRequirements(m_transfer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
m_transfer.enableLimitSwitch(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yval = -m_controller.getLeftY();
    // m_transfer.transferMotor.setVoltage(yval *
    // RobotController.getBatteryVoltage());

    double rpm = yval * 5000;
    m_transfer.transferController.setReference(rpm, ControlType.kVelocity);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transfer.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
