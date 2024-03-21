// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ShooterSubsystem;

public class JogShooters extends Command {
  /** Creates a new JogShooter. */
  private ShooterSubsystem m_shooter;
  private CommandXboxController m_controller;

  public JogShooters(ShooterSubsystem shooter, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_shooter = shooter;
    m_controller = controller;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_shooter.resetRunShooter();
    ;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yval = -m_controller.getLeftY() ;

    double rpm = yval * 5000;
    // m_shooter.topRoller.setVoltage(yval * RobotController.getBatteryVoltage());
    // m_shooter.bottomRoller.setVoltage(yval *
    // RobotController.getBatteryVoltage());

    m_shooter.topController.setReference(rpm, ControlType.kVelocity);
    m_shooter.bottomController.setReference(rpm, ControlType.kVelocity);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_shooter.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
