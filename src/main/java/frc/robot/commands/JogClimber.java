// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class JogClimber extends Command {
  /** Creates a new JogShooter. */
  private ClimberSubsystem m_climber;
  private CommandXboxController m_controller;

  public JogClimber(ClimberSubsystem climber, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_climber = climber;
    m_controller = controller;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_climber.unlockClimber();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yval = -m_controller.getLeftY();
    SmartDashboard.putNumber("ClimbNum", yval);
    if (m_climber.getPositionLeft() > 145 && yval > 0) {
      m_climber.runClimberMotor(0);
    } else if (m_climber.getPositionLeft() > 140 && yval > 0) {
      m_climber.runClimberMotor(yval*0.2);
    } 
    
    else if (m_climber.getPositionLeft() < 10 && yval < 0) {
      m_climber.runClimberMotor(0);
    } else {
      m_climber.runClimberMotor(yval);
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_climber.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
