// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakeSubsystem;

public class JogIntake extends Command {
  /** Creates a new JogIntake. */
  private IntakeSubsystem m_intake;
  private CommandXboxController m_controller;

  public JogIntake(IntakeSubsystem intake, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = intake;
    m_controller = controller;
    addRequirements(m_intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.resetRunIntake();
    m_intake.jogging=true;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yval = -m_controller.getLeftY();
    // m_intake.intakeMotor.setVoltage(yval * RobotController.getBatteryVoltage());
    double rpm = yval * 5000;
    SmartDashboard.putNumber("INTRPMJ", rpm);
    m_intake.intakeController.setReference(rpm, ControlType.kVelocity);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.stopMotor();
    m_intake.jogging=false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
