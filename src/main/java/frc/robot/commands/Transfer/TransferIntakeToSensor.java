// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Transfer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TransferSubsystem;

public class TransferIntakeToSensor extends Command {
  private final TransferSubsystem m_transfer;
  private final IntakeSubsystem m_intake;
  private Debouncer sensorDebouncer;
  private double robotStoppedTime;
  private final SwerveSubsystem m_swerve;
  private double nonoteTime = 100;

  /** Creates a new TransferIntakeToSensor. */
  public TransferIntakeToSensor(TransferSubsystem transfer, IntakeSubsystem intake, SwerveSubsystem swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_transfer = transfer;
    m_intake = intake;
    m_swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    sensorDebouncer = new Debouncer(.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_transfer.runToSensor();

    if (!m_swerve.isStopped())
      robotStoppedTime = 0;

    if (m_swerve.isStopped() && robotStoppedTime == 0)
      robotStoppedTime = Timer.getFPGATimestamp();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transfer.stopMotor();
    m_intake.notePresent = m_transfer.noteAtIntake();
    if (DriverStation.isTeleopEnabled())
      m_intake.stopMotor();
    m_transfer.enableLimitSwitch(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return sensorDebouncer.calculate(m_transfer.noteAtIntake()) ||
    // !m_intake.getRunIntake()
    // || robotStoppedTime != 0 && Timer.getFPGATimestamp() > robotStoppedTime +
    // nonoteTime;

    return sensorDebouncer.calculate(m_transfer.m_limitSwitch.isPressed());

  }
}
