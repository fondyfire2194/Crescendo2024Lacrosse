// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Transfer;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TransferSubsystem;

public class TransferIntakeToSensor extends Command {
  private final TransferSubsystem m_transfer;
  private Debouncer sensorDebouncer;
 
  /** Creates a new TransferIntakeToSensor. */
  public TransferIntakeToSensor(TransferSubsystem transfer) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_transfer = transfer;
  
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

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_transfer.stopMotor();
    m_transfer.enableLimitSwitch(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return sensorDebouncer.calculate(m_transfer.noteAtIntake());
  }
}
