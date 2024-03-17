// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateToAngle extends PIDCommand {
  /** Creates a new RotateToAngle. */
  private SwerveSubsystem m_drive;

  private static PIDController rotatePID = new PIDController(0, 0, 0);

  public RotateToAngle(SwerveSubsystem drive, double angle) {

    super(
        // The controller that the command will use
        rotatePID,
        // This should return the measurement
        () -> drive.getHeadingDegrees(),
        // This should return the setpoint (can also be a constant)
        () -> angle,
        // This uses the output
        output -> {
          drive.drive(0, 0, output * 3, false, false,false);
        }, drive);
    m_drive = drive;

    super.m_controller.setTolerance(2);

    super.m_controller.enableContinuousInput(-180, 180);

    // this number could be changed

    addRequirements(drive);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    getController().setTolerance(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return super.m_controller.atSetpoint();
  }
}
