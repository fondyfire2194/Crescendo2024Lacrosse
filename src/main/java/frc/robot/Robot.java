// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.CameraConstants;
import frc.robot.utils.LLPipelines;
import monologue.Monologue;
import monologue.Annotations.Log;
import monologue.Logged;

public class Robot extends TimedRobot implements Logged {

  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private double m_disableStartTime;

  private double brakeOffTime = 3;

  private double m_startDelay;

  @Log.NT.Once  private double startTime;

  private boolean autoHasRun;

  private boolean firstScan = true;

  @Override
  public void robotInit() {
    if (RobotBase.isReal()) {
      DataLogManager.start();
      DriverStation.startDataLog(DataLogManager.getLog());

    }

    m_robotContainer = new RobotContainer();

    // CameraServer.startAutomaticCapture();

    // Shuffleboard.selectTab("Autonomous");
    Monologue.setupMonologue(m_robotContainer, "/Monologue", false, true);

   DriverStation.startDataLog(DataLogManager.getLog());
   // Monologue.setupMonologue(this, "/Monologue", false, true);
  }

  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();

    m_robotContainer.m_arm.periodicRobot();

    // setFileOnly is used to shut off NetworkTables broadcasting for most logging
    // calls.
    // Basing this condition on the connected state of the FMS is a suggestion only.
    // Monologue.setFileOnly(DriverStation.isDSAttached());
    // This method needs to be called periodically, or no logging annotations will
    // process properly.
    Monologue.updateAll();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
    autoHasRun = false;
    m_robotContainer.m_arm.disable();
    m_robotContainer.m_arm.enableArm = false;
    if (m_robotContainer.m_arm.getCanCoderDeg() < 26)
      m_robotContainer.m_arm.armMotor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void disabledPeriodic() {

    // turn off drive brakes if they are on and robot is not moving
    // allows easier manual pushing of robot

    if (m_robotContainer.m_swerve.driveIsBraked() && m_robotContainer.m_swerve.isStopped()
        && m_disableStartTime == 0)
      m_disableStartTime = Timer.getFPGATimestamp();

    if (m_disableStartTime != 0 && Timer.getFPGATimestamp() > m_disableStartTime
        + brakeOffTime) {
      m_robotContainer.m_swerve.setIdleMode(false);
    }

    m_robotContainer.m_swerve.cameraSelection = m_robotContainer.m_cameraChooser.getSelected();

  }

  @Override
  public void disabledExit() {

    m_robotContainer.checkCAN = false;
  }

  @Override
  public void autonomousInit() {
    m_robotContainer.m_arm.armMotor.setIdleMode(IdleMode.kBrake);
    m_robotContainer.m_arm.enable();
    m_robotContainer.m_arm.enableArm = true;

    LimelightHelpers.setPipelineIndex(CameraConstants.frontLeftCamera.camname,
        LLPipelines.pipelines.APRILTAGALL0.ordinal());
    LimelightHelpers.setPipelineIndex(CameraConstants.frontRightCamera.camname,
        LLPipelines.pipelines.APRILTAGALL0.ordinal());
    LimelightHelpers.setPipelineIndex(CameraConstants.rearCamera.camname,
        LLPipelines.pipelines.NOTE_DETECT8.ordinal());

    m_robotContainer.m_swerve.setIdleMode(true);

    m_robotContainer.m_swerve.cameraSelection = m_robotContainer.m_cameraChooser.getSelected();

    m_startDelay = m_robotContainer.m_startDelayChooser.getSelected();

    m_robotContainer.m_swerve.cameraSelection = m_robotContainer.m_cameraChooser.getSelected();

    m_robotContainer.m_swerve.resetModuleEncoders();

    startTime = Timer.getFPGATimestamp();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (!autoHasRun && Timer.getFPGATimestamp() > startTime + m_startDelay
        && m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
      autoHasRun = true;
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
    m_robotContainer.m_shooter.stopMotors();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    autoHasRun = false;

    m_robotContainer.m_shooter.stopMotors();
    m_robotContainer.m_intake.stopMotor();
    m_robotContainer.m_transfer.stopMotor();

  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.m_arm.armMotor.setIdleMode(IdleMode.kBrake);
    // new ArmShooterByDistance().schedule();

    m_robotContainer.m_swerve.setIdleMode(true);
    m_robotContainer.m_arm.enable();
    m_robotContainer.m_arm.enableArm = true;

    m_robotContainer.m_shooter.stopMotors();
    m_robotContainer.m_intake.stopMotor();
    m_robotContainer.m_transfer.stopMotor();

    LimelightHelpers.setPipelineIndex(CameraConstants.frontLeftCamera.camname,
        LLPipelines.pipelines.APRILTAGALL0.ordinal());
    LimelightHelpers.setPipelineIndex(CameraConstants.frontRightCamera.camname,
        LLPipelines.pipelines.APRILTAGALL0.ordinal());
    LimelightHelpers.setPipelineIndex(CameraConstants.rearCamera.camname,
        LLPipelines.pipelines.NOTE_DETECT8.ordinal());

  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    REVPhysicsSim.getInstance().run();
  }
}
