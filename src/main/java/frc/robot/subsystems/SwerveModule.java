package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.config.SwerveModuleConstants;
//import frc.lib.math.OnboardModuleState;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.GlobalConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Pref;
import frc.robot.RobotContainer;

public class SwerveModule extends SubsystemBase {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private CANSparkMax angleMotor;
  CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;

  private final CANcoder m_turnCancoder;

  private final SparkPIDController driveController;
  private final SparkPIDController angleController;

  private SimpleMotorFeedforward driveFeedforward;

  private SwerveModuleState currentDesiredState = new SwerveModuleState();
  private double angleDegrees;
  private double m_simDrivePosition;
  private double m_simRotatePosition;
  private boolean driveReversed;
  private int loopctr;
  private double characterizationVolts;
  private boolean characterizing;
  private SwerveModuleState previousState = new SwerveModuleState();

  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;
    driveReversed = moduleConstants.driveReversed;
    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();
    this.m_simDrivePosition = 0.0;
    this.m_simRotatePosition = 0.0;
    m_turnCancoder = new CANcoder(moduleConstants.canCoderID, "CV1");

    // var can_config = new CANcoderConfiguration();
    // can_config.MagnetSensor.AbsoluteSensorRange =
    // AbsoluteSensorRangeValue.Unsigned_0To1;
    // m_turnCancoder.getConfigurator().apply(can_config);

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    // try this for faster updates from CAN check can usage befor and after
    driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 15);
    angleMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 15);

    lastAngle = getState().angle;

    driveFeedforward = new SimpleMotorFeedforward(
        SwerveConstants.driveKS, SwerveConstants.driveKV, SwerveConstants.driveKA);

  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous
    // controller which
    // REV and CTRE are not
    // desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
    desiredState = SwerveModuleState.optimize(desiredState, getAngle());

    currentDesiredState = desiredState;
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  public SwerveModuleState getDesiredState() {
    return currentDesiredState;
  }

  public void resetAngleToAbsolute() {
    double angle = ((m_turnCancoder.getAbsolutePosition().getValueAsDouble() * 360)); // -angleOffset
    integratedAngleEncoder.setPosition(angle);
  }

  public void resetAngleEncoder(double angle) {
    integratedAngleEncoder.setPosition(angle);
    this.m_simRotatePosition = 0.0;
  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(Constants.SwerveConstants.angleContinuousCurrentLimit);
    angleMotor.setInverted(Constants.SwerveConstants.angleInvert);
    angleMotor.setIdleMode(Constants.SwerveConstants.angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(Constants.SwerveConstants.angleConversionFactor);
    angleController.setP(Constants.SwerveConstants.angleKP);
    angleController.setI(Constants.SwerveConstants.angleKI);
    angleController.setD(Constants.SwerveConstants.angleKD);
    angleController.setFF(Constants.SwerveConstants.angleKFF);

    // ******************************************************** */

    angleController.setPositionPIDWrappingEnabled(true);
    angleController.setPositionPIDWrappingMinInput(-180);
    angleController.setPositionPIDWrappingMaxInput(180);

    // ******************************************************** */

    angleMotor.enableVoltageCompensation(Constants.SwerveConstants.voltageComp);
    angleMotor.burnFlash();

  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kVelocityOnly);
    driveMotor.setSmartCurrentLimit(Constants.SwerveConstants.driveContinuousCurrentLimit);
    driveMotor.setInverted(driveReversed);
    driveMotor.setIdleMode(Constants.SwerveConstants.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(Constants.SwerveConstants.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Constants.SwerveConstants.driveConversionPositionFactor);
    driveEncoder.setAverageDepth(4);
    driveEncoder.setMeasurementPeriod(32);
    driveController.setP(Constants.SwerveConstants.driveKP);
    driveController.setI(Constants.SwerveConstants.driveKI);
    driveController.setD(Constants.SwerveConstants.driveKD);
    driveController.setFF(Constants.SwerveConstants.driveKFF);// 3.24=max speed
    driveController.setP(0.35 , 1);
    driveController.setI(0, 1);
    driveController.setD(0, 1);
    driveController.setFF(0, 1);
    driveMotor.enableVoltageCompensation(Constants.SwerveConstants.voltageComp);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
  }

  public void setDriveKp() {
    driveController.setP(Pref.getPref("DriveKp"));
  }

  public void setDriveFF() {
    driveController.setFF(Pref.getPref("DriveFF") / Constants.SwerveConstants.kmaxTheoreticalSpeed);
  }

  public double getDriveKp() {
    return driveController.getP();
  }

  public double getDriveFF() {
    return driveController.getFF();
  }

  public void setAngleKp() {
    angleController.setP(Pref.getPref("AngleKp"));
  }

  public double getAngleKp() {
    return angleController.getP();
  }

  public boolean driveIsBraked() {
    return driveMotor.getIdleMode() == IdleMode.kBrake;
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {

    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.kmaxSpeed;
      driveMotor.setVoltage(percentOutput * RobotController.getBatteryVoltage());
      SmartDashboard.putNumber(String.valueOf(moduleNumber) + " driveVolts",
          percentOutput * RobotController.getBatteryVoltage());
      SmartDashboard.putNumber(String.valueOf(moduleNumber) + " driveKv",
          percentOutput * RobotController.getBatteryVoltage() / getDriveVelocity());
    }
    boolean feedforward = true;
    if (!isOpenLoop) {
      if (!feedforward) {
        driveController.setReference(desiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
      } else {

        double feedForward = driveFeedforward.calculate(
            desiredState.speedMetersPerSecond,
            (desiredState.speedMetersPerSecond - previousState.speedMetersPerSecond) / 0.020);

        if (Math.abs(desiredState.speedMetersPerSecond) < .01) {
          feedForward = 0;
        }
        driveController.setReference(
            desiredState.speedMetersPerSecond, ControlType.kVelocity, 1, feedForward, ArbFFUnits.kVoltage);
        previousState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
      }
    }
    if (RobotBase.isSimulation())
      m_simDrivePosition += desiredState.speedMetersPerSecond * GlobalConstants.ROBOT_LOOP_PERIOD;
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.kmaxSpeed * 0.01))
        ? lastAngle
        : desiredState.angle;

    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
    angleDegrees = angle.getDegrees();
    if (RobotBase.isSimulation())
      m_simRotatePosition = angle.getDegrees();
  }

  private Rotation2d getAngle() {
    if (RobotBase.isReal())
      return Rotation2d.fromDegrees(
          MathUtil.inputModulus(integratedAngleEncoder.getPosition(), -180, 180));
    else
      return Rotation2d.fromDegrees(MathUtil.inputModulus(m_simRotatePosition, -180, 180));
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), getAngle());
  }

  public double getVoltage() {
    return driveMotor.getAppliedOutput() * driveMotor.getBusVoltage();
  }

  public double getDriveVelocity() {
    if (RobotBase.isReal())
      return driveEncoder.getVelocity();
    else {
      return currentDesiredState.speedMetersPerSecond;
    }
  }

  public double getDrivePosition() {
    if (RobotBase.isReal())
      return driveEncoder.getPosition();

    else {

      return m_simDrivePosition;

    }
  }

  public double getPositionRadians() {
    return getDrivePosition() / SwerveConstants.wheelRadius;
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber(
        String.valueOf(moduleNumber) + "cancoder", getCancoderDeg());

    // SmartDashboard.putBoolean(String.valueOf(moduleNumber) + " Characterizing",
    // characterizing);
    // SmartDashboard.putNumber(String.valueOf(moduleNumber) + " Characterization
    // Volts", characterizationVolts);
    SmartDashboard.putNumber(String.valueOf(moduleNumber) + " DrivePosiiton", driveEncoder.getPosition());

    if (characterizing) {
      driveMotor.setVoltage(characterizationVolts);
      angleController.setReference(0, ControlType.kPosition);
    }

  }

  private double getCancoderDeg() {
    return m_turnCancoder.getAbsolutePosition().getValueAsDouble() * 360;
  }

  public boolean isStopped() {
    return Math.abs(getDriveVelocity()) < .1;
  }

  public void setIdleMode(boolean brake) {
    if (brake) {
      driveMotor.setIdleMode(IdleMode.kBrake);
      angleMotor.setIdleMode(IdleMode.kBrake);
    } else {
      driveMotor.setIdleMode(IdleMode.kCoast);
      angleMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  public Command clearFaultsCommand() {
    return Commands.parallel(
        Commands.runOnce(() -> driveMotor.clearFaults()),
        Commands.runOnce(() -> angleMotor.clearFaults()));
  }

  public double getFaults() {
    return driveMotor.getFaults() + angleMotor.getFaults() + m_turnCancoder.getFaultField().getValueAsDouble();
  }

  public double getStickyFaults() {
    return driveMotor.getStickyFaults() + angleMotor.getStickyFaults()
        + m_turnCancoder.getStickyFaultField().getValueAsDouble();
  }

  public void setCharacterizationVolts(double volts) {
    characterizationVolts = volts;
    characterizing = true;
  }

  public void stopCharacterizing() {
    characterizationVolts = 0.0;
    characterizing = false;
  }

  @Override
  public void simulationPeriodic() {

  }

  public static double round2dp(double number, int dp) {
    number = Math.round(number * dp);
    number /= 100;
    return number;
  }

}
