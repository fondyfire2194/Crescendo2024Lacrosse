// Copytop (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Pref;

import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

public class ShooterSubsystem extends SubsystemBase {

  public CANSparkMax topRoller;
  public SparkPIDController topController;
  public SparkPIDController bottomController;
  RelativeEncoder topEncoder;

  public CANSparkMax bottomRoller;
  RelativeEncoder bottomEncoder;

  public double commandRPM = 500;

  public boolean m_showScreens;

  private boolean runShooterVel;
  private double topBottomSpeedRatio = 1;

  private SlewRateLimiter topSpeedLimiter = new SlewRateLimiter(2500);
  private SlewRateLimiter bottomSpeedLimiter = new SlewRateLimiter(2500);

  /** Creates a new Shooter. */
  public ShooterSubsystem(boolean showScreens) {
    m_showScreens = showScreens;

    topRoller = new CANSparkMax(Constants.CANIDConstants.topShooterID, MotorType.kBrushless);
    topController = topRoller.getPIDController();
    topEncoder = topRoller.getEncoder();

    configMotor(topRoller, topEncoder, true);

    bottomRoller = new CANSparkMax(Constants.CANIDConstants.bottomShooterID, MotorType.kBrushless);
    bottomEncoder = bottomRoller.getEncoder();
    bottomController = bottomRoller.getPIDController();

    configMotor(bottomRoller, bottomEncoder, true);

    setShooterSpeedRatio(1);

    if (m_showScreens) {

      Shuffleboard.getTab("ShooterSubsystem").add(this)
          .withPosition(0, 0).withSize(2, 1);

      ShuffleboardLayout shootLayout = Shuffleboard.getTab("ShooterSubsystem")
          .getLayout("Shooter", BuiltInLayouts.kList).withPosition(0, 1)
          .withSize(1, 4).withProperties(Map.of("Label position", "TOP"));

      ShuffleboardLayout shootLayout1 = Shuffleboard.getTab("ShooterSubsystem")
          .getLayout("Shooter1", BuiltInLayouts.kList).withPosition(1, 1)
          .withSize(1, 4).withProperties(Map.of("Label position", "TOP"));

      shootLayout1
          .addNumber("CommandRPM", () -> commandRPM)
          .withPosition(1, 1).withSize(1, 1);

      shootLayout1.addNumber(
          "TopRPM", () -> round2dp(getRPMTop(), 0))
          .withPosition(0, 2).withSize(1, 1);

      shootLayout1.addNumber("BottomRPM",
          () -> round2dp(getRPMBottom(), 0))
          .withPosition(1, 2).withSize(1, 1);

      shootLayout1.addNumber("TopAmps", () -> getTopAmps())
          .withPosition(0, 3).withSize(1, 1);

      shootLayout1.addNumber("BottomAmps", () -> getBottomAmps())
          .withPosition(1, 3).withSize(1, 1);

      shootLayout.addBoolean("TopAtSpeed", () -> topAtSpeed(.2))
          .withPosition(0, 4).withSize(1, 1);
      shootLayout.addBoolean("BotAtSpeed", () -> bottomAtSpeed(.2))
          .withPosition(0, 4).withSize(1, 1);

      shootLayout.add("SetTopGains", setTopKpKdKiCommand())
          .withPosition(1, 4).withSize(1, 1);

      shootLayout.add("SetBottomGains", setBottomKpKdKiCommand())
          .withPosition(1, 4).withSize(1, 1);

      shootLayout.addBoolean("StickyFaultTfr", () -> getStickyFaults() != 0)
          .withPosition(3, 3).withSize(1, 1)
          .withProperties(Map.of("colorWhenTrue", "red", "colorWhenFalse", "black"));
      shootLayout
          .add("ClearFaultTfr", clearFaultsCommand())
          .withPosition(4, 3).withSize(1, 1);

    }

    topController.setOutputRange(0, 1);

    bottomController.setOutputRange(0, 1);

  }

  private void configMotor(CANSparkMax motor, RelativeEncoder encoder,
      boolean reverse) {
    motor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(motor, Usage.kAll);
    motor.setSmartCurrentLimit(Constants.ShooterConstants.shooterContinuousCurrentLimit);
    motor.setInverted(reverse);
    motor.setIdleMode(Constants.ShooterConstants.shooterIdleMode);
    encoder.setVelocityConversionFactor(Constants.ShooterConstants.shooterConversionVelocityFactor);
    encoder.setPositionConversionFactor(Constants.ShooterConstants.shooterConversionPositionFactor);
    encoder.setAverageDepth(4);
    encoder.setMeasurementPeriod(32);
    motor.enableVoltageCompensation(Constants.ShooterConstants.voltageComp);
    
    motor.burnFlash();
    encoder.setPosition(0.0);
  }

  public void stopMotors() {
    runShooterVel = false;
    topController.setReference(0, ControlType.kVelocity);
    bottomController.setReference(0, ControlType.kVelocity);
    topRoller.stopMotor();
    bottomRoller.stopMotor();
    topSpeedLimiter.reset(0);
    bottomSpeedLimiter.reset(0);
  }

  public Command stopShooterCommand() {
    return Commands.runOnce(() -> stopMotors());
  }

  public Command startShooterCommand(double rpm) {
    return Commands.parallel(
        Commands.runOnce(() -> commandRPM = rpm),
        Commands.runOnce(() -> setRunShooter(), this));
  }

  public Command startShooterCommandAmp(double rpm) {
    return Commands.parallel(
        Commands.runOnce(() -> commandRPM = rpm),
        Commands.runOnce(() -> setRunShooter(), this));
  }

  public void setRunShooter() {
    runShooterVel = true;
  }

  public void resetRunShooter() {
    runShooterVel = false;
  }

  public boolean getRunShooter() {
    return runShooterVel;
  }

  public void setCommandRPM(double rpm) {
    commandRPM = rpm;
  }

  public double getCommandRPM() {
    return commandRPM;
  }

  public double getRPMTop() {
    if (RobotBase.isReal())
      return topEncoder.getVelocity();
    if (RobotBase.isSimulation() && runShooterVel) {
      return commandRPM;
    } else
      return 0;
  }

  public double getRPMBottom() {
    if (RobotBase.isReal())
      return bottomEncoder.getVelocity();
    if (RobotBase.isSimulation() && runShooterVel) {
      return commandRPM;
    } else
      return 0;
  }

  public void increaseShooterRPM(double val) {
    commandRPM += val;
    if (commandRPM > ShooterConstants.maxShooterMotorRPM)
      commandRPM = ShooterConstants.maxShooterMotorRPM;
  }

  public Command increaseRPMCommand(double val) {
    return Commands.runOnce(() -> increaseShooterRPM(val));
  }

  public void decreaseShooterRPM(double val) {
    commandRPM -= val;
    if (commandRPM < ShooterConstants.minShooterMotorRPM)
      commandRPM = ShooterConstants.minShooterMotorRPM;
  }

  public Command decreaseRPMCommand(double val) {
    return Commands.runOnce(() -> decreaseShooterRPM(val));
  }

  public boolean topAtSpeed(double pct) {
    return commandRPM != 0 && Math.abs(commandRPM - getRPMTop()) < commandRPM / 4;
  }

  public boolean bottomAtSpeed(double pct) {
    return commandRPM != 0 && Math.abs(commandRPM - getRPMBottom()) < commandRPM * pct;
  }

  public boolean bothAtSpeed(double pct) {
    return topAtSpeed(pct) && bottomAtSpeed(pct);
  }

  public void setShooterSpeedRatio(double ratio) {
    if (ratio < 0 || ratio > 1.2)
      ratio = 1;
    topBottomSpeedRatio = ratio;
  }

  public Command setShooterRatioCommand(double ratio) {
    return Commands.runOnce(() -> setShooterSpeedRatio(ratio));
  }

  public Command resetShooterRatioCommand() {
    return Commands.runOnce(() -> setShooterSpeedRatio(1));
  }

  public double getShooterSpeedRatio() {
    return topBottomSpeedRatio;
  }

  public double getTopAmps() {
    return topRoller.getOutputCurrent();
  }

  public double getBottomAmps() {
    return bottomRoller.getOutputCurrent();
  }

  public Command clearFaultsCommand() {
    return Commands.parallel(
        Commands.runOnce(() -> topRoller.clearFaults()),
        Commands.runOnce(() -> bottomRoller.clearFaults()));
  }

  public int getFaults() {
    return topRoller.getFaults() + bottomRoller.getFaults();
  }

  public int getStickyFaults() {
    return topRoller.getStickyFaults() + bottomRoller.getStickyFaults();
  }

  @Override
  public void periodic() {
    // topBottomSpeedRatio = Pref.getPref("ShooterSpeedRatio");

    // SmartDashboard.putNumber("TOPVolts", topRoller.get() * RobotController.getBatteryVoltage());
    // SmartDashboard.putNumber("BotVolts", bottomRoller.get() * RobotController.getBatteryVoltage());

    if (runShooterVel) {
      double bottomrpm = getCommandRPM();
      double toprpm = bottomrpm * topBottomSpeedRatio;

      topController.setReference(topSpeedLimiter.calculate(toprpm), ControlType.kVelocity, 0);
      bottomController.setReference(bottomSpeedLimiter.calculate(bottomrpm), ControlType.kVelocity, 0);
    } else {
      stopMotors();
    }
  }

  public double rpmTrackDistance(double meters) {
    return Constants.shooterRPMMap.get(meters);
  }

  public static double round2dp(double number, int dp) {
    double temp = Math.pow(10, dp);
    double temp1 = Math.round(number * temp);
    return temp1 / temp;
  }

  public Command setTopKpKdKiCommand() {
    return Commands.runOnce(() -> setTopKpKdKi());
  }

  public Command setBottomKpKdKiCommand() {
    return Commands.runOnce(() -> setBottomKpKdKi());
  }

  public void setTopKpKdKi() {

    topController.setFF(Constants.ShooterConstants.topShooterKFF, 0);
    topController.setP(Pref.getPref("ShooterTopKp"), 0);
    topController.setD(Pref.getPref("ShooterTopKd"), 0);
    topController.setI(Pref.getPref("ShooterTopKi"), 0);
  }

  public void setBottomKpKdKi() {
    bottomController.setFF(Constants.ShooterConstants.bottomShooterKFF);
    bottomController.setP(Pref.getPref("ShooterBottomKp"), 0);
    bottomController.setD(Pref.getPref("ShooterBottomKd"), 0);
    bottomController.setI(Pref.getPref("ShooterBottomKi"), 0);
  }

  private final SysIdRoutine sysIdRoutine = new SysIdRoutine(
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          (volts) -> {
            bottomRoller.setVoltage(volts.in(Volts));
            topRoller.setVoltage(volts.in(Volts));
          },
          // log -> {
          // // Record a frame for the shooter motor.
          // log.motor("shooter-wheel")
          // .voltage(
          // m_appliedVoltage.mut_replace(
          // m_shooterMotor.get() * RobotController.getBatteryVoltage(), Volts))
          // .angularPosition(m_angle.mut_replace(m_shooterEncoder.getDistance(),
          // Rotations))
          // .angularVelocity(
          // m_velocity.mut_replace(m_shooterEncoder.getRate(), RotationsPerSecond));
          // },
          log -> {
            log.motor("Top")
                .voltage(Volts.of(topRoller.getAppliedOutput() * topRoller.getBusVoltage()))
                .angularVelocity(Rotations.per(Minute).of(topEncoder.getVelocity()))
                .angularPosition(Rotations.of(topEncoder.getPosition()));
            // log.motor("Bottom")
            // .voltage(Volts.of(bottomRoller.getAppliedOutput() *
            // bottomRoller.getBusVoltage()))
            // .angularVelocity(Rotations.per(Minute).of(bottomEncoder.getVelocity()))
            // .angularPosition(Rotations.of(bottomEncoder.getPosition()));
          },
          this));

  public Command quasistaticForward() {
    return sysIdRoutine.quasistatic(Direction.kForward);
  }

  public Command quasistaticBackward() {
    return sysIdRoutine.quasistatic(Direction.kReverse);
  }

  public Command dynamicForward() {
    return sysIdRoutine.dynamic(Direction.kForward);
  }

  public Command dynamicBackward() {
    return sysIdRoutine.dynamic(Direction.kReverse);
  }

}