package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopSwerve extends Command {
  private SwerveSubsystem s_Swerve;
  private DoubleSupplier translationSup;
  private DoubleSupplier strafeSup;
  private DoubleSupplier rotationSup;
  private BooleanSupplier fieldCentric;
  private BooleanSupplier keepAngle;
  private SlewRateLimiter translationLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter strafeLimiter = new SlewRateLimiter(3.0);
  private SlewRateLimiter rotationLimiter = new SlewRateLimiter(3.0);

  public TeleopSwerve(
      SwerveSubsystem s_Swerve,
      DoubleSupplier translationSup,
      DoubleSupplier strafeSup,
      DoubleSupplier rotationSup,
      BooleanSupplier fieldCentric,
      BooleanSupplier keepAngle) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);

    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.rotationSup = rotationSup;
    this.fieldCentric = fieldCentric;
    this.keepAngle = keepAngle;
  }

  @Override
  public void execute() {
    /* Get Values, Deadband */
    double translationVal = translationLimiter.calculate(
        MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));
    double strafeVal = strafeLimiter.calculate(
        MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));
    double rotationVal = rotationLimiter.calculate(
        MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.SwerveConstants.stickDeadband));

    /* Drive */
    s_Swerve.drive(

        translationVal *= Constants.SwerveConstants.kmaxSpeed,
        strafeVal *= Constants.SwerveConstants.kmaxSpeed,
        rotationVal *= Constants.SwerveConstants.kmaxAngularVelocity,
        !fieldCentric.getAsBoolean(),
        true,
        keepAngle.getAsBoolean());

    // SmartDashboard.putBoolean("FieldCentric", fieldCentric.getAsBoolean());
    // SmartDashboard.putNumber("TransVal", translationVal);
    // SmartDashboard.putNumber("StrafeVal", strafeVal);
    // SmartDashboard.putNumber("RotVal", rotationVal);
  
    

  }
}
