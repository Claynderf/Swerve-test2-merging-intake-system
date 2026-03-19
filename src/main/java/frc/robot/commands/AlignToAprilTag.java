package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import static edu.wpi.first.units.Units.MetersPerSecond;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class AlignToAprilTag extends Command { 
    private final CommandSwerveDrivetrain drivetrain;

    private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric()
          .withDriveRequestType(DriveRequestType.Velocity);
  private static final double kP_turn = 0.05; 
  private static final double kP_forward = 0.08;

  // tx = left/right error, should end at 0
   private static final double targetTX = 0.0;

  // ty = how high/low target appears; tune this for desired stopping distance
   private static final double targetTY = 5.0;

  private final double maxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  private final double maxTurnRate = 1.5;

  public AlignToAprilTag(CommandSwerveDrivetrain drivetrain) {
  this.drivetrain = drivetrain;
  addRequirements(drivetrain);
  }

  @Override
  public void execute() {
  if (!drivetrain.hasDesiredAprilTag()) {
  drivetrain.setControl(
  driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
  );
  return;
 }

 double tx = drivetrain.getAprilTagTX();
 double ty = drivetrain.getAprilTagTY();
 double txError = targetTX - tx;
 double tyError = targetTY - ty;

 double turnOutput = MathUtil.clamp(txError * kP_turn, -maxTurnRate, maxTurnRate);
 double forwardOutput = MathUtil.clamp(tyError * kP_forward, -maxSpeed, maxSpeed);

 if (Math.abs(txError)<1.0){
  turnOutput = 0.0;
 }
 if (Math.abs(tyError)<1.0){
  forwardOutput = 0.0;
 }

 drivetrain.setControl(
 driveRequest
 .withVelocityX(-forwardOutput)
 .withVelocityY(0.0)
 .withRotationalRate(turnOutput)
 );
 }

 @Override
 public void end(boolean interrupted) {
 drivetrain.setControl(
 driveRequest.withVelocityX(0).withVelocityY(0).withRotationalRate(0)
 );
 }

 @Override
 public boolean isFinished() {
 return false;
 }
}