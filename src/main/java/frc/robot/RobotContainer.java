// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AutoLaunch10Sec;
import frc.robot.commands.AutoLaunch5Sec;
import frc.robot.commands.Eject;
import frc.robot.commands.Intake;
import frc.robot.commands.LaunchSequence;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANFuelSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;



public class RobotContainer {
    private double MaxSpeed = 0.75 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    //private final SlewRateLimiter xLimiter = new SlewRateLimiter(3); 

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
private final CANFuelSubsystem fuelSubsystem = new CANFuelSubsystem();
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        // Register Named Commands
        
        NamedCommands.registerCommand("AutoLaunch5Sec", new AutoLaunch5Sec(fuelSubsystem));
        NamedCommands.registerCommand("AutoLaunch10Sec", new AutoLaunch10Sec(fuelSubsystem));

        configureBindings();

        autoChooser = AutoBuilder.buildAutoChooser("Forward-Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);

        CameraServer.startAutomaticCapture();
        CameraServer.startAutomaticCapture();
        
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
       joystick.start().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
        joystick.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // While the left bumper on operator controller is held, intake Fuel
         joystick.leftBumper().whileTrue(new Intake(fuelSubsystem));
        // While the right bumper on the operator controller is held, spin up for 1
        // second, then launch fuel. When the button is released, stop.
        joystick.rightBumper().whileTrue(new LaunchSequence(fuelSubsystem));
        // While the A button is held on the operator controller, eject fuel back out
         // the intake
        joystick.x().whileTrue(new Eject(fuelSubsystem));

        
        joystick.b().whileTrue(
            
                drivetrain.applyRequest(() ->
                drive.withVelocityX(-LimelightHelpers.getTY("limelight") * -0.1) // Drive forward with negative Y (forward)
                    .withVelocityY(LimelightHelpers.getTX("limelight") * 0.05) // Drive left with negative X (left)
                    .withRotationalRate(-LimelightHelpers.getTX("limelight") * -0.05) // Drive counterclockwise with negative X (left)
            )
            );

            
            //joystick.b().whileTrue(new AlignToAprilTag(drivetrain));
           
        
        // Set the default command for the drive subsystem to the command provided by
        // factory with the values provided by the joystick axes on the driver
        // controller. The Y axis of the controller is inverted so that pushing the
        // stick away from you (a negative value) drives the robot forwards (a positive
        // value)
        //driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController));

        fuelSubsystem.setDefaultCommand(fuelSubsystem.run(() -> fuelSubsystem.stop()));

    }
 
    
    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    } 
}
