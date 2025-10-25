package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Climb;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.autos.DriveForwardAuto;

public class RobotContainer {
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    // Subsystems
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    private final Arm arm = new Arm();
    private final Intake intake = new Intake();
    private final Climb climb = new Climb();

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Default drivetrain command with speed scaling
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                // Use trigger state from the controller directly
                boolean slowMode = joystick.leftBumper().getAsBoolean();
                double speedScale = slowMode ? 0.4 : 1.0;
                return drive
                    .withVelocityX(-joystick.getLeftY() * MaxSpeed * speedScale)  // forward/back
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * speedScale)  // strafe
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate * speedScale);
            })
        );

        // Disabled idle
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Field-centric reset on left bumper press
        joystick.leftTrigger().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Swerve wheel pointing
        // Straighten wheels to face field forward when POV Down is pressed
        joystick.povDown().whileTrue(drivetrain.applyRequest(() ->
        point.withModuleDirection(new Rotation2d(0)) // 0 radians = facing field forward
        ));


        // Arm preset positions (tap to set)
        joystick.x().onTrue(new InstantCommand(() -> arm.moveToAngle(0), arm));   // Intake from floor
        joystick.a().onTrue(new InstantCommand(() -> arm.moveToAngle(90), arm));  // Score position
        joystick.y().onTrue(new InstantCommand(() -> arm.moveToAngle(120), arm)); // Fully upright

        // Intake in (hold to run)
        joystick.rightTrigger()
        .whileTrue(new RunCommand(() -> intake.intakeIn(), intake))
        .onFalse(new RunCommand(() -> intake.stop(), intake));

        // Intake out (hold to run)
        joystick.rightBumper()
        .whileTrue(new RunCommand(() -> intake.intakeOut(), intake))
        .onFalse(new RunCommand(() -> intake.stop(), intake));

        // Climber controls
        joystick.povRight().whileTrue(climb.runClimber(1));   // Extend
        joystick.povLeft().whileTrue(climb.runClimber(-1));   // Retract

        // Telemetry
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return new DriveForwardAuto(drivetrain);
    }
}
