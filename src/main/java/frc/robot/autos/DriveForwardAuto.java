package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.Units;

public class DriveForwardAuto extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private Pose2d startingPose;
    private final double targetDistanceMeters = Units.Feet.of(5).in(Units.Meters); // 5 ft = 1.524 m (5ft TAXI DRIVE)
    private final double driveSpeed = 1.0; // meters per second (adjust if too fast)
    private final SwerveRequest.FieldCentric driveRequest = new SwerveRequest.FieldCentric();

    public DriveForwardAuto(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        startingPose = drivetrain.getState().Pose; // Save starting position
    }

    @Override
    public void execute() {
        drivetrain.setControl(
            driveRequest.withVelocityX(driveSpeed) // Forward
                        .withVelocityY(0)          // No strafe
                        .withRotationalRate(0)     // No rotation
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(new SwerveRequest.Idle()); // Stop movement
    }

    @Override
    public boolean isFinished() {
        Pose2d currentPose = drivetrain.getState().Pose;
        double distanceTraveled = currentPose.getTranslation()
                                             .getDistance(startingPose.getTranslation());
        return distanceTraveled >= targetDistanceMeters;
    }
}
