package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    private final TalonFX armMotor = new TalonFX(10); // Change ID if needed
    private final DutyCycleOut control = new DutyCycleOut(0);

    private double targetAngle = 0; // Target position (degrees)
    private static final double MAX_ANGLE = 120; // Adjust using Advanced Scope
    private static final double MIN_ANGLE = 0; //Adjust using Advanced scope
    private static final double P = 0.004; // 0.003–0.005 recommended for smooth hold
    private static final double GEAR_RATIO = 1.0; // Adjust to the GR of the arm
    private static final double TOLERANCE = 1.0; // degrees

    private boolean initialized = false;

    public Arm() {
        // Hold position when idle — prevents arm from falling
        armMotor.setNeutralMode(NeutralModeValue.Brake);
        // Optional: invert if moving in wrong direction
        // armMotor.setInverted(true);
    }

    /** Command to move arm toward a specific angle (in degrees). */
    public void moveToAngle(double angle) {
        targetAngle = clamp(angle, MIN_ANGLE, MAX_ANGLE);
    }

    @Override
    public void periodic() {
        // On first enable, lock arm at current angle
        if (!initialized) {
            targetAngle = getArmAngle();
            initialized = true;
        }

        double currentAngle = getArmAngle();
        double error = targetAngle - currentAngle;

        double output = 0;
        if (Math.abs(error) > TOLERANCE) {
            output = clamp(error * P, -1, 1);
        }
        armMotor.setControl(control.withOutput(output));

        // encoder value print for testing
        System.out.println("Arm Angle (deg): " + String.format("%.2f", currentAngle));

        //Display encoder position and angle live
        SmartDashboard.putNumber("Arm Encoder (Rotations)", armMotor.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("Arm Angle (Degrees)", currentAngle);
        SmartDashboard.putNumber("Arm Target Angle", targetAngle);
    }

    /** Reads the arm’s current angle in degrees. */
    public double getArmAngle() {
        return armMotor.getPosition().getValueAsDouble() * (360.0 / GEAR_RATIO);
    }

    /** Optional manual stop (resumes holding next loop). */
    public void stop() {
        armMotor.setControl(control.withOutput(0));
    }

    /** Helper for keeping values within bounds. */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
