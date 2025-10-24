package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Arm extends SubsystemBase {
    private final TalonFX armMotor = new TalonFX(10); // Change ID to arm motor
    private final DutyCycleOut control = new DutyCycleOut(0);

    private double targetAngle = 0; // target angle in degrees
    private final double MAX_ANGLE = 120;
    private final double MIN_ANGLE = 0;
    private final double P = 0.01; // proportional constant for simple position control, 0.02 faster and 0.005 slower

    public Arm() {
        // Set inversion if needed to reverse
        // armMotor.setInverted(true);

         // Enable brake mode so the motor resists motion when idle
        armMotor.setNeutralMode(com.ctre.phoenix6.signals.NeutralModeValue.Brake);
    }

    /** Moves the arm manually with speed (-1 to 1). */
    public void moveArm(double speed) {
        armMotor.setControl(control.withOutput(speed));
        // Update target angle based on manual motion
        targetAngle = clamp(targetAngle + speed, MIN_ANGLE, MAX_ANGLE);
    }

    /** Moves the arm toward a target angle (degrees) */
    public void moveToAngle(double angle) {
        targetAngle = clamp(angle, MIN_ANGLE, MAX_ANGLE);
        double currentAngle = getArmAngle();
        double output = (targetAngle - currentAngle) * P;

         // Minimum holding power (adjust 0.05 based on arm weight) If this causes problems // it out.
        if (Math.abs(targetAngle - currentAngle) < 2) { // small error range in degrees
        output = Math.copySign(Math.max(Math.abs(output), 0.05), output); //Test and configure (If arm sags, raise MORE POWERRRR!)
        }

        output = clamp(output, -1, 1);
        armMotor.setControl(control.withOutput(output));
    }

    /** Stops the arm motor. */
    public void stop() {
        armMotor.setControl(control.withOutput(0));
    }

    /** Returns the current arm angle (degrees). Replace with your encoder logic. */
    public double getArmAngle() {
        return armMotor.getPosition().getValueAsDouble(); // adjust if needed
    }

    /** Clamp a value between min and max. */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
    // Test this too see if it actually works. If not you can always // it out lol
    /** Increment the target angle manually (e.g., with triggers). */
    public void incrementTarget(double delta) {
        moveToAngle(targetAngle + delta);
    }
}
