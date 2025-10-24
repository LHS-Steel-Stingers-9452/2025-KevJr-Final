package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.DutyCycleOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(11); // Should be the right id
    private final DutyCycleOut control = new DutyCycleOut(0);

    public Intake() {
        // intakeMotor.setInverted(true); // Uncomment if intake runs backwards
    }

    /** Spins intake inwards (pulls coral in). */
    public void intakeIn() {
        intakeMotor.setControl(control.withOutput(7)); // Adjust power as needed
    }

    /** Spins intake outward (outtakes coral). */
    public void intakeOut() {
        intakeMotor.setControl(control.withOutput(-1.2)); // Adjust for your needs
    }

    /** Stops intake motor. */
    public void stop() {
        intakeMotor.setControl(control.withOutput(0));
    }
}
