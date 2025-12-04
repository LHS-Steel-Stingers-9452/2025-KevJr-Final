package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {

    private final TalonFX armMotor;

    private double targetAngle = 21;     // Desired arm angle in degrees
    private double currentSetpoint = 0.0; // Ramped arm angle
    private final double rampRate = 2.0;  // Degrees per loop

    private final double minAngle = 110;   // Safe minimum
    private final double maxAngle = 13; // Safe maximum

    private final double deadband = 1.0;   // Degrees within target
    private final double maxPower = 0.2;   // Motor power (0–1)

    private final double gearRatio = 45;  // Replace with the actual gearbox ratio

    private double softwareOffset = 0.0;   // Software zero offset

    private PositionVoltage positionRequest = new PositionVoltage(0).withSlot(0);

    public Arm() {
        armMotor = new TalonFX(14); // Replace with the motor ID 14
        
        var motorOutputConfig = new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Coast);
        armMotor.getConfigurator().apply(motorOutputConfig);

        var Slot0Configs =
        new Slot0Configs()
            .withKP(0.3);

        armMotor.getConfigurator().apply(Slot0Configs);
        

        //Setting encoder Boundry Limits
  

        // Initialize setpoint to current arm position
        // currentSetpoint = getCurrentAngle();
        // targetAngle = currentSetpoint;
        setDefaultCommand(runArm(0));
    }

    public Command runArm(double speed){
        return run(() -> {
            armMotor.set(speed);
        });
        }

    public Command setPosition(double position) {
        return run(()-> {
            positionRequest.withPosition(position);
            armMotor.setControl(positionRequest);
        });
    }

    /** Get current arm angle in degrees using motor rotations and gear ratio, applying software offset */
    public double getCurrentAngle() {
        // double motorRotations = armMotor.getRotorPosition().getValueAsDouble(); // numeric rotations
        // return (motorRotations / gearRatio) * 360.0 - softwareOffset;
        return armMotor.getRotorPosition().getValueAsDouble();

    }

    /** Treat current arm position as 0 degrees (software zero) */
    public void zeroEncoder() {
        // softwareOffset = (armMotor.getRotorPosition().getValueAsDouble() / gearRatio) * 360.0;
        armMotor.setPosition(0);
        // System.out.println("Arm software encoder zeroed at current position.");
    }

    /** Move arm to a target angle */
    public void moveToAngle(double angle) {
        targetAngle = Math.max(minAngle, Math.min(maxAngle, angle));
    }

    @Override
    public void periodic() {
        // Ramp toward target angle smoothly
        // if (currentSetpoint < targetAngle) {
        //     currentSetpoint = Math.min(currentSetpoint + rampRate, targetAngle);
        // } else if (currentSetpoint > targetAngle) {
        //     currentSetpoint = Math.max(currentSetpoint - rampRate, targetAngle);
        // }

        // Compute error for deadband
        // double error = targetAngle - getCurrentAngle();

        // // Stop motor if within deadband
        // if (Math.abs(error) < deadband) {
        //     armMotor.setControl(new DutyCycleOut(0));
        // } else {
        //     // Apply motor power toward target
        //     double power = Math.signum(error) * maxPower;
        //     armMotor.setControl(new DutyCycleOut(power));
        // }
        // System print to continuously display arm angle
        //System.out.println("Arm Angle: " + getCurrentAngle() + "°, Target: " + targetAngle);

        // SmartDashboard telemetry
        SmartDashboard.putNumber("Arm Angle", getCurrentAngle());
        // SmartDashboard.putNumber("Arm Target", targetAngle);
        SmartDashboard.putNumber("Arm Voltage", armMotor.getMotorVoltage().getValueAsDouble());
    }
}
