package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.subsystems.shooter.ShooterConstants.dcMotorMOI;
import static frc.robot.subsystems.shooter.ShooterConstants.feedMotorGearbox;
import static frc.robot.subsystems.shooter.ShooterConstants.shootKd;
import static frc.robot.subsystems.shooter.ShooterConstants.shootKp;
import static frc.robot.subsystems.shooter.ShooterConstants.shootMotorGearbox;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import org.littletonrobotics.junction.networktables.LoggedNetworkBoolean;

public class ShooterIOSim implements ShooterIO {
  private final DCMotorSim shootMotor;
  private final DCMotorSim feedMotor;
  private final PIDController shootController;

  private final LoggedNetworkBoolean noteDetected =
      new LoggedNetworkBoolean("Tuning/Shooter/SimNotedDetected", false);
  private boolean feedRunningForward = false;
  private boolean feedRunningBackward = false;

  public ShooterIOSim() {
    shootMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(shootMotorGearbox, dcMotorMOI, 1.0),
            shootMotorGearbox);
    feedMotor =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(feedMotorGearbox, dcMotorMOI, 1.0),
            feedMotorGearbox);
    shootController = new PIDController(shootKp, 0.0, shootKd);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    shootMotor.update(0.02);
    inputs.feedConnected = true;
    inputs.shootConnected = true;

    inputs.shootVelocityRadPerSec = shootMotor.getAngularVelocity().in(RadiansPerSecond);
    inputs.shootPositionRad = shootMotor.getAngularPositionRad();

    inputs.feedVelocityRadPerSec = feedMotor.getAngularVelocityRadPerSec();

    inputs.feedAppliedVolts = feedMotor.getInputVoltage();
    inputs.shootAppliedVolts = shootMotor.getInputVoltage();

    inputs.feedCurrentAmps = feedMotor.getCurrentDrawAmps();
    inputs.shootCurrentAmps = shootMotor.getCurrentDrawAmps();

    inputs.noteDetected = noteDetected.get();
  }

  public void setFeedOpenLoop(double output) {
    feedMotor.setInputVoltage(output);
  }

  public void setShootOpenLoop(double output) {
    shootMotor.setInputVoltage(output);
  }

  // public void setShootVelocity(double velocityRadPerSec) {
  // setShootVelocity(velocityRadPerSec, 0.0);
  // }

  public void setShootVelocity(double velocityRadPerSec, double feedforward) {
    // Multiply by 12.0 to match spark max pid
    shootMotor.setInputVoltage(
        MathUtil.clamp(
            shootController.calculate(
                        shootMotor.getAngularVelocity().in(RadiansPerSecond), velocityRadPerSec)
                    * 12.0
                + feedforward,
            -12.0,
            12.0));
  }
}
