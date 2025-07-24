package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ShooterIOSpark implements ShooterIO {
  private final SparkBase feedSpark;
  private final SparkBase shootSpark;

  private final LaserCan laserCan;

  private final RelativeEncoder shootEncoder;

  private final SparkClosedLoopController shootController;

  public ShooterIOSpark() {
    feedSpark = new SparkMax(feedId, MotorType.kBrushless);
    shootSpark = new SparkMax(shootId, MotorType.kBrushless);

    shootEncoder = shootSpark.getEncoder();
    shootController = shootSpark.getClosedLoopController();

    var feedConfig = new SparkMaxConfig();
    feedConfig
        .inverted(wheelInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(80)
        .voltageCompensation(12.0);

    var shootConfig = new SparkMaxConfig();
    shootConfig
        .inverted(wheelInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(40)
        .voltageCompensation(12.0);
    shootConfig
        .encoder
        .positionConversionFactor(wheelEncoderPositionFactor)
        .velocityConversionFactor(wheelEncoderVelocityFactor);
    shootConfig.closedLoop.pidf(shootKp, 0.0, 0.0, 0.0);

    feedSpark.configure(feedConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    shootSpark.configure(
        shootConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    laserCan = new LaserCan(laserCanId);
    try {
      laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
      laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration Failed! " + e);
    }
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.feedConnected = true;
    inputs.shootConnected = true;

    inputs.shootVelocityRadPerSec = shootEncoder.getVelocity();
    inputs.shootPositionRad = shootEncoder.getPosition();

    inputs.feedAppliedVolts = feedSpark.getAppliedOutput() * feedSpark.getBusVoltage();
    inputs.shootAppliedVolts = shootSpark.getAppliedOutput() * shootSpark.getBusVoltage();

    inputs.feedCurrentAmps = feedSpark.getOutputCurrent();
    inputs.shootCurrentAmps = shootSpark.getOutputCurrent();

    Measurement measurement = laserCan.getMeasurement();

    if (measurement.status == LaserCanInterface.LASERCAN_STATUS_VALID_MEASUREMENT) {
      inputs.noteDetected = measurement.distance_mm < detectedTheshold;
    } else {
      inputs.noteDetected = false;
    }
  }

  public void setFeedOpenLoop(double output) {
    feedSpark.setVoltage(output);
  }

  public void setShootOpenLoop(double output) {
    shootSpark.setVoltage(output);
  }

  public void setShootVelocity(double velocityRadPerSec) {
    setShootVelocity(velocityRadPerSec, 0.0);
  }

  public void setShootVelocity(double velocityRadPerSec, double feedforward) {
    shootController.setReference(
        velocityRadPerSec,
        ControlType.kVelocity,
        ClosedLoopSlot.kSlot0,
        feedforward,
        ArbFFUnits.kVoltage);
  }
}
