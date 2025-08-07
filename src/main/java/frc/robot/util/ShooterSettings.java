package frc.robot.util;

import java.util.HashMap;
import java.util.TreeMap;
import org.littletonrobotics.junction.Logger;

public record ShooterSettings(double angle, double power) {
  public static HashMap<Double, ShooterSettings> settingsMap = new HashMap<>();
  private static TreeMap<Double, Double> acceptedAngle = new TreeMap<>();
  private static TreeMap<Double, Double> acceptedPower = new TreeMap<>();

  public double getAngle(double distance) {
    return settingsMap.get(distance).angle;
  }

  public double getPower(double distance) {
    return settingsMap.get(distance).power;
  }

  public static double interpolateAngle(double currentDistance) {
    acceptedAngle.clear();
    settingsMap.forEach(
        (distance, settings) -> {
          acceptedAngle.put(distance, settings.angle);
        });

    Double floorKey = acceptedAngle.floorKey(currentDistance);
    Double ceilingKey = acceptedAngle.ceilingKey(currentDistance);

    if (acceptedAngle.isEmpty()) {
      throw new IllegalStateException("Angle settings map is empty!");
    }

    if (currentDistance <= acceptedAngle.firstKey()) {
      return acceptedAngle.get(acceptedAngle.firstKey());
    }

    if (currentDistance >= acceptedAngle.lastKey()) {
      return acceptedAngle.get(acceptedAngle.lastKey());
    }

    double floorVal = acceptedAngle.get(floorKey); // 0.38
    double ceilingVal = acceptedAngle.get(ceilingKey); // 0.18
    // System.out.println((floorVal + ceilingVal) / 2);
    Logger.recordOutput("AutoAim/TargetAngle", ((floorVal + ceilingVal) / 2));
    return -((floorVal + ceilingVal) / 2);
  }

  public static double interpolatePower(double currentDistance) {
    acceptedPower.clear();
    settingsMap.forEach(
        (distance, settings) -> {
          acceptedPower.put(distance, settings.power);
        });

    Double floorKey = acceptedPower.floorKey(currentDistance);
    Double ceilingKey = acceptedPower.ceilingKey(currentDistance);

    if (floorKey == null || ceilingKey == null) {
      throw new IllegalArgumentException(
          "Not enough data points to interpolate angle at distance: " + currentDistance);
    }

    double floorVal = acceptedPower.get(floorKey); // 0.38
    double ceilingVal = acceptedPower.get(ceilingKey); // 0.18
    Logger.recordOutput("AutoAim/InterpolatedPower", (floorVal + ceilingVal) / 2);
    return ((floorVal + ceilingVal) / 2);
  }

  public static double interpolate(double lowX, double highX, double targetX) {
    double power = 2;

    if (targetX == lowX) return lowX;
    if (targetX == highX) return highX;

    double distLow = Math.abs(targetX - lowX);
    double distHigh = Math.abs(targetX - highX);

    double weightLow = 1.0 / Math.pow(distLow, power);
    double weightHigh = 1.0 / Math.pow(distHigh, power);

    double numerator = weightLow * lowX + weightHigh * highX;
    double denominator = weightLow + weightHigh;

    return numerator / denominator;
  }
}
