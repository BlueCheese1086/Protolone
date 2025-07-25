package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.Interpolatable;

public record ShooterSettings(double angle, double power)
    implements Interpolatable<ShooterSettings> {

  @Override
  public ShooterSettings interpolate(ShooterSettings endValue, double t) {
    return new ShooterSettings(
        MathUtil.interpolate(this.angle(), endValue.angle(), t),
        MathUtil.interpolate(this.power(), endValue.power(), t));
  }
}
