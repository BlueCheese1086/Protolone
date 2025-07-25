// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

public class SplineInterpolation {
  double[] distances = {2.14, 2.17, 2.39, 2.55, 2.88, 3.0, 3.144};
  double[] angles = {0.26, 0.32, 0.245, 0.38, 0.197, 0.28, 0.39};
  double[] powers = {280, 290, 270, 380, 380, 400, 400};

  SplineInterpolator interpolator = new SplineInterpolator();

  PolynomialSplineFunction angleSpline = interpolator.interpolate(distances, angles);
  PolynomialSplineFunction powerSpline = interpolator.interpolate(distances, powers);

  public void interpolate() {
    double distance = 2.6;

    double interpolatedAngle = angleSpline.value(distance);
    double interpolatedPower = powerSpline.value(distance);

    System.out.println("Interpolated Angle: " + interpolatedAngle);
    System.out.println("Interpolated Power: " + interpolatedPower);
  }
}
