/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.Measure;
import org.growingstems.math.Pose2dU;
import org.growingstems.math.Vector2dU;
import org.growingstems.measurements.Angle;
import org.growingstems.measurements.Measurements.Length;
import org.growingstems.measurements.Measurements.Time;
import org.growingstems.measurements.Measurements.Voltage;

public class ConversionUtils {
  public static Pose2dU<Length> fromWpi(
      edu.wpi.first.math.geometry.Pose2d pose2d, Length.Unit lengthUnit) {
    return new Pose2dU<Length>(
        new Length(pose2d.getX(), lengthUnit),
        new Length(pose2d.getY(), lengthUnit),
        fromWpi(pose2d.getRotation()));
  }

  public static Pose2dU<Length> fromWpi(edu.wpi.first.math.geometry.Pose2d pose2dMeters) {
    return fromWpi(pose2dMeters, Length.Unit.METERS);
  }

  public static Vector2dU<Length> fromWpi(
      edu.wpi.first.math.geometry.Translation2d translation2d, Length.Unit lengthUnit) {
    return new Vector2dU<Length>(
        new Length(translation2d.getX(), lengthUnit), new Length(translation2d.getY(), lengthUnit));
  }

  public static Vector2dU<Length> fromWpi(
      edu.wpi.first.math.geometry.Translation2d translation2dMeters) {
    return fromWpi(translation2dMeters, Length.Unit.METERS);
  }

  public static Angle fromWpi(edu.wpi.first.math.geometry.Rotation2d rotation2d) {
    return Angle.radians(rotation2d.getRadians());
  }

  public static edu.wpi.first.math.geometry.Pose2d toWpi(
      Pose2dU<Length> pose, Length.Unit lengthUnit) {
    return new edu.wpi.first.math.geometry.Pose2d(
        pose.getX().getValue(lengthUnit),
        pose.getY().getValue(lengthUnit),
        toWpi(pose.getRotation()));
  }

  public static edu.wpi.first.math.geometry.Pose2d toWpiMeters(Pose2dU<Length> pose) {
    return toWpi(pose, Length.Unit.METERS);
  }

  public static edu.wpi.first.math.geometry.Rotation2d toWpi(Angle angle) {
    return new edu.wpi.first.math.geometry.Rotation2d(angle.asRadians());
  }

  public static edu.wpi.first.math.geometry.Translation2d toWpi(
      Vector2dU<Length> vector, Length.Unit lengthUnit) {
    return new edu.wpi.first.math.geometry.Translation2d(
        vector.getX().getValue(lengthUnit), vector.getY().getValue(lengthUnit));
  }

  public static edu.wpi.first.math.geometry.Translation2d toWpi(Vector2dU<Length> vector) {
    return toWpi(vector, Length.Unit.METERS);
  }

  public static Measure<edu.wpi.first.units.Voltage> toWpi(Voltage voltage) {
    return Volts.of(voltage.asVolts());
  }

  public static Voltage fromWpiVoltage(Measure<edu.wpi.first.units.Voltage> voltage) {
    return Voltage.volts(voltage.baseUnitMagnitude());
  }

  public static Measure<edu.wpi.first.units.Time> toWpi(Time time) {
    return Seconds.of(time.asSeconds());
  }

  public static Time fromWpiTime(Measure<edu.wpi.first.units.Time> time) {
    return Time.seconds(time.baseUnitMagnitude());
  }
}
