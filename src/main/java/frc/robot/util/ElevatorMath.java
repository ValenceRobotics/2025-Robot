package frc.robot.util;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.elevator.ElevatorConstants;

public class ElevatorMath {
  /**
   * Convert {@link Distance} into {@link Angle}
   *
   * @param distance Distance, usually Meters.
   * @return {@link Angle} equivalent to rotations of the motor.
   */
  public static Angle convertDistanceToRotations(Distance distance) {
    return Rotations.of(
        distance.in(Meters)
            / (ElevatorConstants.kElevatorDrumRadiusMeters * 2 * Math.PI)
            * ElevatorConstants.kElevatorGearing);
  }

  /**
   * Convert {@link Angle} into {@link Distance}
   *
   * @param rotations Rotations of the motor
   * @return {@link Distance} of the elevator.
   */
  public static Distance convertRotationsToDistance(Angle rotations) {
    return Meters.of(
        (rotations.in(Rotations) / ElevatorConstants.kElevatorGearing)
            * (ElevatorConstants.kElevatorDrumRadiusMeters * 2 * Math.PI));
  }
}
