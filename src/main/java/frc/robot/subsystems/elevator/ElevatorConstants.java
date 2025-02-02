package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.util.ElevatorMath;

public class ElevatorConstants {

  public static final DCMotor m_elevatorMotors = DCMotor.getNEO(2);
  public static final double kElevatorGearing = 6;
  public static final double kCarriageMassKg = Units.lbsToKilograms(16.142);
  public static final double kElevatorDrumRadiusMeters = Units.inchesToMeters(0.829 / 2);
  public static final double kMinElevatorHeightMeters = 0;
  public static final double kMaxElevatorHeightMeters = Units.inchesToMeters(59.035);

  public static final int motorId1 = 11;
  public static final int motorId2 = 12;

  public static final int currentLimit = 40;
  public static final double kP = 4;
  public static final double kD = 0;

  public static final double maxVelocity =
      ElevatorMath.convertDistanceToRotations(Meters.of(5)).per(Second).in(RPM);
  public static final double maxAcceleration =
      ElevatorMath.convertDistanceToRotations(Meters.of(10))
          .per(Second)
          .per(Second)
          .in(RPM.per(Second));

  public static final double allowedError =
      ElevatorMath.convertDistanceToRotations(Meters.of(0.01)).per(Second).in(RPM);
}
