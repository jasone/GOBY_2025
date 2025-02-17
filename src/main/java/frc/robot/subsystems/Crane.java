// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.util.Optional;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.CraneConstants;
import frc.robot.utilities.SparkUtil;
import frc.robot.utilities.TunablePIDF;
import frc.robot.utilities.ValueCache;

public class Crane extends SubsystemBase {
  private final SparkFlex m_pivotMotor;
  private final SparkFlex m_leftElevatorMotor;
  private final SparkFlex m_rightElevatorMotor;

  private final RelativeEncoder m_pivotEncoder;
  private final RelativeEncoder m_elevatorEncoder;

  private final SparkClosedLoopController m_pivotPID;
  private final SparkClosedLoopController m_leftElevatorPID;

  private final LaserCan m_laserCan;

  private final ValueCache<Double> m_pivotPositionCache;
  private final ValueCache<Double> m_pivotVelocityCache;
  private final ValueCache<Double> m_elevatorPositionCache;
  private final ValueCache<Double> m_elevatorVelocityCache;

  private Translation2d m_setpoint;

  private static final TunablePIDF pivotPIDF = new TunablePIDF("Crane.pivotPIDF",
    CraneConstants.kPivotPIDF);
  private static final TunablePIDF elevatorPIDF = new TunablePIDF("Crane.elevatorPIDF",
    CraneConstants.kElevatorPIDF);

  private ProfiledPIDController m_aController = new ProfiledPIDController(
    pivotPIDF.get().p(),
    pivotPIDF.get().i(),
    pivotPIDF.get().d(),
    new TrapezoidProfile.Constraints(0.0, 0.0) // Dynamically scaled.
  );
  private ProfiledPIDController m_hController = new ProfiledPIDController(
    elevatorPIDF.get().p(),
    elevatorPIDF.get().i(),
    elevatorPIDF.get().d(),
    new TrapezoidProfile.Constraints(0.0, 0.0) // Dynamically scaled.
  );
  private double m_angleSetpoint;
  private double m_heightSetpoint;

  private double m_angleTolerance;
  private double m_heightTolerance;

  private int m_currentSerialNum = 0;
  private double m_startTime = 0.0;
  private boolean m_isVelControlled = false;

  private enum State {
    CRANING,             // Normal crane operation.
    ESTIMATE_H,          // Estimate elevator height to inform homing (skipped if kCompeting).
                         // Low homing mode:
    LO_PIVOT_HOME,       //   Home the pivot.
    LO_ELEVATOR_RAPID,   //   Move close to elevator home (skipped if kCompeting).
    LO_ELEVATOR_HOME,    //   Home the elevator.
                         // High homing mode (disabled if kCompeting):
    HI_ELEVATOR_RAPID_A, //   Move the elevator high enough to allow pivot homing.
    HI_PIVOT_HOME,       //   Home the pivot.
    HI_PIVOT_0,          //   Move the pivot to the 0 (level) position.
    HI_ELEVATOR_RAPID_B, //   Move close to elevator home.
    HI_ELEVATOR_HOME,    //   Home the elevator.
    HI_PIVOT_MAX         //   Move the pivot the max (90 degrees up) position.
  }
  private State m_state;

  public Crane() {
    m_pivotMotor = new SparkFlex(CraneConstants.kPivotMotorID, MotorType.kBrushless);
    SparkUtil.configureMotor(m_pivotMotor, CraneConstants.kPivotMotorConfig);

    m_leftElevatorMotor = new SparkFlex(CraneConstants.kLeftElevatorMotorID, MotorType.kBrushless);
    SparkUtil.configureMotor(m_leftElevatorMotor, CraneConstants.kElevatorMotorConfig);

    m_rightElevatorMotor = new SparkFlex(CraneConstants.kRightElevatorMotorID, MotorType.kBrushless);
    SparkUtil.configureFollowerMotor(m_rightElevatorMotor, CraneConstants.kElevatorMotorConfig, m_leftElevatorMotor);

    m_pivotEncoder = m_pivotMotor.getEncoder();
    m_elevatorEncoder = m_leftElevatorMotor.getEncoder();

    m_pivotPID = m_pivotMotor.getClosedLoopController();
    m_leftElevatorPID = m_leftElevatorMotor.getClosedLoopController();

    m_laserCan = new LaserCan(CraneConstants.kLaserCanID);
    try {
      m_laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
      m_laserCan.setRegionOfInterest(CraneConstants.kRegionOfInterest);
      m_laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_100MS);
    } catch (ConfigurationFailedException e) {
      throw new UncheckedIOException("Failed to configure LaserCAN: " + e, new IOException());
    }

    m_pivotPositionCache =
      new ValueCache<Double>(() -> {
        return m_pivotEncoder.getPosition();
      }, CraneConstants.kValueCacheTtlMicroseconds);
    m_pivotVelocityCache =
      new ValueCache<Double>(() -> {
        return m_pivotEncoder.getVelocity();
      }, CraneConstants.kValueCacheTtlMicroseconds);
    m_elevatorPositionCache =
      new ValueCache<Double>(() -> {
        return m_elevatorEncoder.getPosition();
      }, CraneConstants.kValueCacheTtlMicroseconds);
    m_elevatorVelocityCache =
      new ValueCache<Double>(() -> {
        return m_elevatorEncoder.getVelocity();
      }, CraneConstants.kValueCacheTtlMicroseconds);

    m_state = Constants.kCompeting ? State.LO_PIVOT_HOME : State.ESTIMATE_H;
  }

  private int allocatePosSerialNum() {
    m_isVelControlled = false;
    int serialNum = m_currentSerialNum;
    m_currentSerialNum++;
    return serialNum;
  }

  // Only increases serial number once while controlling with velocity to avoid the serial num getting too large.
  private void allocateVelSerialNum() {
    if (!m_isVelControlled) {
      m_isVelControlled = true;
      m_currentSerialNum++;
    }
  }

  private void moveElevatorToImpl(double height, double heightTolerance) {
    m_heightSetpoint = height;
    m_heightTolerance = heightTolerance;
    m_leftElevatorPID.setReference(height, ControlType.kMAXMotionPositionControl,
      CraneConstants.kElevatorPosPIDFSlot.slot());
  }

  private void movePivotToImpl(double angle, double angleTolerance) {
    m_angleSetpoint = angle;
    m_angleTolerance = angleTolerance;
    m_pivotPID.setReference(angle, ControlType.kMAXMotionPositionControl,
      CraneConstants.kPivotPosPIDFSlot.slot());
  }

  public int moveElevatorTo(double height, double heightTolerance) {
    moveElevatorToImpl(height, heightTolerance);
    return allocatePosSerialNum();
  }

  public int movePivotTo(double angle, double angleTolerance) {
    movePivotToImpl(angle, angleTolerance);
    return allocatePosSerialNum();
  }

  // Tolerance is in absolute difference
  public int moveTo(double height, double angle, double heightTolerance, double angleTolerance) {
    m_heightSetpoint = height;
    m_angleSetpoint = angle;
    m_heightTolerance = heightTolerance;
    m_angleTolerance = angleTolerance;
    moveElevatorToImpl(height, heightTolerance);
    movePivotToImpl(angle, angleTolerance);
    return allocatePosSerialNum();
  }

  public void moveElevatorVel(double velocity) {
    m_leftElevatorPID.setReference(velocity, ControlType.kMAXMotionVelocityControl, CraneConstants.kElevatorVelPIDFSlot.slot());
    allocateVelSerialNum();;
  }

  public void movePivotVel(double velocity) {
    m_pivotPID.setReference(velocity, ControlType.kMAXMotionVelocityControl, CraneConstants.kPivotVelPIDFSlot.slot());
    allocateVelSerialNum();
  }

  private boolean pivotAtSetpoint() {
    double a = m_pivotEncoder.getPosition();
    return Math.abs(a - m_angleSetpoint) <= m_angleTolerance;
  }

  private boolean elevatorAtSetpoint() {
    double h = m_elevatorEncoder.getPosition();
    return Math.abs(h - m_heightSetpoint) <= m_heightTolerance;
  }

  public Optional<Integer> craneAtSetpoint() {
    double currentTime = (double)RobotController.getFPGATime() / 1_000_000.0;
    if (pivotAtSetpoint() && elevatorAtSetpoint()) {
      if (m_startTime == 0.0) {
        m_startTime = currentTime;
      } else if (currentTime - m_startTime >= CraneConstants.kDebouncingTime) {
        return Optional.of(m_currentSerialNum);
      }
    }
    return Optional.empty();
  }

  private void laserCanStatusError(int status) {
    String statusString;
    switch (status) {
      case LaserCan.LASERCAN_STATUS_NOISE_ISSUE: {
        statusString = "Noise issue";
        break;
      }
      case LaserCan.LASERCAN_STATUS_WEAK_SIGNAL: {
        statusString = "Weak signal";
        break;
      }
      case LaserCan.LASERCAN_STATUS_OUT_OF_BOUNDS: {
        statusString = "Out of bounds";
        break;
      }
      case LaserCan.LASERCAN_STATUS_WRAPAROUND: {
        statusString = "Wraparound";
        break;
      }
      default: {
        assert(false);
        statusString = "Unreachable code reached";
        break;
      }
    }
    throw new UncheckedIOException("Failed to get LaserCAN measurement: " + statusString,
      new IOException());
  }

  private void
  initPivotPosition(double a) {
    m_pivotEncoder.setPosition(a);
    m_pivotPositionCache.flush();
  }

  private void
  initElevatorPosition(double h) {
    m_elevatorEncoder.setPosition(h);
    m_elevatorPositionCache.flush();
  }

  private void toStateCraning() {
    m_state = State.CRANING;
  }

  private void toStateLoPivotHome() {
    // XXX Configure pivot to low amperage, and rotate upward slowly.
    m_state = State.LO_PIVOT_HOME;
  }

  private void toStateLoElevatorRapid() {
    double h = m_elevatorPositionCache.get();
    if (h > CraneConstants.kElevatorHomeRapidThreshold) {
      // XXX Rapid move.
      m_state = State.LO_ELEVATOR_RAPID;
    } else {
      m_state = State.LO_ELEVATOR_HOME;
    }
  }

  private void toStateLoElevatorHome() {
    // XXX Configure elevator to low amperage, and move downward slowly.
    m_state = State.LO_ELEVATOR_HOME;
  }

  private void toStateHiElevatorRapidA() {
    // XXX
    m_state = State.HI_ELEVATOR_RAPID_A;
  }

  private void toStateHiPivotHome() {
    // XXX
    m_state = State.HI_PIVOT_HOME;
  }

  private void toStateHiPivot0() {
    // XXX
    m_state = State.HI_PIVOT_0;
  }

  private void toStateHiElevatorRapidB() {
    // XXX
    m_state = State.HI_ELEVATOR_RAPID_B;
  }

  private void toStateHiElevatorHome() {
    // XXX
    m_state = State.HI_ELEVATOR_HOME;
  }

  private void toStateHiPivotMax() {
    // XXX
    m_state = State.HI_PIVOT_MAX;
  }

  private Translation2d getPosition() {
    return new Translation2d(m_pivotPositionCache.get(), m_elevatorPositionCache.get());
  }

  private Translation2d getVelocity() {
    return new Translation2d(m_pivotVelocityCache.get(), m_elevatorVelocityCache.get());
  }

  private Translation2d getDesiredTranslation() {
    return m_setpoint;
  }

  private Translation2d getDeviation(Translation2d position) {
    getDesiredTranslation().minus(position);
  }

  /* Dynamically scale the a,h controller constraints such that the combined a,h component
   * movements combine to follow a "straight" line, i.e. the component movements complete
   * simultaneously. */
  private void scaleAHConstraints(Translation2d position, Translation2d deviation) {
    Rotation2d translationAngle = deviation.getAngle();
    double aFactor = translationAngle.getCos();
    m_aController.setConstraints(new TrapezoidProfile.Constraints(
      aFactor * CraneConstants.kPivotMaxSpeedRadiansPerSecond,
      aFactor * CraneConstants.kPivotMaxAccelerationRadiansPerSecondSquared
    ));
    double hFactor = translationAngle.getSin();
    m_hController.setConstraints(new TrapezoidProfile.Constraints(
      hFactor * CraneConstants.kElevatorMaxSpeedMetersPerSecond,
      hFactor * CraneConstants.kElevatorMaxAcccelerationMetersPerSecondSquared
    ));
  }

  private void resetCrane() {
    Translation2d position = getPosition();
    Translation2d deviation = getDeviation(position);
    Translation2d velocity = getVelocity();

    scaleAHConstraints(position, deviation);
    m_aController.reset(
      deviation.getX(),
      velocity.getX()
    );
    m_hController.reset(
      deviation.getY(),
      velocity.getY()
    );
  }

  private void crane() {
    Translation2d position = getPosition();
    Translation2d deviation = getDeviation(position);

    scaleAHConstraints(position, deviation);
    double aVelocity = m_aController.calculate(deviation.getX());
    double hVelocity = m_hController.calculate(deviation.getY());
    m_pivotMotor.set(aVelocity);
    m_leftElevatorMotor.set(hVelocity);
  }

  @Override
  public void periodic() {
    switch (m_state) {
      case CRANING: {
        crane();
        break;
      }
      case ESTIMATE_H: {
        LaserCan.Measurement m = m_laserCan.getMeasurement();
        if (m == null) {
          // No measurement is available yet.
          break;
        }
        switch (m.status) {
          case LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT: {
            double h = CraneConstants.kLaserCanBaseMeasurement + (m.distance_mm / 1000.0);
            initElevatorPosition(h);
            if (h <= CraneConstants.kLoHiThreshold) {
              toStateLoPivotHome();
            } else {
              toStateHiElevatorRapidA();
            }
            break;
          }
          default: {
            laserCanStatusError(m.status);
            break;
          }
        }
        break;
      }
      case LO_PIVOT_HOME: {
        // XXX If pivot motor amperage is spiked (and velocity 0?), stop motor,
        // init pivot position.

        if (Constants.kCompeting) {
          toStateLoElevatorHome();
        } else {
          toStateLoElevatorRapid();
        }
        break;
      }
      case LO_ELEVATOR_RAPID: {
        // XXX Transition if at setpoint.

        toStateLoElevatorRapid();
        break;
      }
      case LO_ELEVATOR_HOME: {
        // XXX If elevator amperage is spiked (and velocity 0?), stop motor,
        // init elevator position.

        toStateCraning();
        break;
      }
      case HI_ELEVATOR_RAPID_A: {
        // XXX Transition if at setpoint.

        toStateHiPivotHome();
        break;
      }
      case HI_PIVOT_HOME: {
        // XXX Same as LO_PIVOT_HOME.

        toStateHiPivot0();
        break;
      }
      case HI_PIVOT_0: {
        // XXX Transition if at setpoint.

        toStateHiElevatorRapidB();
        break;
      }
      case HI_ELEVATOR_RAPID_B: {
        // XXX Transition if at setpoint.

        toStateHiElevatorHome();
        break;
      }
      case HI_ELEVATOR_HOME: {
        // XXX Same as LO_ELEVATOR_HOME.

        toStateHiPivotMax();
        break;
      }
      case HI_PIVOT_MAX: {
        // XXX Transition if at setpoint.

        toStateCraning();
        break;
      }
    }
  }