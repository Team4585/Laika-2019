package org.huskyrobotics.frc2019.subsystems.superstructure;

//import org.huskyrobotics.frc2019.subsystems.*;
import org.huskyrobotics.frc2019.Constants;
import org.huskyrobotics.frc2019.RobotMap;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.Rotation2d;
import org.ghrobotics.lib.mathematics.units.Rotation2dKt;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitRotationModel;
import org.ghrobotics.lib.wrappers.ctre.FalconSRX;


public class PivotArmObject{
    private FalconSRX<Rotation2d> m_motor;
    NativeUnitRotationModel rotationModel = Constants.PivotArm.kArmNativeunitRotationmodel;

      public static enum EncoderMode {
            None, QuadEncoder, CTRE_MagEncoder_Relative, CTRE_MagEncoder_Absolute
        }

public PivotArmObject(int motorPort, EncoderMode mode) {
    m_motor = new FalconSRX<Rotation2d>(motorPort, rotationModel, TimeUnitsKt.getMillisecond(10));
    if(mode == EncoderMode.QuadEncoder) {
          m_motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
          m_motor.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, 10);
      }
    m_motor.setSensorPhase(true);
    m_motor.configNominalOutputForward(0, Constants.drivetrain.kTimeoutMs);
    m_motor.configNominalOutputReverse(0, Constants.drivetrain.kTimeoutMs);
    m_motor.configPeakOutputForward(1, Constants.drivetrain.kTimeoutMs);
    m_motor.configPeakOutputReverse(-1, Constants.drivetrain.kTimeoutMs);
    
    m_motor.selectProfileSlot(0, 0);
}
public FalconSRX<Rotation2d> getPivot() {
    return m_motor;
  }
public NativeUnitRotationModel getModel() {
    return rotationModel;
  }

  public Rotation2d getDistance() {
    return m_motor.getSensorPosition();
  }

  public Velocity<Rotation2d> getVelocity() {
    return m_motor.getSensorVelocity();
  }

 /* public double getRadianPerSecond() {
    //I need this to return a value of Rad (or degrees) per second, but we might not have to do that necessarily
  }*/

  public double getDegree() {
    return getDistance().getDegree();
  }

  public Rotation2d getClosedLoopError() {
    if (getPivot().getControlMode() != ControlMode.PercentOutput) {
      return rotationModel.toModel(NativeUnitKt.getSTU(m_motor.getClosedLoopError()));
    }
    else {
      return Rotation2dKt.getDegree(0);
    }
  }

  public void zeroEncoder() {
    m_motor.setSensorPosition(Rotation2dKt.getDegree(0));
  }

  public void setNeutralMode(NeutralMode mode) {
      m_motor.setNeutralMode(mode);
  }

  public void stop() {
    getPivot().set(ControlMode.PercentOutput, 0);
  }

  public void setClosedLoopGains(double kp, double ki, double kd, double kf, double iZone, double maxIntegral) {
    m_motor.config_kP(0, kp, 10);
    m_motor.config_kI(0, ki, 10);
    m_motor.config_kD(0, kd, 10);
    m_motor.config_kF(0, kf, 10);
    m_motor.config_IntegralZone(0, (int)Math.round(rotationModel.fromModel(Rotation2dKt.getDegree(iZone)).getValue()), 30);
    m_motor.configMaxIntegralAccumulator(0, maxIntegral, 0);
  }

}
