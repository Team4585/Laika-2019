package org.huskyrobotics.frc2019.subsystems.drive.FalconLibStuff;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SensorTerm;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import org.ghrobotics.lib.mathematics.units.Length;
import org.ghrobotics.lib.mathematics.units.LengthKt;
import org.ghrobotics.lib.mathematics.units.TimeUnitsKt;
import org.ghrobotics.lib.mathematics.units.derivedunits.Velocity;
import org.ghrobotics.lib.mathematics.units.derivedunits.VelocityKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitKt;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitLengthModel;
import org.ghrobotics.lib.mathematics.units.nativeunits.NativeUnitModel;
import org.ghrobotics.lib.wrappers.ctre.FalconSRX;
import org.huskyrobotics.frc2019.Constants;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class FalconGearbox {
    public static enum EncoderMode {
        None, QuadEncoder, CTRE_MagEncoder_Relative, CTRE_MagEncoder_Absolute
    }
    public enum TransmissionSide {
        Left, Right;
    }

    private FalconSRX<Length> m_Master;

    private VictorSPX m_Slave;

    private TransmissionSide side;

    NativeUnitLengthModel lengthModel = Constants.drivetrain.kLeftNativeunitLengthmodel;

    /**
     * Creates a gearbox object using a Talon SRX master and Victor SPX slave.
     * 
     * @param masterPort Master Talon SRX port on the CAN line
     * @param slavePort Slave Victor SPX port on the CAN line
     * @param mode The kind of encoder connected to the Gearbox. Look in enums and CTRE docs for more info.
     * @param side The side of the Tank Drive gearbox
     * @param isInverted Determines if something is inverted (Good to know that one gearbox will always be inverted in tank drive)
     */
    public FalconGearbox(int masterPort, int slavePort, EncoderMode mode, TransmissionSide side, boolean isInverted) {
        if (side == TransmissionSide.Left){
            lengthModel = Constants.drivetrain.kLeftNativeunitLengthmodel;
        }else{ 
            lengthModel = Constants.drivetrain.kRightNativeunitLengthmodel;
        };
        
        m_Master = new FalconSRX<Length>(masterPort, lengthModel, TimeUnitsKt.getMillisecond(10));
        m_Slave = new VictorSPX(slavePort);

        this.side = side;

        if(mode == EncoderMode.QuadEncoder) {
            m_Master.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
            m_Master.configSensorTerm(SensorTerm.Diff0, FeedbackDevice.QuadEncoder, 10);
        }
        //m_Slave.set(ControlMode.Follower, m_Master.getDeviceID());
        m_Slave.follow(m_Master);

        m_Master.configPeakOutputForward(+1.0, 10);
        m_Master.configPeakOutputReverse(-1.0, 10);

        SmartDashboard.putBoolean("Is the " + side.toString() + " transmission inverted", isInverted);
         
        m_Master.setInverted(isInverted);
        m_Slave.setInverted(InvertType.FollowMaster);


    }
      /* Utility Methods */
        public FalconSRX<Length> getMaster() {
            return m_Master;
        }
        
        public List<FalconSRX<Length>> getAllMasters() {
            return Arrays.asList(
              m_Master
            );
        }
        public List<BaseMotorController> getAll(){
            return Arrays.asList(
              m_Master, m_Slave
              );
        }
        
        public NativeUnitLengthModel getModel() {
            return lengthModel;
        }

        /* Drive Base Practicality */
        
        public void zeroEncoder() {
          m_Master.setSensorPosition(LengthKt.getMeter(0));
        }
      
        public void setNeutralMode(NeutralMode mode) {
          for( BaseMotorController motor : getAll() ) {
            motor.setNeutralMode(mode);
          }
        }
      
        public void stop() {
          getMaster().set(ControlMode.PercentOutput, 0);
        }
      
        public void setClosedLoopGains(double kp, double ki, double kd, double kf, double iZone, double maxIntegral) {
          m_Master.config_kP(0, kp, 10);
          m_Master.config_kI(0, ki, 10);
          m_Master.config_kD(0, kd, 10);
          m_Master.config_kF(0, kf, 10);
          m_Master.config_IntegralZone(0, (int)Math.round(lengthModel.fromModel(LengthKt.getMeter(iZone)).getValue()), 30);
          m_Master.configMaxIntegralAccumulator(0, maxIntegral, 0);
        }

        /* Length/Velocity Methods */

        public Length getDistance() {
            return m_Master.getSensorPosition();
        }

        public Velocity<Length> getVelocity() {
            return m_Master.getSensorVelocity();
        }

        public double getFeetPerSecond() {
            return VelocityKt.getFeetPerSecond(getVelocity());
        }

        public double getFeet() {
            return getDistance().getFeet();
        }

        public Length getClosedLoopError() {
            if (getMaster().getControlMode() != ControlMode.PercentOutput) {
              return lengthModel.toModel(NativeUnitKt.getSTU(m_Master.getClosedLoopError()));
            }
            else {
              return LengthKt.getFeet(0);
            }
        }

}