package frc.robot.utils.motorsupplier;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class FalconMotorSupplier {
  public final int port;

  final TalonFXConfiguration motorConfig = new TalonFXConfiguration();

  public FalconMotorSupplier(int motorPort) {
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = 40;

    this.port = motorPort;
  }

  public FalconMotorSupplier withBrake() {
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    return this;
  }

  
  public FalconMotorSupplier withInvert() {
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    return this;
  }

  
  public FalconMotorSupplier withPID(double p, double i, double d){
    motorConfig.Slot0.kP = p;
    motorConfig.Slot0.kI = i;
    motorConfig.Slot0.kD = d;

    return this;
  }
  public FalconMotorSupplier withPIDGravComp(double g, GravityTypeValue GravityType){
    motorConfig.Slot0.kG = g;
    motorConfig.Slot0.GravityType = GravityType;

    return this;
  }

  /**  <pre>
   * 
   * @param ratio the ratio between one unit (m, mm, deg) motion in the mechanism and one rotation in the motor
   *      rotary = gearRatio
   *      linear = Final wheel circumference / gearRatio {until the final wheel}
   * 
   *    Note: circumference calcs in terms of rotations
   * 
   *   Final wheel circumference of a gear,sprocket,pully should be based on its pitch diameter
   *      #25 chain = (Teeth * 2.0193  {PD in mm}) * PI
   * </pre>
   */
  public FalconMotorSupplier withEncoder(double ratio){
    motorConfig.Feedback.SensorToMechanismRatio = ratio;

    return this;
  }

  public FalconMotorSupplier withCurrentLimit(int amps){
    motorConfig.CurrentLimits.SupplyCurrentLimit = amps;
    
    return this;
  }

  public TalonFX get(){
    TalonFX falcon = new TalonFX(port);    

    falcon.getConfigurator().apply(motorConfig);
    
    return falcon;
  }
  
  
  
  @Deprecated
  /** Use a 'VelocityVoltage' PID to achieve the same effect */
  public FalconMotorSupplier withVoltageComp() {
    throw(new NoSuchMethodError("FalconMotorSupplier : Use a 'VelocityVoltage' PID to achieve the same effect"));
  }
  @Deprecated
  /** Falcon motors have safety by default */
  public FalconMotorSupplier withSafety() {
    throw(new NoSuchMethodError("FalconMotorSupplier : Falcon motors have safety by default"));
  }
}

