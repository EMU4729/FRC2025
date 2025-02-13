package frc.robot.utils.motorsupplier;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.units.measure.Distance;

import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.constants.DriveConstants;

public class SparkMotorSupplier extends MotorSupplier<SparkMax> {
  final SparkMaxConfig config = new SparkMaxConfig();


  public SparkMotorSupplier(int port) {
    super(port);
    config.smartCurrentLimit(30);
    config.closedLoop.outputRange(-1, 1);
  }

  @Override
  public MotorSupplier<SparkMax> withSafety() {
    throw new UnsupportedOperationException();
  }

  /** default 30 apms */
  public SparkMotorSupplier withCurrentLimit(int amps){
    config.smartCurrentLimit(amps);
    return this;
  }

  public SparkMotorSupplier withAbsEncoder(double positionFactor, double velocityFactor){
    config.absoluteEncoder.positionConversionFactor(positionFactor);
    config.absoluteEncoder.velocityConversionFactor(velocityFactor);
    config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);


    return this;
  }
  
  public SparkMotorSupplier withRelEncoder(double positionFactor, double velocityFactor){
    config.encoder.positionConversionFactor(positionFactor);
    config.encoder.velocityConversionFactor(velocityFactor);
    config.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);


    return this;
  }

  public SparkMotorSupplier withPID(double p, double i, double d){
    config.closedLoop
        .pidf(p, i, d, 0);

    return this;
  }
  public SparkMotorSupplier withPID(double p, double i, double d, double ff){
    config.closedLoop
        .pidf(p, i, d, ff);

    return this;
  }
  public SparkMotorSupplier withPIDIZone(double absLimit){
    config.closedLoop.iZone(absLimit);
    return this;
  }

  public SparkMotorSupplier withPositionWrapping(double min, double max){
    config.closedLoop
    .positionWrappingEnabled(true)
    .positionWrappingInputRange(min, max);

    return this;
  }

  public SparkMax get() {
    if (port < 0) {
      System.out.println("MotorInfo : motor port num < 0, check port is defined : " + port);
      return new SparkMax(port, MotorType.kBrushed);
    }

    final var spark = new SparkMax(port, MotorType.kBrushed);
    config.inverted(invert);

    if (voltageComp) {
      config.voltageCompensation(12);
    } else {
      config.disableVoltageCompensation();
    }
    
    if(brake){
      config.idleMode(IdleMode.kBrake);
    }

    spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    return spark;
  }
}
