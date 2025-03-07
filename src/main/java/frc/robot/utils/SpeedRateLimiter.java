package frc.robot.utils;

import edu.wpi.first.math.filter.SlewRateLimiter;
import frc.robot.constants.DriveConstants;

public class SpeedRateLimiter extends SlewRateLimiter {
  protected double sign = 0;
  protected boolean lock = false;
  
  public SpeedRateLimiter(double positiveRateLimit, double negativeRateLimit, double initialValue){
    super(positiveRateLimit, negativeRateLimit, initialValue);
  }

  @Override
  public double calculate(double input){
    if(Math.abs(input) > 0){
      sign = input;
    }
    double out = super.calculate(Math.abs(input));
    out = Math.copySign(out, sign);

    return out;
  }

}
