package frc.robot.utils;

public class ClosedSlewRateLimiter {
  private final double positiveRate;
  private final double negativeRate;

  public ClosedSlewRateLimiter(double rate) {
    this(rate, -rate);
  }

  public ClosedSlewRateLimiter(double positiveRate, double negativeRate) {
    this.positiveRate = positiveRate;
    this.negativeRate = negativeRate;
  }

  public double calculate(double desired, double current) {
    if (current == desired){
      return desired;    
    } else if (desired > current) {
      return Math.min(current + positiveRate, desired);
    } else {
      return Math.max(current + negativeRate, desired); //neg rates sign is used specified
    }
  }
  
}
