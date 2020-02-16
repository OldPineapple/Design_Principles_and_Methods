package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.EV3LargeRegulatedMotor;
/**
 * 
 * @author Frank P. Ferrie
 *
 */
public class PController implements UltrasonicController {

  /* Constants */
  private static final int MOTOR_SPEED = 170;
  private static final int FILTER_OUT = 20;

  private final int bandCenter;
  private final int bandWidth;
  private int distance;
  private int filterControl;
/**
 * This method states three parameters
 * And makes sure that the robot can move forward.
 * @param bandCenter
 * @param bandwidth
 */
  public PController(int bandCenter, int bandwidth) {
    this.bandCenter = bandCenter;
    this.bandWidth = bandwidth;
    this.filterControl = 0;

    WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED); // Initialize motor rolling forward
    WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  /**
   * This method checks whether the front left side is a gap, a wall, or empty
   * And lists three conditions
   * For each condition, it sets a speed for each motor
   * and then makes sure the robot can move forward
   */
  public void processUSData(int distance) {

    // rudimentary filter - toss out invalid samples corresponding to null
    // signal.
    // (n.b. this was not included in the Bang-bang controller, but easily
    // could have).
    //
    if (distance >= 255 && filterControl < FILTER_OUT) {
      // bad value, do not set the distance var, however do increment the
      // filter value
      filterControl++;
    } else if (distance >= 255) {
      // We have repeated large values, so there must actually be nothing
      // there: leave the distance alone
      this.distance = distance;
    } else {
      // distance went below 255: reset filter and leave
      // distance alone.
      filterControl = 0;
      this.distance = distance;
    }
    //the first condition is that the robot will keep moving forward in a specific range
	  if(Math.abs(distance)<=bandCenter+bandWidth && Math.abs(distance)>=bandCenter-bandWidth)
	  {
		  WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
		  WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED);
		  WallFollowingLab.leftMotor.forward();
		  WallFollowingLab.rightMotor.forward();
	  }
	  //the second condition is that the robot will quickly turn right if the distance is smaller than that range
	  if(Math.abs(distance)<bandCenter-bandWidth)
	  {
		  int error=bandCenter-bandWidth-distance; //error is the distance that we use to do calculations
		  WallFollowingLab.leftMotor.setSpeed((int) (MOTOR_SPEED+error*9));// speed changing is proportional to "error"
		  WallFollowingLab.rightMotor.setSpeed((int) (MOTOR_SPEED-error*9));
		  WallFollowingLab.leftMotor.forward();
		  WallFollowingLab.rightMotor.forward();
	  }
	  //the third condition is that the robot will turn left in a relatively high speed if the distance is greater than that range
	  if(Math.abs(distance)>bandCenter+bandWidth)
	  {
		  int error=distance-bandCenter-bandWidth;
		  if(error>30)
		  {
			  error=30; //we force the variable "error" to be 30 if it is greater than 30 to avoid sudden acceleration
			  WallFollowingLab.leftMotor.setSpeed((int) (MOTOR_SPEED-error*1)); 
			  WallFollowingLab.rightMotor.setSpeed((int) (MOTOR_SPEED+error*2.5));
			  WallFollowingLab.leftMotor.forward();
			  WallFollowingLab.rightMotor.forward();
		  }
		  else if(error>0 && error<=30) //when the "error" is within 30, we make the turning speed faster
		  {
			  WallFollowingLab.leftMotor.setSpeed((int) (MOTOR_SPEED-error*1.9));
			  WallFollowingLab.rightMotor.setSpeed((int) (MOTOR_SPEED+error*4));
			  WallFollowingLab.leftMotor.forward();
			  WallFollowingLab.rightMotor.forward();
		  }
	  }
	  
	  try {
		Thread.sleep(50);
	} catch (InterruptedException e) {
		e.printStackTrace();
	}
    // TODO: process a movement based on the us distance passed in (P style)
  }


  @Override
  /**
   * This method measures the distance from the wall
   */
  public int readUSDistance() {
    return this.distance;
  }

}