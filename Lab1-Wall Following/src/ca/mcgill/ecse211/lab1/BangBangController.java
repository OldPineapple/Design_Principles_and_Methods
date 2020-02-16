package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.*;
/**
 * This class implements the Bang Bang Controller for Lab1 on the EV3 platform
 * 
 * @author Frank P. Ferrie
 *
 */
public class BangBangController implements UltrasonicController {

  private final int bandCenter;
  private final int bandwidth;
  private final int motorLow;
  private final int motorHigh;
  private int distance;

  /**
   * This method states all parameters we are going to use
   * And makes sure the robot can move forward at start
   * @param bandCenter
   * @param bandwidth
   * @param motorLow
   * @param motorHigh
   */
  public BangBangController(int bandCenter, int bandwidth, int motorLow, int motorHigh) {
    // Default Constructor
    this.bandCenter = bandCenter;
    this.bandwidth = bandwidth;
    this.motorLow = motorLow;
    this.motorHigh = motorHigh;
    WallFollowingLab.leftMotor.setSpeed(motorHigh); // Start robot moving forward
    WallFollowingLab.rightMotor.setSpeed(motorHigh);
    WallFollowingLab.leftMotor.forward();
    WallFollowingLab.rightMotor.forward();
  }

  @Override
  /**
   * This method lists three conditions
   * for each there is a specific speed for each motor
   * and then make sure the robot can move forward
   */
  public void processUSData(int distance) {
    this.distance = distance;
    	 //the first condition is that the robot will keep moving forward in a specific range
	  if(Math.abs(distance)<=bandCenter+bandwidth && Math.abs(distance)>=bandCenter-bandwidth)
	  {
		  WallFollowingLab.leftMotor.setSpeed(motorHigh);
		  WallFollowingLab.rightMotor.setSpeed(motorHigh);
		  WallFollowingLab.leftMotor.forward();
		  WallFollowingLab.rightMotor.forward();
	  }
	//the second condition is that the robot will quickly turn right if the distance is smaller than that range
	  else if(Math.abs(distance)<bandCenter-bandwidth)
	  {
		  WallFollowingLab.leftMotor.setSpeed(motorHigh+motorHigh);// we let the robot to turn very fast 
		  WallFollowingLab.rightMotor.setSpeed(motorHigh-motorLow);// when it is going to turn right
		  WallFollowingLab.leftMotor.forward();
		  WallFollowingLab. rightMotor.forward();
	  }
	//the third condition is that the robot will turn left in a relatively high speed if the distance is greater than that range
	  else if(Math.abs(distance)>bandCenter+bandwidth)
	  {
		  WallFollowingLab.leftMotor.setSpeed(motorHigh-motorLow);//we let the robot to turn relatively slow
		  WallFollowingLab.rightMotor.setSpeed(motorHigh);//when it is going to turn left to avoid colliding the convex corner
		  WallFollowingLab.leftMotor.forward();
		  WallFollowingLab.rightMotor.forward();
	  }
	  try {
		Thread.sleep(50);
	} catch (InterruptedException e) {
		// TODO Auto-generated catch block
		e.printStackTrace();
	}
    // TODO: process a movement based on the us distance passed in (BANG-BANG style)
  }

  @Override
  /**
   * This method measures the distance from the wall
   */
  public int readUSDistance() {
    return this.distance;
  }
}
