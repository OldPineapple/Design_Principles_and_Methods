package ca.mcgill.ecse211.lab3;

import ca.mcgill.ecse211.Navigation.Navigation;
import ca.mcgill.ecse211.lab3.UltrasonicPoller;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class is used to avoid brick using hard code
 * @author Group 65
 *
 */
public class Avoidance {
	public static void ProcessUSData() {
	    int distance = UltrasonicPoller.getDist();
		if(Math.abs(distance) <= 15) {
			Navigation.clockwise(Navigation.WHEEL_RAD, Navigation.WHEEL_RAD, Navigation.TRACK, 90);
			longdrive(Lab3.leftMotor, Lab3.rightMotor, Navigation.WHEEL_RAD, Navigation.WHEEL_RAD, Navigation.TRACK);
			Navigation.anticlockwise(Navigation.WHEEL_RAD, Navigation.WHEEL_RAD, Navigation.TRACK, 90);
			shortdrive(Lab3.leftMotor, Lab3.rightMotor, Navigation.WHEEL_RAD, Navigation.WHEEL_RAD, Navigation.TRACK);
			Navigation.anticlockwise(Navigation.WHEEL_RAD, Navigation.WHEEL_RAD, Navigation.TRACK, 90);
			longdrive(Lab3.leftMotor, Lab3.rightMotor, Navigation.WHEEL_RAD, Navigation.WHEEL_RAD, Navigation.TRACK);
			Navigation.clockwise(Navigation.WHEEL_RAD, Navigation.WHEEL_RAD, Navigation.TRACK, 90);
		} else
			Navigation.travelTo();
	}
	
	/**
	 * This method is to move past the long side of brick.
	 * 
	 * @param leftMotor
	 * @param rightMotor
	 * @param leftRadius
	 * @param rightRadius
	 * @param track
	 */
	  public static void longdrive(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
		      double leftRadius, double rightRadius, double track) {
		    // reset the motors
		    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		      motor.stop();
		      motor.setAcceleration(3000);
		    }

		    // Sleep for 2 seconds
		    try {
		      Thread.sleep(2000);
		    } catch (InterruptedException e) {
		      // There is nothing to be done here
		    }
		    
		      leftMotor.setSpeed(Navigation.FORWARD_SPEED);
		      rightMotor.setSpeed(Navigation.FORWARD_SPEED);

		      leftMotor.rotate(Navigation.convertDistance(leftRadius, 28), true);
		      rightMotor.rotate(Navigation.convertDistance(rightRadius, 28), false);

	  }
	  
	  /**
	   * This method is used to move past short side of brick.
	   *  
	   * @param leftMotor
	   * @param rightMotor
	   * @param leftRadius
	   * @param rightRadius
	   * @param track
	   */
	  public static void shortdrive(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
		      double leftRadius, double rightRadius, double track) {
		    // reset the motors
		    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {leftMotor, rightMotor}) {
		      motor.stop();
		      motor.setAcceleration(3000);
		    }

		    // Sleep for 2 seconds
		    try {
		      Thread.sleep(2000);
		    } catch (InterruptedException e) {
		      // There is nothing to be done here
		    }
		    
		      leftMotor.setSpeed(Navigation.FORWARD_SPEED);
		      rightMotor.setSpeed(Navigation.FORWARD_SPEED);

		      leftMotor.rotate(Navigation.convertDistance(leftRadius, 23), true);
		      rightMotor.rotate(Navigation.convertDistance(rightRadius, 23), false);

	  }
	  
}
