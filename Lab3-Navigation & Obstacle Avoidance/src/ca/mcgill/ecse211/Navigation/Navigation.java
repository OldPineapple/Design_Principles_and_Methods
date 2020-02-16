package ca.mcgill.ecse211.Navigation;

import ca.mcgill.ecse211.lab3.Lab3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
/**
 * This is the class for Navigation
 * @author Group 65
 *
 */
public class Navigation {
	
	public static final int FORWARD_SPEED = 200;
	public static final int ROTATE_SPEED = 150;
	public static final double TILE_SIZE = 30.48;
	public static final double WHEEL_RAD = 2.2;
	public static final double TRACK = 17.1;
	static boolean isNavigating = false;
	boolean avoid = false;
	
	//Set array of values for x and y
	public static final double[] x = {0 * TILE_SIZE, 1 * TILE_SIZE, 2 * TILE_SIZE, 2 * TILE_SIZE, 1 * TILE_SIZE};
	public static final double[] y = {2 * TILE_SIZE, 1 * TILE_SIZE, 2 * TILE_SIZE, 1 * TILE_SIZE, 0 * TILE_SIZE};
	private static Odometer odo;

	/**
	 * This method ensures the robot to turn clockwise when it can turn samller angle in this way.
	 * 
	 * @param leftRadius
	 * @param rightRadius
	 * @param track
	 * @param theta
	 */
	public static void clockwise(double leftRadius, double rightRadius, double track, double theta) {

		  for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {Lab3.leftMotor, Lab3.rightMotor}) {
		    motor.stop();
		    motor.setAcceleration(2000);
		  }

		  // Sleep for 2 seconds
		  try {
		    Thread.sleep(2000);
		  } catch (InterruptedException e) {
		    // There is nothing to be done here
		  }

		  // turn clockwise theta degrees
		  Lab3.leftMotor.setSpeed(ROTATE_SPEED);
		  Lab3.rightMotor.setSpeed(ROTATE_SPEED);

		  Lab3.leftMotor.rotate(convertAngle(leftRadius, track, theta), true);
		  Lab3.rightMotor.rotate(-convertAngle(rightRadius, track, theta), false);
		}
	/**
	 * This method ensures the robot to turn anticlockwise when it can turn smaller angle in this way.
	 * 
	 * @param leftRadius
	 * @param rightRadius
	 * @param track
	 * @param theta
	 */
	public static void anticlockwise(double leftRadius, double rightRadius, double track, double theta) {

		  for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {Lab3.leftMotor, Lab3.rightMotor}) {
		    motor.stop();
		    motor.setAcceleration(3000);
		  }

		  // Sleep for 2 seconds
		  try {
		    Thread.sleep(2000);
		  } catch (InterruptedException e) {
		    // There is nothing to be done here
		  }

		  // turn anticlockwise theta degrees
		  Lab3.leftMotor.setSpeed(ROTATE_SPEED);
		  Lab3.rightMotor.setSpeed(ROTATE_SPEED);

		  Lab3.rightMotor.rotate(convertAngle(leftRadius, track, theta), true);
		  Lab3.leftMotor.rotate(-convertAngle(rightRadius, track, theta), false);
		}
	/**
	 * This method compares all the relations between the angle now and the angle we are going to turn to.
	 * And then decides what direction we should turn in and how much we should turn.
	 */
	public static void turnTo(double theta) {
		
		//Here we can know the angle we have now.
		double heading = odo.getXYT()[2];
		if((theta > heading) && (theta - heading <= 180) && (heading <= 180)) {
			clockwise(WHEEL_RAD, WHEEL_RAD, TRACK, theta - heading);
		}
		else if((theta > heading) && (theta - heading > 180) && (heading <= 180)) {
			anticlockwise(WHEEL_RAD, WHEEL_RAD, TRACK, 360 + heading - theta);
		}
		else if((theta > heading) && (heading > 180)) {
			clockwise(WHEEL_RAD, WHEEL_RAD, TRACK, theta - heading);
		}
		else if((theta < heading) && (heading < 180)) {
			anticlockwise(WHEEL_RAD, WHEEL_RAD, TRACK, heading - theta);
		}
		else if((theta < heading) && (heading - theta <=180) && (heading >= 180)) {
			anticlockwise(WHEEL_RAD, WHEEL_RAD, TRACK, heading - theta);
		}
		else if((theta < heading) && (heading - theta > 180) && (heading >= 180)) {
			clockwise(WHEEL_RAD, WHEEL_RAD, TRACK, 360 + theta - heading);
		}
	}
	
	/**
	 * This method sets the condition of whether the robot is navigating or not.
	 * 
	 * @return
	 */
	boolean isNavigating() {
		return isNavigating;
	}
	
	/**
	 * This method gets the points we will pass. Then we calculate the angle between two points.
	 * And then we let the robot to move to that position and update the value of x, y and theta
	 * in that point for further calculation.
	 */
	public static void travelTo() {
		double xd, yd, theta = 0;
		double currentx, currenty;
		isNavigating = true;
		
		try {
			odo = Odometer.getOdometer();
		} catch (OdometerExceptions e1) {
		}
		for(int i = 0; i < 5; i++) {
			xd = x[i];
			yd = y[i];
			currentx = odo.getXYT()[0]; //This is the value of x at current position.
			currenty = odo.getXYT()[1]; //This is the value of y at current position.
			if (xd > currentx && yd > currenty) {
				theta = Math.atan((xd - currentx) / (yd - currenty)) * 180 / Math.PI;
			}
			else if(xd > currentx && yd == currenty) {
				theta = 90;
			}
			else if (xd > currentx && yd < currenty) {
				theta = 180 + Math.atan((xd - currentx) / (yd - currenty)) * 180 / Math.PI;
			}
			else if (xd == currentx && yd < currenty) {
				theta = 180;
			}
			else if(xd < currentx && yd < currenty) {
				theta = 180 + Math.atan((xd - currentx) / (yd - currenty)) * 180 / Math.PI;
			}
			else if(xd < currentx && yd ==currenty) {
				theta = 270;
			}
			else if(xd < currentx && yd > currenty) {
				theta = 360 + Math.atan((xd - currentx) / (yd - currenty)) * 180 / Math.PI;
			}
			else if(xd == currentx && yd > currenty) {
				theta = 0;
			}
			turnTo(theta);
			isNavigating = false;
		    for (EV3LargeRegulatedMotor motor : new EV3LargeRegulatedMotor[] {Lab3.leftMotor, Lab3.rightMotor}) {
		        motor.stop();
		        motor.setAcceleration(3000);
		    }

		      // Sleep for 2 seconds
		    try {
		      Thread.sleep(2000);
		    } catch (InterruptedException e) {
		        // There is nothing to be done here
		    }

			double Distance = Math.sqrt((xd - currentx) * (xd - currentx) + (yd - currenty) * (yd - currenty));
			Lab3.leftMotor.setSpeed(FORWARD_SPEED);
			Lab3.rightMotor.setSpeed(FORWARD_SPEED);
			
			Lab3.leftMotor.rotate(convertDistance(WHEEL_RAD, Distance), true);
			Lab3.rightMotor.rotate(convertDistance(WHEEL_RAD, Distance), false);
			
			odo.setXYT(xd, yd, theta);
		}
	}
	/**
	  * This method allows the conversion of a distance to the total rotation of each wheel need to
	  * cover that distance.
	  * 
	  * @param radius
	  * @param distance
	  * @return
	  */
	public static int convertDistance(double radius, double distance) {
	    return (int) ((180.0 * distance) / (Math.PI * radius));
	  }

	public static int convertAngle(double radius, double width, double angle) {
	    return convertDistance(radius, Math.PI * width * angle / 360.0);
	  }
	
}
