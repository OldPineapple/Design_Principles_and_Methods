/**
 * This class is meant as a skeleton for the odometer class to be used.
 * 
 * @author Group 65
 * 
 */

package ca.mcgill.ecse211.Navigation;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends OdometerData implements Runnable {

  private OdometerData odoData;
  private static Odometer odo = null; // Returned as singleton

  // Motors and related variables
  private int leftMotorTachoCount;
  private int rightMotorTachoCount;
  private EV3LargeRegulatedMotor leftMotor;
  private EV3LargeRegulatedMotor rightMotor;

  private final double TRACK;
  private final double WHEEL_RAD;

  private static final long ODOMETER_PERIOD = 25; // odometer update period in ms

  /**
   * This is the default constructor of this class. It initiates all motors and variables once.It
   * cannot be accessed externally.
   * 
   * @param leftMotor
   * @param rightMotor
   * @throws OdometerExceptions
   */
  private Odometer(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
      final double TRACK, final double WHEEL_RAD) throws OdometerExceptions {
    odoData = OdometerData.getOdometerData(); // Allows access to x,y,z
                                              // manipulation methods
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;

    // Reset the values of x, y and z to 0
    odoData.setXYT(0, 0, 0);

    this.leftMotorTachoCount = 0;
    this.rightMotorTachoCount = 0;

    this.TRACK = TRACK;
    this.WHEEL_RAD = WHEEL_RAD;

  }

  /**
   * This method is meant to ensure only one instance of the odometer is used throughout the code.
   * 
   * @param leftMotor
   * @param rightMotor
   * @return new or existing Odometer Object
   * @throws OdometerExceptions
   */
  public synchronized static Odometer getOdometer(EV3LargeRegulatedMotor leftMotor,
      EV3LargeRegulatedMotor rightMotor, final double TRACK, final double WHEEL_RAD)
      throws OdometerExceptions {
    if (odo != null) { // Return existing object
      return odo;
    } else { // create object and return it
      odo = new Odometer(leftMotor, rightMotor, TRACK, WHEEL_RAD);
      return odo;
    }
  }

  /**
   * This class is meant to return the existing Odometer Object. It is meant to be used only if an
   * odometer object has been created
   * 
   * @return error if no previous odometer exists
   */
  public synchronized static Odometer getOdometer() throws OdometerExceptions {

    if (odo == null) {
      throw new OdometerExceptions("No previous Odometer exits.");

    }
    return odo;
  }

  /**
   * This method is where the logic for the odometer will run. Use the methods provided from the
   * OdometerData class to implement the odometer.
   */
  // run method (required for Thread)
  public void run() {
    long updateStart, updateEnd;

    double distL, distR, deltaD, deltaT, deltaTdeg, dX, dY;
    double lastTachoL = 0, lastTachoR = 0;
    
    while (true) {
      updateStart = System.currentTimeMillis();
      double theta = odo.getXYT()[2]; //get angle in degree
      double thetarad = theta * Math.PI / 180; //convert degree to radian
      leftMotorTachoCount = leftMotor.getTachoCount(); //get tacho counts for left motor
      rightMotorTachoCount = rightMotor.getTachoCount(); //get tacho counts for right motor
      distL = Math.PI * WHEEL_RAD * (leftMotorTachoCount - lastTachoL) / 180;  //compute left wheel displacement
      distR = Math.PI * WHEEL_RAD * (rightMotorTachoCount - lastTachoR) / 180; //compute right wheel displacement
      lastTachoL = leftMotorTachoCount; //update tacho counts for left wheel
      lastTachoR = rightMotorTachoCount;//update tacho counts for right wheel
      deltaD = 0.5 * (distL+distR); //compute change in displacement
      deltaT = (distL-distR) / TRACK; //compute change in heading
      deltaTdeg = deltaT * 180 / Math.PI; //convert radian to degree
      dX = deltaD * Math.sin(deltaT + thetarad); //calculating the change in x-axis
      dY = deltaD * Math.cos(deltaT + thetarad); //calculating the change in y-axis
      odo.update(dX, dY, deltaTdeg); //update all values to the odometer

      // this ensures that the odometer only runs once every period
      updateEnd = System.currentTimeMillis();
      if (updateEnd - updateStart < ODOMETER_PERIOD) {
        try {
          Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
        } 
        catch (InterruptedException e) {
        }
        
      }
      
    }
    
  }

}