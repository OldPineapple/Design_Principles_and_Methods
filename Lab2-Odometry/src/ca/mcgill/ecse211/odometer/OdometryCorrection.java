/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
//import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private Odometer odometer;

  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();

  }
  
  Port EV3port = LocalEV3.get().getPort("S1"); //Set the connection port for the sensor
  SensorModes modesensor=new EV3ColorSensor(EV3port); //Get color sensor
  SampleProvider IDSensor=modesensor.getMode("RGB"); //Set sensor mode
  float[] samplecolor=new float[IDSensor.sampleSize()]; //Create data buffer
  
  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;
    int countx=0; //count for x
    int county=0; //count for y
    while (true) {
      correctionStart = System.currentTimeMillis();
      double theta = odometer.getXYT()[2]; //Get angle in degree
      IDSensor.fetchSample(samplecolor, 0);
      //LCD.drawString("Intensity"+ samplecolor[0]*1000, 0, 4); //This is used to measure the intensity of the black line and background of board
      double samplecolorthou = samplecolor[0] * 1000; //enlarge the value we detected
      if (samplecolorthou < 140) {
    	  	Sound.beep(); //Let the robot beep once we detect the black line
    	  	if (theta <= 10 || Math.abs(theta - 360) <= 10) {
    	  		odometer.setY(county * 30.48); //When it is moving towards North
    	  		county++;					  //Set the value of y depend on the index of black line(increase)
    	  	}
    	  	
    	  	if (Math.abs(theta - 90) <= 10) {
    	  		odometer.setX(countx * 30.48); //When it is moving towards East
    	  		countx++;					  //Set the value of x depend on index of black line(increase)
    	  	}
    	  	
    	  	if(Math.abs(theta - 180) <= 10) {
    	  		county--;                      //When it is moving towards South
    	  		odometer.setY(county * 30.48); //Set the value of x depend on index of black line(decrease)
    	  	}
    	  	
    	  	if(Math.abs(theta - 270) <= 10) {
    	  		countx--;                      //When it is moving towards West
    	  		odometer.setX(countx * 30.48); //Set the value of x depend on index of black line(decrease)
    	  	}
      }

      // this ensure the odometry correction occurs only once every period
      correctionEnd = System.currentTimeMillis();
      if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
        try {
          Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
        } catch (InterruptedException e) {
          // there is nothing to be done here
        }
      }
    }
  }
}
