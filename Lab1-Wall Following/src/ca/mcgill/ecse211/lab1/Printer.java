package ca.mcgill.ecse211.lab1;

import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
/**
 * This class prints choices we have on EV3
 * @author Frank P. Ferrie
 *
 */
public class Printer extends Thread {

  //
  // In addition to the UltrasonicPoller, the printer thread also operates
  // in the background. Since the thread sleeps for 200 mS each time through
  // the loop, screen updating is limited to 5 Hz.
  //

  private UltrasonicController cont;
  private final int option;
/**
 * This method states two parameters
 * @param option
 * @param cont
 */
  public Printer(int option, UltrasonicController cont) {
    this.cont = cont;
    this.option = option;
  }

  public static TextLCD t = LocalEV3.get().getTextLCD(); // n.b. how the screen is accessed
/**
 * This method prints messages of each choice on EV3
 */
  public void run() {
    while (true) { // operates continuously
      t.clear();
      t.drawString("Controller Type is... ", 0, 0); // print header
      if (this.option == Button.ID_LEFT)
        t.drawString("BangBang", 0, 1);
      else if (this.option == Button.ID_RIGHT)
        t.drawString("P type", 0, 1);
      t.drawString("US Distance: " + cont.readUSDistance(), 0, 2); // print last US reading

      try {
        Thread.sleep(200); // sleep for 200 mS
      } catch (Exception e) {
        System.out.println("Error: " + e.getMessage());
      }
    }
  }
/**
 * This method prints the choice menu
 */
  public static void printMainMenu() { // a static method for drawing
    t.clear(); // the screen at initialization
    t.drawString("left = bangbang", 0, 0);
    t.drawString("right = p type", 0, 1);
  }
}
