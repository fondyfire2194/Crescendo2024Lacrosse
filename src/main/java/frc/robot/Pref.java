/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Collection;
/**
 * Add your docs here.
 */
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Vector;

import edu.wpi.first.wpilibj.Preferences;

public class Pref {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  private static Collection<String> v;
  private static Enumeration<String> e;
  private static String tempString;
  private static double tempDouble;

  public static HashMap<String, Double> prefDict = new HashMap<>();

  static {

    // amp shot
    prefDict.put("AmpArmIncrementDelay", .1);
    prefDict.put("AmpBottomRPM", 500.);
    prefDict.put("AmpArmDegrees", 92.);
    prefDict.put("AmpDegreeIncrement", 10.);
    prefDict.put("AmpTopRPM", 1000.);
    prefDict.put("AmpTransferToShootSpeed", 4000.);

    // align to tag
    prefDict.put("AlignKp", 0.01);

    // swerveangle tune
    prefDict.put("AngleKp", .028);
    // arm
    prefDict.put("armFFKa", 0.);
    prefDict.put("armFFKg", .2);
    prefDict.put("armFFKs", 0.31);
    prefDict.put("armFFKv", 2.0);
    prefDict.put("armKd", 0.);
    prefDict.put("armKi", .5);
    prefDict.put("armKiZone", 0.);
    prefDict.put("armKp", 30.);
    prefDict.put("armUpFFKv", 2.75);

    // drive tune

    prefDict.put("DriveFF", .95);
    prefDict.put("DriveKp", .1);
    prefDict.put("IntakeKp", 0.00035);
    prefDict.put("IntakeSpeed", 4500.);

    prefDict.put("LockNumber", 1.);

    prefDict.put("rotkd", 0.00001);
    prefDict.put("rotki", 0.005);
    prefDict.put("rotkp", 0.003);

    prefDict.put("SensorDistance", 6.);
    // shooter
    prefDict.put("ShooterBottomKd", .0001);
    prefDict.put("ShooterBottomKi", .0);
    prefDict.put("ShooterBottomKp", .0004);
    prefDict.put("ShooterBottomKpFF", .0007);
    
    prefDict.put("ShooterSpeedRatio", 1.);

    prefDict.put("ShooterTopKd", .01);
    prefDict.put("ShooterTopKi", 0.);
    prefDict.put("ShooterTopKp", .0004);
    prefDict.put("ShooterTopKpFF", .0007);

    prefDict.put("TransferIntakingSpeed", 5500.);
    prefDict.put("TransferToShootSpeed", 7000.);

    // climber

    prefDict.put("UnlockNumber", 0.);

  }

  public static void ensureRioPrefs() {
    // init();
    deleteUnused();
    addMissing();
  }

  public static void deleteUnused() {
    v = new Vector<String>();
    v = Preferences.getKeys();
    // v = (Vector<String>) RobotContainer.prefs.getKeys();
    String[] myArray = v.toArray(new String[v.size()]);

    for (int i = 0; i < v.size(); i++) {
      boolean doNotDelete = myArray[i].equals(".type");

      if (!doNotDelete && !prefDict.containsKey(myArray[i]) && Preferences.containsKey(myArray[i])) {
        Preferences.remove(myArray[i]);
      }
    }

  }

  public static void addMissing() {

    Iterator<Map.Entry<String, Double>> it = prefDict.entrySet().iterator();
    while (it.hasNext()) {
      Map.Entry<String, Double> pair = it.next();
      tempString = pair.getKey();
      tempDouble = pair.getValue();
      if (!Preferences.containsKey((tempString)))
        Preferences.setDouble(tempString, tempDouble);
    }
  }

  public static double getPref(String key) {
    if (prefDict.containsKey(key))
      return Preferences.getDouble(key, prefDict.get(key));
    else
      return 0;
  }

  public static void deleteAllPrefs(Preferences Preferences) {
    edu.wpi.first.wpilibj.Preferences.removeAll();
  }

}
