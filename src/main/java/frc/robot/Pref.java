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

    // drive tune
    prefDict.put("DriveKp", .001);
    prefDict.put("DriveFF", .5);

    // angle tune

    prefDict.put("AngleKp", .01);

    // align to tag

    prefDict.put("AlignKp", .01);

    // shooter

    prefDict.put("ShooterSpeedRatio", 0.5);
    prefDict.put("ShooterTopKp", .0007);
    prefDict.put("ShooterTopKd", .001);
    prefDict.put("ShooterTopKi", .001);

    prefDict.put("ShooterTopKpFF", .0007);

    prefDict.put("ShooterBottomKp", .0007);
    prefDict.put("ShooterBottomKd", .001);
    prefDict.put("ShooterBottomKi", .001);

    prefDict.put("ShooterBottomKpFF", .0007);

    prefDict.put("IntakeSpeed", 3000.);
    prefDict.put("IntakeKp", 0.001);

    prefDict.put("TransferToShootSpeed", 2000.);
    prefDict.put("TransferIntakingSpeed", 3000.);
    prefDict.put("SensorDistance", 3.);

    // arm

    prefDict.put("armFFKs", 0.001);
    prefDict.put("armFFKv", 2.01);
    prefDict.put("armUpFFKv", 2.75);
    
    prefDict.put("armFFKa", .00001);
    prefDict.put("armFFKg", .00001);

    prefDict.put("armKp", .00001);
    prefDict.put("armKi", .00001);
    prefDict.put("armKd", .00001);
    prefDict.put("armKIZone", .00001);

    // climber

    prefDict.put("LockNumber", 1.);
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
