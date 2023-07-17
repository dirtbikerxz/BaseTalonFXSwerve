package frc.lib.util.loggingUtil;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * Utilty class for logging data to the DataLogManager.
 * View logs using MechanicalAdvantage's advantage scope (https://github.com/Mechanical-Advantage/AdvantageScope)
 */
public class LogManager {

  private static DataLog log = DataLogManager.getLog();

  // These are array lists of log entry classes from WPI. appending to a log entry automatically adds to the log file.
  private static ArrayList<DoubleLogEntry> doubleLogs = new ArrayList<>();
  private static ArrayList<DoubleArrayLogEntry> doubleArrayLogs = new ArrayList<>();
  private static ArrayList<BooleanLogEntry> booleanLogs = new ArrayList<>();
  private static ArrayList<IntegerLogEntry> intLogs = new ArrayList<>();

  // These are the suppliers, or functions that return values. This is how the values are accessed.
  private static ArrayList<DoubleSupplier> doubleValues = new ArrayList<>();
  private static ArrayList<DoubleSupplier[]> doubleArrayValues = new ArrayList<>();
  private static ArrayList<BooleanSupplier> booleanValues = new ArrayList<>();
  private static ArrayList<IntSupplier> intValues = new ArrayList<>();

  // These are the log entries that are not updated periodically, they just receive individual values.
  private static HashMap<String, DoubleLogEntry> individualDoubleLogs = new HashMap<>();
  private static HashMap<String, DoubleArrayLogEntry> individualDoubleArrayLogs = new HashMap<>();
  private static HashMap<String, BooleanLogEntry> individualBooleanLogs = new HashMap<>();
  private static HashMap<String, IntegerLogEntry> individualIntegerLogs = new HashMap<>();

  

  /**
   * @deprecated Use {@link #addDouble(String, double)} with a single value instead.
   */
  @Deprecated
  public static void addDouble(String name, DoubleSupplier logged) {
    DoubleLogEntry myDoubleLog = new DoubleLogEntry(log, name);
    doubleLogs.add(myDoubleLog);
    doubleValues.add(logged);
  }

  /**
   * Logs a single double value to the log. Do not use with the other addDouble() that takes a double supplier.
   * This will only log the value once, so it should be called periodically or when needed. If you have a function that consistently 
   * returns values, it may be easier to use the double supplier log.
   * 
   * @param name The name of the log. Use / to create subdirectories, and keep names unique.
   * @param value the value to be logged.
   */
  public static void addDouble(String name, double value) {
    if (individualDoubleLogs.containsKey(name)) {
      individualDoubleLogs.get(name).append(value);
    } else {
      individualDoubleLogs.put(name, new DoubleLogEntry(log, name));
      individualDoubleLogs.get(name).append(value);
    }
  }

  /**
   * @deprecated Use {@link #addDoubleArray(String, double[])} with a single value instead.
   */
  @Deprecated
  public static void addDoubleArray(String name, DoubleSupplier[] logged) {
    DoubleArrayLogEntry myDoubleLog = new DoubleArrayLogEntry(log, name);
    doubleArrayLogs.add(myDoubleLog);
    doubleArrayValues.add(logged);
  }

  /**
   * Logs a single double array to the log. Do not use with the other addDoubleArray() that takes a double array supplier.
   * This will only log the value once, so it should be called periodically or when needed. If you have a function that consistently 
   * returns values, it may be easier to use the double array supplier log.
   * 
   * @param name The name of the log. Use / to create subdirectories, and keep names unique.
   * @param value the value to be logged.
   */
  public static void addDoubleArray(String name, double[] value) {
    if (individualDoubleArrayLogs.containsKey(name)) {
      individualDoubleArrayLogs.get(name).append(value);
    } else {
      individualDoubleArrayLogs.put(name, new DoubleArrayLogEntry(log, name));
      individualDoubleArrayLogs.get(name).append(value);
    }
  }

  /**
   * @deprecated Use {@link #addInt(String, int)} with a single value instead.
   */
  @Deprecated
  public static void addInt(String name, IntSupplier logged) {
    IntegerLogEntry IntegerLog = new IntegerLogEntry(log, name);
    intLogs.add(IntegerLog);
    intValues.add(logged);
  }

  /**
   * Logs a single int to the log. Do not use with the other addInt() that takes a int supplier.
   * This will only log the value once, so it should be called periodically or when needed. If you have a function that consistently 
   * returns values, it may be easier to use the int supplier log.
   * 
   * @param name The name of the log. Use / to create subdirectories, and keep names unique.
   * @param value the value to be logged.
   */
  public static void addInt(String name, int value) {
    if (individualIntegerLogs.containsKey(name)) {
      individualIntegerLogs.get(name).append(value);
    } else {
      individualIntegerLogs.put(name, new IntegerLogEntry(log, name));
      individualIntegerLogs.get(name).append(value);
    }
  }

  /**
   * @deprecated Use {@link #addBoolean(String, boolean)} with a single value instead.
   */
  @Deprecated
  public static void addBoolean(String name, BooleanSupplier logged) {
    BooleanLogEntry BooleanLog = new BooleanLogEntry(log, name);
    booleanLogs.add(BooleanLog);
    booleanValues.add(logged);
  }

  /**
   * Logs a single boolean to the log. Do not use with the other addBoolean() that takes a boolean supplier.
   * This will only log the value once, so it should be called periodically or when needed. If you have a function that consistently 
   * returns values, it may be easier to use the boolean supplier log.
   * 
   * @param name The name of the log. Use / to create subdirectories, and keep names unique.
   * @param value the value to be logged.
   */
  public static void addBoolean(String name, boolean value) {
    if (individualBooleanLogs.containsKey(name)) {
      individualBooleanLogs.get(name).append(value);
    } else {
      individualBooleanLogs.put(name, new BooleanLogEntry(log, name));
      individualBooleanLogs.get(name).append(value);
    }
  }

  /**
   * Logs all the values that have been collected. Should be called periodically. 
   */
  public static void log() {
    for (int i = 0; i < doubleLogs.size(); i++) {
      doubleLogs.get(i).append(doubleValues.get(i).getAsDouble());
    }
    for (int i = 0; i < doubleArrayLogs.size(); i++) {
      double[] values = new double[doubleArrayValues.get(i).length];
      for (int j = 0; j < doubleArrayValues.get(i).length; j++) {
        values[j] = doubleArrayValues.get(i)[j].getAsDouble();
      }
      doubleArrayLogs.get(i).append(values);
    }
    for (int i = 0; i < intLogs.size(); i++) {
      intLogs.get(i).append(intValues.get(i).getAsInt());
    }
    for (int i = 0; i < booleanLogs.size(); i++) {
      booleanLogs.get(i).append(booleanValues.get(i).getAsBoolean());
    }
  }
}