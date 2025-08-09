// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.util;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import java.util.function.Supplier;

public class PhoenixUtil {
  /** Attempts to run the command until no error is produced. */
  public static void tryUntilOk(int maxAttempts, Supplier<StatusCode> command) {
    for (int i = 0; i < maxAttempts; i++) {
      var error = command.get();
      if (error.isOK()) break;
    }
  }

  /** Rio signals for synchronized refresh. */
  private static BaseStatusSignal[] rioSignals = new BaseStatusSignal[0];

  /** Registers a set of signals for synchronized refresh. */
  public static void registerSignals(boolean canivore, BaseStatusSignal... signals) {
    BaseStatusSignal[] newSignals = new BaseStatusSignal[rioSignals.length + signals.length];
    System.arraycopy(rioSignals, 0, newSignals, 0, rioSignals.length);
    System.arraycopy(signals, 0, newSignals, rioSignals.length, signals.length);
    rioSignals = newSignals;
  }

  /** Refresh all registered signals. */
  public static void refreshAll() {
    if (rioSignals.length > 0) {
      BaseStatusSignal.refreshAll(rioSignals);
    }
  }
}
