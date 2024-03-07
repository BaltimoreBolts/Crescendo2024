/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

package frc.library;

import edu.wpi.first.wpilibj.RobotController;

import org.growingstems.measurements.Measurements.Time;
import org.growingstems.util.timer.TimeSource;

/**
 * {@link org.growingstems.util.timer.TimeSource} implementation using
 * {@link edu.wpi.first.wpilibj.RobotController#getFPGATime()}. Precision: 1
 * microsecond
 */
public class WpiTimeSource implements TimeSource {
    /**
     * Gets the current time using
     * {@link edu.wpi.first.wpilibj.RobotController#getFPGATime()}
     *
     * @return Current time according to the FPGA.
     */
    @Override
    public Time clockTime() {
        return Time.microseconds(RobotController.getFPGATime());
    }
}
