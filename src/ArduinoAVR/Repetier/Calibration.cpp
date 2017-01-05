/*
 *
 */

#include "Repetier.h"
#if FEATURE_DELTA_AUTO_CALIBRATION

FSTRINGVALUE(Com1::tAutocalibrationStarted, "Auto-calibration started...")
FSTRINGVALUE(Com1::tAutocalibrationFinished, "... auto-calibration finished.")
FSTRINGVALUE(Com1::tNoCalibrationNeeded, "Printer doesn't require calibration")
FSTRINGVALUE(Com1::tIteration, "Iteration ")
FSTRINGVALUE(Com1::tOf, " of ")
FSTRINGVALUE(Com1::tPrinterHasTilt, "Printer's bed has tilt that cannot be compensated with off-sets:")
FSTRINGVALUE(Com1::tDirection, "Direction: ")
FSTRINGVALUE(Com1::tAngle, "Angle: ")
FSTRINGVALUE(Com1::tValue, "Value: ")
FSTRINGVALUE(Com1::tNoTilt, "No tilt detected.")
FSTRINGVALUE(Com1::tCorrectingRadius, "Correcting printer radius: ")
FSTRINGVALUE(Com1::tProbes, "Probes: ")
FSTRINGVALUE(Com1::tAt, "@")
FSTRINGVALUE(Com1::tSemicolon, "; ")
FSTRINGVALUE(Com1::tTo, " -> ")
FSTRINGVALUE(Com1::tFineFlatness, "Printer flatness is fine: ")
FSTRINGVALUE(Com1::tWorkingOnFlatness, "Working on printer flatness...")
FSTRINGVALUE(Com1::tWorkingOnErrors, "Working on delta geometry errors...")
FSTRINGVALUE(Com1::tAllErrorsOk, "All errors within precision: ")
FSTRINGVALUE(Com1::tCorrectingHeight, "Correcting height: ")
FSTRINGVALUE(Com1::tCenter, "0,0")
FSTRINGVALUE(Com1::tDeviation, "Probes deviation: ")
FSTRINGVALUE(Com1::tTakingProbes, "Taking probes:")
FSTRINGVALUE(Com1::tInspectingForTilt, "Inspecting bed surface for tilt...")
FSTRINGVALUE(Com1::tFineTilt, "Bed tilt is fine: ")
FSTRINGVALUE(Com1::tCorrectingOffsets, "Correcting towers offsets: ");
FSTRINGVALUE(Com1::tBetterResultFound,"Found better results (devsq): ")
FSTRINGVALUE(Com1::tSuccess,"Success!")
FSTRINGVALUE(Com1::tStoringBestResults,"Storing best results:")
//FSTRINGVALUE(Com1::t,"")

AutoCalibration::AutoCalibration(float radius) {
	calibration_radius = radius;
	iteration = tilt_direction = 0;
	tilt_value = tilt_angle = height_correction = deviation = printer_flatness_error = .0;
	flatness_corrected = tilt_correceted = errors_corrected = false;
}

/**
 * Runs autocalibration process with given number of iterations.
 * If 'rounds' is greater than 1 it makes given number of rounds of taking probes resulting in average values in each iteration.
 */
void AutoCalibration::run(uint8_t max_iterations, uint8_t probe_rounds) {
	Com::printFLN(Com1::tAutocalibrationStarted);
    bool oldAutolevel = Printer::isAutolevelActive();
    Printer::setAutolevelActive(false);

    float best_devsq = 9999.0f;

	float best_radius;
	int8_t best_offset_a;
	int8_t best_offset_b;
	int8_t best_offset_c;
	float best_rad_correction_a;
	float best_rad_correction_b;
	float best_rad_correction_c;
	float best_diag_correction_a;
	float best_diag_correction_b;
	float best_diag_correction_c;


	while (iteration < max_iterations) {
		prepareForNextIteration();
		Com::printFLN(PSTR("---"));
		Com::printF(Com1::tIteration, iteration);
		Com::printFLN(Com1::tOf, max_iterations);

		takeProbes(probe_rounds);
		if (getDeviation() <= DELTA_CALIBRATION_PRECISION) {
			if (iteration == 1) {
				Com::printFLN(Com1::tNoCalibrationNeeded);
			} else {
				Com::printF(Com1::tSuccess);
			}
			break;
		}

		float devsq_val = devsq(probes, 12);
		if (iteration == 1 || best_devsq > devsq_val) {
			best_radius = Printer::radius0;
			best_offset_a = EEPROM::deltaTowerXOffsetSteps();
			best_offset_b = EEPROM::deltaTowerYOffsetSteps();
			best_offset_c = EEPROM::deltaTowerZOffsetSteps();
			best_rad_correction_a = EEPROM::deltaRadiusCorrectionA();
			best_rad_correction_b = EEPROM::deltaRadiusCorrectionB();
			best_rad_correction_c = EEPROM::deltaRadiusCorrectionC();
			best_diag_correction_a = EEPROM::deltaDiagonalCorrectionA();
			best_diag_correction_b = EEPROM::deltaDiagonalCorrectionB();
			best_diag_correction_c = EEPROM::deltaDiagonalCorrectionC();
			if (iteration > 1) {
				Com::printF(Com1::tBetterResultFound, best_devsq, 4);
				Com::printFLN(Com1::tTo, devsq_val, 4);
			}
			best_devsq = devsq_val;
		}

		if (iteration == max_iterations) {
			break;
		}

		eliminateHeightError();
		eliminateFlatnessError();
		eliminateBedTilt();

		if (! tilt_correceted) {
			correctTilt();
		}

		if (! flatness_corrected) {
			correctFlatnes();
		}

		correctDeltaGeometry();

		if (height_correction != 0.0) {
			Com::printF(Com1::tCorrectingHeight, Printer::zLength, 3);
			Printer::zLength -= height_correction;
			height_correction = 0.0;
			Com::printFLN(Com1::tTo, Printer::zLength, 3);
			Printer::updateDerivedParameter();
		}
	};

//	if (!isEverythingCorrected()) {
//		// TODO report iterations limit exceeded, parameters not stored
//		Com::printWarningFLN()
//	}

    Printer::setAutolevelActive(oldAutolevel);

    // setting best values and store
    if (iteration > 1) {
		Com::printFLN(PSTR("---"));
		Com::printFLN(Com1::tStoringBestResults);
    	Printer::radius0 = best_radius;
		Com::printFLN(PSTR("Printer radius: "), Printer::radius0, 3);
    	HAL::eprSetFloat(EPR_DELTA_HORIZONTAL_RADIUS, Printer::radius0);

    	Com::printF(Com::tEPRTowerXOffset);
		Com::printFLN(Com::tSpace, best_offset_a, 3);
		EEPROM::setDeltaTowerXOffsetSteps(best_offset_a);

    	Com::printF(Com::tEPRTowerYOffset);
		Com::printFLN(Com::tSpace, best_offset_b, 3);
		EEPROM::setDeltaTowerYOffsetSteps(best_offset_b);

    	Com::printF(Com::tEPRTowerZOffset);
		Com::printFLN(Com::tSpace, best_offset_c, 3);
		EEPROM::setDeltaTowerZOffsetSteps(best_offset_c);

		Com::printF(Com::tDeltaRadiusCorrectionA);
		Com::printFLN(Com::tSpace, best_rad_correction_a, 3);
		HAL::eprSetFloat(EPR_DELTA_RADIUS_CORR_A, best_rad_correction_a);

		Com::printF(Com::tDeltaRadiusCorrectionB);
		Com::printFLN(Com::tSpace, best_rad_correction_b, 3);
		HAL::eprSetFloat(EPR_DELTA_RADIUS_CORR_B, best_rad_correction_b);

		Com::printF(Com::tDeltaRadiusCorrectionC);
		Com::printFLN(Com::tSpace, best_rad_correction_c, 3);
		HAL::eprSetFloat(EPR_DELTA_RADIUS_CORR_C, best_rad_correction_c);

		Com::printF(Com::tDeltaDiagonalCorrectionA);
		Com::printFLN(Com::tSpace, best_diag_correction_a, 3);
		HAL::eprSetFloat(EPR_DELTA_DIAGONAL_CORRECTION_A, best_diag_correction_a);

		Com::printF(Com::tDeltaDiagonalCorrectionB);
		Com::printFLN(Com::tSpace, best_diag_correction_b, 3);
		HAL::eprSetFloat(EPR_DELTA_DIAGONAL_CORRECTION_B, best_diag_correction_b);

		Com::printF(Com::tDeltaDiagonalCorrectionC);
		Com::printFLN(Com::tSpace, best_diag_correction_c, 3);
		HAL::eprSetFloat(EPR_DELTA_DIAGONAL_CORRECTION_C, best_diag_correction_c);

//		EEPROM::storeDataIntoEEPROM(false); do not need this, writing necessary parameters directly into EEPROM
		EEPROM::updateChecksum();
		Com::printInfoF(Com::tConfigStoredEEPROM);
		Printer::updateDerivedParameter();
    }

	Printer::homeAxis(true, true, true);
	Printer::updateCurrentPosition();
	Com::printFLN(Com1::tAutocalibrationFinished);
}

void AutoCalibration::correctTilt() {
	if (tilt_angle > .0) {
		// tilt value is height change from center to calibration radius, along the bed it's twice bigger
		if (tilt_value < DELTA_CALIBRATION_PRECISION / 2.0) {
			Com::printFLN(Com1::tFineTilt, tilt_value, 3);
			tilt_correceted = true;
		} else if (tilt_value <= DELTA_CALIBRATION_COMPENSABLE_TILT) {
			Com::printFLN(Com1::tDirection, tilt_direction);
			Com::printFLN(Com1::tAngle, tilt_angle, 3);
			Com::printFLN(Com1::tValue, tilt_value, 3);
			Com::printFLN(Com1::tCorrectingOffsets);
			int16_t osA, osB, osC, osMin;
			osA = cos(radians(DELTA_ALPHA_A - tilt_direction)) * tilt_value * Printer::axisStepsPerMM[X_AXIS] + EEPROM::deltaTowerXOffsetSteps();
			osB = cos(radians(DELTA_ALPHA_B - tilt_direction)) * tilt_value * Printer::axisStepsPerMM[Y_AXIS] + EEPROM::deltaTowerYOffsetSteps();
			osC = cos(radians(DELTA_ALPHA_C - tilt_direction)) * tilt_value * Printer::axisStepsPerMM[Z_AXIS] + EEPROM::deltaTowerZOffsetSteps();
			osMin = RMath::min(RMath::min(osA, osB), osC);
			osA -= osMin;
			osB -= osMin;
			osC -= osMin;

			Com::printF(Com::tEPRTowerXOffset);
			Com::printF(Com::tSpace, EEPROM::deltaTowerXOffsetSteps(), 3);
			EEPROM::setDeltaTowerXOffsetSteps(osA);
			Com::printFLN(Com1::tTo, osA, 3);

			Com::printF(Com::tEPRTowerYOffset);
			Com::printF(Com::tSpace, EEPROM::deltaTowerYOffsetSteps(), 3);
			EEPROM::setDeltaTowerYOffsetSteps(osB);
			Com::printFLN(Com1::tTo, osB, 3);

			Com::printF(Com::tEPRTowerZOffset);
			Com::printF(Com::tSpace, EEPROM::deltaTowerZOffsetSteps(), 3);
			EEPROM::setDeltaTowerZOffsetSteps(osC);
			Com::printFLN(Com1::tTo, osC, 3);

			EEPROM::updateChecksum();

			Printer::updateDerivedParameter();
			tilt_value = 0.0;
		} else {
			Com::printWarningFLN(Com1::tPrinterHasTilt);
			Com::printFLN(Com1::tDirection, tilt_direction);
			Com::printFLN(Com1::tAngle, tilt_angle, 2);
			Com::printFLN(Com1::tValue, tilt_value, 2);
			tilt_correceted = true;
		}
	} else {
		Com::printFLN(Com1::tNoTilt);
		tilt_correceted = true;
	}
}

void AutoCalibration::correctFlatnes() {
	if (abs(printer_flatness_error) > DELTA_CALIBRATION_PRECISION) {
		// I believe that correction for printer radius can be computed
		// based on height error (center vs calibration radius)
		// but didn't find the formula yet.
		// So, for the time being we do correction with a few iterations.
		// TODO try to find the formula

		// Positive value of getPrinterFlatnessError() means printer radius must be bigger
		Com::printFLN(Com1::tCorrectingRadius);
		Com::printFloat(Printer::radius0, 3);
		float correction = printer_flatness_error * 1.618; // * 1.618 - faster approach

		// limit correction to prevent significant changes when probing works incorrect.
		// huge changes may cause probe to go outside bed
		if (abs(correction) > 0.25) {
			if (correction > 0) {
				correction = 0.25;
			} else {
				correction = -0.25;
			}
		}
		Printer::radius0 += correction;
		Com::printFLN(Com1::tTo, Printer::radius0, 3);
		HAL::eprSetFloat(EPR_DELTA_HORIZONTAL_RADIUS, Printer::radius0);
		EEPROM::updateChecksum();
		Printer::updateDerivedParameter();
	} else {
		Com::printFLN(Com1::tFineFlatness, printer_flatness_error, 3);
		flatness_corrected = true;
	}
}

void AutoCalibration::correctDeltaGeometry() {
	float errs[3];
	float errs_average;

	errs[0] = 0.382 * (probes[1] + probes[6] * 2 + probes[7] * 2 + probes[8] * 2) / 7; // TODO find better dependency
	errs[1] = 0.382 * (probes[0] * 2 + probes[5] + probes[10] * 2 + probes[11] * 2) / 7;
	errs[2] = 0.382 * (probes[2] * 2 + probes[3] * 2 + probes[4] * 2 + probes[9]) / 7;
	errs_average = average(errs, 3);
	errs[0] -= errs_average;
	errs[1] -= errs_average;
	errs[2] -= errs_average;

	Com::printF(Com::tDeltaRadiusCorrectionA);
	Com::printF(Com::tSpace, EEPROM::deltaRadiusCorrectionA(), 3);
	HAL::eprSetFloat(EPR_DELTA_RADIUS_CORR_A, EEPROM::deltaRadiusCorrectionA() + errs[0]);
	Com::printFLN(Com1::tTo, EEPROM::deltaRadiusCorrectionA(), 3);

	Com::printF(Com::tDeltaRadiusCorrectionB);
	Com::printF(Com::tSpace, EEPROM::deltaRadiusCorrectionB(), 3);
	HAL::eprSetFloat(EPR_DELTA_RADIUS_CORR_B, EEPROM::deltaRadiusCorrectionB() + errs[1]);
	Com::printFLN(Com1::tTo, EEPROM::deltaRadiusCorrectionB(), 3);

	Com::printF(Com::tDeltaRadiusCorrectionC);
	Com::printF(Com::tSpace, EEPROM::deltaRadiusCorrectionC(), 3);
	HAL::eprSetFloat(EPR_DELTA_RADIUS_CORR_C, EEPROM::deltaRadiusCorrectionC() + errs[2]);
	Com::printFLN(Com1::tTo, EEPROM::deltaRadiusCorrectionC(), 3);

	errs[0] = 0.382 * (probes[1] + probes[7]) / 2; // TODO find better dependency
	errs[1] = 0.382 * (probes[5] + probes[11]) / 2;
	errs[2] = 0.382 * (probes[3] + probes[9]) / 2;
	errs_average = average(errs, 3);
	errs[0] -= errs_average;
	errs[1] -= errs_average;
	errs[2] -= errs_average;

	Com::printF(Com::tDeltaDiagonalCorrectionA);
	Com::printF(Com::tSpace, EEPROM::deltaDiagonalCorrectionA(), 3);
	HAL::eprSetFloat(EPR_DELTA_DIAGONAL_CORRECTION_A, EEPROM::deltaDiagonalCorrectionA() - errs[0]);
	Com::printFLN(Com1::tTo, EEPROM::deltaDiagonalCorrectionA(), 3);

	Com::printF(Com::tDeltaDiagonalCorrectionB);
	Com::printF(Com::tSpace, EEPROM::deltaDiagonalCorrectionB(), 3);
	HAL::eprSetFloat(EPR_DELTA_DIAGONAL_CORRECTION_B, EEPROM::deltaDiagonalCorrectionB() - errs[1]);
	Com::printFLN(Com1::tTo, EEPROM::deltaDiagonalCorrectionB(), 3);

	Com::printF(Com::tDeltaDiagonalCorrectionC);
	Com::printF(Com::tSpace, EEPROM::deltaDiagonalCorrectionC(), 3);
	HAL::eprSetFloat(EPR_DELTA_DIAGONAL_CORRECTION_C, EEPROM::deltaDiagonalCorrectionC() - errs[2]);
	Com::printFLN(Com1::tTo, EEPROM::deltaDiagonalCorrectionC(), 3);

	EEPROM::updateChecksum();
	Printer::updateDerivedParameter();
}

void AutoCalibration::eliminateFlatnessError() {
	printer_flatness_error = average(probes, 12);
	if (abs(printer_flatness_error) < DELTA_CALIBRATION_PRECISION) {
		printer_flatness_error = 0.0;
	} else {
		for (int i = 0; i < 12; i++) {
			probes[i] -= printer_flatness_error;
		}
		updateDeviation();
	}
}

void AutoCalibration::findTilt(int ang1, int ang2, int angStep, float val1, float val2, float valStep, float &ref_devsq) {
	// looking for a combination with minimal devsq
	float coses[12];
	float tvals[12];
	float dsq;
	for (int ang = ang1; ang <= ang2; ang += angStep) {
		for (int i = 0; i < 12; i++) {
			coses[i] = cos(radians(i * 30 - ang));
		}
		for (float val = val1; val <= val2; val += valStep) {
			for (int i = 0; i < 12; i++) {
				tvals[i] = probes[i] - coses[i] * val;
			}
			dsq = devsq(tvals, 12);
			if (dsq < ref_devsq) {
				ref_devsq = dsq;
				tilt_value = val;
				tilt_direction = ang - 180;
			}
		}
	}
}

/**
 * Tries to find bed tilt and remove it from probes results
 */
void AutoCalibration::eliminateBedTilt() {
	if (!tilt_correceted) {
		tilt_value = .0;
		// find candidate points
		int ca1 = 0, ca2 = 0;
		float cv1 = -9999.9, cv2 = 9999.9;
		for (int i = 0; i < 12; i++) {
			if (probes[i] > cv1) {
				cv1 = probes[i];
				ca1 = i;
			}
			if (probes[i] < cv2) {
				cv2 = probes[i];
				ca2 = i;
			}
		}
		// indexes -> degrees
		ca1 *= 30;
		ca2 *= 30;
		// move cv2 to opposite side
		cv2 = -cv2;
		ca2 = (ca2 + 180) % 360;

		// define ranges
		int start_angle = RMath::min(ca1, ca2) - 15;
		int end_angle = RMath::max(ca1, ca2) + 15;
		float start_val = DELTA_CALIBRATION_PRECISION;
		float end_val = RMath::max(cv1, cv2) + DELTA_CALIBRATION_PRECISION;

		float best_devsq = devsq(probes, 12);
		findTilt(start_angle, end_angle, 5, start_val, end_val, DELTA_CALIBRATION_PRECISION, best_devsq);
#if AUTOCALIBRATION_FINE_TILT_SEARCH
		if (tilt_value > 0.0) {
			start_angle = tilt_angle - 5;
			end_angle = tilt_angle + 5;
			start_val = tilt_value - DELTA_CALIBRATION_PRECISION;
			end_val = tilt_value + DELTA_CALIBRATION_PRECISION;
			findTilt(start_angle, end_angle, 1, start_val, end_val, DELTA_CALIBRATION_PRECISION / 10.0, best_devsq);
		}
#endif
	}
	if (tilt_value > .0) { // tilt has been found
		if (tilt_value < DELTA_CALIBRATION_PRECISION/2.0) {
			tilt_value = 0.0;
			tilt_angle = 0;
		} else {
			tilt_angle = degrees(asin(tilt_value/calibration_radius));
			for (int i = 0; i < 12; i++) {
				probes[i] += cos(radians(i * 30 - tilt_direction + 180)) * tilt_value;
			}
			updateDeviation();
		}
	}
}

void AutoCalibration::prepareForNextIteration() {
	iteration++;
	height_correction = deviation = printer_flatness_error = .0;
	for (int i = 0; i < 12; i++) {
		probes[i] = .0;
	}
	// force flatness and tilt checks on each iteration
	tilt_correceted = flatness_corrected = false;
}

/**
 * Takes 13 probes: 1 - center and 12 - around on DELTA_CALIBRATION_RADIUS.
 * If 'rounds' is greater than 1 it makes given number of rounds of taking probes resulting in average values.
 */
void AutoCalibration::takeProbes(uint8_t rounds) {
	if (rounds < 1) {
		rounds = 1;
	}
	Printer::homeAxis(true, true, true);
	Printer::updateCurrentPosition();
	GCode::executeFString(Com::tZProbeStartScript);
	Com::println();
	Com::printFLN(Com1::tTakingProbes);
	for (uint8_t r = 1; r <= rounds; r++) {
		Printer::moveTo(0.0, 0.0, IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
		bool is_first_probe = (r == 1);
		height_correction += ZProbe::runZProbe(is_first_probe, false, Z_PROBE_REPETITIONS, false) + EEPROM::zProbeHeight();
		uint16_t degrees;
		float prX, prY;
		for (uint8_t i = 0; i < 12; i++) {
			degrees = i * 30;
			prX = cos(degrees * DEG_TO_RAD) * calibration_radius;
			prY = sin(degrees * DEG_TO_RAD) * calibration_radius;
			Printer::moveTo(prX, prY, IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
			bool is_last_probe = (i == 11 && r == rounds);
			probes[i] = ZProbe::runZProbe(false, is_last_probe, Z_PROBE_REPETITIONS, false) + EEPROM::zProbeHeight();
		}
	}
	if (rounds > 1) {
		height_correction /= rounds;
		for (uint8_t i = 0; i < 12; i++) {
			probes[i] /= rounds;
		}
	}
	writeProbesOut();
}

void AutoCalibration::eliminateHeightError() {
	for (uint8_t i = 0; i < 12; i++) {
		probes[i] -= height_correction;
	}
}

void AutoCalibration::writeProbesOut() {
	Com::printFloat(height_correction, 3);
	Com::printF(Com1::tSemicolon);
	for (uint8_t i = 0; i < 12; i++) {
		Com::printFloat(probes[i],3);
		Com::printF(Com1::tSemicolon);
	}
	Com::printFloat(getDeviation(), 4);
	Com::printF(Com1::tSemicolon);
	Com::printFloat(devsq(probes, 12), 4);
	Com::println();
}

void AutoCalibration::updateDeviation() {
	float min = 999.9, max = -999.9;
	for (int i = 0; i < 12; i++) {
		min = RMath::min(min, probes[i]);
		max = RMath::max(max, probes[i]);
	}
	deviation = max - min;
}

float AutoCalibration::average(float arr[], int sz) {
	float res = .0;
	for (int i = 0; i < sz; i++) {
		res += arr[i];
	}
	return res / (float) sz;
}

float AutoCalibration::devsq(float arr[], int sz) {
	float avr = average(arr, sz);
	float dsq = .0;
	float dif;
	for (int i = 0; i < sz; i++) {
		dif = arr[i] - avr;
		dsq += dif * dif;
	}
	return dsq;
}

/**
 * Modified version of Printer::runZProbe.
 *
 * Method returns value that means 'Where nozzle would be on z-axis if it goes to position Z0 at current X and Y coordinates?'.
 * Negative values mean nozzle would hit bed.
 */
float ZProbe::runZProbe(bool first,bool last,uint8_t repeat,bool runStartScript) {
    float oldOffX = Printer::offsetX;
    float oldOffY = Printer::offsetY;
    float oldOffZ = Printer::offsetZ;
    if(first) {
        if(runStartScript) {
            GCode::executeFString(Com::tZProbeStartScript);
        }
        Printer::offsetX = -EEPROM::zProbeXOffset();
        Printer::offsetY = -EEPROM::zProbeYOffset();
        Printer::offsetZ = 0; // we correct this with probe height
        PrintLine::moveRelativeDistanceInSteps((Printer::offsetX - oldOffX) * Printer::axisStepsPerMM[X_AXIS],
                                               (Printer::offsetY - oldOffY) * Printer::axisStepsPerMM[Y_AXIS],
                                               0, 0, EEPROM::zProbeXYSpeed(), true, ALWAYS_CHECK_ENDSTOPS);
    }
    Commands::waitUntilEndOfAllMoves();
    int32_t sum = 0, probeDepth;
    int32_t shortMove = static_cast<int32_t>((float)Z_PROBE_SWITCHING_DISTANCE * Printer::axisStepsPerMM[Z_AXIS]); // distance to go up for repeated moves
    int32_t startingZPosition = Printer::currentPositionSteps[Z_AXIS];

    Printer::realDeltaPositionSteps[Z_AXIS] = Printer::currentNonlinearPositionSteps[Z_AXIS]; // update real

    Printer::waitForZProbeStart();
    for(int8_t r = 0; r < repeat; r++) {
        probeDepth = 2 * (Printer::zMaxSteps - Printer::zMinSteps); // probe should always hit within this distance
        Printer::stepsRemainingAtZHit = -1; // Marker that we did not hit z probe
        Printer::setZProbingActive(true);
        PrintLine::moveRelativeDistanceInSteps(0, 0, -probeDepth, 0, EEPROM::zProbeSpeed(), true, true);
        if(Printer::stepsRemainingAtZHit < 0) {
            Com::printErrorFLN(Com::tZProbeFailed);
            return -1;
        }
        Printer::setZProbingActive(false);
        Printer::stepsRemainingAtZHit = Printer::realDeltaPositionSteps[C_TOWER] - Printer::currentNonlinearPositionSteps[C_TOWER]; // nonlinear moves may split z so stepsRemainingAtZHit is only what is left from last segment not total move. This corrects the problem.

        Printer::currentNonlinearPositionSteps[A_TOWER] += Printer::stepsRemainingAtZHit; // Update difference
        Printer::currentNonlinearPositionSteps[B_TOWER] += Printer::stepsRemainingAtZHit;
        Printer::currentNonlinearPositionSteps[C_TOWER] += Printer::stepsRemainingAtZHit;

        Printer::currentPositionSteps[Z_AXIS] += Printer::stepsRemainingAtZHit; // now current position is correct
		sum += Printer::currentPositionSteps[Z_AXIS];
        if(r < repeat - 1) // go only shortes possible move up for repetitions
            PrintLine::moveRelativeDistanceInSteps(0, 0, shortMove, 0, EEPROM::zProbeSpeed() * 10, true, false);
    }
	float distance = static_cast<float>(sum) * Printer::invAxisStepsPerMM[Z_AXIS] / static_cast<float>(repeat);
    PrintLine::moveRelativeDistanceInSteps(0, 0, 5 * Printer::axisStepsPerMM[Z_AXIS], 0, EEPROM::zProbeSpeed(), true, false);
    if(last) {
        oldOffX = Printer::offsetX;
        oldOffY = Printer::offsetY;
        oldOffZ = Printer::offsetZ;
        GCode::executeFString(Com::tZProbeEndScript);
        if(Extruder::current) {
            Printer::offsetX = -Extruder::current->xOffset * Printer::invAxisStepsPerMM[X_AXIS];
            Printer::offsetY = -Extruder::current->yOffset * Printer::invAxisStepsPerMM[Y_AXIS];
            Printer::offsetZ = -Extruder::current->zOffset * Printer::invAxisStepsPerMM[Z_AXIS];
        }
        PrintLine::moveRelativeDistanceInSteps((Printer::offsetX - oldOffX) * Printer::axisStepsPerMM[X_AXIS],
                                               (Printer::offsetY - oldOffY) * Printer::axisStepsPerMM[Y_AXIS],
                                               (Printer::offsetZ - oldOffZ) * Printer::axisStepsPerMM[Z_AXIS], 0, EEPROM::zProbeXYSpeed(), true, ALWAYS_CHECK_ENDSTOPS);
    }
    return distance;
}

/**
 * Corrects printer's height by taking one or four probes.
 * When radius is 0 - takes only one probe at center.
 * With given radius it takes 4 probes: one at center and three against each tower on radius distance from center.
 * Corrects height by average of the probes.
 */
void ZProbe::correctHeight(float radius) {
	// force homing
	Printer::homeAxis(true, true, true);
	Printer::updateCurrentPosition();
	GCode::executeFString(Com::tZProbeStartScript);

	float prX, prY;
	Printer::moveTo(0.0, 0.0, IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
	float h_correction = ZProbe::runZProbe(true, (radius == 0.0), Z_PROBE_REPETITIONS, false);
	if (radius != 0.0) {
		prX = cos(90 * DEG_TO_RAD) * radius;
		prY = sin(90 * DEG_TO_RAD) * radius;
		Printer::moveTo(prX, prY, IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
		h_correction += ZProbe::runZProbe(false, false, Z_PROBE_REPETITIONS, false);
		prX = cos(210 * DEG_TO_RAD) * radius;
		prY = sin(210 * DEG_TO_RAD) * radius;
		Printer::moveTo(prX, prY, IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
		h_correction += ZProbe::runZProbe(false, false, Z_PROBE_REPETITIONS, false);
		prX = cos(330 * DEG_TO_RAD) * radius;
		prY = sin(330 * DEG_TO_RAD) * radius;
		Printer::moveTo(prX, prY, IGNORE_COORDINATE, IGNORE_COORDINATE, EEPROM::zProbeXYSpeed());
		h_correction += ZProbe::runZProbe(false, true, Z_PROBE_REPETITIONS, false);
		h_correction /= 4.0;
	}
	h_correction += EEPROM::zProbeHeight();
    Printer::updateCurrentPosition();
    Printer::zLength -= h_correction;
    Printer::updateDerivedParameter();
    Com::printFLN(Com::tZProbeCorrection, -h_correction, 2);
    Com::printFLN(Com::tZProbePrinterHeight, Printer::zLength, 2);
    Printer::homeAxis(true,true,true);
}

/**
 * Having printer's height set properly (i.e. after calibration of Z=0), one can use this procedure to measure Z-probe offset.
 * It deploys the sensor, takes several probes at center, then updates Z-probe height (i.e. z offset) with average.
 * The sensor is deployed and undeployed on every probe.
 */
void ZProbe::measureZProbeZOffset(uint8_t repeats) {
	float zProbeOffset = 0.0;
	for (uint8_t i = 0; i < repeats; i++) {
		zProbeOffset += runZProbe(true, true, Z_PROBE_REPETITIONS, true);
	}
	zProbeOffset = -zProbeOffset / repeats;
	Com::printFLN(Com::tZProbeHeight, zProbeOffset);
	EEPROM::setZProbeHeight(zProbeOffset);
	Printer::updateCurrentPosition(true);
}

#endif