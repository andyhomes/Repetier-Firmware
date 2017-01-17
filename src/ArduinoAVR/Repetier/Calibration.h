#ifndef CALIBRATION_H_INCLUDED
#define CALIBRATION_H_INCLUDED

#include "HAL.h"

#define AUTOCALIBRATION_FINE_TILT_SEARCH true

class Com1 {
public:
FSTRINGVAR(tAutocalibrationStarted)
FSTRINGVAR(tAutocalibrationFinished)
FSTRINGVAR(tNoCalibrationNeeded)
FSTRINGVAR(tIteration)
FSTRINGVAR(tOf)
FSTRINGVAR(tPrinterHasTilt)
FSTRINGVAR(tDirection)
FSTRINGVAR(tAngle)
FSTRINGVAR(tValue)
FSTRINGVAR(tNoTilt)
FSTRINGVAR(tCorrectingRadius)
FSTRINGVAR(tProbes)
FSTRINGVAR(tAt)
FSTRINGVAR(tSemicolon)
FSTRINGVAR(tTo)
FSTRINGVAR(tFineFlatness)
FSTRINGVAR(tWorkingOnFlatness)
FSTRINGVAR(tWorkingOnErrors)
FSTRINGVAR(tAllErrorsOk)
FSTRINGVAR(tCorrectingHeight)
FSTRINGVAR(tCenter)
FSTRINGVAR(tDeviation)
FSTRINGVAR(tTakingProbes)
FSTRINGVAR(tInspectingForTilt)
FSTRINGVAR(tFineTilt)
FSTRINGVAR(tCorrectingOffsets)
FSTRINGVAR(tBetterResultFound)
FSTRINGVAR(tSuccess)
FSTRINGVAR(tStoringBestResults)
//FSTRINGVAR(t)
};

class AutoCalibration {

private:
	float calibration_radius;

	uint8_t iteration;
	float height_correction;
	float probes[12];
	float deviation;
	float printer_flatness_error;
	int tilt_direction;
	float tilt_value;
	float tilt_angle;

	bool flatness_corrected;
	bool tilt_correceted;
	bool errors_corrected;

	static float average(float arr[], int sz);
	static float devsq(float arr[], int sz);

	void updateDeviation();
	void prepareForNextIteration();

	inline float getDeviation() {if (deviation == .0) updateDeviation(); return deviation; };

	void eliminateHeightError();

	void eliminateFlatnessError();

	void eliminateBedTilt();

	void findTilt(int ang1, int ang2, int angStep, float val1, float val2, float valStep, float &ref_devsq);

	void correctTilt();
	void correctFlatnes();
	void correctDeltaGeometry();

	inline bool isDeltaGeometryCorrected() {
		return errors_corrected;
	}

	inline bool isEverythingCorrected() {
		return flatness_corrected && tilt_correceted && isDeltaGeometryCorrected();
	}

	void writeProbesOut();

public:
	AutoCalibration(float calibration_radius);

    void takeProbes(uint8_t rounds = 1);

	void run(uint8_t max_iterations, uint8_t probe_rounds = 1);
};


class ZProbe {

public:

	static float runZProbe(bool first, bool last, uint8_t repeat, bool runStartScript);

	static void correctHeight(float radius = 0.0);

	static void measureZProbeZOffset(uint8_t repeats = 3);
};

#endif
