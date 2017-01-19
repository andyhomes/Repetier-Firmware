#ifndef CALIBRATION_H_INCLUDED
#define CALIBRATION_H_INCLUDED
#endif

#define DELTA_CALIBRATION_ENABLED ((DRIVE_SYSTEM==DELTA) && FEATURE_Z_PROBE && 1)

#if DELTA_CALIBRATION_ENABLED

#define AUTOCALIBRATION_FINE_TILT_SEARCH true

class Com1 {
public:
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
FSTRINGVAR(tStoringBestResults)
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

//	bool flatness_corrected;
//	bool tilt_correceted;
	bool errors_corrected;

	static float average(float arr[], int sz);
	static float devsq(float arr[], int sz);

	void updateDeviation();
	void prepareForNextIteration();

	inline float getDeviation() {if (deviation == .0) updateDeviation(); return deviation; };

	void eliminateHeightAndFlatnessError();

	void eliminateBedTilt();

	void findTilt(int ang1, int ang2, int angStep, float val1, float val2, float valStep, float &ref_devsq);

	void correctFlatnes();
	void correctDeltaGeometry();

	void writeProbesOut();

public:
	AutoCalibration(float calibration_radius);

    void takeProbes(uint8_t rounds = 1);

	void run(uint8_t max_iterations, uint8_t probe_rounds = 1);
};


class ZProbe {

public:

	static float runZProbe(bool first, bool last, uint8_t repeat = Z_PROBE_REPETITIONS, bool returnError = true);

	static void correctHeight(float radius = 0.0);

	static void measureZProbeHeight(uint8_t repeats = 3);
};

#endif // DELTA_CALIBRATION_ENABLED
