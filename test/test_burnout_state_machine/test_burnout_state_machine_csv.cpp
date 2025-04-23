#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
#include <cmath>
#include <iomanip>                 // NEW
#include "unity.h"
#include "state_estimation/BurnoutStateMachine.h"
#include "state_estimation/LaunchDetector.h"
#include "state_estimation/ApogeeDetector.h"
#include "state_estimation/VerticalVelocityEstimator.h"
#include "state_estimation/ApogeePredictor.h"
#include "data_handling/DataPoint.h"
#include "DataSaver_mock.h"
#include "../CSVMockData.h"
#include "algorithm"
#include <random>

// launch @ 3675568
// burnout @ 3677343 | 1436761
// apogee @ - | 1444561
// - 3674269.348155

static std::default_random_engine rng{42};
static std::normal_distribution<float> timeNoise(0.0f, 0.0f); // ms

void test_burnout_state_machine_with_real_data(void)
{
    CSVDataProvider provider(
        "data/MARTHA_4-13_1.3_B1_transformed.csv",
        100.0f);                                  // 25 Hz sample‑rate data → 40 ms Δt

    LaunchDetector          lp(30, 1000, 40);
    ApogeeDetector           ad;
    VerticalVelocityEstimator vve;
    ApogeePredictor          apogeePredictor(vve);
    ApogeePredictor          quadApogeePredictor(vve);
    DataSaverMock            dataSaver;
    BurnoutStateMachine      sm(&dataSaver, &lp, &ad, &vve);

    // ── CSV LOG ──────────────────────────────────────────────────────────
    std::ofstream log("test/test_burnout_state_machine/burnout_state_machine_log.csv",
                      std::ios::trunc | std::ios::out);
    TEST_ASSERT_TRUE_MESSAGE(log.is_open(),
                             "Could not open logs/burnout_state_machine_log.csv");

    log << "time_ms,ax_g,ay_g,az_g,alt_m,state,estAlt_m,estVel_mps,"
           "predApogee_m,quadPredApogee_m,timeToApogee_s\n";
    log << std::fixed << std::setprecision(3);

    // ── variables for assertions ────────────────────────────────────────
    bool     hasData        = false;
    float    maxAltitude    = -1000.0f;
    uint32_t maxAltTime     = 0;
    uint32_t launchTime     = 0;
    uint32_t coastTime = 0;

    // ── main playback loop ──────────────────────────────────────────────
    while (provider.hasNextDataPoint())
    {
        hasData = true;
        SensorData d = provider.getNextDataPoint();

        uint32_t noise_time = d.time + static_cast<uint32_t>(timeNoise(rng));

        DataPoint ax(noise_time, d.accelx);
        DataPoint ay(noise_time, d.accely);
        DataPoint az(noise_time, d.accelz);
        DataPoint alt(noise_time, d.altitude);

        sm.update(ax, ay, az, alt);


      
        if (sm.getState() == STATE_COAST_ASCENT){
            apogeePredictor.update();    
            quadApogeePredictor.quad_update();  
        }

        float predApogee       = apogeePredictor.getPredictedApogeeAltitude_m();
        float quadPredApogee  = quadApogeePredictor.getPredictedApogeeAltitude_m();
        float timeToApogee     = apogeePredictor.getTimeToApogee_s();
        float estAlt           = vve.getEstimatedAltitude();
        float estVel           = vve.getEstimatedVelocity();

        // ── log one CSV row ────────────────────────────────────────────
        log << d.time << ',' << d.accelx << ',' << d.accely << ','
            << d.accelz << ',' << d.altitude << ',' << static_cast<int>(sm.getState()) << ','
            << estAlt << ',' << estVel << ',' << predApogee << ',' << quadPredApogee << ','
            << timeToApogee << '\n';

        // ── book‑keeping for post‑test assertions ─────────────────────
        if (d.altitude > maxAltitude)
        {
            maxAltitude  = d.altitude;
            maxAltTime   = d.time;
        }
        if (lp.isLaunched() && launchTime == 0)
            launchTime = lp.getLaunchedTime();
    }

    log.close();                                   // flush + close the file

    // ── UNITY assertions (unchanged) ────────────────────────────────────
    TEST_ASSERT_TRUE(hasData);
    TEST_ASSERT_TRUE(lp.isLaunched());
    TEST_ASSERT_TRUE(ad.isApogeeDetected());
    TEST_ASSERT_GREATER_THAN(STATE_ARMED,         sm.getState());
    TEST_ASSERT_GREATER_THAN(STATE_POWERED_ASCENT, sm.getState());
    TEST_ASSERT_GREATER_THAN(STATE_COAST_ASCENT,   sm.getState());

    // TEST_ASSERT_LESS_THAN(
    //     1000,
    //     std::abs(static_cast<int64_t>(maxAltTime) -
    //              static_cast<int64_t>(ad.getApogee().timestamp_ms)));

    TEST_ASSERT_LESS_THAN(
        10, std::abs(maxAltitude - ad.getApogee().data));

    TEST_ASSERT_TRUE(launchTime < ad.getApogee().timestamp_ms);
    TEST_ASSERT_TRUE(launchTime < maxAltTime);
}
