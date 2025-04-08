#include "unity.h"

#include "AirResistanceSimulation.h"
#include "state_estimation/VerticalVelocityEstimator.h"
#include "state_estimation/ApogeePredictor.h"
#include "data_handling/DataPoint.h"
#include "data_handling/CircularArray.h"

#include <fstream>
#include <random>
#include <iostream>
#include <string>

/* ---------- helpers ---------- */
static std::default_random_engine rng{42};
static std::normal_distribution<float> aclNoise(0.0f, 0.55f);   // m/s²  (IMU noise)
static std::normal_distribution<float> altNoise(0.0f, 3.0f);    // m     (baro noise)

/* ---------- Unity fixtures ---------- */
void setUp   (void) {}
void tearDown(void) {}

/* ---------- the actual test ---------- */
void test_apogee_predictor_generates_csv(void)
{
    /* --------------- create simulator --------------- */
    constexpr uint32_t TICK_MS     = 10;
    AirResistanceSimulator sim(/*launch*/       2000,
                               /*motor accel*/  55.0f,
                               /*burn*/         1500,
                               /*tick*/         TICK_MS,
                               /*drag k*/       0.0008f);

    /* --------------- estimation chain --------------- */
    VerticalVelocityEstimator vve(/*acc σ²*/1.05f, /*alt σ²*/10.0f);
    ApogeePredictor apo(vve, /*alpha*/0.2f, /*min climb vel*/1.0f);

    /* --------------- CSV setup ---------------------- */
    std::ofstream csv("apogee_prediction.csv");
    TEST_ASSERT_TRUE_MESSAGE(csv.is_open(), "Failed to open CSV file for writing");

    csv << "timestamp,true_alt,true_vertical_velocity,"
           "est_alt,est_vertical_velocity,true_acl,est_acl,cd,est_apogee\n";

    const float desired_apogee_m = 100.0f;

    float initial_drag_coefficient = sim.getDragCoefficient();

    bool is_post_burnout = false;
    bool check_apogee_prediction = false;


    CircularArray<DataPoint> predicted_apogees(255);


    uint32_t tick_count = 0;

    /* --------------- run the flight ----------------- */
    while (!sim.getHasLanded())
    {
       
        sim.tick();

        /* --- build synthetic sensor readings --- */
        const uint32_t ts   = sim.getCurrentTime();
        const float    aclZ = sim.getInertialVerticalAcl() + 9.81f + aclNoise(rng); // proper accel
        const float    alt  = sim.getAltitude() + altNoise(rng);

        DataPoint aclX(ts, 0.0f);
        DataPoint aclY(ts, 0.0f);
        DataPoint aclZdp(ts, aclZ);
        DataPoint altDp(ts,  alt);

        /* --- update estimator / predictor --- */
        vve.update(aclX, aclY, aclZdp, altDp);

        if (!is_post_burnout && sim.getLaunchTimestamp() > 0 && sim.getLaunchTimestamp() + 1500 < ts)
        {
            // After 1.5 seconds after launch, we are in post burnout
            is_post_burnout = true;
            TEST_ASSERT_TRUE(ts > 1500);
            TEST_ASSERT_FALSE(apo.isPredictionValid());
        }
        
        if (is_post_burnout){
            apo.update();
        }

        /* --- stream a row to the CSV --- */
        csv << ts << ','
            << sim.getAltitude()          << ','
            << sim.getVerticalVel()       << ','
            << vve.getEstimatedAltitude() << ','
            << vve.getEstimatedVelocity() << ','
            << sim.getInertialVerticalAcl() << ','
            << vve.getInertialVerticalAcceleration() << ','
            << sim.getDragCoefficient() << ',';

        if (apo.isPredictionValid()){
            TEST_ASSERT_TRUE(is_post_burnout);
            TEST_ASSERT_TRUE(apo.isPredictionValid());
            csv << apo.getPredictedApogeeAltitude_m();

            TEST_ASSERT_GREATER_THAN_FLOAT_MESSAGE(0.0f, apo.getPredictedApogeeAltitude_m(),
                "Predicted apogee altitude isn't greater than 0.0f");

            // Push to circular array
            tick_count++;
            predicted_apogees.push(DataPoint(ts, apo.getPredictedApogeeAltitude_m()));

            // if predicted is higher than desired, then increase CD otherwith decrease
            // if (apo.getPredictedApogeeAltitude_m() > desired_apogee_m)
            // {
            //     sim.setDragCoefficient(sim.getDragCoefficient() + 0.001f);
            // }
            // else
            // {
            //     // Can only decrease the drag coef, if greater than the initial drag coef
            //     if (sim.getDragCoefficient() > initial_drag_coefficient)
            //     {
            //         sim.setDragCoefficient(sim.getDragCoefficient() - 0.001f);
            //     }
            // }
        }

        // If we are past apogee, check that the oldest value in the predicted apogees was near the now true apogee
        if (sim.getApogeeTimestamp() < ts && sim.getApogeeTimestamp() > 0 && check_apogee_prediction == false)
        {
            check_apogee_prediction = true;
            TEST_ASSERT_TRUE(is_post_burnout);
            TEST_ASSERT_FALSE(apo.isPredictionValid()); // Not valid anymore because we are post apogee
            TEST_ASSERT_TRUE(predicted_apogees.isFull());
            TEST_ASSERT_TRUE(predicted_apogees.getMaxSize() > 0);
            DataPoint oldest = predicted_apogees.getFromHead(predicted_apogees.getMaxSize() - 1);

            std::cout << "Tick count: " << tick_count << std::endl;
            std::cout << "Apogee timestamp: " << sim.getApogeeTimestamp() << std::endl;
            std::cout << "Current timestamp: " << ts << std::endl;
            std::cout << "Predicted apogee: " << apo.getPredictedApogeeAltitude_m() << std::endl;
            std::cout << "True apogee: " << sim.getApogeeAlt() << std::endl;
            std::cout << "Tail apogee timestamp: " << oldest.timestamp_ms << std::endl;
            std::cout << "Tail apogee value: " << oldest.data << std::endl;
            // head
            std::cout << "Head apogee timestamp: " << predicted_apogees.getFromHead(0).timestamp_ms << std::endl;
            std::cout << "Head apogee value: " << predicted_apogees.getFromHead(0).data << std::endl;
            // Get the oldest value in the circular array
            
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1.0f, sim.getApogeeAlt(), oldest.data,
                "Predicted apogee was not close to the true apogee");
        }
        csv << '\n';
    }

    csv.close();
}

/* --------------- main runner ----------------------- */
int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_apogee_predictor_generates_csv);   // existing long‑form test
    return UNITY_END();
}

