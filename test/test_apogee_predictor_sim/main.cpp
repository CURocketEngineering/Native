#include "unity.h"
#include <fstream>
#include <random>
#include <iostream>
#include <string>
#include <sstream>
#include <iostream>

#include "AirResistanceSimulation.h"
#include "state_estimation/VerticalVelocityEstimator.h"
#include "state_estimation/ApogeePredictor.h"
#include "data_handling/DataPoint.h"
#include "data_handling/CircularArray.h"



/* ---------- helpers ---------- */
static std::default_random_engine rng{42};
static std::normal_distribution<float> aclNoise(0.0f, 0.55f);   // m/s²  (IMU noise)
static std::normal_distribution<float> altNoise(0.0f, 3.0f);    // m     (baro noise)

/* ---------- Unity fixtures ---------- */
void setUp   (void) {}
void tearDown(void) {}

/* ---------- the actual tests ---------- */


/*--------test apogee predictor with generated CSV ----------*/
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
    NoiseVariances noise = {1.05F, 10.0F};
    VerticalVelocityEstimator vve(noise);
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


    CircularArray<DataPoint, 10> predicted_apogees(10);


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
        AccelerationTriplet accel = {aclX, aclY, aclZdp};

        /* --- update estimator / predictor --- */
        vve.update(accel, altDp);

        if (!is_post_burnout && sim.getLaunchTimestamp() > 0 && sim.getLaunchTimestamp() + 1500 < ts)
        {
            // After 1.5 seconds after launch, we are in post burnout
            is_post_burnout = true;
            TEST_ASSERT_TRUE(ts > 1500);
            TEST_ASSERT_FALSE(apo.isPredictionValid());
        }
        
        if (is_post_burnout){
            apo.analytic_update();
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
            
            TEST_ASSERT_FLOAT_WITHIN_MESSAGE(1.000f, sim.getApogeeAlt(), oldest.data,
                "Predicted apogee was not close to the true apogee");
        }
        csv << '\n';
    }

    csv.close();
}


/* ---------- the test using real CSV ---------- */
void test_apogee_predictor_with_real_csv(void)
{
    /* --------------- select a file --------------- */
    std::ifstream file(
        //"data/MARTHA_3-8_1.3_B2_SingleID_transformed.csv"
        "data/MARTHA_IREC_2025_B2_transformed.csv"
        //"data/AA Data Collection - Second Launch Trimmed.csv"
    );
    TEST_ASSERT_TRUE_MESSAGE(file.is_open(), "Failed to open CSV file");

    /* --------------- CSV setup ---------------------- */
    std::ofstream csv("apogee_prediction.csv");
    TEST_ASSERT_TRUE_MESSAGE(csv.is_open(), "Failed to open CSV file for writing");

    csv << "timestamp,true_alt,true_vertical_velocity,"
           "est_alt,est_vertical_velocity,true_acl,est_acl,cd,est_apogee\n";

    VerticalVelocityEstimator vve;
    ApogeePredictor apo(vve, 0.2f, 0.5f);  // alpha=0.2, min climb vel=0.5m/s

    CircularArray<DataPoint, 10> predicted_apogees(10);

    std::string line;
    std::getline(file, line); // skip header

    float prev_alt = 0.0f;
    uint32_t prev_ts = 0;

    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string token;

        uint32_t ts;
        float ax, ay, az, alt;

        // Parse CSV columns
        std::getline(ss, token, ','); ts  = std::stoul(token);  // time
        std::getline(ss, token, ','); ax  = std::stof(token);   // accel x
        std::getline(ss, token, ','); ay  = std::stof(token);   // accel y
        std::getline(ss, token, ','); az  = std::stof(token);   // accel z
        for (int i = 0; i < 6; i++) std::getline(ss, token, ','); // skip gyro & mag
        std::getline(ss, token, ','); alt = std::stof(token);   // altitude

        // Compute vertical velocity from altitude
        float vertical_vel = 0.0f;
        if (prev_ts > 0)
            vertical_vel = (alt - prev_alt) / ((ts - prev_ts) * 0.001f); // m/s

        prev_alt = alt;
        prev_ts = ts;

        // Build DataPoints
        DataPoint aclX(ts, ax);
        DataPoint aclY(ts, ay);
        DataPoint aclZ(ts, az);
        DataPoint altDp(ts, alt);

        AccelerationTriplet accel = {aclX, aclY, aclZ};

        // Update estimator and predictor
        vve.update(accel, altDp);
        //apo.poly_update();
        //apo.quad_update();
        apo.analytic_update();
        //apo.update();

        // Stream row to CSV
        csv << ts << ','
            << alt << ','
            << vertical_vel << ','          // true vertical velocity
            << vve.getEstimatedAltitude() << ','
            << vve.getEstimatedVelocity() << ','
            << az << ','
            << vve.getInertialVerticalAcceleration() << ','
            << apo.getdragCoefficient() << ','
            ;

        if (apo.isPredictionValid()) {
            float est_apogee = apo.getPredictedApogeeAltitude_m();
            csv << est_apogee;
            predicted_apogees.push(DataPoint(ts, est_apogee));
        } else {
            csv << 0.0f; // placeholder if prediction not valid
        }

        csv << '\n';
    }

    file.close();
    csv.close();
}

/* --------------- test with multiple real CSVs --------------- */
void test_apogee_predictor_with_multiple_csvs(void)
{
    //files that will be tested
    const char* files[] = {
        "data/MARTHA_3-8_1.3_B2_SingleID_transformed.csv",
        "data/MARTHA_IREC_2025_B2_transformed.csv",
        "data/AA Data Collection - Second Launch Trimmed.csv"
    };
    //constant meters to feet conversion
    const float meters_to_feet = 3.28084f;

    for (const char* filename : files)
    {
        std::ifstream file(filename);
        TEST_ASSERT_TRUE_MESSAGE(file.is_open(), ("Failed to open CSV file: " + std::string(filename)).c_str());

        std::string outname = "apogee_prediction_" + std::string(filename).substr(5);
        std::ofstream csv(outname);
        TEST_ASSERT_TRUE_MESSAGE(csv.is_open(), "Failed to open CSV file for writing");

        csv << "timestamp,true_alt,true_vertical_velocity,"
               "est_alt,est_vertical_velocity,true_acl,est_acl,cd,est_apogee\n";

        VerticalVelocityEstimator vve;
        ApogeePredictor apo(vve, 0.2f, 0.5f);  // alpha=0.2, min climb vel=0.5 m/s

        float prev_alt = 0.0f;
        uint32_t prev_ts = 0;
        float true_apogee = 0.0f;
        uint32_t apogee_ts = 0;

        std::vector<std::pair<uint32_t, float>> predicted_apogees;

        std::string line;
        std::getline(file, line); // skip header

        while (std::getline(file, line))
        {
            std::stringstream ss(line);
            std::string token;

            uint32_t ts;
            float ax, ay, az, alt;

            // Parse CSV columns
            std::getline(ss, token, ','); ts  = std::stoul(token);
            std::getline(ss, token, ','); ax  = std::stof(token);
            std::getline(ss, token, ','); ay  = std::stof(token);
            std::getline(ss, token, ','); az  = std::stof(token);
            for (int i = 0; i < 6; i++) std::getline(ss, token, ','); // skip gyro & mag
            std::getline(ss, token, ','); alt = std::stof(token);

            // Track true apogee
            if (alt > true_apogee) {
                true_apogee = alt;
                apogee_ts = ts;
            }

            // Compute vertical velocity
            float vertical_vel = 0.0f;
            if (prev_ts > 0)
                vertical_vel = (alt - prev_alt) / ((ts - prev_ts) * 0.001f);

            prev_alt = alt;
            prev_ts = ts;

            // Build DataPoints
            DataPoint aclX(ts, ax);
            DataPoint aclY(ts, ay);
            DataPoint aclZ(ts, az);
            DataPoint altDp(ts, alt);
            AccelerationTriplet accel = {aclX, aclY, aclZ};

            // Update estimator and predictor
            vve.update(accel, altDp);

            /* update apogee predictor */
            //apo.poly_update();
            //apo.quad_update();
            apo.analytic_update();
            //apo.update();

            // Stream row to CSV
            csv << ts << ','
                << alt << ','
                << vertical_vel << ','
                << vve.getEstimatedAltitude() << ','
                << vve.getEstimatedVelocity() << ','
                << az << ','
                << vve.getInertialVerticalAcceleration() << ','
                << apo.getdragCoefficient() << ','; // placeholder CD

            if (apo.isPredictionValid()) {
                float est_apogee = apo.getPredictedApogeeAltitude_m();
                csv << est_apogee;
                predicted_apogees.emplace_back(ts, est_apogee);
            } else {
                csv << 0.0f;
            }
            csv << '\n';
        }

        // Find first prediction within N feet of true apogee
        int32_t ts_diff = 0;
        bool found = false;
        //tolerance for true apogee (0.01 =  1% of true apogee)
        const float tolerance_feet = true_apogee * 0.01f;
        for (const auto& dp : predicted_apogees)
        {
            float diff_feet = std::abs(dp.second - true_apogee) * meters_to_feet;
            if (diff_feet <= tolerance_feet)
            {
                ts_diff = apogee_ts - dp.first;
                found = true;
                break;
            }
        }

        if (found) {
            printf("File: %s | Predicted within %.1f ft of true apogee %.2f m, %d ms before actual apogee\n",
                   filename, tolerance_feet, true_apogee, ts_diff);
        } else {
            printf("File: %s | Predictor never got within %.1f ft of true apogee %.2f m\n",
                   filename, tolerance_feet, true_apogee);
        }

        file.close();
        csv.close();
    }
}

/* --------------- test with IREC CSV to see if it passes test --------------- */
void test_apogee_predictor_with_irec_csv_15s_early(void)
{
    const char* filename = "data/MARTHA_IREC_2025_B2_transformed.csv";

    std::ifstream file(filename);
    TEST_ASSERT_TRUE_MESSAGE(file.is_open(), "Failed to open IREC CSV file");

    std::ofstream csv("apogee_prediction_IREC.csv");
    TEST_ASSERT_TRUE_MESSAGE(csv.is_open(), "Failed to open output CSV");

    csv << "timestamp,true_alt,true_vertical_velocity,"
           "est_alt,est_vertical_velocity,true_acl,est_acl,cd,est_apogee\n";

    VerticalVelocityEstimator vve;
    ApogeePredictor apo(vve, 0.2f, 0.5f);

    float prev_alt = 0.0f;
    uint32_t prev_ts = 0;

    float true_apogee = 0.0f;
    uint32_t apogee_ts = 0;

    std::vector<std::pair<uint32_t, float>> predicted_apogees;

    std::string line;
    std::getline(file, line); // skip header

    while (std::getline(file, line))
    {
        std::stringstream ss(line);
        std::string token;

        uint32_t ts;
        float ax, ay, az, alt;

        std::getline(ss, token, ','); ts  = std::stoul(token);
        std::getline(ss, token, ','); ax  = std::stof(token);
        std::getline(ss, token, ','); ay  = std::stof(token);
        std::getline(ss, token, ','); az  = std::stof(token);
        for (int i = 0; i < 6; i++) std::getline(ss, token, ',');
        std::getline(ss, token, ','); alt = std::stof(token);

        /* track true apogee */
        if (alt > true_apogee)
        {
            true_apogee = alt;
            apogee_ts = ts;
        }

        /* compute vertical velocity */
        float vertical_vel = 0.0f;
        if (prev_ts > 0)
            vertical_vel = (alt - prev_alt) / ((ts - prev_ts) * 0.001f);

        prev_alt = alt;
        prev_ts = ts;

        /* build datapoints */
        DataPoint aclX(ts, ax);
        DataPoint aclY(ts, ay);
        DataPoint aclZ(ts, az);
        DataPoint altDp(ts, alt);

        AccelerationTriplet accel = {aclX, aclY, aclZ};

        vve.update(accel, altDp);
        apo.analytic_update();

        /* write CSV */
        csv << ts << ','
            << alt << ','
            << vertical_vel << ','
            << vve.getEstimatedAltitude() << ','
            << vve.getEstimatedVelocity() << ','
            << az << ','
            << vve.getInertialVerticalAcceleration() << ','
            << apo.getdragCoefficient() << ',';

        if (apo.isPredictionValid())
        {
            float est_apogee = apo.getPredictedApogeeAltitude_m();
            csv << est_apogee;
            predicted_apogees.emplace_back(ts, est_apogee);
        }
        else
        {
            csv << 0.0f;
        }

        csv << '\n';
    }

    file.close();
    csv.close();

    /* ---------- check if prediction was within tolerance >= 15s early ---------- */

    const float tolerance_m = true_apogee * 0.01f;  // 1% tolerance
    const uint32_t required_early_ms = 15000;

    bool passed = false;
    int32_t early_ms = 0;

    for (const auto& dp : predicted_apogees)
    {
        float error = fabs(dp.second - true_apogee);
        int32_t time_before_apogee = (int32_t)(apogee_ts - dp.first);

        if (error <= tolerance_m && time_before_apogee >= required_early_ms)
        {
            passed = true;
            early_ms = time_before_apogee;
            break;
        }
    }

    printf("True apogee: %.2f m at %u ms\n", true_apogee, apogee_ts);

    if (passed)
    {
        printf("PASS: Prediction reached within 1%% tolerance %d ms before apogee\n", early_ms);
    }
    else
    {
        printf("FAIL: Predictor did not reach tolerance 15 seconds early\n");
    }

    TEST_ASSERT_TRUE_MESSAGE(
        passed,
        "Apogee predictor did not predict within tolerance at least 15 seconds before apogee"
    );
}

/* --------------- main runner ----------------------- */
int main(void)
{
    UNITY_BEGIN();
    /* test with a generated CSV */
    //RUN_TEST(test_apogee_predictor_generates_csv);   // existing long‑form test


    /* test with a real CSV */
    //RUN_TEST(test_apogee_predictor_with_real_csv);


    /* test with multiple real CSVs and get how far in advance apogee is predicted with in 1% of true apogee */
    //RUN_TEST(test_apogee_predictor_with_multiple_csvs);
    
    /* test with IREC CSV to see if it passes test */
    RUN_TEST(test_apogee_predictor_with_irec_csv_15s_early);
    return UNITY_END();
}

