/**
 * Publishes device GPS and cellular location to Particle Cloud at regular intervals (default 6
 * hours), sleeping in between to minimize power usage. When connected to input power, publishes
 * continuously, default every 10 seconds.
 *
 * To wake the device as soon as USB power is supplied, connect VIN to WKP.
 *
 * Written for the Particle Asset Tracker 3G v2 (Particle Electron, u-blox SARA-U260 cellular modem,
 * u-blox MAX-M8Q GPS receiver).
 */

#include <stdint.h>

#include "CellularHelper.h"
#include "Particle.h"
#include "ubx.h"
#include "PowerCheck.h"

// Threading is required to be able to time out connecting to the cellular network
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(MANUAL);

// CONFIGURATION ===================================================================================

/** Interval between location reports when car is not running (s). */
const int32_t min_publish_interval_sec = 24 * 60 * 60;
/** Interval between debug messages (ms). */
const uint32_t min_print_interval_ms = 1000;
/** Timeout for connecting to the cellular network and Particle cloud (ms). */
const uint32_t max_connect_time_ms = 3 * 60 * 1000;
/**
 * Timeout for acquiring a GPS fix (ms). Must be greater than max_connect_time if assistance is to
 * have immediate benefit.
 */
const uint32_t max_gps_time_ms = 5 * 60 * 1000;
/** Interval between location reports while car is running (ms). */
const uint32_t continuous_publish_interval_ms = 10 * 1000;
/**
 * Padding time before turning cellular and microprocessor off (ms). This allows published events to
 * be sent and the cellular modem to be stopped properly. Without it, the system sometimes wakes up
 * quickly after being put to sleep.
 */
const uint32_t pad_delay_ms = 5000;

// INTERNALS =======================================================================================

// -- SETUP ----------------------------------------------------------------------------------------

void handleGPSMessage(uint16_t msg_class_id, const ubx_buf_t &buf);

ApplicationWatchdog wd(60000, System.reset);
UBX gps(handleGPSMessage);
FuelGauge fuel;
PowerCheck powerCheck;

// Rate limiting and timeout variables
int32_t last_send_unixtime = 0;
uint32_t last_print_ms = 0;
uint32_t gps_begin_ms = 0;
uint32_t connect_begin_ms = 0;

// Times spent in waiting states, used for statistics reporting
uint32_t connect_time_ms = 0;
uint32_t ttff = 0;

// Temporary string buffer for snprintf
char buf[128];

// GPS info
bool fix_valid = false;
double lat = 0.0, lon = 0.0, acc = 0.0, speed_mph = 0.0;
uint8_t num_satellites = 0;
const double mm_per_second_per_mph = 447.04;

// -- Utilities ------------------------------------------------------------------------------------

void resetGPSInfo() {
    fix_valid = false;
    lat = lon = acc = speed_mph = 0.0;
    num_satellites = 0;
    ttff = 0;
}

void handleGPSMessage(uint16_t msg_class_id, const ubx_buf_t &buf) {
    switch (msg_class_id) {
    case UBX_MSG_NAV_PVT:
		fix_valid = ((buf.payload_rx_nav_pvt.flags & UBX_RX_NAV_PVT_FLAGS_GNSSFIXOK) == 1);
		lat = (double)buf.payload_rx_nav_pvt.lat * 1e-7;
		lon	= (double)buf.payload_rx_nav_pvt.lon * 1e-7;
        acc = (double)buf.payload_rx_nav_pvt.hAcc * 1e-3;
        num_satellites = buf.payload_rx_nav_pvt.numSV;
        speed_mph = (double)buf.payload_rx_nav_pvt.gSpeed / mm_per_second_per_mph;

        if (ttff == 0 && fix_valid) {
            ttff = millis() - gps_begin_ms;
        }
        break;
    }
}

/** Print a rate-limited debug message. */
#define PRINT_STATUS(s, ...) { \
    if (last_print_ms == 0 || millis() > last_print_ms + min_print_interval_ms) { \
        last_print_ms = millis(); \
        snprintf(buf, sizeof(buf), s, ## __VA_ARGS__); \
        Serial.write(buf); \
    }}

#define APP_TRACE(s, ...) { \
    snprintf(buf, sizeof(buf), s, ## __VA_ARGS__);  \
    Serial.write(buf);}

#define APP_WAIT_UNTIL(cond, s, ...) { \
    while (!(cond)) { \
        checkin(); \
        PRINT_STATUS(s, ## __VA_ARGS__); \
        delay(1); \
    }}

void checkin() {
    wd.checkin();
    gps.update();
    Particle.process();
}

void app_delay(uint32_t delay_ms) {
    uint32_t delay_start_ms = millis();
    while (millis() < delay_start_ms + delay_ms) {
        checkin();
        delay(1);
    }
}

// -- Main loop ------------------------------------------------------------------------------------

void setup() {
    Serial.begin(9600);

    // Initialize FuelGauge for battery readings
    fuel.wakeup();
    fuel.quickStart();

    powerCheck.setup();
}

void loop() {
    APP_WAIT_UNTIL(
        last_send_unixtime == 0 || Time.now() > last_send_unixtime + min_publish_interval_sec,
        "[%lu] %ld s until publish\n",
        millis(), last_send_unixtime + min_publish_interval_sec - Time.now());

    last_send_unixtime = Time.now();

    APP_TRACE("[%lu] Starting FuelGauge.\n", millis());
    fuel.wakeup();

    APP_TRACE("[%lu] Starting GPS.\n", millis());
    gps_begin_ms = millis();
    resetGPSInfo();
    gps.start();

    APP_TRACE("[%lu] Connecting.\n", millis());
    connect_begin_ms = millis();
    Cellular.on();
    Particle.connect();

    APP_WAIT_UNTIL(
        Particle.connected() || (
            !powerCheck.getHasPower() && millis() > connect_begin_ms + max_connect_time_ms),
        "[%lu] Connecting, %lu s left\n",
        millis(), (connect_begin_ms + max_connect_time_ms - millis()) / 1000);

    if (Particle.connected()) {
        connect_time_ms = millis() - connect_begin_ms;
        APP_TRACE("[%lu] Connected in %lu ms.\n", millis(), connect_time_ms);

        if (!fix_valid) {
            // Commented out because assistance is too expensive on the Particle network
            // gps.assist();
        }

        APP_WAIT_UNTIL(
            fix_valid || millis() > gps_begin_ms + max_gps_time_ms,
            "[%lu] Waiting for fix, %lu s left\n",
            millis(), (gps_begin_ms + max_gps_time_ms - millis()) / 1000);

        if (!fix_valid) {
            // Fall back to cellular location
            APP_TRACE("[%lu] Failed to get GPS fix. Requesting CellLocate.\n", millis());

            checkin();
            CellularHelperLocationResponse cell_loc = CellularHelper.getLocation();
            checkin();
            if (cell_loc.valid) {
                if (Particle.connected()) {
                    APP_TRACE("[%lu] Publishing cellular location %f,%f~%d\n",
                              millis(), cell_loc.lat, cell_loc.lon, cell_loc.uncertainty);
                    snprintf(buf, sizeof(buf), "%f,%f,%d",
                             cell_loc.lat, cell_loc.lon, cell_loc.uncertainty);
                    Particle.publish("cl", buf, PRIVATE);
                }
            } else {
                APP_TRACE("[%lu] Failed to get cellular location.\n", millis());
            }
        }

        // Publish GPS location even if it does not meet acceptability criteria
        if (ttff == 0) {
            ttff = millis() - gps_begin_ms;
        }
        if (Particle.connected()) {
            APP_TRACE("[%lu] Publishing GPS location: %f,%f~%f, %.1f mph, %u satellites.\n",
                      millis(), lat, lon, acc, speed_mph, num_satellites);
            snprintf(buf, sizeof(buf), "%f,%f,%.0f,%.0f,%u",
                     lat, lon, acc, speed_mph, num_satellites);
            Particle.publish("g", buf, PRIVATE);
        }

        // Report statistics
        if (Particle.connected()) {
            APP_TRACE("[%lu] Publishing voltage %.1f, TTFF %lu ms, connect time %lu ms\n",
                      millis(), fuel.getVCell(), ttff, connect_time_ms);
            snprintf(buf, sizeof(buf), "%.1f,%.1f,%.0f",
                     fuel.getVCell(),
                     ttff / 1000.0,
                     connect_time_ms / 1000.0);
            Particle.publish("s", buf, PRIVATE);
        }

        // Publish continuously while car is running
        while (powerCheck.getHasPower()) {
            if (Particle.connected()) {
                APP_TRACE("[%lu] Publishing GPS location: %f,%f~%f, %.1f mph, %u satellites.\n",
                          millis(), lat, lon, acc, speed_mph, num_satellites);
                snprintf(buf, sizeof(buf), "%f,%f,%.0f,%.0f,%u",
                         lat, lon, acc, speed_mph, num_satellites);
                Particle.publish("g", buf, PRIVATE, NO_ACK);
            } else {
                APP_TRACE("[%lu] Not connected; skipping location publish: "
                          "%f,%f~%f, %.1f mph, %u satellites.\n",
                          millis(), lat, lon, acc, speed_mph, num_satellites);
            }

            app_delay(continuous_publish_interval_ms);
        }

        APP_TRACE("[%lu] Waiting %lu ms for messages to go out.\n",
                  millis(), pad_delay_ms);
        app_delay(pad_delay_ms);
    } else {
        APP_TRACE("[%lu] Failed to connect.\n", millis());
    }

    APP_TRACE("[%lu] Sleeping FuelGauge.\n", millis());
    fuel.sleep();

    APP_TRACE("[%lu] Turning cellular off.\n", millis());
    Cellular.off();

    APP_TRACE("[%lu] Stopping GPS.\n", millis());
    gps.stop();

    int32_t sleep_time_sec =
        (last_send_unixtime == 0)
        ? min_publish_interval_sec
        : (last_send_unixtime + min_publish_interval_sec - Time.now());
    if (sleep_time_sec > min_publish_interval_sec) {
        sleep_time_sec = min_publish_interval_sec;
    }
    if (sleep_time_sec > 0) {
        APP_TRACE("[%lu] Sleeping for %ld seconds.\n",
                  millis(), sleep_time_sec);
        app_delay(pad_delay_ms);

        System.sleep(SLEEP_MODE_DEEP, sleep_time_sec);
    }
}
