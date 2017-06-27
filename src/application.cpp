/**
 * Publishes device GPS and cellular location to Particle Cloud at regular intervals (default 15
 * minutes), sleeping in between to minimize power usage.
 *
 * Written for the Particle Asset Tracker 3G v2 (Particle Electron, u-blox SARA-U260 cellular modem,
 * u-blox MAX-M8Q GPS receiver).
 */

#include <stdint.h>

#include "CellularHelper.h"
#include "Particle.h"
#include "ubx.h"

void handleGPSMessage(uint16_t msg_class_id, const ubx_buf_t &buf);

// CONFIGURATION ===================================================================================

/** Interval between location reports (s). */
const int32_t min_publish_interval_sec = 15 * 60;
/** Interval between debug messages (ms). */
const uint32_t min_print_interval_ms = 1000;
/** Timeout for acquiring a GPS fix (ms). */
const uint32_t max_gps_time_ms = 5 * 60 * 1000;
/** Timeout for connecting to the cellular network and Particle cloud (ms). */
const uint32_t max_connect_time_ms = 3 * 60 * 1000;

/**
 * How long after connecting to the cellular network to wait before querying the modem for location
 * and signal strength (ms).
 */
const uint32_t post_connect_delay_ms = 15 * 1000;
/** How long to wait for published events to be sent before turning off the cellular modem. */
const uint32_t pre_sleep_delay_ms = 5 * 1000;

// Use the AT&T IoT network
STARTUP(cellular_credentials_set("m2m.com.attz", "", "", NULL));

// INTERNALS =======================================================================================

// -- SETUP ----------------------------------------------------------------------------------------

// Threading is required to be able to time out connecting to the cellular network
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(MANUAL);

ApplicationWatchdog wd(60000, System.reset);
UBX gps(handleGPSMessage);
FuelGauge fuel;

enum State { IDLE, WAIT_FOR_GPS, WAIT_FOR_CONNECT, PUBLISH, SLEEP };
State state = IDLE;

// Rate limiting and timeout variables
int32_t last_send_unixtime = 0;
uint32_t last_print_ms = 0;
uint32_t gps_begin_ms = 0;
uint32_t last_fix_time = 0;
uint32_t connect_begin_ms = 0;

// Times spent in waiting states, used for statistics reporting
uint32_t connect_time_ms = 0;

// Temporary string buffer for snprintf
char buf[128];

// GPS info
bool fix_valid = false;
double lat = 0.0, lon = 0.0, acc = 0.0, speed_mph = 0.0;
uint8_t num_satellites = 0;

uint32_t ttff = 0;

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
        break;
    }
}

/** Print a debug message about the current state. */
void printStatus() {
    if (last_print_ms == 0 || millis() > last_print_ms + min_print_interval_ms) {
        last_print_ms = millis();

        switch (state) {
        case IDLE:
            snprintf(buf, sizeof(buf),
                     "[%lu] IDLE, time until publish %ld\n",
                     millis(), last_send_unixtime + min_publish_interval_sec - Time.now());
            Serial.write(buf);
            break;
        case WAIT_FOR_GPS:
            snprintf(buf, sizeof(buf),
                     "[%lu] WAIT_FOR_GPS, %f,%f~%f, %.1f mph, %u satellites, TTFF %lu ms\n",
                     millis(),
                     lat, lon, acc, speed_mph, num_satellites, ttff);
            break;
        case WAIT_FOR_CONNECT:
            snprintf(buf, sizeof(buf),
                     "[%lu] WAIT_FOR_CONNECT, cell {lis %d, conn %d, ready %d}, conn %d, %lu ms left\n",
                     millis(),
                     Cellular.listening(),
                     Cellular.connecting(),
                     Cellular.ready(),
                     Particle.connected(),
                     connect_begin_ms + max_connect_time_ms - millis());
            break;
        case PUBLISH:
            snprintf(buf, sizeof(buf), "[%lu] PUBLISH\n", millis());
            break;
        case SLEEP:
            snprintf(buf, sizeof(buf), "[%lu] SLEEP\n", millis());
            break;
        }
        Serial.write(buf);
    }
}

/** Delay for the specified time while maintaining regular services. */
void appDelay(uint32_t delay_ms) {
    uint32_t wait_begin = millis();
    while (millis() < wait_begin + delay_ms) {
        wd.checkin();
        gps.update();
        Particle.process();
        printStatus();
    }
}

// -- Main loop ------------------------------------------------------------------------------------

void setup() {
    // Initialize FuelGauge for battery readings
    fuel.wakeup();
    fuel.quickStart();
}

void loop() {
    wd.checkin();
    gps.update();
    Particle.process();
    printStatus();

    switch (state) {
    case IDLE:
        if (last_send_unixtime == 0 || Time.now() > last_send_unixtime + min_publish_interval_sec) {
            last_send_unixtime = Time.now();

            snprintf(buf, sizeof(buf), "[%lu] Time to publish.\n", millis());
            Serial.write(buf);

            snprintf(buf, sizeof(buf), "[%lu] Starting GPS...\n", millis());
            Serial.write(buf);
            gps_begin_ms = millis();
            resetGPSInfo();
            gps.start();

            state = WAIT_FOR_GPS;
        }
        break;

    case WAIT_FOR_GPS:
        if (fix_valid || millis() > gps_begin_ms + max_gps_time_ms) {

            ttff = millis() - gps_begin_ms;

            if (fix_valid) {
                snprintf(buf, sizeof(buf), "[%lu] Got GPS fix in %lu ms.\n", millis(), ttff);
                Serial.write(buf);
            } else {
                snprintf(buf, sizeof(buf), "[%lu] Failed to get GPS fix.\n", millis());
                Serial.write(buf);
            }

            snprintf(buf, sizeof(buf), "[%lu] Connecting...\n", millis());
            Serial.write(buf);
            connect_begin_ms = millis();
            Cellular.on();
            Particle.connect();
            Particle.keepAlive(30); // Higher frequency keepalives for AT&T IoT network
            state = WAIT_FOR_CONNECT;
        }
        break;

    case WAIT_FOR_CONNECT:
        if (Particle.connected()) {
            connect_time_ms = millis() - connect_begin_ms;
            snprintf(buf, sizeof(buf), "[%lu] Connected in %lu ms.\n", millis(), connect_time_ms);
            Serial.write(buf);

            state = PUBLISH;
        } else if (millis() > connect_begin_ms + max_connect_time_ms) {
            connect_time_ms = millis() - connect_begin_ms;
            snprintf(buf, sizeof(buf), "[%lu] Failed to connect.\n", millis());
            Serial.write(buf);

            state = SLEEP;
        }
        break;

    case PUBLISH:
        if (Particle.connected()) {
            snprintf(buf, sizeof(buf), "[%lu] Waiting %lu ms before querying cellular modem.\n",
                     millis(), post_connect_delay_ms);
            Serial.write(buf);
            appDelay(post_connect_delay_ms);

            CellularHelperLocationResponse cell_loc = CellularHelper.getLocation();
            if (cell_loc.valid) {
                snprintf(buf, sizeof(buf), "[%lu] Cellular location %f,%f~%d\n",
                         millis(), cell_loc.lat, cell_loc.lon, cell_loc.uncertainty);
                Serial.write(buf);
                snprintf(buf, sizeof(buf), "%f,%f,%d",
                         cell_loc.lat, cell_loc.lon, cell_loc.uncertainty);
                Particle.publish("cl", buf, PRIVATE, NO_ACK);
            }

            CellularHelperRSSIQualResponse rssiQual = CellularHelper.getRSSIQual();
            snprintf(buf, sizeof(buf),
                     "[%lu] SoC %.1f, TTFF %lu ms, connect time %lu ms, "
                     "RSSI %d, qual %d.\n",
                     millis(), fuel.getSoC(), ttff, connect_time_ms,
                     rssiQual.rssi, rssiQual.qual);
            Serial.write(buf);
            snprintf(buf, sizeof(buf), "%.1f,%.1f,%.0f,%d",
                     fuel.getSoC(),
                     ttff / 1000.0,
                     connect_time_ms / 1000.0,
                     rssiQual.rssi);
            Particle.publish("s", buf, PRIVATE, NO_ACK);

            gps.assist(cell_loc);

            snprintf(buf, sizeof(buf),
                     "[%lu] Sending fix: %f,%f~%f, %.1f mph, %u satellites.\n",
                     millis(), lat, lon, acc, speed_mph, num_satellites);
            Serial.write(buf);
            snprintf(buf, sizeof(buf), "%f,%f,%.0f,%.0f,%u",
                     lat, lon, acc, speed_mph, num_satellites);
            Particle.publish("g", buf, PRIVATE, NO_ACK);
        }
        state = SLEEP;
        break;

    case SLEEP:
        snprintf(buf, sizeof(buf), "[%lu] Waiting %lu ms for messages to go out.\n",
                 millis(), pre_sleep_delay_ms);
        Serial.write(buf);
        appDelay(pre_sleep_delay_ms);

        snprintf(buf, sizeof(buf), "[%lu] Turning cellular off.\n", millis());
        Serial.write(buf);
        Cellular.off();

        snprintf(buf, sizeof(buf), "[%lu] Stopping GPS.\n", millis());
        Serial.write(buf);
        gps.stop();

        int32_t sleep_time_sec = (last_send_unixtime == 0)
            ? min_publish_interval_sec
            : (last_send_unixtime + min_publish_interval_sec - Time.now());
        if (sleep_time_sec < 0) {
            sleep_time_sec = 10;
        }
        if (sleep_time_sec > min_publish_interval_sec) {
            sleep_time_sec = min_publish_interval_sec;
        }
        snprintf(buf, sizeof(buf), "[%lu] Pausing microcontroller for %ld seconds.\n",
                 millis(), sleep_time_sec);
        Serial.write(buf);
        state = IDLE;
        delay(5000); // Wait for the last few serial messages to send and the cellular modem to turn
                     // off

        fuel.sleep();
        // We only pause the controller instead of using SLEEP_MODE_DEEP because the latter causes
        // intermittent hanging on wake:
        // https://community.particle.io/t/gps-causes-hanging-on-wake-from-sleep-mode-deep/33946
        // A5 is a dummy pin that should not change state
        System.sleep(A5, RISING, SLEEP_NETWORK_STANDBY, sleep_time_sec);
        fuel.wakeup();
        state = IDLE;
        break;
    }
}
