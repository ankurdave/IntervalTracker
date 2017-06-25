/**
 * Publishes device GPS and cellular location to Particle Cloud at regular intervals (default 15
 * minutes), sleeping in between to minimize power usage.
 *
 * Written for the Particle Asset Tracker 3G v2 (Particle Electron, u-blox SARA-U260 cellular modem,
 * u-blox MAX-M8Q GPS receiver).
 *
 * References:
 * - [UBXProtocol] "u-blox 8 / u-blox M8 Receiver Description Including Protocol Specification".
 *   UBX-13003221.
 */

#include "CellularHelper.h"
#include "TinyGPS++.h"

// CONFIGURATION ===================================================================================

/** Interval between location reports (s). */
const long min_publish_interval_sec = 15 * 60;
/** Interval between debug messages (ms). */
const unsigned long min_print_interval_ms = 1000;
/** Timeout for acquiring a GPS fix (ms). */
const unsigned long max_gps_time_ms = 5 * 60 * 1000;
/** Timeout for connecting to the cellular network and Particle cloud (ms). */
const unsigned long max_connect_time_ms = 3 * 60 * 1000;

/**
 * How long after connecting to the cellular network to wait before querying the modem for location
 * and signal strength (ms).
 */
const unsigned long post_connect_delay_ms = 15 * 1000;
/** How long to wait for published events to be sent before turning off the cellular modem. */
const unsigned long pre_sleep_delay_ms = 5 * 1000;

// INTERNALS =======================================================================================

// -- SETUP ----------------------------------------------------------------------------------------

// Threading is required to be able to time out connecting to the cellular network
SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(MANUAL);

ApplicationWatchdog wd(60000, System.reset);
TinyGPSPlus gps;
FuelGauge fuel;

enum State { IDLE, WAIT_FOR_GPS, WAIT_FOR_CONNECT, PUBLISH, SLEEP };
State state = IDLE;

// Rate limiting and timeout variables
long last_send_unixtime = 0;
unsigned long last_print_ms = 0;
unsigned long gps_begin_ms = 0;
unsigned long connect_begin_ms = 0;

// Times spent in waiting states, used for statistics reporting
unsigned long gps_ttff_ms = 0;
unsigned long connect_time_ms = 0;

// Temporary string buffer for snprintf
char buf[64];

// -- GPS management -------------------------------------------------------------------------------

// Backup of navigation database for persistence across GPS power cycles
unsigned char gps_nav_database[8192];
uint32_t gps_nav_database_length = 0;

// UBX commands. See protocol description in [UBXProtocol].
// Set Serial1 input and output to UBX protocol
static const uint8_t ubx_cfg_prt_ubx[] = {
    0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xC0,0x08,0x00,0x00,0x80,0x25,0x00,0x00,0x01,
    0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x8A,0x79};
// Set Serial1 input to UBX protocol and output to NMEA protocol
static const uint8_t ubx_cfg_prt_nmea[] = {
    0xB5,0x62,0x06,0x00,0x14,0x00,0x01,0x00,0x00,0x00,0xC0,0x08,0x00,0x00,0x80,0x25,0x00,0x00,0x01,
    0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x8B,0x7F};
// Request navigation database
static const uint8_t ubx_mga_dbd[] = {
    0xB5,0x62,0x13,0x80,0x00,0x00,0x93,0xCC};

void gpsOn() {
    Serial1.begin(9600);
    pinMode(D6, OUTPUT);

    // Clear Serial1 buffer
    while (Serial1.available() > 0) {
		Serial1.read();
	}

    // Turn on GPS
    digitalWrite(D6, LOW);

    // Allow GPS to initialize before sending commands
    delay(500);
    restore_nav_database();
}

/** Parse NMEA output from GPS for fixes. */
void updateGPS() {
    while (Serial1.available() > 0) {
		gps.encode(Serial1.read());
	}
}

void gpsOff() {
    save_nav_database();

    digitalWrite(D6, HIGH);
}

/**
 * Save GPS navigation database before powering it off. See Section 11.5 of [UBXProtocol],
 * "Preserving Information During Power-Off".
 */
void save_nav_database() {
    snprintf(buf, sizeof(buf), "[%lu] Getting nav database\n", millis());
    Serial.write(buf);

    gps_nav_database_length = 0;

    // Switch to receiving UBX messages
    Serial1.write(ubx_cfg_prt_ubx, sizeof(ubx_cfg_prt_ubx));

    // Flush leftover NMEA data and ACK from the buffer: Drop data until there is 1 second of idle
    // time
    unsigned long last_received = millis();
    while (millis() < last_received + 1000) {
        while (Serial1.available() > 0) {
            last_received = millis();
            Serial1.read();
        }
    }

    // Request database
    Serial1.write(ubx_mga_dbd, sizeof(ubx_mga_dbd));

    // Save database: Read data until there is 1 second of idle time
    last_received = millis();
    while (millis() < last_received + 1000) {
        while (Serial1.available() > 0) {
            last_received = millis();
            unsigned char c = Serial1.read();
            if (gps_nav_database_length < sizeof(gps_nav_database)) {
                gps_nav_database[gps_nav_database_length++] = c;
            }
            // Serial.printf("0x%0.2X ", c);
            // if (gps_nav_database_length % 10 == 0) Serial.print("\n");
        }
    }

    // Switch back to receiving NMEA
    Serial1.write(ubx_cfg_prt_nmea, sizeof(ubx_cfg_prt_nmea));

    snprintf(buf, sizeof(buf), "[%lu] Got %d bytes of database\n",
             millis(), gps_nav_database_length);
    Serial.write(buf);
}

/**
 * Restore GPS navigation database on start to improve cold start performance. See Section 11.5 of
 * [UBXProtocol], "Preserving Information During Power-Off".
 */
void restore_nav_database() {
    if (gps_nav_database_length != 0) {
        snprintf(buf, sizeof(buf), "[%lu] Sending stored nav database to GPS\n", millis());
        Serial.write(buf);

        // Send database
        Serial1.write(gps_nav_database, gps_nav_database_length);

        snprintf(buf, sizeof(buf), "[%lu] Sent %d bytes of database\n", millis(), gps_nav_database_length);
        Serial.write(buf);
    }
}

// -- Utilities ------------------------------------------------------------------------------------

/** Print a debug message about the current state. */
void printStatus() {
    if (last_print_ms == 0 || millis() > last_print_ms + min_print_interval_ms) {
        last_print_ms = millis();

        switch (state) {
        case IDLE:
            snprintf(buf, sizeof(buf),
                "[%lu] IDLE, time until publish %d\n",
                millis(), last_send_unixtime + min_publish_interval_sec - Time.now());
            Serial.write(buf);
            break;
        case WAIT_FOR_GPS:
            snprintf(buf, sizeof(buf),
                "[%lu] WAIT_FOR_GPS, gps valid %d, gps updated %d, %d ms left\n",
                millis(),
                gps.location.isValid(),
                gps.location.isUpdated(),
                gps_begin_ms + max_gps_time_ms - millis());
            break;
        case WAIT_FOR_CONNECT:
            snprintf(buf, sizeof(buf),
                "[%lu] WAIT_FOR_CONNECT, cell {lis %d, conn %d, ready %d}, conn %d, %d ms left\n",
                millis(),
                Cellular.listening(),
                Cellular.connecting(),
                Cellular.ready(),
                Particle.connected(),
                connect_begin_ms + max_connect_time_ms - millis());
            break;
        case PUBLISH:
            snprintf(buf, sizeof(buf), "[%lu] PUBLISH\n");
            break;
        case SLEEP:
            snprintf(buf, sizeof(buf), "[%lu] SLEEP\n");
            break;
        }
        Serial.write(buf);
    }
}

/** Delay for the specified time while maintaining regular services. */
void appDelay(unsigned long delay_ms) {
    unsigned long wait_begin = millis();
    while (millis() < wait_begin + delay_ms) {
        wd.checkin();
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
    updateGPS();
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
            gpsOn();

            state = WAIT_FOR_GPS;
        }
        break;

    case WAIT_FOR_GPS:
        if ((gps.location.isValid() && gps.location.isUpdated())
            || millis() > gps_begin_ms + max_gps_time_ms) {

            gps_ttff_ms = millis() - gps_begin_ms;

            if (gps.location.isValid() && gps.location.isUpdated()) {
                snprintf(buf, sizeof(buf), "[%lu] Got a GPS fix. TTFF: %d ms.\n", millis(), gps_ttff_ms);
                Serial.write(buf);
            } else {
                snprintf(buf, sizeof(buf), "[%lu] Failed to get GPS fix.\n", millis());
                Serial.write(buf);
            }

            gpsOff();

            snprintf(buf, sizeof(buf), "[%lu] Connecting...\n", millis());
            Serial.write(buf);
            connect_begin_ms = millis();
            Cellular.on();
            Particle.connect();
            state = WAIT_FOR_CONNECT;
        }
        break;

    case WAIT_FOR_CONNECT:
        if (Particle.connected()) {
            connect_time_ms = millis() - connect_begin_ms;
            snprintf(buf, sizeof(buf), "[%lu] Connected in %d ms.\n", millis(), connect_time_ms);
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
            if (gps.location.isValid() && gps.location.isUpdated()) {
                snprintf(buf, sizeof(buf), "[%lu] Sending fix: %f,%f.\n",
                              millis(), gps.location.lat(), gps.location.lng());
                Serial.write(buf);
                snprintf(buf, sizeof(buf), "%f,%f",
                         gps.location.lat(), gps.location.lng());
                Particle.publish("g", buf, PRIVATE, NO_ACK);
            }

            snprintf(buf, sizeof(buf), "[%lu] Waiting %d ms before querying cellular modem.\n",
                          millis(), post_connect_delay_ms);
            Serial.write(buf);
            appDelay(post_connect_delay_ms);

            CellularHelperLocationResponse cell_loc = CellularHelper.getLocation();
            if (cell_loc.valid) {
                snprintf(buf, sizeof(buf), "[%lu] Cellular location %f,%f~%d\n",
                              millis(), cell_loc.lat, cell_loc.lon, cell_loc.uncertainty);
                Serial.write(buf);
                snprintf(buf, sizeof(buf), "%f,%f~%d",
                         cell_loc.lat, cell_loc.lon, cell_loc.uncertainty);
                Particle.publish("cl", buf, PRIVATE, NO_ACK);
            }

            CellularHelperRSSIQualResponse rssiQual = CellularHelper.getRSSIQual();
            snprintf(buf, sizeof(buf),
                "[%lu] SoC %.1f, TTFF %lu ms, connect time %lu ms, GPS DB size %u, "
                "RSSI %d, qual %d.\n",
                millis(), fuel.getSoC(), gps_ttff_ms, connect_time_ms, gps_nav_database_length,
                rssiQual.rssi, rssiQual.qual);
            Serial.write(buf);
            snprintf(buf, sizeof(buf), "soc%.1f,ttff%lu,tcon%lu,dbsz%u,rssi%d",
                     fuel.getSoC(), gps_ttff_ms, connect_time_ms, gps_nav_database_length,
                     rssiQual.rssi);
            Particle.publish("s", buf, PRIVATE, NO_ACK);

            CellularHelperEnvironmentResponseStatic<8> envResp;
            CellularHelper.getEnvironment(CellularHelper.ENVIRONMENT_SERVING_CELL, envResp);
            envResp.logResponse();
        }
        state = SLEEP;
        break;

    case SLEEP:
        snprintf(buf, sizeof(buf), "[%lu] Waiting %d ms for messages to go out.\n",
                      millis(), pre_sleep_delay_ms);
        Serial.write(buf);
        appDelay(pre_sleep_delay_ms);

        snprintf(buf, sizeof(buf), "[%lu] Turning cellular off.\n", millis());
        Serial.write(buf);
        Cellular.off();

        long sleep_time_sec = (last_send_unixtime == 0)
            ? min_publish_interval_sec
            : (last_send_unixtime + min_publish_interval_sec - Time.now());
        if (sleep_time_sec < 0) {
            sleep_time_sec = 10;
        }
        if (sleep_time_sec > min_publish_interval_sec) {
            sleep_time_sec = min_publish_interval_sec;
        }
        snprintf(buf, sizeof(buf), "[%lu] Pausing microcontroller for %d seconds.\n", millis(), sleep_time_sec);
        Serial.write(buf);
        state = IDLE;
        delay(5000); // Wait for the last few serial messages to send and the cellular modem to turn
                     // off
        // A5 is a dummy pin that should not trigger a wakeup
        pinMode(A5, INPUT);
        // We only pause the controller instead of using SLEEP_MODE_DEEP because the latter causes
        // intermittent hanging on wake:
        // https://community.particle.io/t/gps-causes-hanging-on-wake-from-sleep-mode-deep/33946
        System.sleep(A5, RISING, SLEEP_NETWORK_STANDBY, sleep_time_sec);
        state = IDLE;
        break;
    }
}
