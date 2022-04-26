/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 * Copyright (c) 2018 Terry Moore, MCCI
 * Copyright (c) 2022 David Taylor, NSW Department of Primary Industries
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with a payload containing
 * a 16-bit counter, using frequency and encryption settings matching
 * those of the The Things Network. It's pre-configured for the
 * Adafruit Feather M0 LoRa.
 *
 * This example also demonstrates how to safely put the Feather into
 * standby mode for low-power operation.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * The network time callback function is copied from the
 * ttn-otaa-network-time example, modified to set the time in the
 * Feathers RTC.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in
 * arduino-lmic/project_config/lmic_project_config.h or from your BOARDS.txt.
 *
 *******************************************************************************/

// Serial1 works with standby mode.
#define serial Serial1
#define USE_SERIAL

#include <lmic.h>
#include <hal/hal.h>
#include <RTCZero.h>

RTCZero rtc;

//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// This EUI must be in little-endian format, so least-significant-byte
// first. For TTN v3 you can register a device with all zeros in the APPEUI.
static const u1_t PROGMEM APPEUI[8]= { FILLMEIN };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]= { FILLMEIN };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the TTN console can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { FILLMEIN };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

void set_delta_alarm();
void user_request_network_time_callback(void *pVoidUserUTCTime, int flagSuccess);

// This is the data sent with the uplink.
static uint16_t counter = 0;

static osjob_t sendjob;

// This flag is used to avoid sleeping before the node had joined the network.
// Not very efficient but it ensures sleeping cannot get in the way of a join.
static bool joined = false;

// This flag is used to decider whether to make a network time request when
// an uplink is set. The network time request adds 1 byte to the payload.
// Once a network time response has been received this is set to true.
// It could be reset once a day or so to try and keep clock drift in check.
static bool timeOk = false;

static bool check_for_standby = false;
static int32_t delta_osticks = -1;
static int32_t delta_seconds = -1;

static uint32_t userUTCTime; // Seconds since the UTC epoch

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 120;

// Pin mapping
//
// Adafruit BSPs are not consistent -- m0 express defs ARDUINO_SAMD_FEATHER_M0,
// m0 defs ADAFRUIT_FEATHER_M0
//
#if defined(ARDUINO_SAMD_FEATHER_M0) || defined(ADAFRUIT_FEATHER_M0)
// Pin mapping for Adafruit Feather M0 LoRa, etc.
// /!\ By default Adafruit Feather M0's pin 6 and DIO1 are not connected.
// Please ensure they are connected.
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
    .spi_freq = 8000000,
};
#elif defined(ARDUINO_AVR_FEATHER32U4)
// Pin mapping for Adafruit Feather 32u4 LoRa, etc.
// Just like Feather M0 LoRa, but uses SPI at 1MHz; and that's only
// because MCCI doesn't have a test board; probably higher frequencies
// will work.
// /!\ By default Feather 32u4's pin 6 and DIO1 are not connected. Please
// ensure they are connected.
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {7, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8,              // LBT cal for the Adafruit Feather 32U4 LoRa, in dB
    .spi_freq = 1000000,
};
#elif defined(ARDUINO_CATENA_4551)
// Pin mapping for Murata module / Catena 4551
const lmic_pinmap lmic_pins = {
        .nss = 7,
        .rxtx = 29,
        .rst = 8,
        .dio = { 25,    // DIO0 (IRQ) is D25
                 26,    // DIO1 is D26
                 27,    // DIO2 is D27
               },
        .rxtx_rx_active = 1,
        .rssi_cal = 10,
        .spi_freq = 8000000     // 8MHz
};
#else
# error "Unknown target"
#endif

// A buffer for printing log messages.
static constexpr int MAX_MSG = 256;
static char msg[MAX_MSG];

// A printf-like function to print log messages prefixed by the current
// LMIC tick value. Don't call it before os_init();
//
// The RTC timestamps will start at 00:00:00, but will update to UTC
// if the DeviceTimeReq is answered.
void log_msg(const char *fmt, ...) {
#ifdef USE_SERIAL
    snprintf(msg, MAX_MSG, "%02d:%02d:%02d / ", rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
    serial.write(msg, strlen(msg));
    snprintf(msg, MAX_MSG, "% 012ld: ", os_getTime());
    serial.write(msg, strlen(msg));
    va_list args;
    va_start(args, fmt);
    vsnprintf(msg, MAX_MSG, fmt, args);
    va_end(args);
    serial.write(msg, strlen(msg));
    serial.println();
#endif
}

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        serial.print('0');
    serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    switch(ev) {
        case EV_JOINING:
            log_msg("EV_JOINING");
            break;
        case EV_JOINED:
            log_msg("EV_JOINED");
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              serial.print("netid: ");
              serial.println(netid, DEC);
              serial.print("devaddr: ");
              serial.println(devaddr, HEX);
              serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  serial.print("-");
                printHex2(artKey[i]);
              }
              serial.println("");
              serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                  if (i != 0) {
                      serial.print("-");
                  }
                  printHex2(nwkKey[i]);
              }
              serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
            // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);

            os_setCallback(&sendjob, do_send);

            joined = true;
            break;
        case EV_JOIN_FAILED:
            log_msg("EV_JOIN_FAILED");
            break;
        case EV_REJOIN_FAILED:
            log_msg("EV_REJOIN_FAILED");
            break;
            break;
        case EV_TXCOMPLETE:
            digitalWrite(LED_BUILTIN, LOW);
            log_msg("EV_TXCOMPLETE (includes waiting for RX windows)");
            if (LMIC.txrxFlags & TXRX_ACK)
              log_msg("Received ack");
            if (LMIC.dataLen) {
              log_msg("Received %d bytes of payload", LMIC.dataLen);
            }

            // Now check if there is time to go to standby mode.
            check_for_standby = true;
            break;
        case EV_TXSTART:
            digitalWrite(LED_BUILTIN, HIGH);
            log_msg("EV_TXSTART");
            break;
        case EV_TXCANCELED:
            log_msg("EV_TXCANCELED");
            break;
        case EV_JOIN_TXCOMPLETE:
            log_msg("EV_JOIN_TXCOMPLETE: no JoinAccept");
            break;

        default:
            log_msg("Unknown event: %u", (unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j) {

    // Schedule the next application level uplink so the interval is constant
    // rather than related to the end of an uplink/downlink.
    // Clear any current scheduled copy of the job because there should never be
    // more than one.
    os_clearCallback(&sendjob);
    os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);

    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        log_msg("OP_TXRXPEND, not sending");
    } else {
        // Keep asing for the time until the server provides it.
        if ( ! timeOk) {
            log_msg("Adding DeviceTimeReq MAC command to uplink.");
            LMIC_requestNetworkTime(user_request_network_time_callback, &userUTCTime);
        }

        // Prepare upstream data transmission at the next possible time.
        log_msg("Sending uplink");
        counter++;
        LMIC_setTxData2(1, (xref2u1_t)&counter, sizeof(counter), 0);
    }
}

void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    serial.begin(115200);
    serial.println("Starting");

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setLinkCheckMode(0);

    // Not used an AS923, maybe required for US_915?
    //LMIC_setDrTxpow(DR_SF7,14);
    //LMIC_selectSubBand(1);

    rtc.begin(true);

    LMIC_startJoining();
}

void loop() {
    os_runloop_once();

    // Do not go into standby mode until joined.
    if ( ! (joined || check_for_standby)) {
        return;
    }

    // This is used to decide whether to sleep so initialise it to
    // a known value.
    delta_seconds = -1;

    // If an uplink/downlink has just finished, see if there is time to sleep
    // before LMIC wants to do anything else.
    if (check_for_standby) {
        // Don't try to sleep again after waking up, go back to busy-loop
        // operation because data will be sent, either due to a sensor read
        // or an internal LMIC operation.
        check_for_standby = false;

        bit_t have_deadline = 0;
        ostime_t timestamp = os_getTime();
        ostime_t deadline = os_getNextDeadline(&have_deadline);

        if (have_deadline) {
            log_msg("Next deadline: %ld", deadline);
            delta_osticks = deadline - timestamp;
            delta_seconds = osticks2ms(delta_osticks)/1000;
            log_msg("Delta from now: %ld, %ld s", delta_osticks, delta_seconds);
        } else {
            log_msg("ERROR: No scheduled jobs, scheduling do_sent() to run now.");
            os_setCallback(&sendjob, do_send);
        }
    }

    if (delta_seconds > 3) {
        // Given the RTC is used for the alarm and it has a 1 second granularity
        // it seems prudent to provide a buffer for waking up in time to allow
        // LMIC to run the next task on time.
        delta_seconds--;
        delta_seconds--;

        // Set the alarm in the RTC to wake just before LMIC wants to
        // do its next job.
        set_delta_alarm();

        log_msg("Standby");

        // Ensure the log output is visible.
        serial.flush();

        // Don't try to sleep again after waking up, go back to busy-loop operation.
        check_for_standby = false;

        // Go into standby mode until woken by an interrupt, eg the RTC alarm.
        rtc.standbyMode();
        log_msg("Woke up");

        // Disable the alarm in case it was set to some short interval and LMIC
        // tasks will run for longer than that. It probably wouldn't cause
        // trouble but may as well be sure.
        rtc.disableAlarm();
    }
}

/*
 * This function is used to set the alarm to a relative time in the future, such as when
 * sleeping between LMIC tasks.
 */
void set_delta_alarm() {
    int32_t ss = (int32_t)rtc.getSeconds();
    int32_t mm = (int32_t)rtc.getMinutes();
    int32_t hh = (int32_t)rtc.getHours();

    // Sanity check.
    if (delta_seconds < 1) {
        delta_seconds = 1;
    }

    int32_t delta = delta_seconds;
    int32_t hh_delta = delta / 3600; delta -= (hh_delta * 3600);
    // Will always be less than 1 hour.
    int32_t mm_delta = delta / 60; delta -= (mm_delta * 60);
    // Will always be less than 1 minute.
    int32_t ss_delta = delta;

    ss += ss_delta;
    if (ss > 59) {
        ss = ss % 60;
        mm_delta++;
    }

    mm += mm_delta;
    if (mm > 59) {
        mm = mm % 60;
        hh_delta++;
    }

    hh = (hh + hh_delta) % 24;

    log_msg("Delta(s) = %d, wake at %02d:%02d:%02d", delta_seconds, hh, mm, ss);

    rtc.setAlarmTime((uint8_t)(hh & 0xff), (uint8_t)(mm & 0xff), (uint8_t)(ss & 0xff));
    rtc.enableAlarm(RTCZero::MATCH_HHMMSS);
}

void user_request_network_time_callback(void *pVoidUserUTCTime, int flagSuccess) {
    // Explicit conversion from void* to uint32_t* to avoid compiler errors
    uint32_t *pUserUTCTime = (uint32_t *) pVoidUserUTCTime;

    // Only ask for one reply, even if it is a failure.
    timeOk = true;

    // A struct that will be populated by LMIC_getNetworkTimeReference.
    // It contains the following fields:
    //  - tLocal: the value returned by os_GetTime() when the time
    //            request was sent to the gateway, and
    //  - tNetwork: the seconds between the GPS epoch and the time
    //              the gateway received the time request
    lmic_time_reference_t lmicTimeReference;

    if (flagSuccess != 1) {
        log_msg("user_request_network_time_callback: Not a success");
        return;
    }

    // Populate "lmic_time_reference"
    flagSuccess = LMIC_getNetworkTimeReference(&lmicTimeReference);
    if (flagSuccess != 1) {
        log_msg("user_request_network_time_callback: LMIC_getNetworkTimeReference didn't succeed");
        return;
    }

    // Update userUTCTime, considering the difference between the GPS and UTC
    // epoch, and the leap seconds
    *pUserUTCTime = lmicTimeReference.tNetwork + 315964800;

    // Add the delay between the instant the time was transmitted and
    // the current time

    // Current time, in ticks
    ostime_t ticksNow = os_getTime();
    // Time when the request was sent, in ticks
    ostime_t ticksRequestSent = lmicTimeReference.tLocal;
    uint32_t requestDelaySec = osticks2ms(ticksNow - ticksRequestSent) / 1000;
    *pUserUTCTime += requestDelaySec;

    // All that gets the sketch to somewhere near UTC.

    // Update the system time with the time read from the network.
    rtc.setEpoch(*pUserUTCTime);
    log_msg("RTC set from network time");
}
