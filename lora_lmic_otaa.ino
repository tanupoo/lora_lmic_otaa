/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#if LMIC_DEBUG_LEVEL > 0 || LMIC_X_DEBUG_LEVEL > 0
void lmic_printf(const char *fmt, ...) {
    char buf[80];
    va_list ap;

    va_start(ap, fmt);
    (void)vsnprintf(buf, sizeof(buf) - 1, fmt, ap);
    va_end(ap);

    // in case we overflowed:
    buf[sizeof(buf) - 1] = '\0';
    Serial.print(buf);
}

/*
 * show memory of bytes specified into Serial.
 */
void
lmic_printmem(u1_t *src, int src_len)
{
    u1_t *sp = src;

    do {
        char buf[33];   // for 16 bytes in memory.
        int buflen = sizeof(buf);
        char *p = buf;
        for (int buflen = sizeof(buf); buflen > 2; buflen -= 2) {
            (void)snprintf(p, buflen, "%02X", *sp++);
            p += 2;
            if (src + src_len == sp)
                break;
        }
        Serial.println(buf);
    } while (sp < src + src_len);
}
#endif

/*
 * APPEUI, DEVEUI must be in **LITTLE ENDIAN** format.
 * APPKEY must be in **BIG ENDIAN** format.
 */
static const u1_t PROGMEM DEVEUI[8]={
    // beef020000001102
    0x02, 0x11, 0x00, 0x00, 0x00, 0x02, 0xef, 0xbe,
};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

static const u1_t PROGMEM APPEUI[8]={
    // beef020000001101
    0x01, 0x11, 0x00, 0x00, 0x00, 0x02, 0xef, 0xbe,
};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

static const u1_t PROGMEM APPKEY[16] = {
    // 0001020304050607 00000000000000ff
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
//  0x07, 0x06, 0x05, 0x04, 0x03, 0x02, 0x01, 0x00,
//  0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t sendbuf[] = {
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00
};
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 60;

// Pin mapping
// https://github.com/matthijskooijman/arduino-lmic/issues/59
const lmic_pinmap lmic_pins = {
    .nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,
    .dio = {2, 6, 7}, // Specify pin numbers for DIO0, 1, 2
};

void onEvent (ev_t ev)
{
    LMIC_X_DEBUG_PRINTF("%lu:", os_getTime());
    switch(ev) {
    case EV_SCAN_TIMEOUT:
        LMIC_X_DEBUG_PRINTS("EV_SCAN_TIMEOUT");
        break;
    case EV_BEACON_FOUND:
        LMIC_X_DEBUG_PRINTS("EV_BEACON_FOUND");
        break;
    case EV_BEACON_MISSED:
        LMIC_X_DEBUG_PRINTS("EV_BEACON_MISSED");
        break;
    case EV_BEACON_TRACKED:
        LMIC_X_DEBUG_PRINTS("EV_BEACON_TRACKED");
        break;
    case EV_JOINING:
        LMIC_X_DEBUG_PRINTS("EV_JOINING");
        break;
    case EV_JOINED:
	Serial.println(F("EV_JOINED"));
#if 0
        // Disable link check validation (automatically enabled
        // during join, but not supported by TTN at this time).
        LMIC_setLinkCheckMode(0);
#endif
#if LMIC_DEBUG_LEVEL > 0 || LMIC_X_DEBUG_LEVEL > 0
        {
            u4_t netid = 0;
            devaddr_t devaddr = 0;
            u1_t nsKey[16];
            u1_t asKey[16];
        
            LMIC_getSessionKeys(&netid, &devaddr, nsKey, asKey);
            Serial.print("netid: ");
            Serial.println(netid, DEC);
            Serial.print("devaddr: ");
            Serial.println(devaddr, HEX);
            Serial.print("nsKey: ");
            lmic_printmem(nsKey, 16);
            Serial.print("asKey: ");
            lmic_printmem(asKey, 16);
        }
#endif
        LMIC_setSeqnoUp(1);
        break;
    case EV_RFU1:
        LMIC_X_DEBUG_PRINTS("EV_RFU1");
        break;
    case EV_JOIN_FAILED:
        LMIC_X_DEBUG_PRINTS("EV_JOIN_FAILED");
        break;
    case EV_REJOIN_FAILED:
        LMIC_X_DEBUG_PRINTS("EV_REJOIN_FAILED");
        break;
    case EV_TXCOMPLETE:
        Serial.println("EV_TXCOMPLETE");
        if (LMIC.txrxFlags & TXRX_ACK)
            LMIC_X_DEBUG_PRINTS("Received ack");
        if (LMIC.dataLen) {
            LMIC_X_DEBUG_PRINTF("Received %u B\n", LMIC.dataLen);
        }
        // Schedule next transmission
        os_setTimedCallback(&sendjob,
                os_getTime()+sec2osticks(TX_INTERVAL),
                do_send);
        break;
    case EV_LOST_TSYNC:
        LMIC_X_DEBUG_PRINTS("EV_LOST_TSYNC");
        break;
    case EV_RESET:
        LMIC_X_DEBUG_PRINTS("EV_RESET");
        break;
    case EV_RXCOMPLETE:
        // data received in ping slot
        LMIC_X_DEBUG_PRINTS("EV_RXCOMPLETE");
        break;
    case EV_LINK_DEAD:
        LMIC_X_DEBUG_PRINTS("EV_LINK_DEAD");
        break;
    case EV_LINK_ALIVE:
        LMIC_X_DEBUG_PRINTS("EV_LINK_ALIVE");
        break;
    case EV_SCAN_FOUND:
        LMIC_X_DEBUG_PRINTS("EV_SCAN_FOUND");
        break;
    case EV_TXSTART:
        LMIC_X_DEBUG_PRINTF("EV_TXSTART: freq=%lu sf=%u\n",
                LMIC.freq, 6+(sf_t)((LMIC.rps)&0x7));
        break;
    default:
        LMIC_X_DEBUG_PRINTF("EV unknown %u", ev);
        break;
    }
}

void do_send(osjob_t* j)
{
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        LMIC_X_DEBUG_PRINTS("OP_TXRXPEND, not sending");
    } else {
        // Prepare upstream data transmission at the next possible time.
        sendbuf[0]++;
        LMIC_setTxData2(1, sendbuf, sizeof(sendbuf), 0);
        LMIC_X_DEBUG_PRINTS("Packet queued");
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

static osjob_t debugjob;

void debug_print(osjob_t* j)
{
#define DEBUG_INTERVAL 10   // seconds
    ostime_t t0 = os_getTime();
    ostime_t t1 = t0 + sec2osticks(DEBUG_INTERVAL);
    LMIC_X_DEBUG_PRINTF("%lu:opmode=%u:dlen=%u\n",
            t0, LMIC.opmode, LMIC.dataLen);
    os_setTimedCallback(&debugjob, t1, debug_print);
}

void setup()
{
#if 1   // Dragino needs to wait for the serial ?
    delay(2000);
    while (!Serial)
        ;
#endif
    Serial.begin(9600);
    Serial.println(F("Starting"));

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();
    LMIC.txpow = 16;    // XXX it should be less then EIRP 16

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
#if LMIC_DEBUG_LEVEL > 0 || LMIC_X_DEBUG_LEVEL > 0
    debug_print(&debugjob);
#endif
}

void loop()
{
    os_runloop_once();
}
