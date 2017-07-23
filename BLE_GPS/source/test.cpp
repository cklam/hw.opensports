#include "mbed.h"
#include "greentea-client/test_env.h"
#include "unity.h"
#include "utest.h"
#include "gnss.h"
extern "C" {
#include "c030_api.h"
}
 
using namespace utest::v1;

// ----------------------------------------------------------------
// COMPILE-TIME MACROS
// ----------------------------------------------------------------

// How long to wait for a GNSS result
#define GNSS_WAIT_SECONDS 120

// ----------------------------------------------------------------
// PRIVATE VARIABLES
// ----------------------------------------------------------------

// ----------------------------------------------------------------
// PRIVATE FUNCTIONS
// ----------------------------------------------------------------

static void printHex (char * pData, uint32_t lenData)
{
    char * pEnd = pData + lenData;
    uint8_t x;

    printf (" 0  1  2  3  4  5  6  7   8  9  A  B  C  D  E  F\n");
    while (pData < pEnd) {
        for (x = 1; (x <= 32) && (pData < pEnd); x++) {
            if (x % 16 == 8) {
                printf ("%02x  ", *pData);
            } else if (x % 16 == 0) {
                printf ("%02x\n", *pData);
            } else {
                printf ("%02x-", *pData);
            }
            pData++;
        }


        if (x % 16 !=  1) {
            printf("\n");
        }
    }
}

// ----------------------------------------------------------------
// TESTS
// ----------------------------------------------------------------

// Test sending a u-blox command over serial
void test_serial_ubx() {
    char buffer[64];
    int responseLength = 0;
    int returnCode;

    GnssSerial *pGnss = new GnssSerial();

    // Initialise the GNSS chip and wait for it to start up
    pGnss->init(NC);
    wait_ms(1000);

    // See ublox7-V14_ReceiverDescrProtSpec section 30.11.15 (CFG-NAV5)
    // Set automotive mode, which should be acknowledged
    memset (buffer, 0, sizeof (buffer));
    buffer[0] = 0x00;
    buffer[1] = 0x01; // Set dynamic config only
    buffer[2] = 0x04; // Automotive
    // Send length is 32 bytes of payload + 6 bytes header + 2 bytes CRC
    TEST_ASSERT_EQUAL_INT (40, pGnss->sendUbx(0x06, 0x24, buffer, 32));
    while (responseLength == 0) {
        // Wait for the required Ack
        returnCode = pGnss->getMessage(buffer, sizeof(buffer));
        if ((returnCode != GnssSerial::WAIT) && (returnCode != GnssSerial::NOT_FOUND)) {
            responseLength = LENGTH(returnCode);
            if ((PROTOCOL(returnCode) == GnssSerial::UBX)) {
                printHex(buffer, responseLength);
                // Ack is  0xb5-62-05-00-02-00-msgclass-msgid-crcA-crcB
                // Nack is 0xb5-62-05-01-02-00-msgclass-msgid-crcA-crcB
                TEST_ASSERT_EQUAL_UINT8(0xb5, buffer[0]);
                TEST_ASSERT_EQUAL_UINT8(0x62, buffer[1]);
                TEST_ASSERT_EQUAL_UINT8(0x05, buffer[2]);
                TEST_ASSERT_EQUAL_UINT8(0x00, buffer[3]);
                TEST_ASSERT_EQUAL_UINT8(0x02, buffer[4]);
                TEST_ASSERT_EQUAL_UINT8(0x00, buffer[5]);
                TEST_ASSERT_EQUAL_UINT8(0x06, buffer[6]);
                TEST_ASSERT_EQUAL_UINT8(0x24, buffer[7]);
            } else if ((PROTOCOL(returnCode) == GnssSerial::NMEA)) {
                printf ("%.*s", responseLength, buffer);
                responseLength = 0;
            } else {
                printHex(buffer, responseLength);
                responseLength = 0;
            }
        }
        wait_ms (100);
    }
}

// Test getting a response from GNSS using the serial interface
void test_serial_time() {
    GnssSerial *pGnss = new GnssSerial();

    bool gotLatLong = false;
    bool gotElevation = false;
    bool gotSpeed = false;
    bool gotTime = false;
    char buffer[256];
    int returnCode;
    double latitude;
    double longitude;
    double elevation;
    double speed;

    printf("GNSS: powering up and waiting up to %d second(s) for something to happen.\n", GNSS_WAIT_SECONDS);
    pGnss->init();

    memset(buffer, 0, sizeof(buffer));
    for (uint32_t x = 0; (x < GNSS_WAIT_SECONDS) && !gotTime; x++)
    {
        while (((returnCode = pGnss->getMessage(buffer, sizeof(buffer))) > 0) &&
                !(gotLatLong && gotElevation && gotSpeed && gotTime))
        {
            int32_t length = LENGTH(returnCode);

            if ((PROTOCOL(returnCode) == GnssParser::NMEA) && (length > 6))
            {
                printf(".");

                // talker is $GA=Galileo $GB=Beidou $GL=Glonass $GN=Combined $GP=GNSS
                if ((buffer[0] == '$') || buffer[1] == 'G')
                {
#define _CHECK_TALKER(s) ((buffer[3] == s[0]) && (buffer[4] == s[1]) && (buffer[5] == s[2]))
                    if (_CHECK_TALKER("GLL"))
                    {
                        char ch;

                        if (pGnss->getNmeaAngle(1, buffer, length, latitude) &&
                            pGnss->getNmeaAngle(3, buffer, length, longitude) &&
                            pGnss->getNmeaItem(6, buffer, length, ch) &&
                            ch == 'A')
                        {
                            gotLatLong = true;
                            latitude *= 60000;
                            longitude *= 60000;
                            printf("\nGNSS: location %.5f %.5f %c.\n", latitude, longitude, ch);
                        }
                    }
                    else if (_CHECK_TALKER("GGA") || _CHECK_TALKER("GNS"))
                    {
                        const char *pTimeString = NULL;

                        // Retrieve the time
                        pTimeString = pGnss->findNmeaItemPos(1, buffer, buffer + length);
                        if (pTimeString != NULL)
                        {
                            gotTime = true;
                            printf("\nGNSS: time is %.6s.", pTimeString);
                        }

                        if (pGnss->getNmeaItem(9, buffer, length, elevation)) // altitude msl [m]
                        {
                            gotElevation = true;
                            printf("\nGNSS: elevation: %.1f.", elevation);
                        }
                    }
                    else if (_CHECK_TALKER("VTG"))
                    {
                        if (pGnss->getNmeaItem(7, buffer, length, speed)) // speed [km/h]
                        {
                            gotSpeed = true;
                            printf("\nGNSS: speed: %.1f.", speed);
                        }
                    }
                }
            }
        }

        wait_ms(1000);
    }

    printf("\n");

    // Depending on antenna positioning we may not be able to get a GNSS fix but we
    // should at least be able to receive the time from a satellite
    TEST_ASSERT(gotTime);
}

// ----------------------------------------------------------------
// TEST ENVIRONMENT
// ----------------------------------------------------------------

// Setup the test environment
utest::v1::status_t test_setup(const size_t number_of_cases) {
    // Setup Greentea with a timeout
    GREENTEA_SETUP(120, "default_auto");
    return verbose_test_setup_handler(number_of_cases);
}

// Test cases
Case cases[] = {
    Case("Ubx command", test_serial_ubx),
    Case("Get time", test_serial_time),
};

Specification specification(test_setup, cases);

// ----------------------------------------------------------------
// MAIN
// ----------------------------------------------------------------

int main() {

    c030_init(); // HACK

    return !Harness::run(specification);
}

// End Of File
