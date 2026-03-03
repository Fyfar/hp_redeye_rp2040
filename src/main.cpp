#define DATA_PIN 2
#define HALF_BIT_RECEIVED_PIN 4
#define HALF_BIT_INTERVAL_US 418
#define MAX_HALF_BITS 30

#define EC_MASK1 0b01111000
#define EC_MASK2 0b11100110
#define EC_MASK3 0b11010101
#define EC_MASK4 0b10001011

#define END_OF_TRANSMISSION_BYTE 4
#define ASCII_OFFSET 128

#include <Arduino.h>
#include <RPi_Pico_TimerInterrupt.h>

constexpr char16_t ROMAN_8_SYMBOLS[18] = {
    u'\u00a0',
    u'\u00f7',
    u'\u00d7',
    u'\u221a',
    u'\u222b',
    u'\u03a3',
    u'\u23f5',
    u'\u03C0',
    u'\u2202',
    u'\u2264',
    u'\u2265',
    u'\u2260',
    u'\u0251',
    u'\u2192',
    u'\u2190',
    u'\u00B5',
    u'\u000A',
    u'\u00B0'};
constexpr uint8_t errorCorrectionsMasks[4] = {EC_MASK1, EC_MASK2, EC_MASK3, EC_MASK4};
constexpr struct {
    uint8_t startHalfBits = 3;
    uint8_t startHalfBitsOffset = 0;
    uint8_t errorCheckingBits = 4;
    uint8_t errorCheckingBitsOffset = 3;
    uint8_t dataBits = 8;
    uint8_t dataBitsOffset = startHalfBits + (errorCheckingBits * 2);  // offset in half-bits, accounting for error checking bits being 2 half-bits each
    uint8_t stopHalfBits = 3;
    uint8_t stopHalfBitsOffset = startHalfBits + (errorCheckingBits * 2) + (dataBits * 2);  // offset in half-bits, accounting for error checking and data bits being 2 half-bits each
} protocolConfig;

volatile bool halfBitBuffer0[MAX_HALF_BITS];
volatile bool halfBitBuffer1[MAX_HALF_BITS];
volatile uint8_t activeBuffer = 0;
volatile bool timerStarted = false;
volatile bool dataReceivedInHalfBit = false;
volatile bool byteReadyForProcessing = false;
volatile bool halfBitPinState = LOW;

RPI_PICO_Timer ITimer1(1);

void __no_inline_not_in_flash_func(dataISR)() {
    if (!timerStarted) {
        ITimer1.enableTimer();
        timerStarted = true;

        digitalWriteFast(HALF_BIT_RECEIVED_PIN, HIGH);  // Indicate half-bit received
        halfBitPinState = HIGH;
    }

    dataReceivedInHalfBit = true;
    byteReadyForProcessing = false;
}

void stopHalfBitTimer() {
    ITimer1.stopTimer();
    timerStarted = false;
}

bool halfBitTimerCallback(struct repeating_timer* t) {
    (void)*t;  // Avoid unused parameter warning
    static uint_fast8_t halfBitBufferIndex = 0;

    halfBitPinState = !halfBitPinState;                        // Toggle half-bit pin state
    digitalWriteFast(HALF_BIT_RECEIVED_PIN, halfBitPinState);  // Reset half-bit received indicator

    activeBuffer == 0 ? (halfBitBuffer0[halfBitBufferIndex] = dataReceivedInHalfBit)
                      : (halfBitBuffer1[halfBitBufferIndex] = dataReceivedInHalfBit);

    halfBitBufferIndex++;
    dataReceivedInHalfBit = false;

    if (halfBitBufferIndex >= MAX_HALF_BITS) {
        halfBitBufferIndex = 0;
        byteReadyForProcessing = true;
        activeBuffer = 1 - activeBuffer;  // Switch active buffer

        stopHalfBitTimer();  // Stop half-bit timer until next byte is received
    }

    return true;  // Return true to keep the timer running
}

uint_fast8_t checkBit(const bool firstHalfBit, const bool secondHalfBit) {
    if (firstHalfBit == 0 && secondHalfBit == 1) {
        return 0;  // Logical '0'
    } else if (firstHalfBit == 1 && secondHalfBit == 0) {
        return 1;  // Logical '1'
    } else {
        return 255;  // Invalid bit (error)
    }
}

bool validateStartHalfBits(const bool* buffer) {
    uint_fast8_t startHalfBitsValue = 0;  // Static variable to hold the calculated start half-bits value
    Serial.print("First 3 half-bits: ");
    for (uint8_t i = protocolConfig.startHalfBitsOffset; i < protocolConfig.startHalfBits; i++) {
        uint8_t bitPosition = protocolConfig.startHalfBits - 1 - i;
        startHalfBitsValue |= (buffer[i] << bitPosition);  // Shift and combine bits into a value

        Serial.print(buffer[i] ? "1" : "0");
    }
    Serial.println();

    return startHalfBitsValue == 0b111;  // Check if the start half-bits match the expected pattern (0b111)
}

bool validateStopHalfBits(const bool* buffer) {
    for (uint8_t i = protocolConfig.stopHalfBitsOffset; i < MAX_HALF_BITS; i++) {
        if (buffer[i] != 0) {
            Serial.print("Invalid stop half-bits detected. Discarding byte. Position: ");
            Serial.println(i);
            return false;  // Stop half-bits should be all 0s, if not, it's an error
        }
    }

    return true;  // Stop half-bits are valid if all are 0s
}

void printErrorCheckingHalfBits(const bool* buffer) {
    Serial.print("Error cheking half-bits: ");
    for (uint8_t i = protocolConfig.errorCheckingBitsOffset; i < protocolConfig.dataBitsOffset; i++) {
        Serial.print(buffer[i] ? "1" : "0");
    }
    Serial.println();
}

uint_fast8_t extractErrorCorrectionBits(const bool* buffer) {
    uint_fast8_t errorCorrectionBits = 0;  // Reset error correction variable before calculating

    Serial.print("Error checking bits: ");
    for (uint8_t i = protocolConfig.errorCheckingBitsOffset; i < protocolConfig.dataBitsOffset; i += 2) {
        uint_fast8_t bitValue = checkBit(buffer[i], buffer[i + 1]);

        if (bitValue == 255) {
            Serial.println("Error correction bit error detected. Discarding value.");
            printErrorCheckingHalfBits(buffer);
            return 255;  // Return 255 to indicate an error in error correction bits
        }

        Serial.print(bitValue);

        uint8_t bitPosition = (i - 3) / 2;                       // Calculate bit position for error correction
        errorCorrectionBits |= (bitValue << (3 - bitPosition));  // Shift and combine bits into errorCorrection variable
    }

    Serial.println();
    Serial.print("Calculated error correction value: 0b");
    Serial.println(errorCorrectionBits, BIN);

    return errorCorrectionBits;
}

uint_fast8_t calculateErrorCorrection(char data) {
    uint_fast8_t errorCorrectionBits = 0;
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t masked = errorCorrectionsMasks[i] & data;
        uint8_t ones = __builtin_popcount(masked);
        if (ones % 2 != 0) {
            errorCorrectionBits |= (1 << (3 - i));  // Set the corresponding error correction bit if the count of ones is odd
        }
    }

    Serial.print("Calculated error correction bits: 0b");
    Serial.println(errorCorrectionBits, BIN);

    return errorCorrectionBits;
}

void printDataHalfBits(const bool* buffer) {
    Serial.print("Data half-bits: ");
    for (uint8_t i = 11; i < 27; i++) {
        Serial.print(buffer[i] ? "1" : "0");
    }
    Serial.println();
}

uint_fast8_t extractDataByte(const bool* buffer) {
    uint_fast8_t dataByte = 0;

    Serial.print("Data bits: ");
    for (uint8_t i = protocolConfig.dataBitsOffset; i < protocolConfig.stopHalfBitsOffset; i += 2) {
        uint_fast8_t bitValue = checkBit(buffer[i], buffer[i + 1]);

        if (bitValue == 255) {
            Serial.println("Data bit error detected, discarding byte.");
            printDataHalfBits(buffer);

            return 255;  // Return 255 to indicate an error in data bits
        }

        Serial.print(bitValue);

        uint8_t bitPosition = (i - 11) / 2;           // Calculate bit position for data bits
        dataByte |= (bitValue << (7 - bitPosition));  // Shift and combine bits into a byte
    }

    Serial.println();
    Serial.print("Decoded byte: 0b");
    Serial.println(dataByte, BIN);

    if (dataByte >= ASCII_OFFSET) {
        Serial.print("Decoded byte (roman symbol): ");
        Serial.println(ROMAN_8_SYMBOLS[dataByte - ASCII_OFFSET]);
    } else {
        Serial.print("Decoded byte (ascii): ");
        Serial.println(dataByte);
    }

    return dataByte;
}

void processReceivedByte(uint8_t bufferToRead) {
    const bool* buffer = bufferToRead == 0 ? (const bool*)halfBitBuffer0 : (const bool*)halfBitBuffer1;

    bool startHalfBitsValid = validateStartHalfBits(buffer);
    bool stopHalfBitsValid = validateStopHalfBits(buffer);

    if (!startHalfBitsValid) {
        Serial.println("Invalid start half-bits. Discarding byte.");
        return;
    }

    if (!stopHalfBitsValid) {
        Serial.println("Invalid stop half-bits. Discarding byte.");
        // return;
    }

    uint_fast8_t errorCorrectionBits = extractErrorCorrectionBits(buffer);  // only 4 bits used for error correction, so uint_fast8_t is sufficient
    uint_fast8_t dataByte = extractDataByte(buffer);
    uint_fast8_t calculateErrorCorrectionBits = calculateErrorCorrection(dataByte);  // Check error correction for the decoded byte;

    if (errorCorrectionBits != -1 && errorCorrectionBits != calculateErrorCorrectionBits) {
        // TODO: In case of error correction mismatch, we could attempt to correct the data by flipping bits and checking if the error correction matches. For now, we will just discard the byte if there is an error correction mismatch.
        Serial.println("Error correction mismatch detected. Discarding byte.");
    }

    Serial.println("--------------------------------");

    if (dataByte == END_OF_TRANSMISSION_BYTE) {
        Serial1.println();  // Send a newline to indicate end of transmission
    } else if (dataByte >= ASCII_OFFSET) {
        Serial1.print(ROMAN_8_SYMBOLS[dataByte - ASCII_OFFSET]);  // Send the decoded Roman symbol over Serial1 (UART)
    } else {
        Serial1.print(dataByte);  // Send the decoded byte over Serial1 (UART)
    }
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);

    pinMode(DATA_PIN, INPUT_PULLUP);
    pinMode(HALF_BIT_RECEIVED_PIN, OUTPUT);

    digitalWriteFast(HALF_BIT_RECEIVED_PIN, LOW);

    attachInterrupt(digitalPinToInterrupt(DATA_PIN), dataISR, FALLING);

    if (ITimer1.attachInterruptInterval(HALF_BIT_INTERVAL_US, halfBitTimerCallback)) {
        Serial.print(F("Starting ITimer1 OK, millis() = "));
        Serial.println(millis());
        ITimer1.stopTimer();  // Start with the timer stopped until the first half-bit is received
    } else {
        Serial.println(F("Can't set ITimer1. Select another freq. or timer"));
    }
}

void loop() {
    if (byteReadyForProcessing) {
        byteReadyForProcessing = false;  // Reset the flag to prevent re-processing the same byte

        digitalWriteFast(HALF_BIT_RECEIVED_PIN, LOW);
        uint8_t bufferToRead = 1 - activeBuffer;
        processReceivedByte(bufferToRead);
    }
}