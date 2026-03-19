#define DATA_PIN 2
#define HALF_BIT_RECEIVED_PIN 3
#define HALF_BIT_INTERVAL_US 419
#define MAX_HALF_BITS 30
#define PWM_IRQ_PIN 15  // any existing pin

#define EC_MASK1 0b01111000
#define EC_MASK2 0b11100110
#define EC_MASK3 0b11010101
#define EC_MASK4 0b10001011

#define DRAWING_MODE_BYTE 27
#define DRAWING_BLOCKS 4                 // LCD is 32 pixels height, and each block is 8 pixels (1 byte), so we need 4 blocks to represent the full height of the LCD
#define MAX_DRAWING_BYTES_PER_BLOCK 166  // represents max line width of HP 82240A/B printer

#define END_OF_TRANSMISSION_BYTE 4
#define ASCII_OFFSET 128

#include <Arduino.h>
#include <hardware/pwm.h>

#include <bitset>

const char* const ROMAN_8_SYMBOLS[18] = {
    " ",  // return regular space instead of non-breaking space to avoid issues with displaying non-breaking space in some terminals
    "÷",
    "×",
    "√",
    "∫",
    "Σ",
    "▶",
    "π",
    "∂",
    "≤",
    "≥",
    "≠",
    "∝",
    "→",
    "←",
    "µ",
    "\n",
    "°"};
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
volatile bool pwmStarted = false;
volatile bool dataReceivedInHalfBit = false;
volatile bool byteReadyForProcessing = false;

uint sliceNum;

bool drawMode = false;
bool readyToDrawBuffer = false;
bool terminationByteReceived = false;
uint8_t drawingBuffer[MAX_DRAWING_BYTES_PER_BLOCK];
uint8_t bytesToDrawInCurrentBlock = 0;

void __no_inline_not_in_flash_func(halfBitPWM_ISR)() {
    static uint_fast8_t halfBitBufferIndex = 0;

    pwm_clear_irq(sliceNum);

    digitalWriteFast(HALF_BIT_RECEIVED_PIN, !digitalReadFast(HALF_BIT_RECEIVED_PIN));  // Reset half-bit received indicator

    activeBuffer == 0 ? (halfBitBuffer0[halfBitBufferIndex] = dataReceivedInHalfBit)
                      : (halfBitBuffer1[halfBitBufferIndex] = dataReceivedInHalfBit);

    halfBitBufferIndex++;
    dataReceivedInHalfBit = false;

    if (halfBitBufferIndex >= MAX_HALF_BITS) {
        halfBitBufferIndex = 0;
        byteReadyForProcessing = true;
        activeBuffer = 1 - activeBuffer;

        sio_hw->gpio_clr = (1 << HALF_BIT_RECEIVED_PIN);  // Reset half-bit received indicator
        pwm_set_enabled(sliceNum, false);                 // Stop the PWM timer until the next byte is received
        pwmStarted = false;
    }
}

void __no_inline_not_in_flash_func(dataISR)(uint gpio, uint32_t events) {
    if (!pwmStarted) {
        pwm_set_counter(sliceNum, 0);  // Reset the timer counter to ensure accurate timing for the first half-bit
        pwm_set_enabled(sliceNum, true);
        pwmStarted = true;

        sio_hw->gpio_set = (1 << HALF_BIT_RECEIVED_PIN);  // Indicate half-bit received
    }

    dataReceivedInHalfBit = true;
    byteReadyForProcessing = false;
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
    for (uint8_t i = protocolConfig.startHalfBitsOffset; i < protocolConfig.startHalfBits; i++) {
        uint8_t bitPosition = protocolConfig.startHalfBits - 1 - i;
        if (!buffer[i]) return false;  // Start half-bits should be all 1s, if not, it's an error
    }

    return true;
}

bool validateStopHalfBits(const bool* buffer) {
    for (uint8_t i = protocolConfig.stopHalfBitsOffset; i < MAX_HALF_BITS; i++) {
        if (buffer[i]) return false;  // Stop half-bits should be all 0s, if not, it's an error
    }

    return true;
}

uint_fast8_t extractErrorCorrectionBits(const bool* buffer) {
    uint_fast8_t errorCorrectionBits = 0;  // Reset error correction variable before calculating

    for (uint8_t i = protocolConfig.errorCheckingBitsOffset; i < protocolConfig.dataBitsOffset; i += 2) {
        uint_fast8_t bitValue = checkBit(buffer[i], buffer[i + 1]);

        if (bitValue == 255) {
            Serial.println("Error correction bit error detected. Discarding value.");
            return 255;  // Return 255 to indicate an error in error correction bits
        }

        uint8_t bitPosition = (i - 3) / 2;                       // Calculate bit position for error correction
        errorCorrectionBits |= (bitValue << (3 - bitPosition));  // Shift and combine bits into errorCorrection variable
    }

    Serial.print("Error correction value: 0b");
    Serial.println(errorCorrectionBits, BIN);

    return errorCorrectionBits;
}

uint_fast8_t calculateErrorCorrection(uint_fast8_t data) {
    uint_fast8_t errorCorrectionBits = 0;
    for (uint8_t i = 0; i < 4; i++) {
        uint_fast8_t masked = errorCorrectionsMasks[i] & data;
        uint8_t ones = __builtin_popcount(masked);
        if (ones % 2 != 0) {
            errorCorrectionBits |= (1 << (3 - i));  // Set the corresponding error correction bit if the count of ones is odd
        }
    }

    return errorCorrectionBits;
}

uint_fast8_t extractDataByte(const bool* buffer) {
    uint_fast8_t dataByte = 0;

    for (uint8_t i = protocolConfig.dataBitsOffset; i < protocolConfig.stopHalfBitsOffset; i += 2) {
        uint_fast8_t bitValue = checkBit(buffer[i], buffer[i + 1]);

        if (bitValue == 255) return 255;  // Return 255 to indicate an error in data bits

        uint8_t bitPosition = (i - 11) / 2;           // Calculate bit position for data bits
        dataByte |= (bitValue << (7 - bitPosition));  // Shift and combine bits into a byte
    }

    Serial.print("Decoded byte: 0b");
    Serial.println(dataByte, BIN);

    if (drawMode) {
        return dataByte;  // If we are in drawing mode, we will return the raw byte value for processing as drawing data, without interpreting it as ASCII or Roman symbols, since the drawing data can contain any byte value and is not meant to be interpreted as text.
    }

    if (dataByte >= ASCII_OFFSET + 18) {
        Serial.println("Decoded byte is out of bounds for defined ROMAN_8_SYMBOLS.");
    } else if (dataByte >= ASCII_OFFSET && dataByte < ASCII_OFFSET + 18) {
        Serial.print("Decoded byte (roman symbol): ");
        Serial.println(ROMAN_8_SYMBOLS[dataByte - ASCII_OFFSET]);
    } else {
        Serial.print("Decoded byte (ascii): ");
        Serial.println((char)dataByte);
    }

    return dataByte;
}

void sendSymbolDataByte(uint_fast8_t dataByte) {
    if (dataByte >= ASCII_OFFSET + 18) {
        Serial1.print("?");  // Send a placeholder character for out-of-bounds values over Serial1 (UART)
    } else if (dataByte >= ASCII_OFFSET) {
        Serial1.print(ROMAN_8_SYMBOLS[dataByte - ASCII_OFFSET]);  // Send the decoded Roman symbol over Serial1 (UART)
    } else {
        Serial1.print((char)dataByte);  // Send the decoded byte over Serial1 (UART)
    }
}

void disableDrawingMode() {
    if (drawMode) {
        Serial.println("Drawing mode OFF");  // Send drawing mode status over Serial (USB)
        Serial1.println(DRAWING_MODE_BYTE);  // Send drawing mode status over Serial1 (
    }
    drawMode = false;
}

void enableDrawingMode() {
    if (!drawMode) {
        Serial.println("Drawing mode ON");   // Send drawing mode status over Serial (USB)
        Serial1.println(DRAWING_MODE_BYTE);  // Send drawing mode status over Serial1 (UART)
    }
    drawMode = true;
}

void sendDrawBufferToUART() {
    uint8_t buffer[bytesToDrawInCurrentBlock + 1];  // +1 for null terminator

    Serial.println("Sending drawing buffer to UART:");  // Log the action of sending the drawing buffer
    for (uint8_t bitIndex = 0; bitIndex < 8; bitIndex++) {
        for (uint8_t index = 0; index < bytesToDrawInCurrentBlock; index++) {
            uint8_t mask = 1 << bitIndex;
            buffer[index] = (drawingBuffer[index] & mask) ? '.' : ' ';
        }
        buffer[bytesToDrawInCurrentBlock] = '\n';                           // Null-terminate the buffer to safely print as a string
        uart_write_blocking(uart0, buffer, bytesToDrawInCurrentBlock + 1);  // Send each byte of the drawing buffer over Serial1 (UART)
    }

    bytesToDrawInCurrentBlock = 0;
    Serial.println("Finished drawing block.");
}

void processDrawingByte(uint_fast8_t dataByte) {
    static uint8_t drawingByteNumber = 0;
    static uint8_t drawingBlock = 0;

    if (bytesToDrawInCurrentBlock == 0) {  // If this is the first byte of a new drawing block, we will treat it as the byte count for how many bytes of drawing data to expect for this block, which allows for variable width drawing blocks up to the maximum defined by MAX_DRAWING_BYTES_PER_BLOCK.
        bytesToDrawInCurrentBlock = dataByte;
        Serial.print("Receiver will wait ");
        Serial.print(bytesToDrawInCurrentBlock);
        Serial.println(" bytes of drawing data for the next block of LCD (8 lines).");
        return;
    }

    drawingBuffer[drawingByteNumber++] = dataByte;

    if (drawingByteNumber >= bytesToDrawInCurrentBlock) {
        readyToDrawBuffer = true;
        drawingByteNumber = 0;
        drawingBlock++;
        Serial.println("Ready to draw buffer.");
    }

    if (drawingBlock >= DRAWING_BLOCKS) {
        drawingBlock = 0;  // Reset drawing block count for the next set of drawing data
        disableDrawingMode();
    }
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
        Serial.println("Invalid stop half-bits. Just ignore it.");
    }

    uint_fast8_t errorCorrectionBits = extractErrorCorrectionBits(buffer);  // only 4 bits used
    uint_fast8_t dataByte = extractDataByte(buffer);

    if (dataByte == 255) {
        Serial.println("Data byte error detected. Discarding byte.");
        return;
    }

    uint_fast8_t calculateErrorCorrectionBits = calculateErrorCorrection(dataByte);  // Check error correction for the decoded byte;

    if (errorCorrectionBits != 255 && errorCorrectionBits != calculateErrorCorrectionBits) {
        // TODO: In case of error correction mismatch, we could attempt to correct the data by flipping bits and checking if the error correction matches. For now, we will just discard the byte if there is an error correction mismatch.
        Serial.println("Error correction mismatch detected. Ignoring for now.");
    }

    switch (dataByte) {
        case END_OF_TRANSMISSION_BYTE:
            Serial1.println();  // Send a newline to indicate end of transmission
            terminationByteReceived = true;
            break;
        case DRAWING_MODE_BYTE:
            enableDrawingMode();  // Enable drawing mode and pass the previous data byte for context
            break;
        default:
            if (drawMode) {
                processDrawingByte(dataByte);
            } else {
                sendSymbolDataByte(dataByte);
            }
            break;
    }

    Serial.println("--------------------------------");
}

void setupPWM_ISR() {
    gpio_set_irq_enabled_with_callback(DATA_PIN, GPIO_IRQ_EDGE_FALL, true, &dataISR);

    sliceNum = pwm_gpio_to_slice_num(PWM_IRQ_PIN);
    pwm_clear_irq(sliceNum);              // Clear any pending PWM interrupts
    pwm_set_irq_enabled(sliceNum, true);  // Enable PWM interrupt for the selected slice

    irq_set_exclusive_handler(PWM_IRQ_WRAP, halfBitPWM_ISR);  // Set the PWM interrupt handler for the selected slice
    irq_set_enabled(PWM_IRQ_WRAP, true);                      // Enable the PWM interrupt

    pwm_config config = pwm_get_default_config();
    pwm_config_set_clkdiv_int(&config, 133);                 // Set clock divider to achieve 1 MHz PWM frequency (assuming 133 MHz system clock)
    pwm_config_set_wrap(&config, HALF_BIT_INTERVAL_US - 1);  // Set wrap value for the desired half-bit interval
    pwm_init(sliceNum, &config, false);
}

void setup() {
    Serial.begin(115200);
    Serial1.begin(115200);

    pinMode(DATA_PIN, INPUT_PULLUP);
    pinMode(HALF_BIT_RECEIVED_PIN, OUTPUT);

    digitalWriteFast(HALF_BIT_RECEIVED_PIN, LOW);

    setupPWM_ISR();
}

void loop() {
    if (byteReadyForProcessing) {
        byteReadyForProcessing = false;

        uint8_t bufferToRead = 1 - activeBuffer;
        processReceivedByte(bufferToRead);
    }

    if (readyToDrawBuffer && terminationByteReceived) {
        readyToDrawBuffer = false;
        terminationByteReceived = false;
        sendDrawBufferToUART();
    }
}