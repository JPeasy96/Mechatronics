
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>
#include "ti/devices/msp432p4xx/inc/msp.h"

/* Standard Includes */
#include <stdint.h>
#include <stdbool.h>

#include "printf.h"
// project must contain, at the same level as this main.c file,
// the printf.c and printf.h files from the online library;
// they are available on canvas.


// STATE VARIABLE DECLARATION (TODO: will need additional content)
// state for the double command
volatile bool handlingDoubleCommand;
volatile int16_t parsedValue;
volatile bool debounce_enabled;
volatile bool red_led_status;
volatile char rgb_led_status;

// halts the watchdog timer
void disableWDT(){
    WDT_A_holdTimer();
}

// Configures the given pins on the given port as LEDs, initializing all of them to Low.
void configLED(uint_fast8_t port, uint_fast8_t pin){
    GPIO_setAsOutputPin(port, pin);
    GPIO_setOutputLowOnPin(port, pin);
}

void configButton(uint_fast8_t port, uint_fast8_t pin, uint32_t interruptNumber){
    /* Clears interrupt flags and enable interrupts on the selected button */
    MAP_GPIO_clearInterruptFlag(port, pin);
    MAP_GPIO_enableInterrupt(port, pin);
    MAP_Interrupt_enableInterrupt(interruptNumber);
    MAP_GPIO_setAsInputPinWithPullUpResistor(port, pin);
    MAP_GPIO_interruptEdgeSelect(port, pin, GPIO_HIGH_TO_LOW_TRANSITION);
}

// Configures Timer_A0 for PWM generation on CCRS 1, 2, and 3.
// Timer_A0's Period is 256, duty cycle of given CCR is TIMER_A0->CCR[n]/256.
void configTimerA0for3xPWM(){
    //    TIMER_A0->CTL = 0x0110;
        TIMER_A0->CTL = TIMER_A_CTL_SSEL__ACLK // Sources Timer_A0 from the A Clock ...
                            | TIMER_A_CTL_ID__1 // ... with a divider of 1 ...
                            | TIMER_A_CTL_MC__UP; // accumulating in Up mode (ccr0 stores period)
        TIMER_A0->CCR[0] = 256; // Sets period to 256

        //Set CCRs for Compare mode with Toggle/Set output mode
        TIMER_A0->CCTL[1] = TIMER_A_CCTLN_OUTMOD_6;
        TIMER_A0->CCTL[2] = TIMER_A_CCTLN_OUTMOD_6;
        TIMER_A0->CCTL[3] = TIMER_A_CCTLN_OUTMOD_6;

        // Initialize CCRs so it starts in reliable state (50% duty cycle chosen for testing's sake)
        TIMER_A0->CCR[1]  = 127; //RED
        TIMER_A0->CCR[2]  = 127; //GREEN
        TIMER_A0->CCR[3]  = 127; //BLUE
}

void configTimer32forDebouncer(uint32_t timer, uint32_t interruptNumber, uint32_t initialValue){
    // Configures the Timer32 module for debouncing purposes
    Timer32_initModule(timer, TIMER32_PRESCALER_1, TIMER32_32BIT, TIMER32_FREE_RUN_MODE);
    Timer32_clearInterruptFlag(interruptNumber);
    Interrupt_enableInterrupt(interruptNumber);
    Timer32_setCount(timer, initialValue);
}

void configTimer32forStopwatch(uint32_t timer, uint32_t interruptNumber, uint32_t initialValue){
    //Configures the Timer32 module to act as a stopwatch
    Timer32_initModule(timer, TIMER32_PRESCALER_256, TIMER32_32BIT, TIMER32_FREE_RUN_MODE);
    Timer32_clearInterruptFlag(interruptNumber);
    Interrupt_enableInterrupt(interruptNumber);
    Timer32_setCount(timer, initialValue);
}

/* UART Configuration Parameter. These are the configuration parameters to
 * make the eUSCI A UART module to operate with a 9600 baud rate. These
 * values were calculated using the online calculator that TI provides
 * at:
 *http://software-dl.ti.com/msp430/msp430_public_sw/mcu/msp430/MSP430BaudRateConverter/index.html
 */
const eUSCI_UART_Config uartConfig =
{
        EUSCI_A_UART_CLOCKSOURCE_SMCLK,          // SMCLK Clock Source
        78,                                      // BRDIV = 78
        2,                                       // UCxBRF = 2
        0,                                       // UCxBRS = 0
        EUSCI_A_UART_NO_PARITY,                  // No Parity
        EUSCI_A_UART_LSB_FIRST,                  // LSB First
        EUSCI_A_UART_ONE_STOP_BIT,               // One stop bit
        EUSCI_A_UART_MODE,                       // UART mode
        EUSCI_A_UART_OVERSAMPLING_BAUDRATE_GENERATION  // Oversampling
};

// initializes EUSCI_A0 for UART (with pins 1.2 & 1.3, and the associated receive interrupt)
// doesnt enable transmit interrupt anticipating the use of the printf library from online, which busy-waits.
void configUART(){
    UART_initModule(EUSCI_A0_BASE, &uartConfig);

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
                GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    UART_enableModule(EUSCI_A0_BASE);

    UART_enableInterrupt(EUSCI_A0_BASE, EUSCI_A_UART_RECEIVE_INTERRUPT);
    Interrupt_enableInterrupt(INT_EUSCIA0);
}

// Duplicating UART lines for EUSCI_A0 for Logic Analyzer
const uint8_t port1_mapping[] =
{
        PMAP_NONE,      PMAP_NONE,      PMAP_UCA0RXD,      PMAP_UCA0TXD,
        PMAP_NONE,      PMAP_NONE,      PMAP_NONE,   PMAP_NONE
};
// output both the PWM to the RGB LED and the EUSCI_A0 TX line for the Logic Analyzer
const uint8_t port2_mapping[] =
{
        //Port P2:
        PMAP_TA0CCR1A,  PMAP_TA0CCR2A,  PMAP_TA0CCR3A,      PMAP_NONE,
        PMAP_NONE,      PMAP_NONE,      PMAP_NONE,   PMAP_UCA0TXD
};

// Duplicates EUSCI_A0 TX line onto P2.7  while leaving P1.2 and P1.3 intact
// Also configures the RGB LED to be driven by the TimerA0 CCRs 1, 2, 3
void duplicateUARTOutputAndRedirectPWM(){
    PMAP_configurePorts(port1_mapping, PMAP_P1MAP, 2,
                        PMAP_ENABLE_RECONFIGURATION);

    PMAP_configurePorts(port2_mapping, PMAP_P2MAP, 4,
                PMAP_DISABLE_RECONFIGURATION);

    /* Selecting P1.2 and P1.3, also 2.7 in UART mode */
    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P1,
            GPIO_PIN2 | GPIO_PIN3, GPIO_PRIMARY_MODULE_FUNCTION);

    GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P2,
                GPIO_PIN7, GPIO_PRIMARY_MODULE_FUNCTION);

    // Selecting P2.0, 2.1, 2.2 in TimerA0 CCR output mode (PWM):
    GPIO_setAsPeripheralModuleFunctionOutputPin(GPIO_PORT_P2,
                GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2, GPIO_PRIMARY_MODULE_FUNCTION);
}

int main(void)
{
    disableWDT();

    // State Variable Initialization: (TODO: will need additional content)
    handlingDoubleCommand = 0; //Not currently handling double command
    red_led_status = 0; //Stopwatch is not running
    rgb_led_status = 'r'; //RGB led initially set to RED
    parsedValue = 0; //Not currently parsing values
    debounce_enabled = 0;



    // System Configuration: (TODO: will need additional content)
    CS_setDCOFrequency(12000000); //12MHz DCO & CPU
    CS_initClockSignal(CS_SMCLK, CS_DCOCLK_SELECT, CS_CLOCK_DIVIDER_1); //12Mhz SM Clock from DCO
    CS_setReferenceOscillatorFrequency(CS_REFO_128KHZ); //128kHz for REF0
    CS_initClockSignal(CS_ACLK, CS_REFOCLK_SELECT, CS_CLOCK_DIVIDER_1); // 128kHz for AClk, from REF0

    configLED(GPIO_PORT_P1, GPIO_PIN0);
    configLED(GPIO_PORT_P2, GPIO_PIN0 | GPIO_PIN1 | GPIO_PIN2);

    configButton(GPIO_PORT_P1, GPIO_PIN1 | GPIO_PIN4, INT_PORT1); //Configure buttons P1.1 and P1.4 to interrupt Port 1

    configTimerA0for3xPWM();
    configTimer32forDebouncer(TIMER32_0_BASE, TIMER32_0_INTERRUPT, 12000);
    configTimer32forStopwatch(TIMER32_1_BASE, TIMER32_1_INTERRUPT, UINT32_MAX);

    configUART();
    duplicateUARTOutputAndRedirectPWM();

    Interrupt_enableMaster();

    while(1)
    {
        MAP_PCM_gotoLPM0();
    }
}

/* EUSCI A0 UART ISR*/
void EUSCIA0_IRQHandler(void)
{
    uint32_t status = UART_getEnabledInterruptStatus(EUSCI_A0_BASE);
    UART_clearInterruptFlag(EUSCI_A0_BASE, status);

    if(status & EUSCI_A_UART_RECEIVE_INTERRUPT_FLAG)
    {
        volatile uint8_t readdata;
        readdata = UART_receiveData(EUSCI_A0_BASE);

        uint32_t ticks_passed;
        uint32_t time_passed;

        if (handlingDoubleCommand == 0) {
            //Switch statement to interpret data input
            switch (readdata) {
                case 'r' :
                    rgb_led_status = 'r'; // Prepare for red PWM value
                    handlingDoubleCommand = 1;
                    break;
                case 'g' :
                    rgb_led_status = 'g'; // Prepare for green PWM value
                    handlingDoubleCommand = 1;
                    break;
                case 'b' :
                    rgb_led_status = 'b'; // Prepare for blue PWM value
                    handlingDoubleCommand = 1;
                    break;
                case '!' :
                    Timer32_haltTimer(TIMER32_1_BASE); // Stop timer
                    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
                    ticks_passed = UINT32_MAX - Timer32_getValue(TIMER32_1_BASE);
                    time_passed = ticks_passed*256/12000000;
                    UART_transmitData(EUSCI_A0_BASE, time_passed); // Log previous time via UART
                    Timer32_setCount(TIMER32_1_BASE, UINT32_MAX); // Reset timer to initial value
                    break;
                case 's' :
                    // Start and stop timer accordingly
                    if (red_led_status == 0) {
                        red_led_status = 1;
                        MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
                        Timer32_startTimer(TIMER32_1_BASE, 1);
                        break;
                    } else if (red_led_status == 1) {
                        red_led_status = 0;
                        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
                        Timer32_haltTimer(TIMER32_1_BASE);
                        break;
                    }
                case 'p' :
                    ticks_passed = UINT32_MAX - Timer32_getValue(TIMER32_1_BASE);
                    time_passed = ticks_passed*256/12000000;
                    UART_transmitData(EUSCI_A0_BASE, time_passed); // Log current stopwatch time without disturbing timer
                    break;
                default :
                    printf(EUSCI_A0_BASE, "%c?\n", readdata);
                }
            }

            if (handlingDoubleCommand == 1) {
                // Parse PWM value and power LEDs accordingly
                parsedValue = UART_receiveData(EUSCI_A0_BASE);

                switch (rgb_led_status) {
                    case 'r' :
                        TIMER_A0->CCR[1] = parsedValue;
                        handlingDoubleCommand = 0;
                        break;
                    case 'g' :
                        TIMER_A0->CCR[2] = parsedValue;
                        handlingDoubleCommand = 0;
                        break;
                    case 'b' :
                        TIMER_A0->CCR[3] = parsedValue;
                        handlingDoubleCommand = 0;
                        break;
                }
            }
    }

}

void PORT1_IRQHandler(void) {

    uint32_t status = MAP_GPIO_getEnabledInterruptStatus(GPIO_PORT_P1);
    MAP_GPIO_clearInterruptFlag(GPIO_PORT_P1, status);

    if (debounce_enabled == 0) {
        if (status & GPIO_PIN1) {
        // Switch statement for when P1.1 is pushed
        // Start or stop stopwatch accordingly
        // Begin debounce timer

            debounce_enabled = 1;
            Timer32_setCount(TIMER32_0_BASE, 12000);
            Timer32_startTimer(TIMER32_0_BASE, 1);

            switch (red_led_status) {
                case 0:
                    red_led_status = 1;
                    MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);
                    Timer32_startTimer(TIMER32_1_BASE, 1);
                    break;
                case 1:
                    red_led_status = 0;
                    MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);
                    Timer32_haltTimer(TIMER32_1_BASE);
                    break;
            }
        } else if (status & GPIO_PIN4) {
            //Communicate time to console via UART without disturbing the stopwatch
            // Begin debounce timer

            debounce_enabled = 1;
            Timer32_setCount(TIMER32_0_BASE, 12000);
            Timer32_startTimer(TIMER32_0_BASE, 1);

            uint32_t ticks_passed = UINT32_MAX - Timer32_getValue(TIMER32_1_BASE);
            uint32_t time_passed = ticks_passed*256/12000000;
            UART_transmitData(EUSCI_A0_BASE, time_passed);
        }
    }
}

void T32_INT0_IRQHandler(void)
{
    // Clear debounce flag to allow button presses again
    debounce_enabled = 0;
    Timer32_clearInterruptFlag(TIMER32_0_BASE);
}

void T32_INT1_IRQHandler(void)
{
    // Clear interrupt flag if/when stopwatch runs out

    Timer32_clearInterruptFlag(TIMER32_1_BASE);
}
