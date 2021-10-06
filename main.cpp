#include "mbed.h"
#include "common.h"
#include "bdm.h"
#include "can232.h"
#include "t8can.h"
#include <cstdint>
#include "combiadapter.h"
#include "interfaces.h"

// constants
#define CMD_BUF_LENGTH      32              ///< command buffer size

// static variables
static char cmd_buffer[CMD_BUF_LENGTH];     ///< command string buffer

// private functions
uint8_t execute_just4trionic_cmd();
void show_just4trionic_help();

Thread combiThread;

int main()
{
    // fast serial speed
    //pc.baud(921600);
    pc.baud(115200);
    // the address of the function to be attached (leds_off) and the interval (0.1 seconds)
    // This 'ticker' turns off the activity LEDs so that they don't stay on if something has gone wrong
    ticker.attach(&leds_off, 0.1);
    
    combiThread.start(combi_thread);
    // clear incoming buffer
    // sometimes TeraTerm gets 'confused'. johnc does this in his code
    // hopefully this will fix the problem
    // unfortunately it doesn't, but it seems like a good idea
    char rx_char;
    while (pc.readable())
        rx_char = pc.getc();

    show_just4trionic_help();

    // main loop
    *cmd_buffer = '\0';
    char ret;
    while (true) {
        if (pc.readable()) {
            // turn Error LED off for next command
            led4 = 0;
            rx_char = pc.getc();
            switch (rx_char) {
                    // end-of-command reached
                case TERM_OK :
                    // execute command and return flag via USB
                    timer.reset();
                    timer.start();
                    ret = execute_just4trionic_cmd();
                    show_just4trionic_help();
                    pc.putc(ret);
                    // reset command buffer
                    *cmd_buffer = '\0';
                    // light up LED
                    ret == TERM_OK ? led3 = 1 : led4 = 1;
                    break;
                    // another command char
                default:
                    // store in buffer if space permits
                    if (StrLen(cmd_buffer) < CMD_BUF_LENGTH - 1) {
                        StrAddc(cmd_buffer, rx_char);
                    }
                    break;
            }
        }
    }
}

//-----------------------------------------------------------------------------
/**
    Executes a command and returns result flag (does not transmit the flag
    itself).

    @return                    command flag (success / failure)
*/
uint8_t execute_just4trionic_cmd()
{
    switch (*cmd_buffer) {
        case 'b':
        case 'B':
            bdm();
            return TERM_OK;
        case 'o':
        case 'O':
            can232();
            return TERM_OK;
        case '8':
            t8_can();
            return TERM_OK;
        case 't':
            sensor.printTemp();
            return TERM_OK;
        case 'h':
        case 'H':
            return TERM_OK;
        default:
            break;
    }

// unknown command
    return TERM_ERR;
}

void show_just4trionic_help()
{
#ifdef DEBUG
    printf("*************************\r\n");
    printf("** D E B U G B U I L D **\r\n");
    printf("*************************\r\n");
#endif
    printf("=========================\r\n");
    printf("Just4Trionic Release %d.%d\r\n", FW_VERSION_MAJOR, FW_VERSION_MINOR);
    printf("=========================\r\n");
    printf("Modes Menu\r\n");
    printf("=========================\r\n");
    printf("b/B - Enter BDM mode\r\n");
    printf("o/O - Enter Lawicel CAN mode\r\n");
    printf("8   - Enter Trionic8 CAN mode\r\n");
    printf("t   - MAX6675 Thermocouple\r\n");
    printf("\r\n");
    printf("h/H - show this help menu\r\n");
    printf("\r\n");
    return;
}