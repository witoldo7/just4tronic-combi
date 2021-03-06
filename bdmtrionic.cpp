/*******************************************************************************

bdmtrionic.cpp
(c) 2010 by Sophie Dexter

General purpose BDM functions for Just4Trionic by Just4pLeisure

A derivative work based on:
//-----------------------------------------------------------------------------
//    CAN/BDM adapter firmware
//    (C) Janis Silins, 2010
//    $id$
//-----------------------------------------------------------------------------

********************************************************************************

WARNING: Use at your own risk, sadly this software comes with no guarantees.
This software is provided 'free' and in good faith, but the author does not
accept liability for any damage arising from its use.

*******************************************************************************/

#include "interfaces.h"
#include "bdmcpu32.h"
#include "bdmdriver.h"
#include "bdmtrionic.h"

// structure for command address/value pairs
struct mempair_t {
    uint32_t addr;            ///< target address
    uint16_t val;            ///< word value
};

// word write algorithm (29Fxxx)
static const struct mempair_t am29_write [] = {
    {0xaaaa, 0xaaaa}, {0x5554, 0x5555}, {0xaaaa, 0xa0a0},
};

// chip erase algorithms
static const struct mempair_t am29_erase [] = {
    {0xaaaa, 0xaaaa}, {0x5554, 0x5555}, {0xaaaa, 0x8080},
    {0xaaaa, 0xaaaa}, {0x5554, 0x5555}, {0xaaaa, 0x1010}
};

// reset algorithms
//static const struct mempair_t am29_reset = {0xfffe, 0xf0f0};
static const struct mempair_t am29_reset [] = {
    {0xaaaa, 0xaaaa}, {0x5554, 0x5555}, {0xaaaa, 0xf0f0},
};

// chip id algorithms
static const struct mempair_t am29_id [] = {
    {0xaaaa, 0xaaaa}, {0x5554, 0x5555}, {0xaaaa, 0x9090},
};

// ;-)
static const struct mempair_t flash_tag [] = {
    {0x7fe00, 0xFF4A}, {0x7fe02, 0x7573}, {0x7fe04, 0x7434}, {0x7fe06, 0x704C},
    {0x7fe08, 0x6569}, {0x7fe0a, 0x7375}, {0x7fe0c, 0x7265}, {0x7fe0e, 0x3B29},
};

// local functions
bool erase_am29();
bool erase_am28(const uint32_t* start_addr, const uint32_t* end_addr);
bool get_flash_id(uint8_t* make, uint8_t* type);

bool run_bdm_driver(uint32_t addr, uint32_t maxtime);

//-----------------------------------------------------------------------------
/**
    Dumps contents of a memory block from [start_addr] up to, but not including,
    the [end_addr] as long words (word-aligned addresses). MCU must be in
    background mode. The operation interrupts if the break character is
    received.

    @param        start_addr        block start address
    @param        end_addr        block end address

    @return                        status flag
*/


uint8_t dump_flash(const uint32_t* start_addr, const uint32_t* end_addr)
{

    // check parametres
    if (*start_addr > *end_addr) {
        return TERM_ERR;
    }

    // dump memory contents
    uint32_t curr_addr = *start_addr;
    uint32_t value;

    while ((curr_addr < *end_addr) && (pc.getc() != TERM_BREAK)) {
        // read long word
        if (curr_addr > *start_addr) {
            if (memdump_long(&value) != TERM_OK) {
                return TERM_ERR;
            }
        } else {
            if (memread_long(&value, &curr_addr) != TERM_OK) {
                return TERM_ERR;
            }
        }

        // send memory value to host
        printf("%08lX", value);
        printf("\r\n");

        // add the terminating character
        if (curr_addr < *end_addr - 4) {
            pc.putc(TERM_OK);
            // light up the activity LED
            ACTIVITYLEDON;
        }

        curr_addr += 4;
    }

    return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
    Dumps the contents of a T5 ECU to a BIN file on the mbed 'disk'
    from [start_addr] up to, but not including, the [end_addr].
    MCU must be in background mode.

    @param        start_addr        block start address
    @param        end_addr        block end address

    @return                        status flag
*/

uint8_t dump_trionic()
{

    // Configure the MC68332 register values to prepare for flashing
    printf("I am trying to discover what type of Trionic ECU I am connected to...\r\n");
    prep_t5_do();
    // Work out what type of FLASH chips we want to make a dump file for
    uint8_t make;
    uint8_t type;
    get_flash_id(&make, &type);
    // set up chip-specific functions
    bool (*reset_func)();
    uint32_t flash_size;

    switch (type) {
        case AMD29BL802C:
            printf("I have found AMD29BL802C type FLASH chips; I must be connected to a T8 ECU :-)\r\n");
            reset_func = &reset_am29;
            flash_size = T8FLASHSIZE;
            break;
        case AMD29F400B:
        case AMD29F400T:
            printf("I have found AMD29F400 type FLASH chips; I must be connected to a T7 ECU :-)\r\n");
            reset_func = &reset_am29;
            flash_size = T7FLASHSIZE;
            break;
        case AMD29F010:
        case SST39SF010:
        case AMICA29010L:
            printf("I have found 29/39F010 type FLASH chips; I must be connected to a repaired T5.5 ECU :-)\r\n");
            reset_func = &reset_am29;
            flash_size = T55FLASHSIZE;
            break;
        case ATMEL29C010:
            printf("I have found Atmel 29C010 type FLASH chips; I must be connected to a repaired T5.5 ECU :-)\r\n");
            reset_func = &reset_am29;
            flash_size = T55FLASHSIZE;
            break;
        case AMD28F010:
        case INTEL28F010:
            printf("I have found 28F010 type FLASH chips; I must be connected to a T5.5 ECU :-)\r\n");
            reset_func = &reset_am28;
            flash_size = T55FLASHSIZE;
            break;
        case AMD28F512:
        case INTEL28F512:
            printf("I have found 28F512 type FLASH chips; I must be connected to a T5.2 ECU :-)\r\n");
            reset_func = &reset_am28;
            flash_size = T52FLASHSIZE;
            break;
        case ATMEL29C512:
            printf("I have found Atmel 29C512 type FLASH chips; I must be connected to a repaired T5.2 ECU :-)\r\n");
            reset_func = &reset_am28;
            flash_size = T52FLASHSIZE;
            break;
        default:
            // unknown flash type
            printf("I could not work out what FLASH chips or TRIONIC ECU I am connected to :-(\r\n");
            return TERM_ERR;
    }

    // reset the FLASH chips
    if (!reset_func()) return TERM_ERR;

    printf("Creating FLASH dump file...\r\n");
    FILE *fp = fopen("/local/original.bin", "w");    // Open "original.bin" on the local file system for writing
    if (!fp) {
        perror ("The following error occured");
        return TERM_ERR;
    }

// dump memory contents
    uint32_t addr = 0x00;
    uint32_t long_value;

// setup start address to dump from
    if (memread_long_cmd(&addr) != TERM_OK) return TERM_ERR;

    timer.reset();
    timer.start();
    printf("  0.00 %% complete.\r");
    while (addr < flash_size) {
        uint16_t byte_count = 0;
        while (byte_count < FILE_BUF_LENGTH) {
            // get long word
            if (memget_long(&long_value) != TERM_OK) return TERM_ERR;
            // send memory value to file_buffer before saving to mbed 'disk'
            file_buffer[byte_count++] = ((uint8_t)(long_value >> 24));
            file_buffer[byte_count++] = ((uint8_t)(long_value >> 16));
            file_buffer[byte_count++] = ((uint8_t)(long_value >> 8));
            file_buffer[byte_count++] = ((uint8_t)long_value);
        }
        fwrite(file_buffer, 1, FILE_BUF_LENGTH, fp);
        if (ferror (fp)) {
            fclose (fp);
            printf ("Error writing to the FLASH BIN file.\r\n");
            return TERM_ERR;
        }
        printf("%6.2f\r", 100*(float)addr/(float)flash_size );
        // make the activity led twinkle
        ACTIVITYLEDON;
        addr += FILE_BUF_LENGTH;
    }
    printf("100.00\r\n");
    // should 'clear' the BDM connection here but bdm_clear won't compile from here
    // instead do a memread (or anything really) but ignore the result because it's not needed for anything
    memread_long(&long_value, &addr);
    timer.stop();
    printf("Getting the FLASH dump took %#.1f seconds.\r\n",timer.read());
    fclose(fp);
    return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
    Erases the flash memory chip starting from [start_addr] up to, but not
    including [end_addr] and optionally verifies the result; MCU must be in
    background mode.

    @param        flash_type        type of flash chip
    @param        start_addr        flash start address
    @param        end_addr        flash end address

    @return                        status flag
*/
uint8_t erase_flash(const char* flash_type, const uint32_t* start_addr,
                    const uint32_t* end_addr)
{
    // AM29Fxxx chips (retrofitted to Trionic 5.x; original to T7)
    if (strncmp(flash_type, "29f010", 6) == 0 ||
            strncmp(flash_type, "29f400", 6) == 0) {
        return erase_am29() ? TERM_OK : TERM_ERR;
    }

    // AM28F010 chip (Trionic 5.x original)
    if (strncmp(flash_type, "28f010", 6) == 0) {
        return erase_am28(start_addr, end_addr) ? TERM_OK : TERM_ERR;
    }

    return TERM_ERR;
}

//-----------------------------------------------------------------------------
/**
    Writes a batch of long words to the flash starting from [start_addr]. The
    operation interrupts if a break character is received. MCU must be in
    background mode.

    @param        flash_type        type of flash chip
    @param        start_addr        block start address

    @return                        status flag
*/
uint8_t write_flash(const char* flash_type, const uint32_t* start_addr)
{
    // set up chip-specific functions
    bool (*reset_func)(void);
    bool (*flash_func)(const uint32_t*, uint16_t);

    // AM29Fxxx chips (retrofitted to Trionic 5.x, original to T7)
    if (strncmp(flash_type, "29f010", 6) == 0 ||
            strncmp(flash_type, "29f400", 6) == 0) {
        reset_func = &reset_am29;
        flash_func = &flash_am29;
    } else if (strncmp(flash_type, "28f010", 6) == 0) {
        // AM28F010 chip (Trionic 5.x original)
        reset_func = &reset_am28;
        flash_func = &flash_am28;
    } else {
        // unknown flash type
        return TERM_ERR;
    }

    // reset the flash
    if (!reset_func()) {
        return TERM_ERR;
    }

    uint32_t curr_addr = *start_addr;
    if (strncmp(flash_type, "29f010", 6) == 0) {
        curr_addr = 0;
    }

    int rx_char = 0;
    char rx_buf[8];
    char* rx_ptr;
    uint32_t long_value;
    bool ret = true;

    // ready to receive data
    pc.putc(TERM_OK);

    while (true) {
        // receive long words from USB
        printf("receive long words from USB\r\n");
        rx_ptr = rx_buf;
        do {
            rx_char = pc.getc();
            if (rx_char != EOF) {
                // have got all characters for one long word
                if (rx_ptr > &rx_buf[7]) {
                    ret = (rx_char == TERM_OK);
                    break;
                }

                // save the character
                *rx_ptr++ = (char)rx_char;
            }
        } while (rx_char != TERM_OK && rx_char != TERM_BREAK);
        // end writing
        printf("end writing\r\n");
        if (!ret || rx_char == TERM_BREAK) {
            break;
        }

        // convert value to long word
        printf("convert value to long word\r\n");
        if (!ascii2int(&long_value, rx_buf, 8)) {
            ret = false;
            break;
        }
        printf("long value %08lx \r\n", long_value);

        // write the first word
        printf("write the first word\r\n");
        if (!flash_func(&curr_addr, (uint16_t)(long_value >> 16))) {
            ret = false;
            break;
        }
        curr_addr += 2;
        // write the second word
        printf("write the second word\r\n");
        if (!flash_func(&curr_addr, (uint16_t)long_value)) {
            ret = false;
            break;
        }
        curr_addr += 2;

        // light up the activity LED
        ACTIVITYLEDON;
    }

    // reset flash
    return (reset_func() && ret) ? TERM_OK : TERM_ERR;
}

//-----------------------------------------------------------------------------
/**
    Writes a BIN file to the flash starting from [start_addr].
    The operation ends when no more bytes can be read from the BIN file.
    MCU must be in background mode.

    @param        flash_type        type of flash chip
    @param        start_addr        block start address

    @return                        status flag
*/
uint8_t flash_trionic()
{
    // Configure the MC68332 register values to prepare for flashing
    printf("I am trying to discover what type of Trionic ECU I am connected to...\r\n");
    prep_t5_do();
    // Work out what type of FLASH chips we want to program
    uint8_t make;
    uint8_t type;
    get_flash_id(&make, &type);
    // set up chip-specific functions
    bool (*reset_func)();
    bool (*flash_func)(const uint32_t*, uint16_t);
    uint32_t flash_size;

    switch (type) {
        case AMD29BL802C:
            printf("I have found AMD29BL802C type FLASH chips; I must be connected to a T8 ECU :-)\r\n");
            reset_func = &reset_am29;
            flash_func = &flash_am29;
            flash_size = T8FLASHSIZE;
            break;
        case AMD29F400B:
        case AMD29F400T:
            printf("I have found AMD29F400 type FLASH chips; I must be connected to a T7 ECU :-)\r\n");
            reset_func = &reset_am29;
            flash_func = &flash_am29;
            flash_size = T7FLASHSIZE;
            break;
        case AMD29F010:
        case SST39SF010:
        case AMICA29010L:
            printf("I have found 29/39F010 type FLASH chips; I must be connected to a repaired T5.5 ECU :-)\r\n");
            reset_func = &reset_am29;
            flash_func = &flash_am29;
            flash_size = T55FLASHSIZE;
            break;
        case ATMEL29C010:
            printf("I have found Atmel 29C010 type FLASH chips; I must be connected to a repaired T5.5 ECU :-)\r\n");
            reset_func = &reset_am29;
            flash_func = NULL;
            flash_size = T55FLASHSIZE;
            break;
        case AMD28F010:
        case INTEL28F010:
            printf("I have found 28F010 type FLASH chips; I must be connected to a T5.5 ECU :-)\r\n");
            reset_func = &reset_am28;
            flash_func = &flash_am28;
            flash_size = T55FLASHSIZE;
            break;
        case AMD28F512:
        case INTEL28F512:
            printf("I have found 28F512 type FLASH chips; I must be connected to a T5.2 ECU :-)\r\n");
            reset_func = &reset_am28;
            flash_func = &flash_am28;
            flash_size = T52FLASHSIZE;
            break;
        case ATMEL29C512:
            printf("I have found Atmel 29C512 type FLASH chips; I must be connected to a repaired T5.2 ECU :-)\r\n");
            reset_func = &reset_am29;
            flash_func = NULL;
            flash_size = T52FLASHSIZE;
            break;
        default:
            // unknown flash type
            printf("I could not work out what FLASH chips or TRIONIC ECU I am connected to :-(\r\n");
            return TERM_ERR;
    }
    // reset the FLASH chips
    if (!reset_func()) return TERM_ERR;


    printf("Checking the FLASH BIN file...\r\n");
    FILE *fp = fopen("/local/modified.bin", "r");    // Open "modified.bin" on the local file system for reading
//    FILE *fp = fopen("/local/original.bin", "r");    // Open "original.bin" on the local file system for reading
    if (!fp) {
        printf("Error: I could not find the BIN file MODIFIED.BIN\r\n");;
        return TERM_ERR;
    }
    // obtain file size - it should match the size of the FLASH chips:
    fseek (fp , 0 , SEEK_END);
    uint32_t file_size = ftell (fp);
    rewind (fp);

    // read the initial stack pointer value in the BIN file - it should match the value expected for the type of ECU
    uint8_t stack_bytes[4] = {0, 0, 0, 0};
    uint32_t stack_long = 0;
    if (!fread(&stack_bytes[0],1,4,fp)) return TERM_ERR;
    rewind (fp);
    for(uint32_t i=0; i<4; i++) {
        (stack_long <<= 8) |= stack_bytes[i];
    }

    if (flash_size == T52FLASHSIZE && (file_size != T52FLASHSIZE || stack_long != T5POINTER)) {
        fclose(fp);
        printf("The BIN file does not appear to be for a T5.2 ECU :-(\r\n");
        printf("BIN file size: %#10lx, FLASH chip size: %#010lx, Pointer: %#10lx.\r\n", file_size, flash_size, stack_long);
        return TERM_ERR;
    }
    if (flash_size == T55FLASHSIZE && (file_size != T55FLASHSIZE || stack_long != T5POINTER)) {
        fclose(fp);
        printf("The BIN file does not appear to be for a T5.5 ECU :-(\r\n");
        printf("BIN file size: %#10lx, FLASH chip size: %#010lx, Pointer: %#10lx.\r\n", file_size, flash_size, stack_long);
        return TERM_ERR;
    }
    if (flash_size == T7FLASHSIZE && (file_size != T7FLASHSIZE || stack_long != T7POINTER)) {
        fclose(fp);
        printf("The BIN file does not appear to be for a T7 ECU :-(\r\n");
        printf("BIN file size: %#10lx, FLASH chip size: %#010lx, Pointer: %#10lx.\r\n", file_size, flash_size, stack_long);
        return TERM_ERR;
    }
    if (flash_size == T8FLASHSIZE && (file_size != T8FLASHSIZE || stack_long != T8POINTER)) {
        fclose(fp);
        printf("The BIN file does not appear to be for a T8 ECU :-(\r\n");
        printf("BIN file size: %#10lx, FLASH chip size: %#010lx, Pointer: %#10lx.\r\n", file_size, flash_size, stack_long);
        return TERM_ERR;
    }

    uint32_t curr_addr = 0;

    switch (type) {
        case AMD29BL802C:
        case AMD29F400T:
        case AMD29F010:
        case SST39SF010:
        case AMICA29010L:
        case ATMEL29C010:
        case AMD28F010:
        case INTEL28F010:
        case AMD28F512:
        case INTEL28F512:
        case ATMEL29C512: {
            uint8_t flashDriver[] = {\
                                     0x60,0x00,0x04,0x0C,\
                                     0x7C,0x2F,0x2D,0x5C,0x2A,0x0D,0x00,0x00,\
                                     0x02,0x03,0x00,0x03,0x41,0xFA,0xFF,0xF6,\
                                     0x10,0xBB,0x30,0xEE,0x70,0x01,0x4A,0xFA,\
                                     0x52,0x43,0x4E,0x75,\
                                     0x20,0x7C,0x00,0xFF,0xFA,0x00,0x08,0x10,\
                                     0x00,0x04,0x66,0x4E,0xD0,0xFC,0x00,0x04,\
                                     0x10,0xFC,0x00,0x7F,0x08,0x10,0x00,0x03,\
                                     0x67,0xFA,0xD0,0xFC,0x00,0x1C,0x42,0x10,\
                                     0xD0,0xFC,0x00,0x23,0x30,0xBC,0x3F,0xFF,\
                                     0xD0,0xFC,0x00,0x04,0x70,0x07,0x30,0xC0,\
                                     0x30,0xBC,0x68,0x70,0xD0,0xFC,0x00,0x06,\
                                     0x30,0xC0,0x30,0xFC,0x30,0x30,0x30,0xC0,\
                                     0x30,0xBC,0x50,0x30,0xD0,0xFC,0x01,0xBE,\
                                     0x70,0x40,0x30,0xC0,0x30,0x80,0x30,0x3C,\
                                     0x55,0xF0,0x4E,0x71,0x51,0xC8,0xFF,0xFC,\
                                     0x60,0x18,0xD0,0xFC,0x00,0x08,0x30,0xFC,\
                                     0x69,0x08,0x08,0x10,0x00,0x09,0x67,0xFA,\
                                     0x31,0x3C,0x68,0x08,0xD0,0xFC,0x00,0x48,\
                                     0x42,0x50,0x4E,0x75,\
                                     0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,\
                                     0x00,0x00,0x2C,0x3C,0x00,0x00,0x55,0x55,\
                                     0x2E,0x3C,0x00,0x00,0xAA,0xAA,0x2A,0x46,\
                                     0x53,0x8D,0x2C,0x47,0x45,0xF8,0x00,0x00,\
                                     0x47,0xFA,0xFF,0xDE,0x3C,0x87,0x3A,0x86,\
                                     0x3C,0xBC,0x90,0x90,0x36,0xDA,0x36,0x92,\
                                     0x3C,0x87,0x3A,0x86,0x3C,0xBC,0xF0,0xF0,\
                                     0x20,0x3A,0xFF,0xC6,0x72,0x02,0x48,0x41,\
                                     0x74,0x01,0x0C,0x00,0x00,0x25,0x67,0x50,\
                                     0x0C,0x00,0x00,0xB8,0x67,0x4A,0x74,0x04,\
                                     0x0C,0x00,0x00,0x5D,0x67,0x42,0x74,0x01,\
                                     0xE3,0x99,0x0C,0x00,0x00,0xA7,0x67,0x38,\
                                     0x0C,0x00,0x00,0xB4,0x67,0x32,0x74,0x02,\
                                     0x0C,0x00,0x00,0x20,0x67,0x2A,0x0C,0x00,\
                                     0x00,0xA4,0x67,0x24,0x0C,0x00,0x00,0xB5,\
                                     0x67,0x1E,0x74,0x04,0x0C,0x00,0x00,0xD5,\
                                     0x67,0x16,0x74,0x03,0xE3,0x99,0x0C,0x00,\
                                     0x00,0x23,0x67,0x0C,0xE3,0x99,0x0C,0x00,\
                                     0x00,0x81,0x67,0x04,0x72,0x00,0x74,0x00,\
                                     0x47,0xFA,0xFF,0x6A,0x26,0x81,0x47,0xFA,\
                                     0xFF,0x68,0x16,0x82,0x4E,0x75,\
                                     0x45,0x72,0x61,0x73,0x69,0x6E,0x67,0x20,\
                                     0x46,0x4C,0x41,0x53,0x48,0x20,0x63,0x68,\
                                     0x69,0x70,0x73,0x0D,0x0A,0x00,0x41,0xFA,\
                                     0xFF,0xE8,0x70,0x01,0x4A,0xFA,0x12,0x3A,\
                                     0xFF,0x42,0x53,0x01,0x67,0x16,0x53,0x01,\
                                     0x67,0x00,0x00,0xB8,0x53,0x01,0x67,0x00,\
                                     0x01,0x0A,0x53,0x01,0x67,0x00,0x01,0x3A,\
                                     0x60,0x00,0x01,0x3A,0x4B,0xF8,0x00,0x00,\
                                     0x24,0x3A,0xFF,0x1C,0x26,0x02,0x3A,0xBC,\
                                     0xFF,0xFF,0x3A,0xBC,0xFF,0xFF,0x42,0x55,\
                                     0x4A,0x35,0x28,0xFF,0x67,0x28,0x7A,0x19,\
                                     0x1B,0xBC,0x00,0x40,0x28,0xFF,0x42,0x35,\
                                     0x28,0xFF,0x72,0x15,0x4E,0x71,0x51,0xC9,\
                                     0xFF,0xFC,0x1B,0xBC,0x00,0xC0,0x28,0xFF,\
                                     0x72,0x0C,0x4E,0x71,0x51,0xC9,0xFF,0xFC,\
                                     0x4A,0x35,0x28,0xFF,0x66,0x06,0x53,0x82,\
                                     0x66,0xCC,0x60,0x04,0x53,0x45,0x66,0xD0,\
                                     0x42,0x55,0x4A,0x55,0x4A,0x05,0x67,0x00,\
                                     0x00,0xE4,0x24,0x03,0x50,0xC4,0x2A,0x3C,\
                                     0x03,0xE8,0x03,0xE8,0x72,0x20,0x1B,0x81,\
                                     0x28,0xFF,0x1B,0x81,0x28,0xFF,0x32,0x3C,\
                                     0x55,0xF0,0x4E,0x71,0x51,0xC9,0xFF,0xFC,\
                                     0x4E,0xBA,0xFE,0x20,0x1B,0xBC,0x00,0xA0,\
                                     0x28,0xFF,0x72,0x0C,0x4E,0x71,0x51,0xC9,\
                                     0xFF,0xFC,0xB8,0x35,0x28,0xFF,0x66,0x08,\
                                     0x48,0x45,0x53,0x82,0x66,0xE6,0x60,0x04,\
                                     0x53,0x45,0x66,0xC8,0x42,0x55,0x4A,0x55,\
                                     0x4A,0x45,0x67,0x00,0x00,0x98,0x60,0x00,\
                                     0x00,0x90,0x70,0x01,0x42,0x83,0x1D,0x87,\
                                     0x08,0x00,0x1B,0x86,0x08,0x00,0x1D,0xBC,\
                                     0x00,0x80,0x08,0x00,0x1D,0x87,0x08,0x00,\
                                     0x1B,0x86,0x08,0x00,0x1D,0xBC,0x00,0x10,\
                                     0x08,0x00,0x2A,0x00,0x4E,0xBA,0xFD,0xCC,\
                                     0x20,0x05,0x1A,0x30,0x09,0x90,0x08,0x05,\
                                     0x00,0x07,0x66,0x20,0x08,0x05,0x00,0x05,\
                                     0x67,0xE8,0x1A,0x30,0x09,0x90,0x08,0x05,\
                                     0x00,0x07,0x66,0x10,0x1D,0x87,0x08,0x00,\
                                     0x1B,0x86,0x08,0x00,0x1D,0xBC,0x00,0xF0,\
                                     0x08,0x00,0x60,0x40,0x53,0x80,0x67,0xAE,\
                                     0x60,0x36,0x42,0x83,0x3C,0x87,0x3A,0x86,\
                                     0x3C,0xBC,0x00,0x80,0x3C,0x87,0x3A,0x86,\
                                     0x3C,0xBC,0x00,0x10,0x4E,0xBA,0xFD,0x84,\
                                     0x3A,0x15,0x08,0x05,0x00,0x07,0x66,0x18,\
                                     0x08,0x05,0x00,0x05,0x67,0xEE,0x3A,0x15,\
                                     0x08,0x05,0x00,0x07,0x66,0x0A,0x3C,0x87,\
                                     0x3A,0x86,0x3C,0xBC,0x00,0xF0,0x60,0x04,\
                                     0x42,0x80,0x60,0x02,0x70,0x01,0x4E,0x75,\
                                     0x47,0xFB,0x01,0x70,0x00,0x00,0x04,0x4C,\
                                     0x28,0x49,0x24,0x3C,0x00,0x00,0x01,0x00,\
                                     0x12,0x3A,0xFD,0xD8,0x53,0x01,0x67,0x14,\
                                     0x53,0x01,0x67,0x5A,0x53,0x01,0x67,0x00,\
                                     0x00,0xBC,0x53,0x01,0x67,0x00,0x01,0x00,\
                                     0x60,0x00,0x01,0x2E,0x10,0x33,0x28,0xFF,\
                                     0x0C,0x00,0x00,0xFF,0x67,0x28,0x7A,0x19,\
                                     0x19,0xBC,0x00,0x40,0x28,0xFF,0x19,0x80,\
                                     0x28,0xFF,0x72,0x15,0x4E,0x71,0x51,0xC9,\
                                     0xFF,0xFC,0x19,0xBC,0x00,0xC0,0x28,0xFF,\
                                     0x72,0x0C,0x4E,0x71,0x51,0xC9,0xFF,0xFC,\
                                     0xB0,0x34,0x28,0xFF,0x66,0x06,0x53,0x82,\
                                     0x66,0xCA,0x60,0x04,0x53,0x05,0x66,0xD0,\
                                     0x42,0x55,0x4A,0x55,0x4A,0x05,0x67,0x00,\
                                     0x00,0xE8,0x60,0x00,0x00,0xE0,0x20,0x0C,\
                                     0xD0,0x82,0xC0,0xBC,0x00,0x00,0x00,0x01,\
                                     0x08,0x40,0x00,0x00,0x16,0x33,0x28,0xFF,\
                                     0x0C,0x03,0x00,0xFF,0x67,0x48,0x1D,0x87,\
                                     0x08,0x00,0x1B,0x86,0x08,0x00,0x1D,0xBC,\
                                     0x00,0xA0,0x08,0x00,0x19,0x83,0x28,0xFF,\
                                     0xC6,0x3C,0x00,0x80,0x18,0x34,0x28,0xFF,\
                                     0x1A,0x04,0xC8,0x3C,0x00,0x80,0xB8,0x03,\
                                     0x67,0x24,0x08,0x05,0x00,0x05,0x67,0xEC,\
                                     0x18,0x34,0x28,0xFF,0xC8,0x3C,0x00,0x80,\
                                     0xB8,0x03,0x67,0x12,0x1D,0x87,0x08,0x00,\
                                     0x1B,0x86,0x08,0x00,0x1D,0xBC,0x00,0xF0,\
                                     0x08,0x00,0x60,0x00,0x00,0x84,0x53,0x82,\
                                     0x66,0xA6,0x60,0x78,0x36,0x33,0x28,0xFE,\
                                     0x0C,0x43,0xFF,0xFF,0x67,0x3A,0x3C,0x87,\
                                     0x3A,0x86,0x3C,0xBC,0x00,0xA0,0x39,0x83,\
                                     0x28,0xFE,0xC6,0x7C,0x00,0x80,0x38,0x34,\
                                     0x28,0xFE,0x3A,0x04,0xC8,0x7C,0x00,0x80,\
                                     0xB8,0x43,0x67,0x1C,0x08,0x05,0x00,0x05,\
                                     0x67,0xEC,0x38,0x34,0x28,0xFE,0xC8,0x7C,\
                                     0x00,0x80,0xB8,0x43,0x67,0x0A,0x3C,0x87,\
                                     0x3A,0x86,0x3C,0xBC,0x00,0xF0,0x60,0x38,\
                                     0x55,0x82,0x66,0xB8,0x60,0x2E,0x3C,0x87,\
                                     0x3A,0x86,0x3C,0xBC,0xA0,0xA0,0x39,0xB3,\
                                     0x28,0xFE,0x28,0xFE,0x55,0x82,0x66,0xF6,\
                                     0x32,0x3C,0x55,0xF0,0x4E,0x71,0x51,0xC9,\
                                     0xFF,0xFC,0x34,0x3C,0x01,0x00,0x36,0x33,\
                                     0x28,0xFE,0xB6,0x74,0x28,0xFE,0x66,0x08,\
                                     0x55,0x82,0x66,0xF2,0x42,0x80,0x60,0x02,\
                                     0x70,0x01,0x4E,0x75,\
                                     0x4F,0xFB,0x01,0x70,0x00,0x00,0x02,0xF0,\
                                     0x4E,0xBA,0xFC,0x08,0x4E,0xBA,0xFC,0x82,\
                                     0x4E,0xBA,0xFD,0x30,0x4A,0xFA,0x42,0x80,\
                                     0x22,0x40,0x4E,0xBA,0xFE,0x88,0x4A,0xFA,\
                                     0xD2,0xFC,0x01,0x00,0x60,0xF4
                                    };

            //if (prep_t5_do() != TERM_OK) return TERM_ERR;
            // Set Program counter to start of BDM driver code
            uint32_t driverAddress = 0x100000;
            if (sysreg_write(0x0, &driverAddress) != TERM_OK) break;
            for (uint32_t i = 0; i < sizeof(flashDriver); i++) {
                if(memwrite_byte(&driverAddress, flashDriver[i]) != TERM_OK) return false;
                driverAddress++;
            }
//            if (!bdmLoadMemory(flashDriver, driverAddress, sizeof(flashDriver))) break;

            timer.reset();
            timer.start();
            printf("Erasing FLASH chips...\r\n");
            printf("This can take up to a minute for a T8,\r\n");
            printf("30s for a T7 or 15s for a T5 ECU.\r\n");
            // execute the erase algorithm in the BDM driver
            // write the buffer - should complete within 200 milliseconds
            // Typical and Maximum Chip Programming times are 9 and 27 seconds for Am29BL802C
            // Typical Chip erase time for Am29BL802C is 45 secinds, not including 0x00 programming prior to erasure.
            // Allow for at least worst case 27 seconds programming to 0x00 + 3(?) * 45 typical erase time (162 seconds)
            // Allow at least 200 seconds erase time 2,000 * (100ms + BDM memread time)
            // NOTE: 29/39F010 and 29F400 erase times are considerably lower

//            if (sysreg_write(0x0, &driverAddress) != TERM_OK) return TERM_ERR;
//            break;
            do {
                if (!bdmRunDriver(0x0, 200000)) {
                    printf("WARNING: An error occured when I tried to erase the FLASH chips :-(\r\n");
                    return TERM_ERR;
                }
            } while (bdmProcessSyscall() == CONTINUE);

//            if (!run_bdm_driver(0x0, 200000)) {
//                printf("WARNING: An error occured when I tried to erase the FLASH chips :-(\r\n");
//               return TERM_ERR;
//            }
            printf("Erasing took %#.1f seconds.\r\n",timer.read());

            printf("Programming the FLASH chips...\r\n");

// ready to receive data
            printf("  0.00 %% complete.\r");
            while (curr_addr < flash_size) {
                // receive bytes from BIN file - break if no more bytes to get
                if (!fread(file_buffer,1,0x100,fp)) {
                    fclose(fp);
                    printf("Error reading the BIN file MODIFIED.BIN");
                    break;
                }
                if (!bdmLoadMemory((uint8_t*)file_buffer, 0x100700, 0x100)) break;
                // write the buffer - should complete within 200 milliseconds
                if (!bdmRunDriver(0x0, 200)) break;
//                if (!run_bdm_driver(0x0, 200)) break;

                printf("%6.2f\r", 100*(float)curr_addr/(float)flash_size );
                // make the activity LED twinkle
                ACTIVITYLEDON;
                curr_addr += 0x100;
            }
            break;
        }
        // johnc's original method
        case AMD29F400B:        /// a sort of dummy 'placeholder' as the 'B' chip isn't ever fitted to T7 ECUS
        default: {
            timer.reset();
            timer.start();

            // reset the FLASH chips
            printf("Reset the FLASH chip(s) to prepare them for Erasing\r\n");
            if (!reset_func()) return TERM_ERR;

            switch (type) {
                    // AM29Fxxx chips (retrofitted to Trionic 5.x; original to T7)
                case AMD29BL802C:
                case AMD29F400B:
                case AMD29F400T:
                case AMD29F010:
                case SST39SF010:
                case AMICA29010L:
                    printf("Erasing 29BL802/F400/010 type FLASH chips...\r\n");
                    if (!erase_am29()) {
                        printf("WARNING: An error occured when I tried to erase the FLASH chips :-(\r\n");
                        return TERM_ERR;
                    }
                    break;
                    // AM28F010 chip (Trionic 5.x original)
                case AMD28F010:
                case INTEL28F010:
                case AMD28F512:
                case INTEL28F512:
                    printf("Erasing 28F010/512 type FLASH chips...\r\n");
                    if (!erase_am28(&curr_addr, &flash_size)) {
                        printf("WARNING: An error occured when I tried to erase the FLASH chips :-(\r\n");
                        return TERM_ERR;
                    }
                    break;
                case ATMEL29C010:
                case ATMEL29C512:
                    printf("Atmel FLASH chips do not require ERASEing :-)\r\n");
                    break;
                default:
                    // unknown flash type - shouldn't get here hence "Strange!"
                    printf("Strange! I couldn't work out how to erase the FLASH chips in the TRIONIC ECU that I am connected to :-(\r\n");
                    return TERM_ERR;
            }

            timer.stop();
            printf("Erasing took %#.1f seconds.\r\n",timer.read());

            printf("Programming the FLASH chips...\r\n");

            timer.reset();
            timer.start();

            uint16_t word_value = 0;

// ready to receive data
            printf("  0.00 %% complete.\r");
            while (curr_addr < flash_size) {
                // receive bytes from BIN file
                //Get a byte - break if no more bytes to get
                if (!fread(&file_buffer[0],1,0x2,fp)) {
                    fclose(fp);
                    printf("Error reading the BIN file MODIFIED.BIN");
                    break;
                }
                for(uint32_t i=0; i<2; i++) {
                    (word_value <<= 8) |= file_buffer[i];
                }

                // write the word if it is not 0xffff
                if (word_value != 0xffff) {
                    if (!flash_func(&curr_addr, word_value)) break;
                }

                if (!(curr_addr % 0x80)) {
                    printf("%6.2f\r", 100*(float)curr_addr/(float)flash_size );
                    // make the activity LED twinkle
                    ACTIVITYLEDON;
                }
                curr_addr += 2;
            }
        }
    }
    timer.stop();
    fclose(fp);

    if (curr_addr == flash_size) {
        printf("100.00\r\n");
        printf("Programming took %#.1f seconds.\r\n",timer.read());

        // "Just4pleisure;)" 'tag' in the empty space at the end of the FLASH chip
        // Removed for now because it conflicts with some information that Dilemma places in this empty space
        // and because it is unsafe for Trionic 8 ECUs which have much bigger BIN files and FLASH chips
        //        reset_func();
        //        for (uint8_t i = 0; i < 8; ++i) {
        //            memread_word(&word_value, &flash_tag[i].addr);
        //            flash_func(&flash_tag[i].addr, (flash_tag[i].val & word_value));
        //        }

    } else {
        printf("\r\n");
        printf("Programming took %#.1f seconds.\r\n",timer.read());
        printf("WARNING: Oh dear, I couldn't program the FLASH at address 0x%08lx.\r\n", curr_addr);
    }

    // reset flash
    return (reset_func() && (curr_addr == flash_size)) ? TERM_OK : TERM_ERR;
}

//-----------------------------------------------------------------------------
/**
Resets an AM29Fxxx flash memory chip. MCU must be in background mode.

@param                          none

@return                         succ / fail
*/
bool reset_am29(void)
{
    // execute the reset command
    //    uint32_t addr = 0xfffe;
    //    return (memwrite_word(&addr, 0xf0f0) == TERM_OK);
    // execute the algorithm
    for (uint8_t i = 0; i < 3; ++i) {
        if (memwrite_word(&am29_reset[i].addr, am29_reset[i].val) != TERM_OK) return false;
    }
    return true;
}

//-----------------------------------------------------------------------------
/**
Erases an AM29Fxxx flash memory chip and verifies the result; MCU must be
in background mode.

@return                        succ / fail
*/
bool erase_am29()
{
    // reset flash
    if (!reset_am29()) {
        return false;
    }
    printf("Erasing AMD 29Fxxx FLASH chips.\r\n");
    printf("This can take up to a minute for a T8,\r\n");
    printf("30s for a T7 or 15s for a T5 ECU.\r\n");
    // execute the algorithm
    for (uint8_t i = 0; i < 6; ++i) {
        if (memwrite_word(&am29_erase[i].addr, am29_erase[i].val) != TERM_OK) {
            reset_am29();
            return false;
        }
    }

    // verify the result
    uint32_t addr = 0x0;
    uint16_t verify_value;

    printf("  0.0 seconds.\r");
    timeout.reset();
    timeout.start();
    while (timeout.read() < 200.0) {
        // Typical and Maximum Chip Programming times are 9 and 27 seconds for Am29BL802C
        // Typical Chip erase time for Am29BL802C is 45 secinds, not including 0x00 programming prior to erasure.
        // Allow for at least worst case 27 seconds programming to 0x00 + 3(?) * 45 typical erase time (162 seconds)
        // Allow at least 200 seconds erase time 2,000 * (100ms + BDM memread time)
        // NOTE: 29/39F010 and 29F400 erase times are considerably lower
        if (memread_word(&verify_value, &addr) == TERM_OK && verify_value == 0xffff) {
            // erase completed normally
            reset_am29();
            printf("\n");
            return true;
        }
        // make the activity LED twinkle
        ACTIVITYLEDON;
        printf("%5.1f\r", timeout.read());
    }
    // erase failed
    printf("\n");
    reset_am29();
    return false;
}

//-----------------------------------------------------------------------------
/**
Writes a word to AM29Fxxx flash memory chip and optionally verifies the
result; MCU must be in background mode.

@param        addr        destination address
@param        val            value

@return                    succ / fail
*/
bool flash_am29(const uint32_t* addr, uint16_t value)
{

    // execute the algorithm
    for (uint8_t i = 0; i < 3; ++i) {
        if (memwrite_word(&am29_write[i].addr, am29_write[i].val) != TERM_OK) {
            reset_am29();
            return false;
        }
    }
    // write the value
    if (memwrite_word(addr, value) != TERM_OK) {
        reset_am29();
        return false;
    }
    // verify the result
    timeout.reset();
    timeout.start();
    while (timeout.read_us() < 500) {
        // Typical and Maximum Word Programming times are 9us and 360us for Am29BL802C
        // Allow at least 500 microseconds program time 500 * (1us + BDM memread time)
        // NOTE: 29/39F010 and 29F400 programming times are considerably lower
        uint16_t verify_value;
        if ((memread_word(&verify_value, addr) == TERM_OK) &&
                (verify_value == value)) {
            // flashing successful
            return true;
        }
    }
    // writing failed
    reset_am29();
    return false;
}


//-----------------------------------------------------------------------------
/**
Writes a word to a FLASH memory chip and checks the result
MCU must be in background mode.

@param        addr        BDM driver address (0 to continue)
@param        maxtime     how long to allow driver to execute (milliseconds)

@return                    succ / fail
*/
bool run_bdm_driver(uint32_t addr, uint32_t maxtime)
{
    // Start BDM driver and allow it up to 200 milliseconds to update 256 Bytes
    // Upto 25 pulses per byte, 16us per pulse, 256 Bytes
    // 25 * 16 * 256 = 102,400us plus overhead for driver code execution time
    // Allowing up to 200 milliseconds seems like a good allowance.
    uint32_t driverAddress = addr;
    if (run_chip(&driverAddress) != TERM_OK) {
        printf("Failed to start BDM driver.\r\n");
        return false;
    }
    timeout.reset();
    timeout.start();
    // T5 ECUs' BDM interface seem to have problems when the running the CPU and
    // sometimes shows the CPU briefly switching between showing BDM mode or that
    // the CPU is running.
    // I 'debounce' the interface state to workaround this erratic bahaviour
    for (uint32_t debounce = 0; debounce < 5; debounce++) {
        while (IS_RUNNING) {
            debounce = 0;
            if (timeout.read_ms() > maxtime) {
                printf("Driver did not return to BDM mode.\r\n");
                timeout.stop();
                return false;
            }
        }
        wait_us(1);
    }
    timeout.stop();
    // Check return code in D0 register (0 - OK, 1 - FAILED)
    uint32_t result = 1;
    if (adreg_read(&result, 0x0) != TERM_OK) {
        printf("Failed to read BDM register.\r\n");
        return false;
    }
    return (result == 1) ? false : true;
}


//-----------------------------------------------------------------------------
/**
Resets a AM28Fxxx flash memory chip. MCU must be in background mode.

@param      start_addr      flash start address

@return                     succ / fail
*/
bool reset_am28(void)
{
    uint32_t start_addr = 0x0;
    return (memwrite_word_write_word(&start_addr, 0xffff, 0xffff) == TERM_OK);
}

//-----------------------------------------------------------------------------
/**
Erases an AM28Fxxx flash memory chip and verifies the result; MCU must be
in background mode.

@param      start_addr      flash start address
@param      end_addr        flash end address

@return                     succ / fail
*/
bool erase_am28(const uint32_t* start_addr, const uint32_t* end_addr)
{

    // check the addresses
    if (!start_addr || !end_addr) return false;

    // reset flash
    if (!reset_am28()) return false;

    // write zeroes over entire flash space
    uint32_t addr = *start_addr;

    printf("First write 0x00 to all FLASH addresses.\r\n");
    printf("  0.00 %% complete.\r");
    while (addr < *end_addr) {
        if (!flash_am28(&addr, 0x0000)) return false;
        addr += 2;
        //        // feedback to host computer
        //        pc.putc(TERM_OK);
        if (!(addr % 0x80)) {
            // make the activity LED twinkle
            ACTIVITYLEDON;
            printf("%6.2f\r", 100*(float)addr/(float)*end_addr );
        }
    }
    printf("\n");

    // erase flash
    addr = *start_addr;
    uint8_t verify_value;

    printf("Now erasing FLASH and verfiying that all addresses are 0xFF.\r\n");
    printf("  0.00 %% complete.\r");
    uint16_t pulse_cnt = 0;
    if (memwrite_byte_cmd(NULL) != TERM_OK) {
        reset_am28();
        return false;
    }
    while ((++pulse_cnt < 1000) && (addr < *end_addr)) {
        // issue the erase command
        if (memwrite_write_byte(&addr, 0x20) != TERM_OK ||
                memwrite_write_byte(&addr, 0x20) != TERM_OK) break;
        thread_sleep_for(10);

        while (addr < *end_addr) {
            // issue the verify command
            if (memwrite_read_byte(&addr, 0xa0) != TERM_OK) break;
            //            wait_us(6);
            // check the written value
            if (memread_write_byte(&verify_value, &addr) != TERM_OK) break;
            if (verify_value != 0xff) break;
            // succeeded need to check next address
            addr++;
            // make the activity LED twinkle
            ACTIVITYLEDON;
            if (!(addr % 0x80)) {
                // make the activity LED twinkle
                ACTIVITYLEDON;
                printf("%6.2f\r", 100*(float)addr/(float)*end_addr );
            }
        }
    }
    printf("\n");
    // the erase process ends with a BDM_WRITE + BDM_BYTESIZE command left in the BDM
    // it is safe to use it to put one of the FLASH chips into read mode and thereby
    // leave the BDM ready for the next command
    memwrite_nop_byte(start_addr, 0x00);

    reset_am28();
    // check for success
    return (addr == *end_addr) ? true : false;
}

//-----------------------------------------------------------------------------
/**
Writes a byte to AM28Fxxx flash memory chip and verifies the result
A so called 'mask' method checks the FLASH contents and only tries
to program bytes that need to be programmed.
MCU must be in background mode.

@param      addr        destination address
@param      val         value

@return                 succ / fail
*/
bool flash_am28(const uint32_t* addr, uint16_t value)
{

    if (!addr) return false;

    uint8_t pulse_cnt = 0;
    uint16_t verify_value = 0;
    uint16_t mask_value = 0xffff;

    // put flash into read mode and read address
    if (memwrite_word_read_word(&verify_value, addr, 0x0000) != TERM_OK)  return false;
    // return if FLASH already has the correct value - e.g. not all of the FLASH is used and is 0xff
    if (verify_value == value) return true;

    while (++pulse_cnt < 25) {

        // set a mask
        if ((uint8_t)verify_value == (uint8_t)value)
            mask_value &= 0xff00;
        if ((uint8_t)(verify_value >> 8) == (uint8_t)(value >> 8))
            mask_value &= 0x00ff;

        // write the new value
        if (memwrite_word_write_word(addr, (0x4040 & mask_value), value) != TERM_OK) break;
        // NOTE the BDM interface is slow enough that there is no need for a 10us delay before verifying
        // issue the verification command
        // NOTE the BDM interface is slow enough that there is no need for a 6us delay before reading back
        if (memwrite_word_read_word(&verify_value, addr, (0xc0c0 & mask_value)) != TERM_OK) break;
        // check if flashing was successful;
        if (verify_value == value) return true;
    }

    // something went wrong; reset the flash chip and return failed
    reset_am28();
    return false;
}

//-----------------------------------------------------------------------------
/**
Does the equivalent of do prept5.do in BD32
Sets up all of the control registers in the MC68332 so that we can program
the FLASH chips

@param                  none

@return                 succ / fail
*/

//uint8_t prep_t5_do(void) {
uint8_t prep_t8_do(void)
{

    // reset and freeze the MC68332 chip
    if (restart_chip() != TERM_OK) return TERM_ERR;

    // set the 'fc' registers to allow supervisor mode access
    uint32_t long_value = 0x05;
    if (sysreg_write(0x0e, &long_value) != TERM_OK) return TERM_ERR;
    if (sysreg_write(0x0f, &long_value) != TERM_OK) return TERM_ERR;

    // Set MC68332 to 16 MHz (actually 16.78 MHz) (SYNCR)
    long_value = 0x00fffa04;
    if (memwrite_word(&long_value, 0x7f00) != TERM_OK) return TERM_ERR;

    // Disable watchdog and monitors (SYPCR)
    long_value = 0x00fffa21;
    if (memwrite_byte(&long_value, 0x00) != TERM_OK) return TERM_ERR;


    // Chip select pin assignments (CSPAR0)
    long_value = 0x00fffa44;
    if (memwrite_word(&long_value, 0x3fff) != TERM_OK) return TERM_ERR;

    // Boot Chip select read only, one wait state (CSBARBT)
    long_value = 0x00fffa48;
    if (memwrite_word(&long_value, 0x0007) != TERM_OK) return TERM_ERR;
    if (memfill_word(0x6870) != TERM_OK) return TERM_ERR;

    // Chip select 1 and 2 upper lower bytes, zero wait states (CSBAR1, CSOR1, CSBAR2, CSBAR2)
    long_value = 0x00fffa50;
    if (memwrite_word(&long_value, 0x0007) != TERM_OK) return TERM_ERR;
    if (memfill_word(0x3030) != TERM_OK) return TERM_ERR;
    if (memfill_word(0x0007) != TERM_OK) return TERM_ERR;
    if (memfill_word(0x5030) != TERM_OK) return TERM_ERR;

    // PQS Data - turn on VPPH (PORTQS)
    long_value = 0x00fffc14;
    if (memwrite_word(&long_value, 0x0040) != TERM_OK) return TERM_ERR;

    // PQS Data Direction output (DDRQS)
    long_value = 0x00fffc17;
    if (memwrite_byte(&long_value, 0x40) != TERM_OK) return TERM_ERR;
    // wait for programming voltage to be ready
    thread_sleep_for(10);

    //    // Enable internal 2kByte RAM of 68332 at address 0x00100000 (TRAMBAR)
    //    long_value = 0x00fffb04;
    //    if (memwrite_word(&long_value, 0x1000) != TERM_OK) return TERM_ERR;
    return TERM_OK;
}

//-----------------------------------------------------------------------------
/**
Does the equivalent of do prept5/7/8.do in BD32
Sets up all of the control registers in the MC68332/377 so that we can
program the FLASH chips

@param                  none

@return                 succ / fail
*/

uint8_t prep_t5_do(void)
{
    // Make sure that BDM clock is SLOW
    bdm_clk_mode(SLOW);
    // reset and freeze the MC68332/377 chip
    if (restart_chip() != TERM_OK) return TERM_ERR;

    // define some variables to store address and data values
    uint32_t long_value = 0x05;
    uint16_t verify_value;

    // set the 'fc' registers to allow supervisor mode access
    if (sysreg_write(0x0e, &long_value) != TERM_OK) return TERM_ERR;
    if (sysreg_write(0x0f, &long_value) != TERM_OK) return TERM_ERR;

    // Read MC68332/377 Module Control Register (SIMCR/MCR)
    // and use the value to work out if ECU is a T5/7 or a T8
    long_value = 0x00fffa00;
    if (memread_word(&verify_value, &long_value) != TERM_OK) return TERM_ERR;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//    verify_value = 0x7E4F;
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // MC68377 MCR = x111111x01001111 binary after a reset
    if ((verify_value & 0x7E4F) == 0x7E4F) {
        printf ("I have found a Trionic 8 ECU.\r\n");
        // Stop system protection (MDR bit 0 STOP-SYS-PROT)
        long_value = 0x00fffa04;
        if (memwrite_byte(&long_value, 0x01) != TERM_OK) return TERM_ERR;
        // Set MC68377 to double it's default speed (16 MHz?) (SYNCR)
        long_value = 0x00fffa08;
        // First set the MFD part (change 4x to 8x)
        if (memwrite_word(&long_value, 0x6908) != TERM_OK) return TERM_ERR;
        // wait for everything to settle (should really check the PLL lock register)
        thread_sleep_for(100);
        // Now set the RFD part (change /2 to /1)
        if (memwrite_word(&long_value, 0x6808) != TERM_OK) return TERM_ERR;
        // Disable watchdog and monitors (SYPCR)
        long_value = 0x00fffa50;
        if (memwrite_word(&long_value, 0x0000) != TERM_OK) return TERM_ERR;
        // Enable internal 6kByte RAM of 68377 at address 0x00100000 (DPTRAM)
        long_value = 0x00fff684;
        if (memwrite_word(&long_value, 0x1000) != TERM_OK) return TERM_ERR;
        // can use fast or turbo or nitrous BDM clock mode once ECU has been prepped and CPU clock is ??MHz
//        bdm_clk_mode(NITROUS);
    }
// MC68332 SIMCR = 0000x00011001111 binary after a reset
    //if ((verify_value & 0x00CF) == 0x00CF) {
    else {
        printf ("I have found a Trionic 5 or 7 ECU.\r\n");
        // Set MC68332 to 16 MHz (actually 16.78 MHz) (SYNCR)
        long_value = 0x00fffa04;
        if (memwrite_word(&long_value, 0x7f00) != TERM_OK) return TERM_ERR;
        // Disable watchdog and monitors (SYPCR)
        long_value = 0x00fffa21;
        if (memwrite_byte(&long_value, 0x00) != TERM_OK) return TERM_ERR;
        // Chip select pin assignments (CSPAR0)
        long_value = 0x00fffa44;
        if (memwrite_word(&long_value, 0x3fff) != TERM_OK) return TERM_ERR;
        // Boot Chip select read only, one wait state (CSBARBT)
        long_value = 0x00fffa48;
        if (memwrite_word(&long_value, 0x0007) != TERM_OK) return TERM_ERR;
        if (memfill_word(0x6870) != TERM_OK) return TERM_ERR;
        // Chip select 1 and 2 upper lower bytes, zero wait states (CSBAR1, CSOR1, CSBAR2, CSBAR2)
        long_value = 0x00fffa50;
        if (memwrite_word(&long_value, 0x0007) != TERM_OK) return TERM_ERR;
        if (memfill_word(0x3030) != TERM_OK) return TERM_ERR;
        if (memfill_word(0x0007) != TERM_OK) return TERM_ERR;
        if (memfill_word(0x5030) != TERM_OK) return TERM_ERR;
        // PQS Data - turn on VPPH (PORTQS)
        long_value = 0x00fffc14;
        if (memwrite_word(&long_value, 0x0040) != TERM_OK) return TERM_ERR;
        // PQS Data Direction output (DDRQS)
        long_value = 0x00fffc17;
        if (memwrite_byte(&long_value, 0x40) != TERM_OK) return TERM_ERR;
        // wait for programming voltage to be ready
        thread_sleep_for(10);
        // Enable internal 2kByte RAM of 68332 at address 0x00100000 (TRAMBAR)
        long_value = 0x00fffb04;
        if (memwrite_word(&long_value, 0x1000) != TERM_OK) return TERM_ERR;
        // can use fast or turbo BDM clock mode once ECU has been prepped and CPU clock is 16MHz
//        bdm_clk_mode(TURBO);
//        bdm_clk_mode(FAST);
    }
    return TERM_OK;
}


//-----------------------------------------------------------------------------
/**
    Works out what type of flash chip is fitted in the ECU by reading
    the manufacturer byte codes.
    It is enough to use the 29Fxxx flash id algorithm because 28Fxxx
    FLASH chips ignore the first few writes needed by the 29Fxxx chips
    MCU must be in background mode.

    @param                  make (out)
                            type (out)

    @return                 succ / fail
*/
bool get_flash_id(uint8_t* make, uint8_t* type)
{

    uint32_t  addr = 0x0;
    uint32_t value;
    bool ret;
    // read id bytes algorithm for 29F010/400 FLASH chips
    for (uint8_t i = 0; i < 3; ++i) {
        //printf("Getting FLASH chip ID.\r\n");
        if (memwrite_word(&am29_id[i].addr, am29_id[i].val) != TERM_OK) {
            printf("There was an error when I tried to request the FLASH chip ID.\r\n");
            return false;
        }
    }
    if (memread_long(&value, &addr) != TERM_OK) {
        printf("Error Reading FLASH chip types in get_flash_id\r\n");
        return false;
    }
//    *make = (uint8_t)(value >> 24);
//    *type = (uint8_t)(value >> 8);
    *make = (uint8_t)(value >> 16);
    *type = (uint8_t)(value);
    printf("FLASH id bytes: %08lx, make: %02x, type: %02x\r\n", value, *make, *type);
    switch (*type) {
        case AMD29BL802C:
        case AMD29F400B:
        case AMD29F400T:
        case AMD29F010:
        case AMD28F010:
        case INTEL28F010:
        case AMD28F512:
        case INTEL28F512:
            ret = true;
        default:
            ret = false;
    }
    return ret;
}

//-----------------------------------------------------------------------------
//    EOF
//-----------------------------------------------------------------------------
