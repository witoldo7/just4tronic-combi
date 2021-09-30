#include "gmlan.h"

Timer   GMLANtimer;


void GMLANTesterPresentAll()
{
    char GMLANMsg[] = GMLANTesterPresentFunctional;
    can_send_timeout(GMLANALLNODES, GMLANMsg, 3, GMLANPTCT);
    ACTIVITYLEDON;
}

void GMLANTesterPresent(uint32_t ReqID, uint32_t RespID)
{
    char GMLANMsg[] = GMLANTesterPresentPhysical;
    can_send_timeout(ReqID, GMLANMsg, 2, GMLANPTCT);
    can_wait_timeout(RespID, GMLANMsg, 2, GMLANPTCT);
    ACTIVITYLEDON;
}

// All steps needed in preparation for using a bootloader ('Utility File' in GMLAN parlance)
bool GMLANprogrammingSetupProcess(uint32_t ReqID, uint32_t RespID)
{
    GMLANTesterPresent(ReqID, RespID);
    printf("Starting Diagnostic session\r\n");
    if (!GMLANinitiateDiagnostic(ReqID, RespID, GMLANdisableAllDTCs)) {
        printf("Unable to start Diagnostic session\r\n");
        return false;
    }
    printf("Disabling Normal Communincation Messages\r\n");
    if (!GMLANdisableNormalCommunication(ReqID, RespID)) {
        printf("Unable to tell T8 to disable normal communication messages\r\n");
        return false;
    }
    printf("Report Programmed State\r\n");
    if (!GMLANReportProgrammedState(ReqID, RespID)) {
        printf("Unable to determine ECU programmed state\r\n");
    }
    printf("Requesting program mode\r\n");
    if (!GMLANProgrammingMode(ReqID, RespID, GMLANRequestProgrammingNormal)) {
        printf("Unable to request programming mode\r\n");
        return false;
    }
    printf("Starting program session\r\n");
    if (!GMLANProgrammingMode(ReqID, RespID, GMLANEnableProgrammingMode)) {
        printf("Unable to start program session\r\n");
        return false;
    }
    thread_sleep_for(500);   // was 5
    return true;
}


// All steps needed to transfer and start a bootloader ('Utility File' in GMLAN parlance)
bool GMLANprogrammingUtilityFileProcess(uint32_t ReqID, uint32_t RespID, const uint8_t UtilityFile[])
{
    uint16_t i = 0, j = 0, k = 0;
    uint32_t StartAddress = 0x102400;
    uint16_t txpnt = 0;
    char iFrameNumber = 0x21;
    char GMLANMsg[8];
//
    GMLANTesterPresent(ReqID, RespID);
    GMLANtimer.start();
    printf("Waiting for Permission to send bootloader\r\n");
    thread_sleep_for(500);
    if (!GMLANRequestDownload(ReqID, RespID, GMLANRequestDownloadModeNormal)) {
        printf("Unable to request Bootloader Upload\r\n");
        return false;
    }
    printf("Sending Bootloader\r\n");
    printf("  0.00 %% complete.\r");
//
    for (i=0; i<0x46; i++) {
        if (!GMLANDataTransferFirstFrame(ReqID, RespID, 0xF0, GMLANDOWNLOAD, StartAddress)) {
            printf("Unable to start Bootloader Upload\r\n");
            return false;
        }
        iFrameNumber = 0x21;
        for (j=0; j < 0x22; j++) {
            GMLANMsg[0] = iFrameNumber;
            for (k=1; k<8; k++)
                GMLANMsg[k] = UtilityFile[txpnt++];
            if (!can_send_timeout(ReqID, GMLANMsg, 8, GMLANPTCT)) {
                printf("Unable to send Bootloader\r\n");
                return false;
            }
            ++iFrameNumber &= 0x2F;
            thread_sleep_for(2);     // was 2
        }
        if (!GMLANDataTransferBlockAcknowledge(RespID))
            return false;
        if (GMLANtimer.read_ms() > 2000) {
            GMLANTesterPresent(ReqID, RespID);
            GMLANtimer.reset();
        }
        StartAddress += 0xea;
        printf("%6.2f\r", 100*(float)txpnt/(float)(16384+(70*4)) );
    }
    thread_sleep_for(1);
    if (!GMLANDataTransferFirstFrame(ReqID, RespID, 0x0A, GMLANDOWNLOAD, StartAddress)) {
        printf("Unable to finish Bootloader Upload\r\n");
        return false;
    }
    iFrameNumber = 0x21;
    GMLANMsg[0] = iFrameNumber;
    for (k=0; k<7; k++) {
        GMLANMsg[k+1] = UtilityFile[txpnt++];
    }
    if (!can_send_timeout(ReqID, GMLANMsg, 8, GMLANPTCT)) {
        printf("Unable to finish sending Bootloader\r\n");
        return false;
    }
    if (!can_wait_timeout(RespID, GMLANMsg, 8, GMLANPTCT))
        return false;
    printf("%6.2f\r\n", (float)100 );
    printf("Starting the bootloader\r\n");
    if (!GMLANDataTransfer(ReqID, RespID, 0x06, GMLANEXECUTE, 0x00102460)) {
        printf("Unable to start the Bootloader\r\n");
        return false;
    }
    thread_sleep_for(500);   // was 100
    return true;
}

bool GMLANinitiateDiagnostic(uint32_t ReqID, uint32_t RespID, char level)
{
    char GMLANMsg[] = GMLANinitiateDiagnosticOperation;
    GMLANMsg[2] = level;
    if (!can_send_timeout(ReqID, GMLANMsg, 3, GMLANPTCT))
        return false;
    if (ReqID == T8USDTREQID) return true;
    if (!can_wait_timeout(RespID, GMLANMsg, 8, GMLANPTCT))
        return false;
    while (GMLANMsg[0] == 0x03 && GMLANMsg[1] == 0x7F && GMLANMsg[2] == 0x10 && GMLANMsg[3] == 0x78)
        if (!can_wait_timeout(RespID, GMLANMsg, 8, GMLANPTCTENHANCED))
            return false;
    if ( GMLANMsg[0] == 0x03 && GMLANMsg[1] == 0x7F && GMLANMsg[2] == 0x10 )
        GMLANShowReturnCode(GMLANMsg[3]);
    return (GMLANMsg[0] == 0x01 && GMLANMsg[1] == 0x50) ? true : false;
}


bool GMLANdisableNormalCommunication(uint32_t ReqID, uint32_t RespID)
{
    char GMLANMsg[] = GMLANdisableCommunication;
    if (!can_send_timeout(ReqID, GMLANMsg, 2, GMLANPTCT))
        return false;
    if (ReqID == T8USDTREQID) return true;
    if (!can_wait_timeout(RespID, GMLANMsg, 8, GMLANPTCT))
        return false;
    while (GMLANMsg[0] == 0x03 && GMLANMsg[1] == 0x7F && GMLANMsg[2] == 0x28 && GMLANMsg[3] == 0x78)
        if (!can_wait_timeout(RespID, GMLANMsg, 8, GMLANPTCTENHANCED))
            return false;
    if ( GMLANMsg[0] == 0x03 && GMLANMsg[1] == 0x7F && GMLANMsg[2] == 0x28 )
        GMLANShowReturnCode(GMLANMsg[3]);
    return (GMLANMsg[0] == 0x01 && GMLANMsg[1] == 0x68) ? true : false;
}


bool GMLANReportProgrammedState(uint32_t ReqID, uint32_t RespID)
{
    char GMLANMsg[] = GMLANReportProgrammed;
    if (!can_send_timeout(ReqID, GMLANMsg, 2, GMLANPTCT))
        return false;
    if (!can_wait_timeout(RespID, GMLANMsg, 8, GMLANPTCT))
        return false;
    while (GMLANMsg[0] == 0x03 && GMLANMsg[1] == 0x7F && GMLANMsg[2] == 0xA2 && GMLANMsg[3] == 0x78)
        if (!can_wait_timeout(RespID, GMLANMsg, 8, GMLANPTCTENHANCED))
            return false;
    if ( GMLANMsg[0] == 0x03 && GMLANMsg[1] == 0x7F && GMLANMsg[2] == 0xA2 )
        GMLANShowReturnCode(GMLANMsg[3]);
    return (GMLANMsg[0] == 0x02 && GMLANMsg[1] == 0xE2) ? true : false;
}


bool GMLANProgrammingMode(uint32_t ReqID, uint32_t RespID, char mode)
{
    char GMLANMsg[] = GMLANProgramming;
    GMLANMsg[2] = mode;
    if (!can_send_timeout(ReqID, GMLANMsg, 3, GMLANPTCT))
        return false;
    if (mode == GMLANEnableProgrammingMode)
        return true;                            // No response expected when enabling program mode
    if (!can_wait_timeout(RespID, GMLANMsg, 8, GMLANPTCT))
        return false;
    while (GMLANMsg[0] == 0x03 && GMLANMsg[1] == 0x7F && GMLANMsg[2] == 0xA5 && GMLANMsg[3] == 0x78)
        if (!can_wait_timeout(RespID, GMLANMsg, 8, GMLANPTCTENHANCED))
            return false;
    if ( GMLANMsg[0] == 0x03 && GMLANMsg[1] == 0x7F && GMLANMsg[2] == 0xA5 )
        GMLANShowReturnCode(GMLANMsg[3]);
    return (GMLANMsg[0] == 0x01 && GMLANMsg[1] == 0xE5) ? true : false;
}

bool GMLANSecurityAccessRequest(uint32_t ReqID, uint32_t RespID, char level, uint16_t& seed)
{
    char GMLANMsg[] = GMLANSecurityAccessSeed;
    GMLANMsg[2] = level;
    if (!can_send_timeout(ReqID, GMLANMsg, 3, GMLANPTCT))
        return false;
    if (!can_wait_timeout(RespID, GMLANMsg, 8, GMLANPTCT))
        return false;
    while (GMLANMsg[0] == 0x03 && GMLANMsg[1] == 0x7F && GMLANMsg[2] == 0x27 && GMLANMsg[3] == 0x78)
        if (!can_wait_timeout(RespID, GMLANMsg, 8, GMLANPTCTENHANCED))
            return false;
    seed = GMLANMsg[3] << 8 | GMLANMsg[4];
    if ( GMLANMsg[0] == 0x03 && GMLANMsg[1] == 0x7F && GMLANMsg[2] == 0x27 )
        GMLANShowReturnCode(GMLANMsg[3]);
    return (GMLANMsg[0] == 0x04 && GMLANMsg[1] == 0x67 && GMLANMsg[2] == level) ? true : false;
}

bool GMLANSecurityAccessSendKey(uint32_t ReqID, uint32_t RespID, char level, uint16_t key)
{
    char GMLANMsg[] = GMLANSecurityAccessKey;
    GMLANMsg[2] = level+1;
    GMLANMsg[3] = (key >> 8) & 0xFF;
    GMLANMsg[4] = key & 0xFF;
    if (!can_send_timeout(ReqID, GMLANMsg, 5, GMLANPTCT))
        return false;
    if (!can_wait_timeout(RespID, GMLANMsg, 8, GMLANPTCT))
        return false;
    while (GMLANMsg[0] == 0x03 && GMLANMsg[1] == 0x7F && GMLANMsg[2] == 0x27 && GMLANMsg[3] == 0x78)
        if (!can_wait_timeout(RespID, GMLANMsg, 8, GMLANPTCTENHANCED))
            return false;
    if ( GMLANMsg[0] == 0x03 && GMLANMsg[1] == 0x7F && GMLANMsg[2] == 0x27 )
        GMLANShowReturnCode(GMLANMsg[3]);
    return (GMLANMsg[0] == 0x02 && GMLANMsg[1] == 0x67 && GMLANMsg[2] == level+1) ? true : false;
}

bool GMLANRequestDownload(uint32_t ReqID, uint32_t RespID, char dataFormatIdentifier)
{
    char GMLANMsg[] = GMLANRequestDownloadMessage;
    GMLANMsg[2] = dataFormatIdentifier;
    if (!can_send_timeout(ReqID, GMLANMsg, 7, GMLANPTCT))
        return false;
    if (!can_wait_timeout(RespID, GMLANMsg, 8, GMLANPTCT))
        return false;
    while (GMLANMsg[0] == 0x03 && GMLANMsg[1] == 0x7F && GMLANMsg[2] == 0x34 && GMLANMsg[3] == 0x78) {
        printf("Waiting\r\n");
        GMLANTesterPresent(T8REQID, T8RESPID);
        if (!can_wait_timeout(RespID, GMLANMsg, 8, GMLANPTCTENHANCED))
            return false;
    }
    if ( GMLANMsg[0] == 0x03 && GMLANMsg[1] == 0x7F && GMLANMsg[2] == 0x34 )
        GMLANShowReturnCode(GMLANMsg[3]);
    return (GMLANMsg[0] == 0x01 && GMLANMsg[1] == 0x74) ? true : false;
}


bool GMLANDataTransfer(uint32_t ReqID, uint32_t RespID, char length, char function, uint32_t address)
{
    char GMLANMsg[8];
    GMLANMsg[0] = length;
    GMLANMsg[1] = 0x36;
    GMLANMsg[2] = function;
    GMLANMsg[3] = (char) (address >> 24);
    GMLANMsg[4] = (char) (address >> 16);
    GMLANMsg[5] = (char) (address >> 8);
    GMLANMsg[6] = (char) (address);
    GMLANMsg[7] = 0xaa;
    if (!can_send_timeout(ReqID, GMLANMsg, 8, GMLANPTCT))
        return false;
    if (!can_wait_timeout(RespID, GMLANMsg, 8, GMLANPTCT))
        return false;
    while (GMLANMsg[0] == 0x03 && GMLANMsg[1] == 0x7F && GMLANMsg[2] == 0x36 && GMLANMsg[3] == 0x78)
        if (!can_wait_timeout(RespID, GMLANMsg, 8, GMLANPTCTENHANCED))
            return false;
    if ( GMLANMsg[0] == 0x03 && GMLANMsg[1] == 0x7F && GMLANMsg[2] == 0x36 )
        GMLANShowReturnCode(GMLANMsg[3]);
    return (GMLANMsg[0] == 0x01 && GMLANMsg[1] == 0x76) ? true : false;
}


bool GMLANDataTransferFirstFrame(uint32_t ReqID, uint32_t RespID, char length, char function, uint32_t address)
{
    char GMLANMsg[8];
    GMLANMsg[0] = 0x10;
    GMLANMsg[1] = length;
    GMLANMsg[2] = 0x36;
    GMLANMsg[3] = function;
    GMLANMsg[4] = (char) (address >> 24);
    GMLANMsg[5] = (char) (address >> 16);
    GMLANMsg[6] = (char) (address >> 8);
    GMLANMsg[7] = (char) (address);
    if (!can_send_timeout(ReqID, GMLANMsg, 8, GMLANPTCT))
        return false;
    if (!can_wait_timeout(RespID, GMLANMsg, 8, GMLANPTCT))
        return false;
    while (GMLANMsg[0] == 0x03 && GMLANMsg[1] == 0x7F && GMLANMsg[2] == 0x36 && GMLANMsg[3] == 0x78)
        if (!can_wait_timeout(RespID, GMLANMsg, 8, GMLANPTCTENHANCED))
            return false;
    if ( GMLANMsg[0] == 0x03 && GMLANMsg[1] == 0x7F && GMLANMsg[2] == 0x36 )
        GMLANShowReturnCode(GMLANMsg[3]);
    return (GMLANMsg[0] == 0x30 && GMLANMsg[1] == 0x00 && GMLANMsg[2] == 0x00) ? true : false;
}


bool GMLANDataTransferConsecutiveFrame(uint32_t ReqID, char framenumber, char data[8])
{
    char GMLANMsg[8];
    GMLANMsg[0] = framenumber;
    for (char k = 0; k < 7; k++ )
        GMLANMsg[k+1] = data[k];
    return (can_send_timeout(ReqID, GMLANMsg, 8, GMLANPTCT)) ? true : false;
}

bool GMLANDataTransferBlockAcknowledge(uint32_t RespID)
{
    char GMLANMsg[8];
    if (!can_wait_timeout(RespID, GMLANMsg, 8, GMLANPTCT)) {
        printf("\r\nI did not receive a block acknowledge message\r\n");
        return false;
    }
    if (GMLANMsg[0] != 0x01 && GMLANMsg[1] != 0x76) {
        while (GMLANMsg[0] == 0x03 && GMLANMsg[1] == 0x7F && GMLANMsg[2] == 0x36 && GMLANMsg[3] == 0x78) {
            printf("\r\nI'm waiting for a block acknowledge message\r\n");
            if (!can_wait_timeout(RespID, GMLANMsg, 8, GMLANPTCTENHANCED)) {
                printf("I did not receive a block acknowledge message after enhanced timeout\r\n");
                return false;
            }
        }
        if ( GMLANMsg[0] == 0x03 && GMLANMsg[1] == 0x7F && GMLANMsg[2] == 0x36 ) {
            GMLANShowReturnCode(GMLANMsg[3]);
            return false;
        }
        if (GMLANMsg[0] != 0x01 && GMLANMsg[1] != 0x76) {
            printf("\r\nEXITING due to an unexpected CAN message\r\n");
            return false;
        }
    }
    return true;
}

bool GMLANReturnToNormalMode(uint32_t ReqID, uint32_t RespID)
{
    char GMLANMsg[] = GMLANReturnToNormalModeMessage;
    if (!can_send_timeout(ReqID, GMLANMsg, 2, GMLANPTCT))
        return false;
    if (!can_wait_timeout(RespID, GMLANMsg, 8, GMLANPTCT))
        return false;
    while (GMLANMsg[0] == 0x03 && GMLANMsg[1] == 0x7F && GMLANMsg[2] == 0x20 && GMLANMsg[3] == 0x78)
        if (!can_wait_timeout(RespID, GMLANMsg, 8, GMLANPTCTENHANCED))
            return false;
    if ( GMLANMsg[0] == 0x03 && GMLANMsg[1] == 0x7F && GMLANMsg[2] == 0x20 )
        GMLANShowReturnCode(GMLANMsg[3]);
    return (GMLANMsg[0] == 0x01 && GMLANMsg[1] == 0x60) ? true : false;
}


void GMLANShowReturnCode(char returnCode)
{
    switch (returnCode) {
        case 0x11 :
            printf("0x11 - ServiceNotSupported\r\n");
            break;
        case 0x12 :
            printf("0x12 - SubFunctionNotSupported-InvalidFormat\r\n");
            break;
        case 0x22 :
            printf("0x22 - ConditionsNotCorrectOrRequestSequenceError\r\n");
            break;
        case 0x31 :
            printf("0x31 - RequestOutOfRange\r\n");
            break;
        case 0x35 :
            printf("0x35 - InvalidKey\r\n");
            break;
        case 0x36 :
            printf("0x36 - ExceededNumberOfAttempts, ECU is now locked!\r\n");
            break;
        case 0x37 :
            printf("0x37 - RequiredTimeDelayNotExpired\r\n");
            break;
        case 0x78 :
            printf("0x78 - RequestCorrectlyReceived-ResponsePending\r\n");
            break;
        case 0x81 :
            printf("0x81 - SchedulerFull\r\n");
            break;
        case 0x83 :
            printf("0x83 - VoltageOutOfRangeFault\r\n");
            break;
        case 0x85 :
            printf("0x85 - GeneralProgrammingFailure\r\n");
            break;
        case 0x99 :
            printf("0x99 - ReadyForDownload-DTCStored\r\n");
            break;
        case 0xE3 :
            printf("0xE3 - DeviceControlLimitsExceeded\r\n");
            break;
        default :
            printf("Unknown failure Return Code ?!?\r\n");
    }
}
