
#include "max6675.h"
#include "mstd_type_traits"

max6675::max6675(PinName miso, PinName sclk, PinName cs) :
    max(NC, miso, sclk), _cs(cs)
{
    max.format(16,1);   // set 16 bit SPI format
    max.frequency(400000);
}

float max6675::gettemp(int cf)
{
    float temp  = 0;
    int tempByte= 0;
    
    _cs = 0;
    wait_us(1);     // wait to stablize
    tempByte  =  max.write(0);
    wait_us(1);     // wait to finish
    _cs = 1;

    if (tempByte & (1<<2)) { // faulty or no sensor connected
        return -99;
    } else {
        temp = (tempByte)/32.0f;
    }
    if(cf) {
        temp = (temp*9.0f/5.0f) + 32.0f; // Convert value to ˚F
    }
    return temp;
}

void max6675::printTemp(void)
{
    printf("[HMAX6675 Thermocouple!\r\n");
    float temp = gettemp(0);
    printf("Temp: %4.2f%cC \r\n", temp, 176);
}
