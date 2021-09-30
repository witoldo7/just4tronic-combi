#ifndef max6675_h
#define max6675_h

#include "mbed.h"

/*
#include "mbed.h"
#include "max6675.h"

max6675 sensor(D5,D3,D6);  //miso, sclk, cs
Serial pc(USBTX,USBRX);

int main()
{
    pc.baud(921600);
    pc.printf("\033[0m\033[2J\033[HMAX6675 Thermocouple!\r\n\n\n");

    int cf = 0; // 0 Centigrade, 1 Fahrenheit

    while (1) {

        float temp = sensor.gettemp(cf);
        if (cf) {
            printf(" Temp: %4.2f%cF \n\033[2K\033[1A",temp,176);
        } else {
            printf(" Temp: %4.2f%cC \n\033[2K\033[1A",temp,176);
        }
        wait_ms(250);   // requires 250mS for temperature conversion process
    }
}
*/


class max6675
{    
  public:
  
    max6675(PinName miso, PinName sclk, PinName cs);
        
    // read temperature 0 Centigrade, 1 Fahrenheit       
    float gettemp(int cf);    
    void printTemp(void);
    
  private:  
    SPI max;
    DigitalOut _cs;
    Timer t;    
};

#endif
