#include<avr/io.h>
#include<util/delay.h>
#include<avr/lcd.h>
#include<avr/adc.h>
#include<avr/timer.h>

void main()
{

DDRA=0;
int x=0,count=0;
PORTA=0xff;
DDRC=0xff;
lcd_init();
adc_init();
timer_ctc_init();
while(1)
{
x=adc_read(0);
//y=adc_read(1);
if(x>700)
{
//while(x>700);
OCR0=150;
count++;

}

dis_cmd(0x8f);
dis_number(count);


//dis_cmd(0xCf);
//dis_number(y);
_delay_ms(100);

dis_cmd(1);
_delay_ms(1);


}
}
//*/
//OCR0=150;

