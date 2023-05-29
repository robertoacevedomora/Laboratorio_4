//Incluyo las bibliotecas
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>//LCD-SERIAL
#include <math.h>//LCD-SERIAL

#include "clock.h"
#include "console.h"
#include "sdram.h"
#include "lcd-spi.h"
#include "gfx.h"

#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>




//Giroscopio
#define GYR_RNW			(1 << 7) /* Write when zero */
#define GYR_MNS			(1 << 6) /* Multiple reads when 1 */
#define GYR_WHO_AM_I		0x0F
#define GYR_OUT_TEMP		0x26
#define GYR_STATUS_REG		0x27
#define GYR_CTRL_REG1		0x20
#define GYR_CTRL_REG1_PD	(1 << 3)
#define GYR_CTRL_REG1_XEN	(1 << 1)
#define GYR_CTRL_REG1_YEN	(1 << 0)
#define GYR_CTRL_REG1_ZEN	(1 << 2)
#define GYR_CTRL_REG1_BW_SHIFT	4
#define GYR_CTRL_REG4		0x23
#define GYR_CTRL_REG4_FS_SHIFT	4

#define GYR_OUT_X_L		0x28
#define GYR_OUT_X_H		0x29
#define GYR_OUT_Y_L		0x2A
#define GYR_OUT_Y_H		0x2B
#define GYR_OUT_Z_L		0x2C
#define GYR_OUT_Z_H		0x2D

//Faltan agregar ejes y,z.

#define L3GD20_SENSITIVITY_250DPS  (0.00875F) //Sensibilidad


//Funcion para configurar la comunicacion spi, en este caso hay que usar SPI5. 
//Utilizamos GPIOA y GPIOE. GPIO3, GPI05, GPIO6,GPIO7.
static void spi_setup(void)
{
	rcc_periph_clock_enable(RCC_SPI5);
	/* For spi signal pins */
	rcc_periph_clock_enable(RCC_GPIOA);
	/* For spi mode select on the l3gd20 */
	rcc_periph_clock_enable(RCC_GPIOE);

	/* Setup GPIOE3 pin for spi mode l3gd20 select. */
	gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO3);
	/* Start with spi communication disabled */
	gpio_set(GPIOE, GPIO3);

	/* Setup GPIO pins for AF5 for SPI1 signals. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE,
			GPIO5 | GPIO6 | GPIO7);
	gpio_set_af(GPIOA, GPIO_AF5, GPIO5 | GPIO6 | GPIO7);

	//spi initialization;
    //Comunicacion con el giroscopio.
	spi_set_master_mode(SPI5);
	spi_set_baudrate_prescaler(SPI5, SPI_CR1_BR_FPCLK_DIV_64);
	spi_set_clock_polarity_0(SPI5);
	spi_set_clock_phase_0(SPI5);
	spi_set_full_duplex_mode(SPI5);
	spi_set_unidirectional_mode(SPI5); /* bidirectional but in 3-wire */
	//spi_set_data_size(SPI5, SPI_CR1_DS_8BIT);
	spi_enable_software_slave_management(SPI5);
	spi_send_msb_first(SPI5);
	spi_set_nss_high(SPI5);
	//spi_enable_ss_output(SPI1);
	//spi_fifo_reception_threshold_8bit(SPI5);
	SPI_I2SCFGR(SPI5) &= ~SPI_I2SCFGR_I2SMOD;
	spi_enable(SPI5);
}

//Hasta aqui todo bien
//Cambiamos aqui el purto A por el B para no generar confuciones.
static void usart_setup(void)
{
	/* Enable clocks for GPIO port A (for GPIO_USART2_TX) and USART2. */
	rcc_periph_clock_enable(RCC_USART2);
	rcc_periph_clock_enable(RCC_GPIOB);

	/* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
	gpio_mode_setup(GPIOB, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO2 | GPIO3);
	gpio_set_af(GPIOB, GPIO_AF7, GPIO2| GPIO3);

	/* Setup UART parameters. */
	usart_set_baudrate(USART2, 115200);
	usart_set_databits(USART2, 8);
	usart_set_stopbits(USART2, USART_STOPBITS_1);
	usart_set_mode(USART2, USART_MODE_TX_RX);
	usart_set_parity(USART2, USART_PARITY_NONE);
	usart_set_flow_control(USART2, USART_FLOWCONTROL_NONE);

	/* Finally enable the USART. */
	usart_enable(USART2);
}

//Configurando el USART, puerto B pines 2 y 3.

//Funcion para configurar pines como entradas y salidas. Elejimos el puerto D.
static void gpio_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOD);
	gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
		GPIO8 | GPIO9 | GPIO10 | GPIO11 | GPIO12 | GPIO13 |
		GPIO14 | GPIO15);
    gpio_mode_setup(GPIOD, GPIO_MODE_INPUT, GPIO_PUPD_NONE,
		GPIO4 | GPIO5 ); //Para onexiones posteriores, pensando en USB, etc
}


//GPIO configurados
//Eliminamos de momento esta funcion, no hace falta (my_usart_print_int)

//De momento dejamos clock_setup


static void giro_setup(void)
{
    gpio_clear(GPIOE, GPIO3);
	spi_send(SPI5, GYR_CTRL_REG1);
	spi_read(SPI5);
	spi_send(SPI5, GYR_CTRL_REG1_PD | GYR_CTRL_REG1_XEN |
			GYR_CTRL_REG1_YEN | GYR_CTRL_REG1_ZEN |
			(3 << GYR_CTRL_REG1_BW_SHIFT));
	spi_read(SPI5);
	gpio_set(GPIOE, GPIO3);

	gpio_clear(GPIOE, GPIO3);
	spi_send(SPI5, GYR_CTRL_REG4);
	spi_read(SPI5);
	spi_send(SPI5, (1 << GYR_CTRL_REG4_FS_SHIFT));
	spi_read(SPI5);
	gpio_set(GPIOE, GPIO3);    
}

//Funcion que lee y retorna el valor de los 3 ejes
struct Giroscopio 
{
    int16_t gyr_x;
    int16_t gyr_y;
    int16_t gyr_z;
};
typedef struct Giroscopio read_giro;

read_giro Giro_ejes(void);
read_giro Giro_ejes(void)
{
    read_giro eje;

    gpio_clear(GPIOE, GPIO3);
	spi_send(SPI5, GYR_WHO_AM_I | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	spi_read(SPI5);
	gpio_set(GPIOE, GPIO3);

	gpio_clear(GPIOE, GPIO3);
	spi_send(SPI5, GYR_STATUS_REG | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	spi_read(SPI5);
	gpio_set(GPIOE, GPIO3);

	gpio_clear(GPIOE, GPIO3);
	spi_send(SPI5, GYR_OUT_TEMP | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	spi_read(SPI5);
	gpio_set(GPIOE, GPIO3);

	gpio_clear(GPIOE, GPIO3);
	spi_send(SPI5, GYR_OUT_X_L | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	eje.gyr_x=spi_read(SPI5);    //En gyr_x se guarda el valor
	gpio_set(GPIOE, GPIO3);

	gpio_clear(GPIOE, GPIO3);
	spi_send(SPI5, GYR_OUT_X_H | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	eje.gyr_x|=spi_read(SPI5) << 8;  //Desplazamiento y mascar or
	gpio_set(GPIOE, GPIO3);

    gpio_clear(GPIOE, GPIO3);
	spi_send(SPI5, GYR_OUT_Y_L | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	eje.gyr_y=spi_read(SPI5);    //En gyr_x se guarda el valor
	gpio_set(GPIOE, GPIO3);

	gpio_clear(GPIOE, GPIO3);
	spi_send(SPI5, GYR_OUT_Y_H | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	eje.gyr_y|=spi_read(SPI5) << 8;  //Desplazamiento y mascar or
	gpio_set(GPIOE, GPIO3);

    gpio_clear(GPIOE, GPIO3);
	spi_send(SPI5, GYR_OUT_Z_L | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	eje.gyr_z=spi_read(SPI5);    //En gyr_x se guarda el valor
	gpio_set(GPIOE, GPIO3);

	gpio_clear(GPIOE, GPIO3);
	spi_send(SPI5, GYR_OUT_Z_H | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	eje.gyr_z|=spi_read(SPI5) << 8;  //Desplazamiento y mascar or
	gpio_set(GPIOE, GPIO3);

	eje.gyr_x = eje.gyr_x*L3GD20_SENSITIVITY_250DPS;
	eje.gyr_y = eje.gyr_y*L3GD20_SENSITIVITY_250DPS;
	eje.gyr_z = eje.gyr_z*L3GD20_SENSITIVITY_250DPS;

    return eje;

}

/***********Convertidor analogico digital, funciones del archivo ejemplo********************/
//Congiura el convertidor analogico digital
static void adc_setup(void)
{
	//Prints the ADC value on PA0 (adc channel 0) on the console, eliminamo el A1
   // gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0); //Revisarlo, solo usamos uno adc
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);

	adc_power_off(ADC1);
	adc_disable_scan_mode(ADC1);
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_3CYC);

	adc_power_on(ADC1);

}

//Funcion para leer el convertidor analogico digital
static uint16_t read_adc_naiive(uint8_t channel)
{
	uint8_t channel_array[16];
	channel_array[0] = channel;
	adc_set_regular_sequence(ADC1, 1, channel_array);
	adc_start_conversion_regular(ADC1);
	while (!adc_eoc(ADC1));
	uint16_t reg16 = adc_read_regular(ADC1);
	return reg16;
}

//Para ver los valores en la consola(spi.mems)
int print_decimal(int);

int print_decimal(int num)
{
	int		ndx = 0;
	char	buf[10];
	int		len = 0;
	char	is_signed = 0;

	if (num < 0) {
		is_signed++;
		num = 0 - num;
	}
	buf[ndx++] = '\000';
	do {
		buf[ndx++] = (num % 10) + '0';
		num = num / 10;
	} while (num != 0);
	ndx--;
	if (is_signed != 0) {
		console_putc('-');
		len++;
	}
	while (buf[ndx] != '\000') {
		console_putc(buf[ndx--]);
		len++;
	}
	return len; /* number of characters printed */
}

int main(void)
{
	//uint8_t temp;
	//int16_t gyr_x; //Variable de 16 bits que guarda el valor de gyr_x. Hacer otras dos para y y z.
	
	//Las dejamos porque se van a utilizar
    read_giro eje;
	//Despues de la declaracion, inicializo en cero
	//eje.gyr_x = 0;
	//eje.gyr_y = 0;
	//eje.gyr_z = 0;
	
	gpio_setup();
	usart_setup();
	spi_setup();
    giro_setup();
    adc_setup();
	clock_setup();//LCD-SERIAL
	console_setup(115200);//LCD-SERIAL
	sdram_init();
	lcd_spi_init();
	console_puts("LCD Initialized\n");
	console_puts("Should have a checker pattern, press any key to proceed\n");
	msleep(2000);
/*	(void) console_getc(1); */
	gfx_init(lcd_draw_pixel, 240, 320);
	//gfx_fillScreen(LCD_GREY);
	
	//Para realizar la lectura hay que formatear, de entero a cadena de caracteres.
	//Creamos una cadena de caracteres
	//Encabezados
	char gyrpe_x[10];
	char gyrpe_y[10];
	char gyrpe_z[10];
	
	//Valores
	char gyrp_x[10];
	char gyrp_y[10];
	char gyrp_z[10];

	//Inicializar valores en cero
	
    
    // Comandos configuracion para el giroscopio, queda mejor como una nueva funcion porque el main lleva mas cosas.
	/**gpio_clear(GPIOE, GPIO3);
	spi_send(SPI1, GYR_CTRL_REG1);
	spi_read(SPI1);
	spi_send(SPI1, GYR_CTRL_REG1_PD | GYR_CTRL_REG1_XEN |
			GYR_CTRL_REG1_YEN | GYR_CTRL_REG1_ZEN |
			(3 << GYR_CTRL_REG1_BW_SHIFT));
	spi_read(SPI1);
	gpio_set(GPIOE, GPIO3);

	gpio_clear(GPIOE, GPIO3);
	spi_send(SPI1, GYR_CTRL_REG4);
	spi_read(SPI1);
	spi_send(SPI1, (1 << GYR_CTRL_REG4_FS_SHIFT));
	spi_read(SPI1);
	gpio_set(GPIOE, GPIO3);**/

	while (1) {

		//Formatear los valores de los ejes a cadenas de caracteres


		sprintf(gyrpe_x, "%s", "Eje x:");       // Escribir el string "X:" en la cadena print_x
		sprintf(gyrp_x, "%d",  eje.gyr_x);
		sprintf(gyrpe_y, "%s", "Eje y:");       // Escribir el string "X:" en la cadena print_x
		sprintf(gyrp_y, "%d",  eje.gyr_y);
		sprintf(gyrpe_z, "%s", "Eje z:");       // Escribir el string "X:" en la cadena print_x
		sprintf(gyrp_z, "%d",  eje.gyr_z);

//Este blopque de codigo viene del archivo example, lcd-serial.c 
		
		gfx_fillScreen(LCD_CYAN); //Lena la pantalla principal
		gfx_setCursor(15, 36);
		gfx_setTextColor(LCD_BLUE, LCD_BLACK);
		gfx_setTextSize(2);
		gfx_puts("GIROSCOPIO");
		
	    gfx_setCursor(15,70);
		gfx_setTextSize(1);
		gfx_setTextColor(LCD_BLACK,LCD_GREEN);
		gfx_puts(gyrp_x);

		gfx_setCursor(15,100);
		gfx_setTextSize(1);
		gfx_setTextColor(LCD_BLACK,LCD_GREEN);
		gfx_puts(gyrp_y);

		gfx_setCursor(15,130);
		gfx_setTextSize(1);
		gfx_setTextColor(LCD_BLACK,LCD_GREEN);
		gfx_puts(gyrp_z);

		gfx_setCursor(15,190);
		gfx_setTextSize(2);
		gfx_setTextColor(LCD_BLACK,LCD_GREEN);
		gfx_puts(gyrpe_x);
		
		//gfx_fillCircle(120, 160, 40, LCD_YELLOW);
		lcd_show_frame();

//Esto permite lectura de registros y del eje x, hay que agregarle la parte alta del eje x y los ejes z y y.
//Eliminamos los 8, cambiamos SPI1 por SPI5. Tambien segun la rutina de las diapositvas falta agregarle un read, que
//en este caso aprece con un temp.		

//Para el ciclo
		int i;
		for (i = 0; i < 80000; i++)    /* Wait a bit. */
			__asm__("nop");
	}

	return 0;
}
