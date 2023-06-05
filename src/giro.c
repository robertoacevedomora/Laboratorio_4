//Incluyo las bibliotecas
//libs
#include <errno.h>
#include <stdio.h>
#include <unistd.h>
#include <stdint.h>    //Del archivo lcd-spi.h
#include <math.h>     //Del archivo lcd-spi.h
#include <string.h>

#include "clock.h"   //Del archivo lcd-spi.h
#include "console.h" //Del archivo lcd-spi.h
#include "sdram.h"   //Del archivo lcd-spi.h   
#include "lcd-spi.h" //Del archivo lcd-spi.h
#include "gfx.h"

#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/dac.h>
#include <libopencm3/stm32/rcc.h>   //Del archivo spi.c, examples f3
#include <libopencm3/stm32/usart.h> //Del archivo spi.c, examples f3
#include <libopencm3/stm32/spi.h>   //Del archivo spi.c, examples f3
#include <libopencm3/stm32/gpio.h>  //Del archivo spi.c, examples f3




//Del archivo spi.c, examples, stm32, f3. Para utilizar el giroscopio 
#define GYR_RNW			(1 << 7) /* Write when zero */
#define GYR_MNS			(1 << 6) /* Multiple reads when 1 */
#define GYR_WHO_AM_I		0x0F      //Registro identificacion
#define GYR_OUT_TEMP		0x26      //Registro temperatura
#define GYR_STATUS_REG		0x27      //Registro estado
#define GYR_CTRL_REG1		0x20      //Registro control
#define GYR_CTRL_REG1_PD	(1 << 3)  //Registro power_down
#define GYR_CTRL_REG1_XEN	(1 << 1)  //Habilito eje x
#define GYR_CTRL_REG1_YEN	(1 << 0)  //Habilito eje y
#define GYR_CTRL_REG1_ZEN	(1 << 2)  //Habilito eje z
#define GYR_CTRL_REG1_BW_SHIFT	4     //Ancho de banda
#define GYR_CTRL_REG4		0x23      //SPI serial interface mode selection.
#define GYR_CTRL_REG4_FS_SHIFT	4     //Full scale selection

#define GYR_OUT_X_L		0x28 //Direccion eje x. X-axis angular rate data. The value is expressed as two’s complement.
#define GYR_OUT_X_H		0x29 //Direccion eje x
#define GYR_OUT_Y_L		0x2A //Direccion eje y. Y-axis angular rate data. The value is expressed as two’s complement.
#define GYR_OUT_Y_H		0x2B //Direccion eje y
#define GYR_OUT_Z_L		0x2C //Direccion eje z. Z-axis angular rate data. The value is expressed as two’s complement.
#define GYR_OUT_Z_H		0x2D //Direccion eje z


#define Sensibilidad  (0.00875F) //Sensibilidad, caracteristica mecanica 8.75


//Funcion para configurar la comunicacion SPI.
//Funcion extraida del archivo spi.c, de los ejemplos. 
//Los puertos se elijieron segun el archivo f4, spi-mems.c
static void spi_setup(void)
{
	//Se cambiaron los puertos A -> C y E -> F.
	//Se cambio el SPI, SPI1 -> SPI5.
	rcc_periph_clock_enable(RCC_SPI5);  //Habilito reloj SPI5.
	rcc_periph_clock_enable(RCC_GPIOC); //Habilito reloj Puerto C
	rcc_periph_clock_enable(RCC_GPIOF); //Hbailito reloj Puerto F

	gpio_mode_setup(GPIOC, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO1); //Pin 1, puerto C como salida
/* Start with spi communication disabled */
	gpio_set(GPIOC, GPIO1);
//Pin 7, 8 y 9 del puerto F funciones SPI.
	gpio_mode_setup(GPIOF, GPIO_MODE_AF, GPIO_PUPD_NONE,
			GPIO7 | GPIO8 | GPIO9);                       
	gpio_set_af(GPIOF, GPIO_AF5, GPIO7 | GPIO8 | GPIO9);

	//SPI initialization
	//Inicializar y configurar el protocolo SPI
    //Comunicacion con el giroscopio.
	spi_set_master_mode(SPI5);
	spi_set_baudrate_prescaler(SPI5, SPI_CR1_BR_FPCLK_DIV_64);
	spi_set_clock_polarity_0(SPI5); //Polaridad del reloj
	spi_set_clock_phase_0(SPI5);    //Fase del reloj
	spi_set_full_duplex_mode(SPI5);  //Comuniacion duplex
	spi_set_unidirectional_mode(SPI5);  
	spi_enable_software_slave_management(SPI5);
	spi_send_msb_first(SPI5);
	spi_set_nss_high(SPI5);
	SPI_I2SCFGR(SPI5) &= ~SPI_I2SCFGR_I2SMOD;
	spi_enable(SPI5); //Habilita el protrocolo SPI
}


//Funcion extraida del archivo spi.c, de los ejemplos. 
static void usart_setup(void)
{
	
	/* Setup GPIO pin GPIO_USART2_TX/GPIO9 on GPIO port A for transmit. */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
	gpio_set_af(GPIOA, GPIO_AF7, GPIO9);

	//Configuramos los parametros para el USART
	usart_set_baudrate(USART1, 115200); //Velocidad
	usart_set_databits(USART1, 8);     //Bits
	usart_set_stopbits(USART1, USART_STOPBITS_1); //Paradas
	usart_set_mode(USART1, USART_MODE_TX);        //Configuro USART1 para la transmision
	usart_set_parity(USART1, USART_PARITY_NONE); //Elimino bits de paridad
	usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE); //Elimino control de flujo
	usart_enable(USART1);                                     //Habilito USART 1
}

//Configurando el USART, puerto B pines 2 y 3.

static void led_con_setup(void)
{
	rcc_periph_clock_enable(RCC_GPIOG); //Habilitamos reloj del puerto G
	rcc_periph_clock_enable(RCC_GPIOA);  //Habilitamos reloj del puerto A
	gpio_mode_setup(GPIOA, GPIO_MODE_INPUT, GPIO_PUPD_NONE, GPIO0);  //Puerto A, pin 0 como entrada. Boton
	gpio_mode_setup(GPIOG, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, //Puerto G, pin 13 como salida. LED
		GPIO13);
    gpio_mode_setup(GPIOG, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE,
		GPIO14); //Puerto G, pin 14 como salida. LED
}


//GPIO configurados
//Eliminamos de momento esta funcion, no hace falta (my_usart_print_int)

//giro_setup, tomamos como referencia el ejemplo spi.c, especificamente la funcion main
//Control ejes y mode de poder
static void giro_setup(void)
{
    gpio_clear(GPIOC, GPIO1); //Modificamos el chip select para poner a funcionar el giroscopio.
	spi_send(SPI5, GYR_CTRL_REG1); //Enviamos registro
	spi_read(SPI5); //Leemos la respuesta
	spi_send(SPI5, GYR_CTRL_REG1_PD | GYR_CTRL_REG1_XEN |
			GYR_CTRL_REG1_YEN | GYR_CTRL_REG1_ZEN |
<<<<<<< HEAD
			(3 << GYR_CTRL_REG1_BW_SHIFT)); //Enviamos registros de control eje x, y y z
	spi_read(SPI5); //Leemos la respuesta
	gpio_set(GPIOC, GPIO1); // Modificamos otra vez el chip select para poner detener el funcionamiento del giroscopio.
    
	//Configuracion dps y modo spi
	gpio_clear(GPIOC, GPIO1); //Modificamos el chip select para poner a funcionar el giroscopio.
	spi_send(SPI5, GYR_CTRL_REG4); //Enviamos el registro
	spi_read(SPI5); //Lectura de la respuesta
	spi_send(SPI5, (1 << GYR_CTRL_REG4_FS_SHIFT)); //Enviamos escala de medicion
	spi_read(SPI5); //Lectura de la respuesta
	gpio_set(GPIOC, GPIO1);  // Modificamos otra vez el chip select para poner detener el funcionamiento del giroscopio.  
=======

			(3 << GYR_CTRL_REG1_BW_SHIFT));
	spi_read(SPI5);
	gpio_set(GPIOC, GPIO1);

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_CTRL_REG4);
	spi_read(SPI5);
	spi_send(SPI5, (1 << GYR_CTRL_REG4_FS_SHIFT));
	spi_read(SPI5);
	gpio_set(GPIOC, GPIO1);    
>>>>>>> 8fe111d5bfb8ea5c319a166c405a68ae443d0426
}

//Estructura Giroscopio, con los 3 ejes
//Funcion que lee y retorna el valor de los 3 ejes
struct Giroscopio 
{
    int16_t gyr_x; //Variable de 16 bits, para guardar parte alta y baja del eje x.
    int16_t gyr_y; //Variable de 16 bits, para guardar parte alta y baja del eje y. 
    int16_t gyr_z; //Variable de 16 bits, para guardar parte alta y baja del eje z. 
};
typedef struct Giroscopio read_giro;

read_giro leer_ejes(void);
read_giro leer_ejes(void)
{
    read_giro eje;
//Parte explicada en la presentacion para enviar y recibir datos, hacerlo para cada uno de los ejes
    gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_WHO_AM_I | 0x80); //Leer el WHO I AM , or con 0x80. Registro de identificación del dispositivo
	spi_read(SPI5);
	spi_send(SPI5, 0);
	spi_read(SPI5);
	gpio_set(GPIOC, GPIO1);

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_STATUS_REG | GYR_RNW); //Registro de  lectura
	spi_read(SPI5);
	spi_send(SPI5, 0);
	spi_read(SPI5);
	gpio_set(GPIOC, GPIO1);

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_OUT_TEMP | GYR_RNW); //Registro de  lectura
	spi_read(SPI5);
	spi_send(SPI5, 0);
	spi_read(SPI5);
	gpio_set(GPIOC, GPIO1);

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_OUT_X_L | GYR_RNW); //Registro de  lectura
	spi_read(SPI5);
	spi_send(SPI5, 0);
	eje.gyr_x=spi_read(SPI5);    //En gyr_x se guarda el valor
	gpio_set(GPIOC, GPIO1);

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_OUT_X_H | GYR_RNW); //Registro de  lectura
	spi_read(SPI5);
	spi_send(SPI5, 0);
	eje.gyr_x|=spi_read(SPI5) << 8;  //Desplazamiento y mascar or
	gpio_set(GPIOC, GPIO1);

    gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_OUT_Y_L | GYR_RNW); //Registro de  lectura
	spi_read(SPI5);
	spi_send(SPI5, 0);
	eje.gyr_y=spi_read(SPI5);    //En gyr_x se guarda el valor
	gpio_set(GPIOC, GPIO1);

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_OUT_Y_H | GYR_RNW); //Registro de  lectura
	spi_read(SPI5);
	spi_send(SPI5, 0);
	eje.gyr_y|=spi_read(SPI5) << 8;  //Desplazamiento y mascar or
	gpio_set(GPIOC, GPIO1);

    gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_OUT_Z_L | GYR_RNW); //Registro de  lectura
	spi_read(SPI5);
	spi_send(SPI5, 0);
	eje.gyr_z=spi_read(SPI5);    //En gyr_x se guarda el valor
	gpio_set(GPIOC, GPIO1);

	gpio_clear(GPIOC, GPIO1);
	spi_send(SPI5, GYR_OUT_Z_H | GYR_RNW); //Registro de  lectura
	spi_read(SPI5);
	spi_send(SPI5, 0);
	eje.gyr_z|=spi_read(SPI5) << 8;  //Desplazamiento y mascar or
	gpio_set(GPIOC, GPIO1);

	eje.gyr_x = eje.gyr_x*Sensibilidad; //Sensibilidad del giroscopio
	eje.gyr_y = eje.gyr_y*Sensibilidad; //Sensibilidad del giroscopio
	eje.gyr_z = eje.gyr_z*Sensibilidad; //Sensibilidad del giroscopio

    return eje;

}

/***********Convertidor analogico digital, funciones del archivo ejemplo********************/
//Configura el convertidor analogico digital, referencias del ejemplo adc-dac-printf.c
static void adc_setup(void)
{
	//Imprime el valor del convertidor analogico digital en el pin 1 del puerto A.
	gpio_mode_setup(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO1);//Configuramos el pin 1 del puerto A como entrada.

	adc_power_off(ADC1);      //Apagamos el convertidor
	adc_disable_scan_mode(ADC1); //Quitamos el scan mode
	adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_3CYC); //Muestreamos

	adc_power_on(ADC1); //Encendemos

}

//Funcion para leer el valor analogico de ADC1, ejemplo adc-dac-printf.
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

	if (num < 0) { //Revisa si el numero es negativo o positivo
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
	
    read_giro eje;
	//Despues de la declaracion, inicializo en cero
	eje.gyr_x = 0;
	eje.gyr_y = 0;
	eje.gyr_z = 0;	
	//Cadena de caracteres para agregar el valor de los ejes y mensaje
	char gyrp_x[10];
	char gyrp_y[10];
	char gyrp_z[10];
	//Conexion USART
	char * USB_cone = "OFF"; 
	char cone[10];
	bool conexion = false; 
	
	//Nivel de la bateria
	float nivel; //Variable nivel de bateria
	nivel = 0;
	char nivel_p[10];		
	uint16_t ana_dig;              //Entero de la funcion adc

	//Auxiliar
	char aux[10] = "";
	
	
	//Declaracion de funciones 

	console_setup(115200);//LCD-SERIAL
	clock_setup();//LCD-SERIAL
	rcc_periph_clock_enable(RCC_USART1);    // Habilitacion del reloj del periferico USART1
	rcc_periph_clock_enable(RCC_ADC1);     //Habilitacion adc, aun falta gregar estas dos partes bien.
	led_con_setup();
	adc_setup();
	sdram_init();
	usart_setup(); //Funciones para el correcto funcionamiento del giroscopio.
	spi_setup();
    giro_setup();
	lcd_spi_init();
	gfx_init(lcd_draw_pixel, 240, 320);

	


	while (1) {

		//Formatear los valores de los ejes a cadenas de caracteres


		sprintf(gyrp_x, "%s", "Eje x:");       
		sprintf(aux, "%d",  eje.gyr_x);
		strcat(gyrp_x ,aux); 

		sprintf(gyrp_y, "%s", "Eje y:");       
		sprintf(aux, "%d",  eje.gyr_y);
		strcat(gyrp_y ,aux);

		sprintf(gyrp_z, "%s", "Eje z:");       //Cadena, si se pone uno por uno se difuclta y la pantalla se traba.
		sprintf(aux, "%d",  eje.gyr_z);
		strcat(gyrp_z ,aux);

//Mensajes pantalla LCD ejes

//Si se ponen muchas cosas no carga
//Este blopque de codigo viene del archivo example, lcd-serial.c 
		
		gfx_fillScreen(LCD_CYAN); //Lena la pantalla principal
		gfx_setCursor(15, 20);
		gfx_setTextColor(LCD_BLUE, LCD_WHITE);
		gfx_setTextSize(2);
		gfx_puts("Giroscopio");
//Elegir colores que resalten
	    gfx_setCursor(15,70);
		gfx_setTextSize(2);
		gfx_setTextColor(LCD_BLUE,LCD_WHITE);
		gfx_puts(gyrp_x);
//setCursor, posicion en pantalla donde colocamos los datos
//TextSize, tamano de las letras. Probamos con 3 y 1, pero 2 se ve bien
//puts, agregamos la cadena de caracteres despues de cambiar el formato del valor de los ejes
		gfx_setCursor(15,100);
		gfx_setTextSize(2);
		gfx_setTextColor(LCD_BLUE,LCD_WHITE);
		gfx_puts(gyrp_y);


		gfx_setCursor(15,130);
		gfx_setTextSize(2);
		gfx_setTextColor(LCD_BLUE,LCD_WHITE);
		gfx_puts(gyrp_z);


 //Mensaje pantalla nivel bateria       
	
	    sprintf(nivel_p, "%s", "Nivel:");
		sprintf(aux, "%f",  nivel);
		strcat(nivel_p, aux);
	

		gfx_setCursor(15,160);
		gfx_setTextSize(2);
		gfx_setTextColor(LCD_BLUE,LCD_WHITE);
		gfx_puts(nivel_p);



 //Mensaje pantalla LCD conexion

		sprintf(cone, "%s", "Conexion:");
		sprintf(aux, "%s",  USB_cone);
		strcat(cone, aux);

		gfx_setCursor(15,190);
		gfx_setTextSize(2);
		gfx_setTextColor(LCD_BLUE,LCD_WHITE);
		gfx_puts(cone);




		//Funciones pantalla
		lcd_show_frame();
		gpio_clear(GPIOC, GPIO1); 
		eje = leer_ejes(); //Leer el valor de los ejes
		gpio_set(GPIOC, GPIO1); 
        ana_dig = read_adc_naiive(1);
		nivel = ((ana_dig*9)/4095);
		

//Esto permite lectura de registros y del eje x, hay que agregarle la parte alta del eje x y los ejes z y y.
//Eliminamos los 8, cambiamos SPI1 por SPI5. Tambien segun la rutina de las diapositvas falta agregarle un read, que
//en este caso aprece con un temp.		

		if (conexion)                               
		{ 
			USB_cone = "ON";            // Indica si puerto funciona
			print_decimal(eje.gyr_x);             
			console_puts("\t");
       	 	print_decimal(eje.gyr_y);
			console_puts("\t");
        	print_decimal(eje.gyr_z); 
			console_puts("\t");
			print_decimal(nivel); 
			USB_cone = "ON"; 
			console_puts("\n");
			// Toggle del pin 13, puerto G, parpadeo indica envio exitoso 
			gpio_toggle(GPIOG, GPIO13);     
		}
		
		else{                                     
			USB_cone = "OFF";        
			//Conexion OFF, no parpadea y se apaga
			gpio_clear(GPIOG, GPIO13);             
		}

		
		if (nivel<7)
		{   
			// Pin 14 parpadea, indica bateria baja nivel menor a 7
			gpio_toggle(GPIOG, GPIO14);
		}

		else gpio_clear(GPIOG, GPIO14); //Bateria alta, apaga el led pin 14 puerto G
		

		//Revisa si el boton esta en alto o bajo
		if (gpio_get(GPIOA, GPIO0)) {      
			
			if (conexion) {
				conexion = false; //No hay conexion
				gpio_clear(GPIOG, GPIO13); 
			}
			
			else conexion = true;
		}
//Para el ciclo
		int i;
		for (i = 0; i < 80000; i++)    /* Wait a bit. */
			__asm__("nop");
	}

	return 0;
}
