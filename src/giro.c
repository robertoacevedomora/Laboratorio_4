#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/gpio.h>

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
	spi_set_data_size(SPI5, SPI_CR2_DS_8BIT);
	spi_enable_software_slave_management(SPI5);
	spi_send_msb_first(SPI5);
	spi_set_nss_high(SPI5);
	//spi_enable_ss_output(SPI1);
	spi_fifo_reception_threshold_8bit(SPI5);
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
static void clock_setup(void)
{
	rcc_clock_setup_hsi(&rcc_hsi_configs[RCC_CLOCK_HSI_64MHZ]);
}

static void giro_setup(void)
{
    gpio_clear(GPIOE, GPIO3);
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
	gpio_set(GPIOE, GPIO3);    
}

//Funcion que lee y retorna el valor de los 3 ejes
struct Giroscopio read_giro(void)
{
    struct Giroscopio ejes;

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
	ejes.x=spi_read(SPI5);    //En gyr_x se guarda el valor
	gpio_set(GPIOE, GPIO3);

	gpio_clear(GPIOE, GPIO3);
	spi_send(SPI5, GYR_OUT_X_H | GYR_RNW);
	spi_read(SPI5);
	spi_send(SPI5, 0);
	ejes.x|=spi_read(SPI5) << 8;  //Desplazamiento y mascar or
	gpio_set(GPIOE, GPIO3);

}



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
//Faltan agregar ejes y,z.

int main(void)
{
	uint8_t temp;
	int16_t gyr_x; //Variable de 16 bits que guarda el valor de gyr_x. Hacer otras dos para y y z.
	//Las dejamos porque se van a utilizar
    clock_setup();
	gpio_setup();
	usart_setup();
	spi_setup();
    
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
//Esto permite lectura de registros y del eje x, hay que agregarle la parte alta del eje x y los ejes z y y.
//Eliminamos los 8, cambiamos SPI1 por SPI5. Tambien segun la rutina de las diapositvas falta agregarle un read, que
//en este caso aprece con un temp.		
       /** gpio_clear(GPIOE, GPIO3);
		spi_send(SPI1, GYR_WHO_AM_I | GYR_RNW);
		spi_read(SPI1);
		spi_send(SPI1, 0);
		spi_read(SPI1);
		gpio_set(GPIOE, GPIO3);

		gpio_clear(GPIOE, GPIO3);
		spi_send(SPI1, GYR_STATUS_REG | GYR_RNW);
		spi_read(SPI1);
		spi_send(SPI1, 0);
		spi_read(SPI1);
		gpio_set(GPIOE, GPIO3);

		gpio_clear(GPIOE, GPIO3);
		spi_send(SPI1, GYR_OUT_TEMP | GYR_RNW);
		spi_read(SPI1);
		spi_send(SPI1, 0);
		spi_read(SPI1);
		gpio_set(GPIOE, GPIO3);

		gpio_clear(GPIOE, GPIO3);
		spi_send(SPI1, GYR_OUT_X_L | GYR_RNW);
		spi_read(SPI1);
		spi_send(SPI1, 0);
		gyr_x=spi_read(SPI1);    //En gyr_x se guarda el valor
		gpio_set(GPIOE, GPIO3);

		gpio_clear(GPIOE, GPIO3);
		spi_send(SPI1, GYR_OUT_X_H | GYR_RNW);
		spi_read(SPI1);
		spi_send(SPI1, 0);
		gyr_x|=spi_read(SPI1) << 8;  //Desplazamiento y mascar or
		gpio_set(GPIOE, GPIO3); **/

//Para el ciclo
		int i;
		for (i = 0; i < 80000; i++)    /* Wait a bit. */
			__asm__("nop");
	}

	return 0;
}
