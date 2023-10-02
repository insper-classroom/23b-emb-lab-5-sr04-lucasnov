#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_CALC_DIST_STACK_SIZE           (1024*6/sizeof(portSTACK_TYPE))
#define TASK_CALC_DIST_STACK_PRIORITY       (tskIDLE_PRIORITY)

#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

#define TRIG_PIO     PIOD
#define TRIG_PIO_ID  ID_PIOD
#define TRIG_PIO_PIN 30
#define TRIG_PIO_PIN_MASK (1 << TRIG_PIO_PIN)

#define ECHO_PIO     PIOC
#define ECHO_PIO_ID  ID_PIOC
#define ECHO_PIO_PIN 13
#define ECHO_PIO_PIN_MASK (1u << ECHO_PIO_PIN)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);

static void BUT_init(void);

static void RTT_init(void);

void echo_callback(void);

QueueHandle_t xQueueEcho;
SemaphoreHandle_t xSemaphore;

char string[2];

/************************/
/* RTOS application funcs                                               */
/************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************/
/* handlers / callbacks                                                 */
/************************/

void but_callback(void) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(xSemaphore, &xHigherPriorityTaskWoken);
}

void ckho_callback(void) {
	double n;
	if (pio_get(ECHO_PIO, PIO_INPUT, ECHO_PIO_PIN_MASK))
	{
		RTT_init(7500, 0,0);
	}
	
	else
	{
		n = rtt_read_timer_value(RTT);
		xQueueSendFromISR(xQueueEcho, &n, 0);
	}

}
/************************/
/* TASKS                                                                */
/************************/

static void task_oled(void *pvParameters) {
	
	gfx_mono_ssd1306_init();
	BUT_init();
	for (;;)
	{
		if (xSemaphoreTake(xSemaphore, 1000))
		{
			pin_toggle(TRIG_PIO, TRIG_PIO_PIN_MASK);
		}
	}
}

static void task_calc_dist(void *pvParameters) {
	
	double n;
	double d;
	
	gfx_mono_ssd1306_init();
	gfx_mono_draw_string("distância  (cm)", 0, 0, &sysfont);
	BUT_init();
	for (;;) {
		if (xQueueReceive(xQueueEcho, &n, pdMS_TO_TICKS(5)))
		{
			d = (double)(n * 341) * 100 / (2 * 7500);
			sprintf(string, "%f", d);
			gfx_mono_draw_string(string, 50, 16, &sysfont);
		}
	}
}

/************************/
/* funcoes                                                              */
/************************/

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}


static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}


static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

	uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);
	
	if (rttIRQSource & RTT_MR_ALMIEN) {
		uint32_t ul_previous_time;
		ul_previous_time = rtt_read_timer_value(RTT);
		while (ul_previous_time == rtt_read_timer_value(RTT));
		rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
	}

	/* config NVIC */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 4);
	NVIC_EnableIRQ(RTT_IRQn);

	/* Enable RTT interrupt */
	if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
	else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
	
}

static void BUT_init(void) {

	pmc_enable_periph_clk(BUT_PIO_ID);
	pmc_enable_periph_clk(TRIG_PIO_ID);
	pmc_enable_periph_clk(ECHO_PIO_ID);

	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);
	
	pio_set_output(TRIG_PIO, TRIG_PIO_PIN_MASK, 0, 0, 0);
	pio_set_input(ECHO_PIO, ECHO_PIO_PIN_MASK, PIO_DEFAULT);
	pio_enable_interrupt(ECHO_PIO, ECHO_PIO_PIN_MASK);
	pio_handler_set(ECHO_PIO, ECHO_PIO_ID, ECHO_PIO_PIN_MASK, PIO_IT_EDGE , ECHO_callback);
	
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);
	NVIC_EnableIRQ(ECHO_PIO_ID);
	NVIC_SetPriority(ECHO_PIO_ID, 4);

	
}

/************************/
/* main                                                                 */
/************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();
	xQueueEcho = XQueueCreate(32, sizeof(double));
	xSemaphore = xSemaphoreCreateBinary();
	if (xQueueEcho == NULL){
		printf("falhou a fila \n");
	}
	if (xSemaphore == NULL){
		printf("falhou o semaforo \n");
	}
	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS n�o deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}