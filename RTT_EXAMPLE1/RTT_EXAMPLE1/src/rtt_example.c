#include "asf.h"
#include "stdio_serial.h"
#include "conf_board.h"
#include "conf_clock.h"

/** Device state: in the main menu. */
#define STATE_MAIN_MENU      0
/** Device state: user is setting an alarm time. */
#define STATE_SET_ALARM      1

// LED DA PLACA
//#define LED0_PIO_ID	    ID_PIOC
//#define LED0_PIO        PIOC
//#define LED0_PIN		8
//#define LED0_PIN_MASK   (1<<LED0_PIN)

#define LED0_PIO_ID	    ID_PIOD
#define LED0_PIO        PIOD
#define LED0_PIN		20
#define LED0_PIN_MASK   (1<<LED0_PIN)

/** Current device state. */
volatile uint8_t g_uc_state;

void pin_toggle(Pio *pio, uint32_t mask);

/**
 * \brief RTT configuration function.
 *
 * Configure the RTT to generate a one second tick, which triggers the RTTINC
 * interrupt.
 */
static void configure_rtt(void)
{
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
#if SAM4N || SAM4S || SAM4E || SAM4C || SAM4CP || SAM4CM || SAMV71 || SAMV70 || SAME70 || SAMS70
	rtt_sel_source(RTT, false);
#endif
	rtt_init(RTT, 32768);
	//rtt_init(RTT, 24576);

	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT));

	/* Enable RTT interrupt */
	NVIC_DisableIRQ(RTT_IRQn);
	NVIC_ClearPendingIRQ(RTT_IRQn);
	NVIC_SetPriority(RTT_IRQn, 0);
	NVIC_EnableIRQ(RTT_IRQn);
	rtt_enable_interrupt(RTT, RTT_MR_RTTINCIEN);
}

/**
 * \brief Interrupt handler for the RTT.
 *
 * Display the current time on the terminal.
 */

void RTT_Handler(void)
{
	uint32_t ul_status;
	/* Get RTT status */
	ul_status = rtt_get_status(RTT);

	/* Time has changed, refresh display */
	if ((ul_status & RTT_SR_RTTINC) == RTT_SR_RTTINC) {
		pmc_enable_periph_clk(LED0_PIO_ID);
		pin_toggle(LED0_PIO, LED0_PIN_MASK);
		pmc_disable_periph_clk(LED0_PIO_ID);
	}
}

/**
*  Toggle pin controlado pelo PIO (out)
*/
void pin_toggle(Pio *pio, uint32_t mask){
	if (pio_get_output_data_status(pio, mask)) {
		pio_clear(pio, mask);
		} else {
			pio_set(pio, mask);
	}
}


void LED_init(int estado){
	pmc_enable_periph_clk(LED0_PIO_ID);
	pio_set_output(LED0_PIO, LED0_PIN_MASK, estado, 0, 0);
}

/**
 * \brief Application entry point for RTT example.
 *
 * Initialize the RTT, display the current time and allow the user to
 * perform several actions: clear the timer, set an alarm, etc.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Configure RTT */
	configure_rtt();

	/* Initialize state machine */
	g_uc_state = STATE_MAIN_MENU;
	
	LED_init(0);
	
	pmc_set_fast_startup_input(PMC_FSMR_RTCAL);
	supc_set_wakeup_mode(SUPC, SUPC_WUMR_RTCEN_ENABLE);

	/* User input loop */
	while (1) {
		//pmc_sleep(SAM_PM_SMODE_BACKUP);
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}
}
