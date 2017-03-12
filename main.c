#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_uart.h"
#include "inc/hw_emac.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/uart.h"
#include "driverlib/udma.h"
#include "driverlib/emac.h"

#include "FreeRTOS.h"
#include "task.h"

#define NUM_TX_DESCRIPTORS 3
#define NUM_RX_DESCRIPTORS 3
tEMACDMADescriptor RxDescriptor[NUM_TX_DESCRIPTORS];
tEMACDMADescriptor TxDescriptor[NUM_RX_DESCRIPTORS];
volatile uint32_t RxDescIndex;
volatile uint32_t TxDescIndex;

#define RX_BUFFER_SIZE 1536
uint8_t *RxBuffer[NUM_RX_DESCRIPTORS] = {{NULL}};

void write_packet(uint8_t *data, uint32_t data_length)
{
	while(TxDescriptor[TxDescIndex].ui32CtrlStatus & DES0_TX_CTRL_OWN);
	TxDescIndex = TxDescIndex == NUM_TX_DESCRIPTORS - 1 ? 0 : TxDescIndex+1;
	TxDescriptor[TxDescIndex].pvBuffer1 = data;
	TxDescriptor[TxDescIndex].ui32Count = data_length;
	TxDescriptor[TxDescIndex].ui32CtrlStatus = (DES0_TX_CTRL_LAST_SEG |
			DES0_TX_CTRL_FIRST_SEG | DES0_TX_CTRL_INTERRUPT |
			DES0_TX_CTRL_IP_ALL_CKHSUMS | DES0_TX_CTRL_CHAINED |
			DES0_TX_CTRL_OWN);
	EMACTxDMAPollDemand(EMAC0_BASE);
}

void ethernet_int_handler(void)
{
	uint32_t status = EMACIntStatus(EMAC0_BASE, true);
	EMACIntClear(EMAC0_BASE, status);
	if(status & EMAC_INT_RECEIVE)
	{
		uint32_t frame_len = 0;
		if(!(RxDescriptor[RxDescIndex].ui32CtrlStatus & DES0_RX_CTRL_OWN))
		{
			if(!(RxDescriptor[RxDescIndex].ui32CtrlStatus & DES0_RX_STAT_ERR))
			{
				if(RxDescriptor[RxDescIndex].ui32CtrlStatus & DES0_RX_STAT_LAST_DESC)
				{
					frame_len = ((RxDescriptor[RxDescIndex].ui32CtrlStatus & DES0_RX_STAT_FRAME_LENGTH_M) >> DES0_RX_STAT_FRAME_LENGTH_S);
					/*
					 * TODO: process received frame
					 */
				}
			}
			RxDescriptor[RxDescIndex].ui32CtrlStatus = DES0_RX_CTRL_OWN;
			RxDescIndex = RxDescIndex == NUM_RX_DESCRIPTORS - 1 ? 0 : RxDescIndex+1;
		}
	}
}

void init_ethernet(uint32_t clk)
{
	/*
	 * hardware setup:
	 */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_EMAC0);
	SysCtlPeripheralReset(SYSCTL_PERIPH_EMAC0);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_EPHY0);
	SysCtlPeripheralReset(SYSCTL_PERIPH_EPHY0);

	while(!SysCtlPeripheralReady(SYSCTL_PERIPH_EMAC0));

	EMACPHYConfigSet(EMAC0_BASE, EMAC_PHY_TYPE_INTERNAL | EMAC_PHY_INT_MDIX_EN |
			EMAC_PHY_AN_100B_T_FULL_DUPLEX);

	EMACReset(EMAC0_BASE);

	EMACInit(EMAC0_BASE, clk, EMAC_BCONFIG_MIXED_BURST |
			EMAC_BCONFIG_PRIORITY_FIXED, 4, 4, 0);

	EMACConfigSet(EMAC0_BASE, (EMAC_CONFIG_FULL_DUPLEX |
			EMAC_CONFIG_CHECKSUM_OFFLOAD | EMAC_CONFIG_7BYTE_PREAMBLE |
			EMAC_CONFIG_IF_GAP_96BITS | EMAC_CONFIG_USE_MACADDR0 |
			EMAC_CONFIG_SA_INSERT | EMAC_CONFIG_BO_LIMIT_1024),
			(EMAC_MODE_RX_STORE_FORWARD | EMAC_MODE_TX_STORE_FORWARD |
			EMAC_MODE_TX_THRESHOLD_64_BYTES | EMAC_MODE_RX_THRESHOLD_64_BYTES),
			0);

	/*
	 * memory descriptor initalization:
	 */

	for(int i = 0; i < NUM_TX_DESCRIPTORS; i++)
	{
		TxDescriptor[i].ui32Count = DES1_TX_CTRL_SADDR_INSERT;
		TxDescriptor[i].DES3.pLink = (i  == (NUM_TX_DESCRIPTORS - 1))
				? TxDescriptor : &TxDescriptor[i + 1];
		TxDescriptor[i].ui32CtrlStatus = (DES0_TX_CTRL_LAST_SEG |
				DES0_TX_CTRL_FIRST_SEG | DES0_TX_CTRL_INTERRUPT |
				DES0_TX_CTRL_CHAINED | DES0_TX_CTRL_IP_ALL_CKHSUMS);
	}

	for(int i = 0; i < NUM_RX_DESCRIPTORS; i++)
	{
		RxBuffer[i] = pvPortMalloc(RX_BUFFER_SIZE);
		configASSERT(RxBuffer[i] != NULL);
	}

	for(int i = 0; i < NUM_RX_DESCRIPTORS; i++)
	{
		RxDescriptor[i].ui32CtrlStatus = 0;
		RxDescriptor[i].ui32Count = (DES1_RX_CTRL_CHAINED |
				(RX_BUFFER_SIZE << DES1_RX_CTRL_BUFF1_SIZE_S));
		RxDescriptor[i].pvBuffer1 = RxBuffer[i];
		RxDescriptor[i].DES3.pLink = (i == (NUM_RX_DESCRIPTORS - 1))
				? RxDescriptor : &RxDescriptor[i + 1];
	}
	EMACRxDMADescriptorListSet(EMAC0_BASE, RxDescriptor);
	EMACTxDMADescriptorListSet(EMAC0_BASE, TxDescriptor);
	RxDescIndex = 0;
	TxDescIndex = NUM_TX_DESCRIPTORS - 1;

	/*
	 * set mac address:
	 */
	uint8_t mac_addr[6] = {0x00, 0xff, 0x00, 0xff, 0x00, 0xff};
	EMACAddrSet(EMAC0_BASE, 0, mac_addr);

	/*
	 * wait for the link to become active:
	 */
	while((EMACPHYRead(EMAC0_BASE, 0, EPHY_BMSR) & EPHY_BMSR_LINKSTAT) == 0);

	/*
	 * address filtering:
	 */

	EMACFrameFilterSet(EMAC0_BASE, (EMAC_FRMFILTER_SADDR |
			EMAC_FRMFILTER_PASS_MULTICAST | EMAC_FRMFILTER_PASS_NO_CTRL));

	EMACIntClear(EMAC0_BASE, EMACIntStatus(EMAC0_BASE, false));

	for(int i = 0; i < NUM_RX_DESCRIPTORS; i++)
	{
		RxDescriptor[i].ui32CtrlStatus |= DES0_RX_CTRL_OWN;
	}

	EMACTxEnable(EMAC0_BASE);
	EMACRxEnable(EMAC0_BASE);

	IntEnable(INT_EMAC0);

	EMACIntEnable(EMAC0_BASE, EMAC_INT_RECEIVE);
}

void task_blink_f(void *parameters)
{
	uint8_t test_frame[64] = {0x00,0x00,0x00,0x00,0x00,0x00};
	while(1)
	{
		vTaskDelay(250);
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_PIN_0 |
				GPIO_PIN_4);
		vTaskDelay(1000);
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4, 0);

		write_packet(test_frame, 64);
	}
}

void task_blink_n(void *parameters)
{
	while(1)
	{
		vTaskDelay(400);
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_4, GPIO_PIN_0 |
				GPIO_PIN_4);
		vTaskDelay(400);
		GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_4, 0);
	}
}

int main(void)
{
    uint32_t clk = SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 120000000);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF));

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    while(!SysCtlPeripheralReady(SYSCTL_PERIPH_GPION));

    init_ethernet(clk);

    GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);

    xTaskCreate(task_blink_f, "BLINK_F", 100, NULL, NULL, NULL);
    xTaskCreate(task_blink_n, "BLINK_N", 100, NULL, NULL, NULL);

    vTaskStartScheduler();
	return 0;
}

void vApplicationStackOverflowHook(void)
{
	while(1)
	{
	}
}

