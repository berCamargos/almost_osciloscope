#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

#include "stm32f4xx_conf.h"
#include "stm32f4xx.h"
#include "main.h"

#include "usbd_cdc_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"
#include "usbd_cdc_vcp.h"

// Private variables
volatile uint32_t time_var1, time_var2;
__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

#define ADC3_DR_ADDRESS     ((uint32_t)0x4001224C)
#define ADC_CDR_ADDRESS    ((uint32_t)0x40012308)

#define N_REP 10000
#define BUFF_SIZE 10
#define DMA_BUFFER_SIZE 512 
#define DMA_BUFFER_TARGET 1 

__IO uint16_t ADC3ConvertedValue[DMA_BUFFER_SIZE];

// Private function prototypes
void Delay(volatile uint32_t nCount);
void init();
void calculation_test();
void test_adc();
void test_adc_led();
void ADC3_CH12_DMA_Config(void);

int main(void) {
	init();

	/*
	 * Disable STDOUT buffering. Otherwise nothing will be printed
	 * before a newline character or when the buffer is flushed.
	 */
	setbuf(stdout, NULL);



	//calculation_test();
    test_adc_led();
    //test_adc();
	for(;;) {
		GPIO_SetBits(GPIOD, GPIO_Pin_15);
		Delay(1000);
		GPIO_ResetBits(GPIOD, GPIO_Pin_15);
		Delay(1000);
	}

	return 0;
}

void DMA2_Stream0_IRQHandler(void) // Called at 1 KHz for 200 KHz sample rate, LED Toggles at 500 Hz
{
    /* Test on DMA Stream Half Transfer interrupt */
    if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_HTIF0))
    {
        /* Clear DMA Stream Half Transfer interrupt pending bit */
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_HTIF0);
        /* Turn LED3 off: Half Transfer */
		GPIO_SetBits(GPIOD, GPIO_Pin_12);
        // Add code here to process first half of buffer (ping)
        //VCP_send_buffer(&ADC3ConvertedValue, DMA_BUFFER_SIZE/2);
        VCP_send_buffer(&ADC3ConvertedValue[0], (DMA_BUFFER_SIZE/2)*sizeof(uint16_t));
    }
    /* Test on DMA Stream Transfer Complete interrupt */
    if(DMA_GetITStatus(DMA2_Stream0, DMA_IT_TCIF0))
    {
        /* Clear DMA Stream Transfer Complete interrupt pending bit */
        DMA_ClearITPendingBit(DMA2_Stream0, DMA_IT_TCIF0);
        /* Turn LED3 on: End of Transfer */
		GPIO_ResetBits(GPIOD, GPIO_Pin_12);
        VCP_send_buffer(&ADC3ConvertedValue[DMA_BUFFER_SIZE/2], (DMA_BUFFER_SIZE/2)*sizeof(uint16_t));
        // Add code here to process second half of buffer (pong)
    }
}

void adc_configuration(void)
{
    ADC_InitTypeDef       ADC_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    GPIO_InitTypeDef      GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

    /* Configure ADC3 Channel11 pin as analog input ******************************/
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    
    /* ADC Common Init **********************************************************/
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);
    
    /* ADC3 Init ****************************************************************/
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;//DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_Rising;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_T2_TRGO;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 2;

    ADC_Init(ADC1, &ADC_InitStructure);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 2, ADC_SampleTime_3Cycles);
    ADC_RegularChannelConfig(ADC1, ADC_Channel_12, 1, ADC_SampleTime_3Cycles);
    ADC_SoftwareStartConv(ADC1);

    /* Enable ADC1 */
    ADC_Cmd(ADC1, ENABLE);
}

void tim_configuration(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    
    /* Time base configuration */
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = (84000000 / 20000) - 1; // 20 KHz, from 84 MHz TIM2CLK (ie APB1 = HCLK/4, TIM2CLK = HCLK/2)
    TIM_TimeBaseStructure.TIM_Prescaler = 1 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    /* TIM2 TRGO selection */
    TIM_SelectOutputTrigger(TIM2, TIM_TRGOSource_Update); // ADC_ExternalTrigConv_T2_TRGO
    /* TIM2 enable counter */
    TIM_Cmd(TIM2, ENABLE);
}

void dma_configuration(void)
{
    DMA_InitTypeDef  DMA_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    
    /* Enable the DMA Stream IRQ Channel */
    NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    
    /* Enable ADC1, DMA2 and GPIO clocks ****************************************/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
    
    /* DMA2 Stream0 channel0 configuration **************************************/
    DMA_InitStructure.DMA_Channel = DMA_Channel_0;  
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&ADC1->DR;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&ADC3ConvertedValue[0];
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = DMA_BUFFER_SIZE;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream0, &DMA_InitStructure);
    DMA_ITConfig(DMA2_Stream0, DMA_IT_TC | DMA_IT_HT, ENABLE);
    DMA_Cmd(DMA2_Stream0, ENABLE);
    
    /* Enable DMA request after last transfer (Single-ADC mode) */
    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);
    
    /* Enable ADC1 DMA */
    ADC_DMACmd(ADC1, ENABLE);
    

}

void test_adc_led() {
	float a = 1.001;
	int iteration = 0;

	for(;;) {
        if (ADC3ConvertedValue[DMA_BUFFER_TARGET] > 3000)
    		GPIO_SetBits(GPIOD, GPIO_Pin_14);
        else
    		GPIO_ResetBits(GPIOD, GPIO_Pin_14);
        if (ADC3ConvertedValue[0] > 3000)
    		GPIO_SetBits(GPIOD, GPIO_Pin_13);
        else
    		GPIO_ResetBits(GPIOD, GPIO_Pin_13);
		Delay(100);
        //printf("%d\n\r", ADC3ConvertedValue[DMA_BUFFER_TARGET]);
	}
}

void test_adc() {
	float a = 1.001;
	int iteration = 0;

    uint8_t* aaa;
    aaa = malloc(BUFF_SIZE*10);
    memset(aaa, 'a', BUFF_SIZE*10);
	for(;;) {
        if (iteration % 2)
    		GPIO_SetBits(GPIOD, GPIO_Pin_13);
        else
    		GPIO_ResetBits(GPIOD, GPIO_Pin_13);

		time_var2 = 0;
        VCP_send_buffer(&ADC3ConvertedValue, DMA_BUFFER_SIZE*sizeof(uint16_t));
        //VCP_send_buffer(&ADC3ConvertedValue, BUFF_SIZE);
		iteration++;
		Delay(500);
	}
}

void calculation_test() {
	float a = 1.001;
	int iteration = 0;

    uint8_t* aaa;
    aaa = malloc(BUFF_SIZE*10);
    memset(aaa, 10, BUFF_SIZE*10);
	for(;;) {
        if (iteration % 2)
    		GPIO_SetBits(GPIOD, GPIO_Pin_12);
        else
    		GPIO_ResetBits(GPIOD, GPIO_Pin_12);
		Delay(500);

		time_var2 = 0;
		for (int i = 0;i < N_REP;i++) {
			//a += 0.01 * sqrtf(a);
            if (iteration % 2)
                VCP_send_buffer(aaa, BUFF_SIZE);
            else
                VCP_send_buffer(aaa, BUFF_SIZE*10);
		}

		printf("\n\n\n\nTime:      %f ms\n\r", ((float)time_var2)/N_REP);
		printf("Iteration: %i\n\r", iteration);
		printf("Value:     %.5f\n\n\r", a);

		iteration++;
	}
}

void init() {
	GPIO_InitTypeDef  GPIO_InitStructure;

	// ---------- SysTick timer -------- //
	if (SysTick_Config(SystemCoreClock / 1000)) {
		// Capture error
		while (1){};
	}

	// ---------- GPIO -------- //
	// GPIOD Periph clock enable
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	// Configure PD12, PD13, PD14 and PD15 in output pushpull mode
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);

	// ------------- USB -------------- //
	USBD_Init(&USB_OTG_dev,
	          USB_OTG_FS_CORE_ID,
	          &USR_desc,
	          &USBD_CDC_cb,
	          &USR_cb);
    adc_configuration();
    tim_configuration();
    dma_configuration();
}

/*
 * Called from systick handler
 */
void timing_handler() {
	if (time_var1) {
		time_var1--;
	}

	time_var2++;
}

/*
 * Delay a number of systick cycles (1ms)
 */
void Delay(volatile uint32_t nCount) {
	time_var1 = nCount;
	while(time_var1){};
}

/*
 * Dummy function to avoid compiler error
 */
void _init() {

}

