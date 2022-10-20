#include "usart.h"



UART_HandleTypeDef huart1;  // UART_HandleTypeDef �ṹ�����


/*************************************************************************************************
*	�� �� ��:	HAL_UART_MspInit
*	��ڲ���:	huart - UART_HandleTypeDef����ı���������ʾ����Ĵ���
*	�� �� ֵ:	��
*	��������:	��ʼ����������
*	˵    ��:	��		
*************************************************************************************************/

void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	if(huart->Instance==USART1)
	{
		__HAL_RCC_USART1_CLK_ENABLE();		// ���� USART1 ʱ��

		GPIO_USART1_TX_CLK_ENABLE;				// ���� USART1 TX ���ŵ� GPIO ʱ��
		GPIO_USART1_RX_CLK_ENABLE;				// ���� USART1 RX ���ŵ� GPIO ʱ��
		
		/** USART1 GPIO Configuration
		PA9     ------> USART1_TX
		PA10     ------> USART1_RX	*/
		
		GPIO_InitStruct.Pin 			= USART1_TX_PIN;					// TX����
		GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;				// �����������
		GPIO_InitStruct.Pull 		= GPIO_PULLUP;						// ����
		GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;	// �ٶȵȼ� 
		GPIO_InitStruct.Alternate 	= GPIO_AF1_USART1;				// ����ΪUSART1
		HAL_GPIO_Init(USART1_TX_PORT, &GPIO_InitStruct);

		GPIO_InitStruct.Pin 			= USART1_RX_PIN;					// RX����
		HAL_GPIO_Init(USART1_RX_PORT, &GPIO_InitStruct);		
	}

}

/*************************************************************************************************
*	�� �� ��:	USART1_Init
*	��ڲ���:	��
*	�� �� ֵ:	��
*	��������:	��ʼ����������
*	˵    ��:	��		 
*************************************************************************************************/

void USART1_Init(void)
{
	huart1.Instance				 			= USART1;                        // ����1
	huart1.Init.BaudRate 					= 115200;                        // ������
	huart1.Init.WordLength 					= UART_WORDLENGTH_8B;            // 8λ���ݿ��
	huart1.Init.StopBits 					= UART_STOPBITS_1;               // ֹͣλ1
	huart1.Init.Parity 						= UART_PARITY_NONE;              // ��ʹ����żУ��
	huart1.Init.Mode 							= UART_MODE_TX_RX;               // ȫ˫��ģʽ
	huart1.Init.HwFlowCtl 					= UART_HWCONTROL_NONE;           // ��ʹ��Ӳ��������
	huart1.Init.OverSampling 				= UART_OVERSAMPLING_16;          // 16��������ģʽ
	huart1.Init.OneBitSampling 			= UART_ONE_BIT_SAMPLE_DISABLE;   // ��ֹ���β�����ʹ��3�β�����ȷ�����ݸ��ӿɿ�
	huart1.Init.ClockPrescaler 			= UART_PRESCALER_DIV1;           // �ں�ʱ�ӷ�Ƶϵ�������Ϊ64M
	huart1.AdvancedInit.AdvFeatureInit 	= UART_ADVFEATURE_NO_INIT;			// ��ʹ�ý��׹���

	HAL_UART_Init(&huart1);		// ��ʼ����������

}

/*************************************************************************************************
*	�� �� ��:	fputc
*	��ڲ���:	ch - Ҫ������ַ� ��  f - �ļ�ָ�루�����ò�����
*	�� �� ֵ:	����ʱ�����ַ�������ʱ���� EOF��-1��
*	��������:	�ض��� fputc ������Ŀ����ʹ�� printf ����
*	˵    ��:	��		
*************************************************************************************************/

int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 100);	// ���͵��ֽ�����
	return (ch);
}

