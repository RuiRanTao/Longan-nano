/*!
    \file  main.c
    \brief running led
    
    \version 2019-6-5, V1.0.0, firmware for GD32VF103
*/

/*
    Copyright (c) 2019, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32vf103.h"
#include "gd32vf103_usart.h"

// #include "gd32vf103v_eval.h"
#include "systick.h"
#include <stdio.h>

#define ARRAYNUM(arr_nanme)      (uint32_t)(sizeof(arr_nanme) / sizeof(*(arr_nanme)))
#define TRANSMIT_SIZE            (ARRAYNUM(txbuffer) - 1)

uint16_t len;
uint16_t t;
uint8_t flag_d=0;
uint8_t txbuffer[] = "\n\rUSART interrupt test\n\r";
uint8_t str_1[32]="abc";
uint8_t rxbuffer[32];
uint8_t rxtrue[32];
uint8_t time=150;
uint16_t USART_RX_STA=0; 


uint8_t tx_size = TRANSMIT_SIZE;
uint8_t rx_size = 32;
__IO uint8_t txcount = 0; 
__IO uint16_t rxcount = 0; 

/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/


int _put_char(int ch)
{
    usart_data_transmit(USART0, (uint8_t) ch );
    while ( usart_flag_get(USART0, USART_FLAG_TBE)== RESET){
    }

    return ch;
}

void _put_string(char *str)
{
    uint8_t i = 0;
    do
    {
	_put_char(*(str+i));
	i++;
    }while(*(str+i) != '\0');
    // 等待发送完成
    while(usart_flag_get(USART0,USART_FLAG_TC) == RESET);
}

int equal_1(uint8_t str[32])
{
    printf("%s\n", rxtrue);
    for(t=0; t<31; t++)
    if(str[t]!=rxtrue[t])
    {
        return 0;
    }
    return 1;
}


int main(void)
{
    /* enable the LED clock */
    rcu_periph_clock_enable(RCU_GPIOC);
    rcu_periph_clock_enable(RCU_GPIOA);

    /* enable USART clock */
    rcu_periph_clock_enable(RCU_USART0);

    /* connect port to USARTx_Tx */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_9);

    /* connect port to USARTx_Rx */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_10);



    /* configure LED GPIO port */
    gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_13);
    gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_1 | GPIO_PIN_2);

    gpio_bit_reset(GPIOC, GPIO_PIN_13);
    gpio_bit_reset(GPIOA, GPIO_PIN_1 | GPIO_PIN_2);

   /* USART configure */
    usart_deinit(USART0);
    usart_baudrate_set(USART0, 115200);
    usart_word_length_set(USART0, USART_WL_8BIT);
    usart_stop_bit_set(USART0, USART_STB_1BIT);
    usart_parity_config(USART0, USART_PM_NONE);
    usart_hardware_flow_rts_config(USART0, USART_RTS_DISABLE);
    usart_hardware_flow_cts_config(USART0, USART_CTS_DISABLE);
    usart_receive_config(USART0, USART_RECEIVE_ENABLE);
    usart_transmit_config(USART0, USART_TRANSMIT_ENABLE);
    usart_enable(USART0);

        /* USART interrupt configuration */
    eclic_global_interrupt_enable();
    eclic_priority_group_set(ECLIC_PRIGROUP_LEVEL3_PRIO1);
    eclic_irq_enable(USART0_IRQn, 1, 0);

    /* enable USART TBE interrupt */  
    usart_interrupt_enable(USART0, USART_INT_RBNE);

    while(1){
        
		if(USART_RX_STA&0x8000)
		{					   
			len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
			printf("\r\nYourmessage:\r\n\r\n");
			for(t=0;t<len;t++)
			{
				usart_data_transmit(USART0, rxbuffer[t]);//向串口1发送数据
				while(usart_flag_get(USART0,USART_FLAG_TC)!=SET);//等待发送结束
			}
			printf("\r\n\r\n");//插入换行
			USART_RX_STA=0;
		}

        // if(equal_1(str_1)){
        //     printf("    ^~^\r\n");
        // }

        // printf("a usart transmit test example!\r\n");
        /* turn on LED1, turn off LED4 */
        gpio_bit_set(GPIOC, GPIO_PIN_13);
        gpio_bit_reset(GPIOA, GPIO_PIN_1);
        delay_1ms(time);

        /* turn on LED2, turn off LED1 */
        gpio_bit_set(GPIOA, GPIO_PIN_2);
        gpio_bit_reset(GPIOC, GPIO_PIN_13);
        delay_1ms(time);

        /* turn on LED3, turn off LED2 */
        gpio_bit_set(GPIOA, GPIO_PIN_1);
        gpio_bit_reset(GPIOA, GPIO_PIN_2);
        delay_1ms(time);
        _put_char('c');
        _put_string("string\n");
    
    }
}

