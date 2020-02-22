/*!
    \file  gd32vf103_it.c
    \brief interrupt service routines

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

#include "gd32vf103_it.h"
#include "gd32vf103v_eval.h"

extern uint8_t tx_size;
extern uint8_t rx_size;
extern __IO uint8_t txcount;
extern __IO uint16_t rxcount;
extern uint8_t rxbuffer[32];
extern uint8_t txbuffer[];
extern uint16_t t;
extern uint8_t rxtrue[32];
extern uint8_t flag_d;
extern uint16_t USART_RX_STA; 

/*!
    \brief      this function handles USART RBNE interrupt request and TBE interrupt request
    \param[in]  none
    \param[out] none
    \retval     none
*/


void USART0_IRQHandler(void)
{
        uint8_t ucTemp;
    if(RESET != usart_interrupt_flag_get(USART0, USART_INT_FLAG_RBNE)){
        ucTemp = usart_data_receive(USART0);  
        if((USART_RX_STA&0x8000)==0)//接收未完成
            {
                if(USART_RX_STA&0x4000)//接收到了0x0d
                {
                    if(ucTemp!=0x0a)USART_RX_STA=0;//接收错误,重新开始
                    else USART_RX_STA|=0x8000;	//接收完成了 
                }else //还没收到0X0D
                {	
                    if(ucTemp==0x0d)USART_RX_STA|=0x4000;
                    else
                    {
                        rxbuffer[USART_RX_STA&0X3FFF]=ucTemp;
                        USART_RX_STA++;
                        if(USART_RX_STA>(rx_size-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
                    }		 
                }
            }  	
    }	
} 			