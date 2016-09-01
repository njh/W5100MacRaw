//*****************************************************************************
//
//! \file w5100.c
//! \brief W5100 HAL Interface.
//! \version 1.0.0
//! \date 2013/10/21
//! \par  Revision history
//!       <2013/10/21> 1st Release
//! \author MidnightCow
//!
//! Copyright (c)  2013, WIZnet Co., LTD.
//! All rights reserved.
//!
//! Redistribution and use in source and binary forms, with or without
//! modification, are permitted provided that the following conditions
//! are met:
//!
//!     * Redistributions of source code must retain the above copyright
//! notice, this list of conditions and the following disclaimer.
//!     * Redistributions in binary form must reproduce the above copyright
//! notice, this list of conditions and the following disclaimer in the
//! documentation and/or other materials provided with the distribution.
//!     * Neither the name of the <ORGANIZATION> nor the names of its
//! contributors may be used to endorse or promote products derived
//! from this software without specific prior written permission.
//!
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//! CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//! SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//! INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//! ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
//! THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

#include "w5100.h"

#include <SPI.h>


/**
 * @brief Default function to select chip.
 * @note This function help not to access wrong address. If you do not describe this function or register any functions,
 * null function is called.
 */
void wizchip_cs_select(void)
{
    digitalWrite(SS, LOW);
}

/**
 * @brief Default function to deselect chip.
 * @note This function help not to access wrong address. If you do not describe this function or register any functions,
 * null function is called.
 */
void wizchip_cs_deselect(void)
{
    digitalWrite(SS, HIGH);
}

/**
 * @brief Default function to read in SPI interface.
 * @note This function help not to access wrong address. If you do not describe this function or register any functions,
 * null function is called.
 */
uint8_t wizchip_spi_readbyte(void)
{
    return SPI.transfer(0);
}

/**
 * @brief Default function to write in SPI interface.
 * @note This function help not to access wrong address. If you do not describe this function or register any functions,
 * null function is called.
 */
void 	wizchip_spi_writebyte(uint8_t wb)
{
    SPI.transfer(wb);
}


void wizchip_sw_reset(void)
{
    uint8_t gw[4], sn[4], sip[4];
    uint8_t mac[6];
    getSHAR(mac);
    getGAR(gw);
    getSUBR(sn);
    getSIPR(sip);

    setMR(MR_RST);

    getMR(); // for delay
    setSHAR(mac);
    setGAR(gw);
    setSUBR(sn);
    setSIPR(sip);
}

int8_t wizchip_init(uint8_t* txsize, uint8_t* rxsize)
{
    int8_t i;
    int8_t tmp = 0;
    wizchip_sw_reset();
    if(txsize)
    {
        tmp = 0;
        for(i = 0 ; i < _WIZCHIP_SOCK_NUM_; i++)
        {
            tmp += txsize[i];
            if(tmp > 16) return -1;
        }
        for(i = 0 ; i < _WIZCHIP_SOCK_NUM_; i++)
            setSn_TXBUF_SIZE(i, txsize[i]);
    }
    if(rxsize)
    {
        tmp = 0;
        for(i = 0 ; i < _WIZCHIP_SOCK_NUM_; i++)
        {
            tmp += rxsize[i];
            if(tmp > 16) return -1;
        }

        for(i = 0 ; i < _WIZCHIP_SOCK_NUM_; i++)
            setSn_RXBUF_SIZE(i, rxsize[i]);
    }
    return 0;
}

/**
@brief  This function writes the data into W5200 registers.
*/
void     WIZCHIP_WRITE(uint32_t AddrSel, uint8_t wb )
{
    //WIZCHIP_CRITICAL_ENTER();
    wizchip_cs_select();

    wizchip_spi_writebyte(0xF0);
    wizchip_spi_writebyte((AddrSel & 0xFF00) >>  8);
    wizchip_spi_writebyte((AddrSel & 0x00FF) >>  0);
    wizchip_spi_writebyte(wb);    // Data write (write 1byte data)

    wizchip_cs_deselect();
    //WIZCHIP_CRITICAL_EXIT();
}
/**
@brief  This function reads the value from W5200 registers.
*/
uint8_t  WIZCHIP_READ(uint32_t AddrSel)
{
    uint8_t ret;

    //WIZCHIP_CRITICAL_ENTER();
    wizchip_cs_select();

    wizchip_spi_writebyte(0x0F);
    wizchip_spi_writebyte((AddrSel & 0xFF00) >>  8);
    wizchip_spi_writebyte((AddrSel & 0x00FF) >>  0);
    ret = wizchip_spi_readbyte();

    wizchip_cs_deselect();
    //WIZCHIP_CRITICAL_EXIT();
    return ret;
}


/**
@brief  This function writes into W5200 memory(Buffer)
*/
void     WIZCHIP_WRITE_BUF(uint32_t AddrSel, uint8_t* pBuf, uint16_t len)
{
    uint16_t i = 0;

    //WIZCHIP_CRITICAL_ENTER();
    wizchip_cs_select();   //M20150601 : Moved here.

    for(i = 0; i < len; i++)
    {
        //M20160715 : Depricated "M20150601 : Remove _select() to top-side"
        //            CS should be controlled every SPI frames
        wizchip_cs_select();
        wizchip_spi_writebyte(0xF0);
        wizchip_spi_writebyte((((uint16_t)(AddrSel+i)) & 0xFF00) >>  8);
        wizchip_spi_writebyte((((uint16_t)(AddrSel+i)) & 0x00FF) >>  0);
        wizchip_spi_writebyte(pBuf[i]);    // Data write (write 1byte data)
        //M20160715 : Depricated "M20150601 : Remove _select() to top-side"
        wizchip_cs_deselect();
    }

    wizchip_cs_deselect();  //M20150601 : Moved here.
    //WIZCHIP_CRITICAL_EXIT();
}

/**
@brief  This function reads into W5200 memory(Buffer)
*/

void     WIZCHIP_READ_BUF (uint32_t AddrSel, uint8_t* pBuf, uint16_t len)
{
    uint16_t i = 0;
    //WIZCHIP_CRITICAL_ENTER();
    wizchip_cs_select();   //M20150601 : Moved here.

    for(i = 0; i < len; i++)
    {
        //M20160715 : Depricated "M20150601 : Remove _select() to top-side"
        //            CS should be controlled every SPI frames
        wizchip_cs_select();
        wizchip_spi_writebyte(0x0F);
        wizchip_spi_writebyte((uint16_t)((AddrSel+i) & 0xFF00) >>  8);
        wizchip_spi_writebyte((uint16_t)((AddrSel+i) & 0x00FF) >>  0);
        pBuf[i] = wizchip_spi_readbyte();
        //M20160715 : Depricated "M20150601 : Remove _select() to top-side"
        wizchip_cs_deselect();
    }

    wizchip_cs_deselect();    //M20150601 : Moved Here.
    //WIZCHIP_CRITICAL_EXIT();
}

///////////////////////////////////
// Socket N regsiter IO function //
///////////////////////////////////

uint16_t getSn_TX_FSR(uint8_t sn)
{
    uint16_t val=0,val1=0;
    do
    {
        val1 = WIZCHIP_READ(Sn_TX_FSR(sn));
        val1 = (val1 << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_TX_FSR(sn),1));
        if (val1 != 0)
        {
            val = WIZCHIP_READ(Sn_TX_FSR(sn));
            val = (val << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_TX_FSR(sn),1));
        }
    } while (val != val1);
    return val;
}


uint16_t getSn_RX_RSR(uint8_t sn)
{
    uint16_t val=0,val1=0;
    do
    {
        val1 = WIZCHIP_READ(Sn_RX_RSR(sn));
        val1 = (val1 << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_RX_RSR(sn),1));
        if (val1 != 0)
        {
            val = WIZCHIP_READ(Sn_RX_RSR(sn));
            val = (val << 8) + WIZCHIP_READ(WIZCHIP_OFFSET_INC(Sn_RX_RSR(sn),1));
        }
    } while (val != val1);
    return val;
}

/////////////////////////////////////
// Sn_TXBUF & Sn_RXBUF IO function //
/////////////////////////////////////
uint32_t getSn_RxBASE(uint8_t sn)
{
    int8_t  i;
    uint32_t rxbase = _WIZCHIP_IO_RXBUF_;
    for(i = 0; i < sn; i++)
        rxbase += getSn_RxMAX(i);

    return rxbase;
}

uint32_t getSn_TxBASE(uint8_t sn)
{
    int8_t  i;
    uint32_t txbase = _WIZCHIP_IO_TXBUF_;
    for(i = 0; i < sn; i++)
        txbase += getSn_TxMAX(i);
    return txbase;
}

/**
@brief  This function is being called by send() and sendto() function also. for copy the data form application buffer to Transmite buffer of the chip.

This function read the Tx write pointer register and after copy the data in buffer update the Tx write pointer
register. User should read upper byte first and lower byte later to get proper value.
And this function is being used for copy the data form application buffer to Transmite
buffer of the chip. It calculate the actual physical address where one has to write
the data in transmite buffer. Here also take care of the condition while it exceed
the Tx memory uper-bound of socket.

*/
void wiz_send_data(uint8_t sn, uint8_t *wizdata, uint16_t len)
{
    uint16_t ptr;
    uint16_t size;
    uint16_t dst_mask;
    uint16_t dst_ptr;

    ptr = getSn_TX_WR(sn);

    dst_mask = ptr & getSn_TxMASK(sn);
    dst_ptr = getSn_TxBASE(sn) + dst_mask;

    if (dst_mask + len > getSn_TxMAX(sn))
    {
        size = getSn_TxMAX(sn) - dst_mask;
        WIZCHIP_WRITE_BUF(dst_ptr, wizdata, size);
        wizdata += size;
        size = len - size;
        dst_ptr = getSn_TxBASE(sn);
        WIZCHIP_WRITE_BUF(dst_ptr, wizdata, size);
    }
    else
    {
        WIZCHIP_WRITE_BUF(dst_ptr, wizdata, len);
    }

    ptr += len;

    setSn_TX_WR(sn, ptr);
}


/**
@brief  This function is being called by recv() also. This function is being used for copy the data form Receive buffer of the chip to application buffer.

This function read the Rx read pointer register
and after copy the data from receive buffer update the Rx write pointer register.
User should read upper byte first and lower byte later to get proper value.
It calculate the actual physical address where one has to read
the data from Receive buffer. Here also take care of the condition while it exceed
the Rx memory uper-bound of socket.
*/
void wiz_recv_data(uint8_t sn, uint8_t *wizdata, uint16_t len)
{
    uint16_t ptr;
    uint16_t size;
    uint16_t src_mask;
    uint16_t src_ptr;

    ptr = getSn_RX_RD(sn);

    src_mask = (uint32_t)ptr & getSn_RxMASK(sn);
    src_ptr = (getSn_RxBASE(sn) + src_mask);


    if( (src_mask + len) > getSn_RxMAX(sn) )
    {
        size = getSn_RxMAX(sn) - src_mask;
        WIZCHIP_READ_BUF(src_ptr, wizdata, size);
        wizdata += size;
        size = len - size;
        src_ptr = getSn_RxBASE(sn);
        WIZCHIP_READ_BUF(src_ptr, wizdata, size);
    }
    else
    {
        WIZCHIP_READ_BUF(src_ptr, wizdata, len);
    }

    ptr += len;

    setSn_RX_RD(sn, ptr);
}

void wiz_recv_ignore(uint8_t sn, uint16_t len)
{
    uint16_t ptr;

    ptr = getSn_RX_RD(sn);

    ptr += len;
    setSn_RX_RD(sn,ptr);
}
