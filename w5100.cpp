//*****************************************************************************
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


void wizchip_cs_select(void)
{
    digitalWrite(SS, LOW);
}

void wizchip_cs_deselect(void)
{
    digitalWrite(SS, HIGH);
}



void wizchip_sw_reset(void)
{
    uint8_t mac[6];
    getSHAR(mac);

    setMR(MR_RST);
    getMR(); // for delay

    setSHAR(mac);
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

void     wizchip_write(uint16_t AddrSel, uint8_t wb )
{
    //WIZCHIP_CRITICAL_ENTER();
    wizchip_cs_select();

    SPI.transfer(0xF0);
    SPI.transfer((AddrSel & 0xFF00) >>  8);
    SPI.transfer((AddrSel & 0x00FF) >>  0);
    SPI.transfer(wb);    // Data write (write 1byte data)

    wizchip_cs_deselect();
    //WIZCHIP_CRITICAL_EXIT();
}
{
    uint8_t ret;

    //WIZCHIP_CRITICAL_ENTER();
    wizchip_cs_select();

    SPI.transfer(0x0F);
    SPI.transfer((AddrSel & 0xFF00) >>  8);
    SPI.transfer((AddrSel & 0x00FF) >>  0);
    ret = SPI.transfer(0);

    wizchip_cs_deselect();
    //WIZCHIP_CRITICAL_EXIT();
    return ret;
}


void wizchip_write_buf(uint16_t AddrSel, const uint8_t* pBuf, uint16_t len)
{
    uint16_t i = 0;

    //WIZCHIP_CRITICAL_ENTER();
    wizchip_cs_select();

    for(i = 0; i < len; i++)
    {
        wizchip_cs_select();
        SPI.transfer(0xF0);
        SPI.transfer(((AddrSel+i) & 0xFF00) >>  8);
        SPI.transfer(((AddrSel+i) & 0x00FF) >>  0);
        SPI.transfer(pBuf[i]);    // Data write (write 1byte data)
        wizchip_cs_deselect();
    }

    wizchip_cs_deselect();
    //WIZCHIP_CRITICAL_EXIT();
}


void     wizchip_read_buf(uint16_t AddrSel, uint8_t* pBuf, uint16_t len)
{
    uint16_t i = 0;
    //WIZCHIP_CRITICAL_ENTER();
    wizchip_cs_select();

    for(i = 0; i < len; i++)
    {
        wizchip_cs_select();
        SPI.transfer(0x0F);
        SPI.transfer(((AddrSel+i) & 0xFF00) >>  8);
        SPI.transfer(((AddrSel+i) & 0x00FF) >>  0);
        pBuf[i] = SPI.transfer(0);
        wizchip_cs_deselect();
    }

    wizchip_cs_deselect();
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
        val1 = wizchip_read(Sn_TX_FSR(sn));
        val1 = (val1 << 8) + wizchip_read(WIZCHIP_OFFSET_INC(Sn_TX_FSR(sn),1));
        if (val1 != 0)
        {
            val = wizchip_read(Sn_TX_FSR(sn));
            val = (val << 8) + wizchip_read(WIZCHIP_OFFSET_INC(Sn_TX_FSR(sn),1));
        }
    } while (val != val1);
    return val;
}


uint16_t getSn_RX_RSR(uint8_t sn)
{
    uint16_t val=0,val1=0;
    do
    {
        val1 = wizchip_read(Sn_RX_RSR(sn));
        val1 = (val1 << 8) + wizchip_read(WIZCHIP_OFFSET_INC(Sn_RX_RSR(sn),1));
        if (val1 != 0)
        {
            val = wizchip_read(Sn_RX_RSR(sn));
            val = (val << 8) + wizchip_read(WIZCHIP_OFFSET_INC(Sn_RX_RSR(sn),1));
        }
    } while (val != val1);
    return val;
}

/////////////////////////////////////
// Sn_TXBUF & Sn_RXBUF IO function //
/////////////////////////////////////
uint16_t getSn_RxBASE(uint8_t sn)
{
    int8_t  i;
    uint16_t rxbase = _WIZCHIP_IO_RXBUF_;
    for(i = 0; i < sn; i++)
        rxbase += getSn_RxMAX(i);

    return rxbase;
}

uint16_t getSn_TxBASE(uint8_t sn)
{
    int8_t  i;
    uint16_t txbase = _WIZCHIP_IO_TXBUF_;
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
void wiz_send_data(uint8_t sn, const uint8_t *wizdata, uint16_t len)
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
        wizchip_write_buf(dst_ptr, wizdata, size);
        wizdata += size;
        size = len - size;
        dst_ptr = getSn_TxBASE(sn);
        wizchip_write_buf(dst_ptr, wizdata, size);
    }
    else
    {
        wizchip_write_buf(dst_ptr, wizdata, len);
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

    src_mask = ptr & getSn_RxMASK(sn);
    src_ptr = (getSn_RxBASE(sn) + src_mask);


    if( (src_mask + len) > getSn_RxMAX(sn) )
    {
        size = getSn_RxMAX(sn) - src_mask;
        wizchip_read_buf(src_ptr, wizdata, size);
        wizdata += size;
        size = len - size;
        src_ptr = getSn_RxBASE(sn);
        wizchip_read_buf(src_ptr, wizdata, size);
    }
    else
    {
        wizchip_read_buf(src_ptr, wizdata, len);
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



const int MacRawSockNum = 0;


int8_t wiz_begin(const uint8_t mac_address[6])
{
    pinMode(SS, OUTPUT);
    wizchip_cs_deselect();

    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV4); // 4 MHz?
    SPI.setBitOrder(MSBFIRST);
    SPI.setDataMode(SPI_MODE0);

    wizchip_init(NULL, NULL);

    setSHAR(mac_address);

    setSn_MR(MacRawSockNum, Sn_MR_MACRAW);
    setSn_CR(MacRawSockNum, Sn_CR_OPEN);

    /* wait to process the command... */
    while( getSn_CR(MacRawSockNum) ) ;

    if (getSn_SR(MacRawSockNum) != SOCK_MACRAW) {
        Serial.println("Failed to put socket 0 into MACRaw mode");
        return -1;
    }

    // Success
    return 0;
}


int wiz_read_frame(uint8_t *buffer, uint16_t bufsize)
{
    uint16_t len = getSn_RX_RSR(MacRawSockNum);
    if ( len > 0 )
    {
        uint8_t head[2];
        uint16_t data_len=0;

        wiz_recv_data(MacRawSockNum, head, 2);
        setSn_CR(MacRawSockNum, Sn_CR_RECV);
        while(getSn_CR(MacRawSockNum));

        data_len = head[0];
        data_len = (data_len<<8) + head[1];
        data_len -= 2;

        if(data_len > bufsize)
        {
            Serial.println("Packet is bigger than buffer");
            return 0;
        }

        wiz_recv_data( MacRawSockNum, buffer, data_len );
        setSn_CR(MacRawSockNum, Sn_CR_RECV);
        while(getSn_CR(MacRawSockNum));

        return data_len;
    }

    return 0;
}

int16_t wiz_send_frame(const uint8_t *buf, uint16_t len)
{
    uint16_t freesize = 0;

    // check size not to exceed MAX size.
    freesize = getSn_TxMAX(MacRawSockNum);
    if (len > freesize) len = freesize;

    // Wait for space in the transmit buffer
    while(1)
    {
        freesize = getSn_TX_FSR(MacRawSockNum);
        if(getSn_SR(MacRawSockNum) == SOCK_CLOSED) {
            Serial.println("Socket closed");
            return -1;
        }
        if(len <= freesize) break;
    };
    wiz_send_data(MacRawSockNum, buf, len);


    setSn_CR(MacRawSockNum, Sn_CR_SEND);
    /* wait to process the command... */
    while(getSn_CR(MacRawSockNum));
    while(1)
    {
        uint8_t tmp = getSn_IR(MacRawSockNum);
        if(tmp & Sn_IR_SENDOK)
        {
            setSn_IR(MacRawSockNum, Sn_IR_SENDOK);
            Serial.println("Sn_IR_SENDOK");
            break;
        }
        else if(tmp & Sn_IR_TIMEOUT)
        {
            setSn_IR(MacRawSockNum, Sn_IR_TIMEOUT);
            Serial.println("Timeout");
            return -1;
        }
    }

    return len;
}
