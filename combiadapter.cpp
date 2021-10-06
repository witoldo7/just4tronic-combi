/*
Copyright (c) 2021 Witold Olechowski

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#include "combiadapter.h"
#include "interfaces.h"
#include "mbed.h"
#include <cstdint>
#include "canutils.h"
#include "bdmcpu32.h"

bool CombiReceivePacket(packet_t *packet, uint32_t timeout);
bool CombiSendReplyPacket(packet_t *reply, packet_t *source, uint8_t *data, uint16_t data_len, uint8_t term, uint32_t timeout);
bool CombiSendPacket(packet_t *packet, uint32_t timeout);

uint8_t version[2] = {0x03, 0x01};
uint8_t data_buff[64];
uint8_t egt_temp[5] = {0};
Thread can_rx_thd;
Thread egt_thd;

void egt_read_thd(void) {
    while(true) {
        float temp = sensor.gettemp(0);
        egt_temp[0] = temp == -99.0f ? 0 : 1;
        memcpy((float *)(egt_temp + 1), &temp, sizeof(float));
        thread_sleep_for(300);
    }
}

void can_read_thd(void) {
    CANMessage can_MsgRx;
    while(true) {
        if (can.read(can_MsgRx)) {
            led2 = 1;
            packet_t combiPacket;
            uint8_t buff[15] = {0};
            buff[0] = can_MsgRx.id & 0xFF;
            buff[1] = (can_MsgRx.id >> 8) & 0xFF;
            buff[2] = (can_MsgRx.id >> 16) & 0xFF;
            buff[3] = (can_MsgRx.id >> 24) & 0xFF;
            buff[12] = can_MsgRx.len;
            for (int i = 0; i < can_MsgRx.len; i++) {
                buff[4+i] = can_MsgRx.data[i];
            }
            combiPacket.cmd_code = cmd_can_frame;
            combiPacket.data_len = 15;
            combiPacket.data = buff;
            CombiSendPacket(&combiPacket, 0);
        }
    }
    return;
}

bool exec_cmd_bdm(packet_t *rx_packet, packet_t *tx_packet) {
    switch(rx_packet->cmd_code) {
        case cmd_bdm_stop_chip:
            return (stop_chip() == TERM_OK) && CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack, 1000);
        case cmd_bdm_reset_chip:
            return (reset_chip() == TERM_OK) && CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack, 1000);
        case cmd_bdm_run_chip:
            if (rx_packet->data_len == 4) {
                uint32_t addr = rx_packet->data[3] | (uint32_t)*rx_packet->data << 24 | (uint32_t)rx_packet->data[1] << 16 
                | (uint32_t)rx_packet->data[2] << 8;
            return (run_chip(&addr) == TERM_OK) && CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack, 1000);
            }
            break;
        case cmd_bdm_step_chip:
            return (step_chip() == TERM_OK) && CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack, 1000);
        case cmd_bdm_restart_chip:
            return (restart_chip() == TERM_OK) && CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack, 1000);
        case cmd_bdm_mem_read:
            if (rx_packet->data_len < 2) {
                return false;
            }
            if (rx_packet->data_len == 2) {
                //memdump_word(unsigned short *result);
            }
            
            return CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack, 1000);
        case cmd_bdm_mem_write:
            return CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack, 1000);
        case cmd_bdm_sysreg_read:
            return CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack, 1000);
        case cmd_bdm_sysreg_write:
            return CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack, 1000);
        case cmd_bdm_adreg_read:
            return CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack, 1000);
        case cmd_bdm_adreg_write:
            return CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack, 1000);
        case cmd_bdm_read_flash:
            return CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack, 1000);
        case cmd_bdm_erase_flash:
            return CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack, 1000);
        case cmd_bdm_write_flash:
            return CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack, 1000);
        case cmd_bdm_pinstate:
            return CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack, 1000);
    }
    
    return false;
}

bool exec_cmd_can(packet_t *rx_packet, packet_t *tx_packet) {
    switch(rx_packet->cmd_code) {
        case cmd_can_open:
            if (rx_packet->data_len == 1) {
                if (*rx_packet->data != 0x1) {
                    can_close();
                    can.attach(NULL);
                    can_rx_thd.terminate();
                    egt_thd.terminate();
                    return CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack, 1000);
                }
                can_open();
                
                can_add_filter(2, 0x645);         //645h - CIM
                can_add_filter(2, 0x7E0);         //7E0h -
                can_add_filter(2, 0x7E8);         //7E8h -
                can_add_filter(2, 0x311);         //311h -
                can_add_filter(2, 0x5E8);         //5E8h 
                
                can.attach(NULL);
                //can.mode(CAN::LocalTest);
                can_rx_thd.start(&can_read_thd);
                egt_thd.start(&egt_read_thd);
                return CombiSendReplyPacket(tx_packet, rx_packet, 0, 0, cmd_term_ack, 1000);
            }
        break;
        case cmd_can_bitrate:
            if (rx_packet->data_len == 4) {
                uint32_t bitrate = rx_packet->data[3] | (uint32_t)*rx_packet->data << 24 | (uint32_t)rx_packet->data[1] << 16 
                                | (uint32_t)rx_packet->data[2] << 8;
                can_configure(2, bitrate, false);
                return CombiSendReplyPacket(tx_packet, rx_packet, (uint8_t *)0x0, 0, cmd_term_ack, 1000);
            }
            break;
        break;
        case cmd_can_txframe:
            if (rx_packet->data_len != 15) {
                return false;
            }
            uint32_t id = (uint32_t)rx_packet->data[0] 
                        | (uint32_t)(rx_packet->data[1] << 8) 
                        | (uint32_t)(rx_packet->data[2] << 16)
                        | (uint32_t)(rx_packet->data[3] << 24);
            uint8_t length = rx_packet->data[12];
            uint8_t data[length];
            for (uint8_t i = 0; i < length; i++) {
                data[i] = rx_packet->data[4 + i];
            }

            can_send_timeout(id, (char*)data, length, rx_packet->data[13], rx_packet->data[14], 500);
            return CombiSendReplyPacket(tx_packet, rx_packet, (uint8_t *)0x0, 0, cmd_term_ack, 1000);
        break;
    }
    return false;
}

bool exec_cmd_board(packet_t *rx_packet, packet_t *tx_packet) {
    switch(rx_packet->cmd_code) {
        case cmd_brd_fwversion:
            return CombiSendReplyPacket(tx_packet, rx_packet, version, 2, cmd_term_ack, 1000);
        case cmd_brd_adcfilter:
            return CombiSendReplyPacket(tx_packet, rx_packet, (uint8_t *)0x0, 0, cmd_term_ack, 1000);
        case cmd_brd_adc:
            return CombiSendReplyPacket(tx_packet, rx_packet, (uint8_t *)0x0, 0, cmd_term_ack, 1000);
        case cmd_brd_egt:
            return CombiSendReplyPacket(tx_packet, rx_packet, egt_temp, 5, cmd_term_ack, 1000);
    }
    return false;
}

void combi_thread() {
    bool ret = false;
    bool state = false;
    packet_t rx_packet;
    packet_t tx_packet;

    while (1) {
        bool state = CombiReceivePacket(&rx_packet, 0);
        if ((state != false) && (rx_packet.term == cmd_term_ack)) {
            uint8_t cmd = rx_packet.cmd_code & 0xe0;
            switch (cmd) {
                case 0x20:
                    ret = exec_cmd_board(&rx_packet, &tx_packet);
                    break;
                case 0x40:
                    ret = exec_cmd_bdm(&rx_packet, &tx_packet);
                    break;
                case cmd_can_open:
                    ret = exec_cmd_can(&rx_packet, &tx_packet);
                    break;
                default:
                    ret = false;
            }
            if (ret != true) {
                CombiSendReplyPacket(&tx_packet, &rx_packet, 0, 0, cmd_term_nack, 1000);
            }
        }
    }
}

bool CombiReceivePacket(packet_t *packet, uint32_t timeout) {
    if (!combi.readable()) {
        return false;
    }
    bool state = false;
    uint8_t buffer[3] = {0};

    //read cmd, size, 
    state = combi.receive(buffer, 3);
    if (state == true) {
        packet->cmd_code = buffer[0];
        packet->data_len = (uint16_t)((buffer[1] & 0xffffU) << 8) | (uint16_t)buffer[2];
    }
    if (packet->data_len > 0) {
        state = combi.receive(data_buff, packet->data_len);
        packet->data = data_buff;
    }
    
    state = combi.receive(buffer, 1);
    packet->term = buffer[0];

    if (packet->term != cmd_term_ack) {
        state = false;
    }
    return state;
}

bool CombiSendReplyPacket(packet_t *reply, packet_t *source, uint8_t *data, uint16_t data_len, uint8_t term, uint32_t timeout){
    bool state;
    if ((reply == (packet_t *)0x0) || (source == (packet_t *)0x0)) {
        state = false;
    } else {
        reply->cmd_code = source->cmd_code;
        reply->data_len = data_len;
        if ((data != (uint8_t *)0x0) && (data_len != 0)) {
            reply->data = data;
        }
        reply->term = term;
        state = CombiSendPacket(reply, timeout);
    }
    return state;
}

bool CombiSendPacket(packet_t *packet, uint32_t timeout) {
    uint8_t *data_ptr;
    uint8_t buffer[64] = {0};
    uint16_t size = 0;
    (void) timeout;

    if (packet != (packet_t *)0x0) {
        buffer[0] = packet->cmd_code;
        buffer[1] = (uint8_t)(packet->data_len >> 8);
        buffer[2] = (uint8_t)packet->data_len;
        if (*(packet->data) != 0 && packet->data_len != 0) {
            data_ptr = packet->data;
            for (uint16_t cnt = 0; cnt < packet->data_len; cnt++) {
                buffer[3 + cnt] = *data_ptr;
                data_ptr++;
            }
            size = packet->data_len + 3;
            buffer[size] = packet->term;
            size++;
        } else {
            buffer[3] = packet->term;
            size = 4;
        }
        combi.send((uint8_t *)buffer, size);
        return true;
    }
  return false;
}