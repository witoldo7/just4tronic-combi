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

#include "CombiAdapter.h"
#include "interfaces.h"
#include "mbed_retarget.h"
#include "sizedefs.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include "canutils.h"
#include "string.h"

bool CombiReceivePacket(packet_t *packet, uint32_t timeout);
bool CombiSendReplyPacket(packet_t *reply, packet_t *source, uint8_t *data, uint16_t data_len, uint8_t term, uint32_t timeout);
bool CombiSendPacket(packet_t *packet, uint32_t timeout);

uint8_t version[2] = {0x03, 0x01};
uint8_t data_buff[64];
Thread  t;

void read_can_message(void) {
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
    printf("exec_bdm\r\n");
    return true;
}

bool exec_cmd_can(packet_t *rx_packet, packet_t *tx_packet) {
    //printf("exec_can\r\n");
    switch(rx_packet->cmd_code) {
        case cmd_can_open:
            if (rx_packet->data_len == 1) {
                if (*rx_packet->data != 0x1) {
                    printf("can close\r\n");
                    can_close();
                    can.attach(NULL);
                    return CombiSendReplyPacket(tx_packet, rx_packet, (uint8_t *)0x0, 0, cmd_term_ack, 1000);
                }
                printf("can open\r\n");
                can_open();
                
                can_add_filter(2, 0x645);         //645h - CIM
                can_add_filter(2, 0x7E0);         //7E0h -
                can_add_filter(2, 0x7E8);         //7E8h -
                can_add_filter(2, 0x311);         //311h -
                can_add_filter(2, 0x5E8);         //5E8h 
                
                can.attach(NULL);
                //can.mode(CAN::LocalTest);
                t.start(&read_can_message);
                return CombiSendReplyPacket(tx_packet, rx_packet, (uint8_t *)0x0, 0, cmd_term_ack, 1000);
            }
        break;
        case cmd_can_bitrate:
            printf("can bitrate\r\n");
            if (rx_packet->data_len == 4) {
                uint32_t bitrate = rx_packet->data[3] | (uint32_t)*rx_packet->data << 0x18 | (uint32_t)rx_packet->data[1] << 0x10 
                                | (uint32_t)rx_packet->data[2] << 8;
                printf("can bitrate %lu \r\n", bitrate);
                can_configure(2, bitrate, false);

                return CombiSendReplyPacket(tx_packet, rx_packet, (uint8_t *)0x0, 0, cmd_term_ack, 1000);
            }
            break;
        break;
        case cmd_can_txframe:
            if (rx_packet->data_len != 15) {
                printf("can txframe, wrong frame\r\n");
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
            
            #ifdef DEBUG
                printf("send txframe id: %lx, len: %x, data:", id, length);
                for (int i = 0; i < length; i++) {
                    printf(" %02x", data[i]);
                }
                printf("\r\n");
            #endif
            can_send_timeout(id, (char*)data, length, rx_packet->data[13], rx_packet->data[14], 500);
            return CombiSendReplyPacket(tx_packet, rx_packet, (uint8_t *)0x0, 0, cmd_term_ack, 1000);
        break;
    }
    return false;
}

bool exec_cmd_board(packet_t *rx_packet, packet_t *tx_packet) {
    //printf("exec_board\r\n");
    switch(rx_packet->cmd_code) {
        case cmd_brd_fwversion:
            printf("brd_fwversion\r\n");
            return CombiSendReplyPacket(tx_packet, rx_packet, version, 2, cmd_term_ack, 1000);
        case cmd_brd_adcfilter:
            printf("brd_adcfilter\r\n");
            return CombiSendReplyPacket(tx_packet, rx_packet, (uint8_t *)0x0, 0, cmd_term_ack, 1000);
        case cmd_brd_adc:
            printf("brd_adc\r\n");
            return CombiSendReplyPacket(tx_packet, rx_packet, (uint8_t *)0x0, 0, cmd_term_ack, 1000);
        case cmd_brd_egt:
            //printf("brd_egt\r\n");
            return CombiSendReplyPacket(tx_packet, rx_packet, (uint8_t *)0x0, 0, cmd_term_ack, 1000);
        default:
            printf("brd_unhandled cmd: %x", rx_packet->cmd_code);
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
    #ifdef DEBUG
        printf("\r\nReceivedPacket size: %hu, cmd: %02x, term: %02x, val:", packet->data_len, packet->cmd_code, packet->term);
        for(uint16_t i = 0; i < packet->data_len; i++) {
            printf(" %02x", packet->data[i]);
        }
        printf("\r\n");
    #endif
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
        #ifdef DEBUG
            printf("SendPacket size: %hu, cmd: %02x, data:", size, buffer[0]);
            for(uint8_t i = 0; i < packet->data_len; i++) 
                printf(" %02x", buffer[3+i]);
            printf("\r\n");
        #endif
        combi.send((uint8_t *)buffer, size);
        return true;
    }
  return false;
}