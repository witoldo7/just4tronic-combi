/*
 * Copyright (c) 2018-2019, Arm Limited and affiliates.
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "stdint.h"
#include "usbcombi.h"
#include "EndpointResolver.h"
#include "AsyncOp.h"
#include "usb_phy_api.h"

static const uint8_t cdc_line_coding_default[7] = {0x80, 0x25, 0x00, 0x00, 0x00, 0x00, 0x08};

#define DEFAULT_CONFIGURATION (1)

#define CDC_SET_LINE_CODING        0x20
#define CDC_GET_LINE_CODING        0x21
#define CDC_SET_CONTROL_LINE_STATE 0x22

// Control Line State bits
#define CLS_DTR   (1 << 0)
#define CLS_RTS   (1 << 1)

#define CDC_MAX_PACKET_SIZE    32
#define INT_EP_SIZE             8

class USBCombi::AsyncWrite: public AsyncOp {
public:
    AsyncWrite(USBCombi *serial, uint8_t *buf, uint32_t size):
        serial(serial), tx_buf(buf), tx_size(size), result(false)
    {

    }

    virtual ~AsyncWrite()
    {

    }

    virtual bool process()
    {
        uint32_t actual_size = 0;
        serial->send_nb(tx_buf, tx_size, &actual_size, true);
        tx_size -= actual_size;
        tx_buf += actual_size;
        if (tx_size == 0) {
            result = true;
            return true;
        }

        // Start transfer if it hasn't been
        serial->_send_isr_start();
        return false;
    }

    USBCombi *serial;
    uint8_t *tx_buf;
    uint32_t tx_size;
    bool result;
};

class USBCombi::AsyncRead: public AsyncOp {
public:
    AsyncRead(USBCombi *serial, uint8_t *buf, uint32_t size, uint32_t *size_read, bool read_all)
        :   serial(serial), rx_buf(buf), rx_size(size), rx_actual(size_read), all(read_all), result(false)
    {

    }

    virtual ~AsyncRead()
    {

    }

    virtual bool process()
    {
        uint32_t actual_size = 0;
        serial->receive_nb(rx_buf, rx_size, &actual_size);
        rx_buf += actual_size;
        *rx_actual += actual_size;
        rx_size -= actual_size;
        if ((!all && *rx_actual > 0) || (rx_size == 0)) {
            // Wake thread if request is done
            result = true;
            return true;
        }

        serial->_receive_isr_start();
        return false;
    }

    USBCombi *serial;
    uint8_t *rx_buf;
    uint32_t rx_size;
    uint32_t *rx_actual;
    bool all;
    bool result;
};

class USBCombi::AsyncWait: public AsyncOp {
public:
    AsyncWait(USBCombi *serial)
        :   serial(serial)
    {

    }

    virtual ~AsyncWait()
    {

    }

    virtual bool process()
    {
        return true;
    }

    USBCombi *serial;
};

USBCombi::USBCombi(bool connect_blocking, uint16_t vendor_id, uint16_t product_id, uint16_t product_release)
    : USBDevice(get_usb_phy(), vendor_id, product_id, product_release)

{
    _init();
    if (connect_blocking) {
        connect();
        wait_ready();
    } else {
        init();
    }
}

USBCombi::USBCombi(USBPhy *phy, uint16_t vendor_id, uint16_t product_id, uint16_t product_release)
    : USBDevice(phy, vendor_id, product_id, product_release)
{
    _init();
}

USBCombi::~USBCombi()
{
    deinit();
}

void USBCombi::_init()
{
    memcpy(_cdc_line_coding, cdc_line_coding_default, sizeof(_cdc_line_coding));

    EndpointResolver resolver(endpoint_table());
    resolver.endpoint_ctrl(CDC_MAX_PACKET_SIZE);
    _int_in = resolver.endpoint_in(USB_EP_TYPE_INT, CDC_MAX_PACKET_SIZE);
    _bulk_in = resolver.endpoint_in(USB_EP_TYPE_BULK, CDC_MAX_PACKET_SIZE);
    (void)resolver.endpoint_out(USB_EP_TYPE_BULK, CDC_MAX_PACKET_SIZE); //FIXME: ep5
    _bulk_out = resolver.endpoint_out(USB_EP_TYPE_BULK, CDC_MAX_PACKET_SIZE);
    
    MBED_ASSERT(resolver.valid());

    _tx_in_progress = false;
    _tx_buf = _tx_buffer;
    _tx_size = 0;

    _rx_in_progress = false;
    _rx_buf = _rx_buffer;
    _rx_size = 0;
}

void USBCombi::callback_reset()
{
    assert_locked();
    /* Called in ISR context */

};

void USBCombi::callback_state_change(DeviceState new_state)
{
    assert_locked();
    /* Called in ISR context */

    if (new_state != Configured) {
    }
}

void USBCombi::callback_request(const setup_packet_t *setup)
{
    assert_locked();
    /* Called in ISR context */

    RequestResult result = PassThrough;
    uint8_t *data = NULL;
    uint32_t size = 0;

    /* Only process class-specific requests */
    if (setup->bmRequestType.Type == CLASS_TYPE) {
        switch (setup->bRequest) {
            case CDC_GET_LINE_CODING:
                result = Send;
                data = _cdc_line_coding;
                size = 7;
                break;
            case CDC_SET_LINE_CODING:
                result = Receive;
                data = _cdc_new_line_coding;
                size = 7;
                break;
            case CDC_SET_CONTROL_LINE_STATE:
                result = Success;
                break;
            default:
                result = Failure;
                break;
        }
    }
    complete_request(result, data, size);
}

void USBCombi::callback_request_xfer_done(const setup_packet_t *setup, bool aborted)
{
    assert_locked();
    /* Called in ISR context */

    if (aborted) {
        complete_request_xfer_done(false);
        return;
    }

    bool success = false;

    /* Process class-specific requests */
    if (setup->bmRequestType.Type == CLASS_TYPE) {
        if ((setup->bRequest == CDC_SET_LINE_CODING) && (setup->wLength == 7)) {
            if (memcmp(_cdc_line_coding, _cdc_new_line_coding, 7)) {
                memcpy(_cdc_line_coding, _cdc_new_line_coding, 7);

                const uint8_t *buf = _cdc_line_coding;
                int baud = buf[0] + (buf[1] << 8)
                           + (buf[2] << 16) + (buf[3] << 24);
                int stop = buf[4];
                int bits = buf[6];
                int parity = buf[5];

                line_coding_changed(baud, bits, parity, stop);
            }
            success = true;
        }
        if (setup->bRequest == CDC_GET_LINE_CODING) {
            success = true;
        }
    }

    complete_request_xfer_done(success);
}

void USBCombi::callback_set_configuration(uint8_t configuration)
{
    assert_locked();
    /* Called in ISR context */

    bool ret = false;
    if (configuration == DEFAULT_CONFIGURATION) {
        // Configure endpoints > 0
        endpoint_add(_int_in, CDC_MAX_PACKET_SIZE, USB_EP_TYPE_INT);
        endpoint_add(_bulk_in, CDC_MAX_PACKET_SIZE, USB_EP_TYPE_BULK, &USBCombi::_send_isr);
        endpoint_add(_bulk_out, CDC_MAX_PACKET_SIZE, USB_EP_TYPE_BULK, &USBCombi::_receive_isr);

        read_start(_bulk_out, _rx_buf, sizeof(_rx_buffer));
        _rx_in_progress = true;

        ret = true;
    }

    complete_set_configuration(ret);
}

void USBCombi::callback_set_interface(uint16_t interface, uint8_t alternate)
{
    assert_locked();
    complete_set_interface(true);
}


bool USBCombi::ready()
{
    lock();

    bool ready = true;

    unlock();
    return ready;
}

void USBCombi::wait_ready()
{
    lock();

    AsyncWait wait_op(this);
    _connected_list.add(&wait_op);

    unlock();

    wait_op.wait(NULL);
}

int USBCombi::putc(int c)
{
    if (send((uint8_t *)&c, 1)) {
        return c;
    } else {
        return -1;
    }
}

int USBCombi::getc()
{
    uint8_t c = 0;
    if (receive(&c, sizeof(c))) {
        return c;
    } else {
        return -1;
    }
}

uint8_t USBCombi::available()
{
    USBCombi::lock();

    uint8_t size = 0;
    if (!_rx_in_progress) {
        size = _rx_size > 0xFF ? 0xFF : _rx_size;
    }

    USBCombi::unlock();
    return size;
}

bool USBCombi::send(uint8_t *buffer, uint32_t size)
{
    lock();

    AsyncWrite write_op(this, buffer, size);
    _tx_list.add(&write_op);

    unlock();

    write_op.wait(NULL);
    return write_op.result;
}

void USBCombi::send_nb(uint8_t *buffer, uint32_t size, uint32_t *actual, bool now)
{
    lock();

    *actual = 0;
    if (!_tx_in_progress) {
        uint32_t free = sizeof(_tx_buffer) - _tx_size;
        uint32_t write_size = free > size ? size : free;
        if (size > 0) {
            memcpy(_tx_buf, buffer, write_size);
        }
        _tx_size += write_size;
        *actual = write_size;
        if (now) {
            _send_isr_start();
        }
    }

    unlock();
}

void USBCombi::_send_isr_start()
{
    assert_locked();

    if (!_tx_in_progress && _tx_size) {
        if (USBDevice::write_start(_bulk_in, _tx_buffer, _tx_size)) {
            _tx_in_progress = true;
        }
    }
}

/*
* Called by when CDC data is sent
* Warning: Called in ISR
*/
void USBCombi::_send_isr()
{
    assert_locked();
    write_finish(_bulk_in);
    _tx_buf = _tx_buffer;
    _tx_size = 0;
    _tx_in_progress = false;

    _tx_list.process();
    if (!_tx_in_progress) {
        data_tx();
    }
}

bool USBCombi::receive(uint8_t *buffer, uint32_t size,  uint32_t *size_read)
{
    lock();

    bool read_all = size_read == NULL;
    uint32_t size_read_dummy;
    uint32_t *size_read_ptr = read_all ? &size_read_dummy : size_read;
    *size_read_ptr = 0;
    AsyncRead read_op(this, buffer, size, size_read_ptr, read_all);
    _rx_list.add(&read_op);

    unlock();

    read_op.wait(NULL);
    return read_op.result;
}

void USBCombi::receive_nb(uint8_t *buffer, uint32_t size,  uint32_t *size_read)
{
    *size_read = 0;
    if ( !_rx_in_progress) {
        // Copy data over
        uint32_t copy_size = _rx_size > size ? size : _rx_size;
        memcpy(buffer, _rx_buf, copy_size);
        *size_read = copy_size;
        _rx_buf += copy_size;
        _rx_size -= copy_size;
        if (_rx_size == 0) {
            _receive_isr_start();
        }
    }
}

void USBCombi::_receive_isr_start()
{
    if ((_rx_size == 0) && !_rx_in_progress) {
        // Refill the buffer
        read_start(_bulk_out, _rx_buffer, sizeof(_rx_buffer));
        _rx_in_progress = true;
    }
}

/*
* Called by when CDC data is received
* Warning: Called in ISR
*/
void USBCombi::_receive_isr()
{
    assert_locked();

    MBED_ASSERT(_rx_size == 0);
    _rx_buf = _rx_buffer;
    _rx_size = read_finish(_bulk_out);
    _rx_in_progress = false;
    _rx_list.process();
    if (!_rx_in_progress) {
        data_rx();
    }

}

const uint8_t *USBCombi::device_desc()
{
    uint8_t ep0_size = endpoint_max_packet_size(0x00);
    uint8_t device_descriptor_temp[] = {
        18,                   // bLength
        1,                    // bDescriptorType
        0, 0x02,           // bcdUSB
        255,                    // bDeviceClass
        0,                    // bDeviceSubClass
        0,                    // bDeviceProtocol
        ep0_size,             // bMaxPacketSize0
        (uint8_t)(LSB(vendor_id)), (uint8_t)(MSB(vendor_id)),  // idVendor
        (uint8_t)(LSB(product_id)), (uint8_t)(MSB(product_id)),// idProduct
        0x00, 0x01,           // bcdDevice
        1,                    // iManufacturer
        2,                    // iProduct
        3,                    // iSerialNumber
        1                     // bNumConfigurations
    };
    MBED_ASSERT(sizeof(device_descriptor_temp) == sizeof(device_descriptor));
    memcpy(device_descriptor, device_descriptor_temp, sizeof(device_descriptor));
    return device_descriptor;
}

const uint8_t *USBCombi::string_iinterface_desc()
{
    static const uint8_t stringIinterfaceDescriptor[] = {
        0x08,
        STRING_DESCRIPTOR,
        'H', 0, 'U', 0, 'J', 0,
    };
    return stringIinterfaceDescriptor;
}

const uint8_t *USBCombi::string_iserial_desc()
{
    static const uint8_t string_iserial_descriptor[] = {
        0x0A,                                                           /*bLength*/
        STRING_DESCRIPTOR,                                              /*bDescriptorType 0x03*/
        'B', 0, 'E', 0, 'E', 0, 'F', 0,                                 /*bString iSerial - BEEF*/
    };
    return string_iserial_descriptor;
}

const uint8_t *USBCombi::string_iproduct_desc()
{
    static const uint8_t string_iproduct_descriptor[] = {
        0x1A,
        STRING_DESCRIPTOR,
        'C', 0, 'o', 0, 'm', 0, 'b', 0, 'i', 0,
        'A', 0, 'd', 0, 'a', 0, 'p', 0, 't', 0,
        'e', 0, 'r', 0,
    };
    return string_iproduct_descriptor;
}


const uint8_t *USBCombi::string_imanufacturer_desc()
{
    static const uint8_t string_imanufacturer_descriptor[] = {
        0x14,                                            /*bLength*/
        STRING_DESCRIPTOR,                               /*bDescriptorType 0x03*/
        'S', 0, 'h', 0, 't', 0, 'e', 0, 'p', 0,
        'c', 0, 'e', 0, 'l', 0, 'l', 0, /*bString iManufacturer - mbed.org*/
    };
    return string_imanufacturer_descriptor;
}


#define CONFIG1_DESC_SIZE (9+9+5+5 + 4+5+7 + 9+7+7)

const uint8_t *USBCombi::configuration_desc(uint8_t index)
{
    uint8_t config_descriptor_temp[] = {
        // configuration descriptor
        CONFIGURATION_DESCRIPTOR_LENGTH,// bLength
        CONFIGURATION_DESCRIPTOR,   // bDescriptorType
        LSB(CONFIG1_DESC_SIZE),    // wTotalLength
        MSB(CONFIG1_DESC_SIZE),
        2,                          // bNumInterfaces
        1,                          // bConfigurationValue
        0,                          // iConfiguration
        0xc0,                       // bmAttributes
        50,                         // bMaxPower

        // Interface 0 setting 0

        // interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
        INTERFACE_DESCRIPTOR_LENGTH,// bLength
        INTERFACE_DESCRIPTOR,       // bDescriptorType
        0,                          // bInterfaceNumber
        0,                          // bAlternateSetting
        1,                          // bNumEndpoints
        2,                       // bInterfaceClass
        2,                       // bInterfaceSubClass
        1,                       // bInterfaceProtocol
        0,                          // iInterface

        // CDC Header Functional Descriptor, CDC Spec 5.2.3.1, Table 26
        5,                      // bFunctionLength
        0x24,                   // bDescriptorType
        0x00,                   // bDescriptorSubtype
        0x10, 0x01,             // bcdCDC

        // Call Management Functional Descriptor, CDC Spec 5.2.3.2, Table 27
        5,                      // bFunctionLength
        0x24,                   // bDescriptorType
        0x01,                   // bDescriptorSubtype
        0x01,                   // bmCapabilities
        1,                      // bDataInterface

                // Abstract Control Management Functional Descriptor, CDC Spec 5.2.3.3, Table 28
        4,                      // bFunctionLength
        0x24,                   // bDescriptorType
        0x02,                   // bDescriptorSubtype
        0x02,                   // bmCapabilities

        // Union Functional Descriptor, CDC Spec 5.2.3.8, Table 33
        5,                      // bFunctionLength
        0x24,                   // bDescriptorType
        0x06,                   // bDescriptorSubtype
        0,                      // bMasterInterface
        1,                      // bSlaveInterface0

        // endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
        ENDPOINT_DESCRIPTOR_LENGTH, // bLength
        ENDPOINT_DESCRIPTOR,        // bDescriptorType
        _int_in,                   // bEndpointAddress
        E_INTERRUPT,                // bmAttributes (0x02=bulk)
        LSB(INT_EP_SIZE),           // wMaxPacketSize (LSB)
        MSB(INT_EP_SIZE),           // wMaxPacketSize (MSB)
        10,                          // bInterval

                // interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
        INTERFACE_DESCRIPTOR_LENGTH,// bLength
        INTERFACE_DESCRIPTOR,       // bDescriptorType
        1,                          // bInterfaceNumber
        0,                          // bAlternateSetting
        2,                          // bNumEndpoints
        10,                       // bInterfaceClass
        0,                       // bInterfaceSubClass
        0,                       // bInterfaceProtocol
        0,                          // iInterface

                // endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
        ENDPOINT_DESCRIPTOR_LENGTH, // bLength
        ENDPOINT_DESCRIPTOR,        // bDescriptorType
        _bulk_out,                   // bEndpointAddress
        E_BULK,                     // bmAttributes (0x02=bulk)
        LSB(CDC_MAX_PACKET_SIZE),           // wMaxPacketSize (LSB)
        MSB(CDC_MAX_PACKET_SIZE),           // wMaxPacketSize (MSB)
        0,                          // bInterval

               // endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
        ENDPOINT_DESCRIPTOR_LENGTH, // bLength
        ENDPOINT_DESCRIPTOR,        // bDescriptorType
        _bulk_in,                    // bEndpointAddress
        E_BULK,                     // bmAttributes (0x02=bulk)
        LSB(CDC_MAX_PACKET_SIZE),           // wMaxPacketSize (LSB)
        MSB(CDC_MAX_PACKET_SIZE),           // wMaxPacketSize (MSB)
        1,                          // bInterval
    };

    if (index == 0) {
        MBED_ASSERT(sizeof(config_descriptor_temp) == sizeof(_config_descriptor));
        memcpy(_config_descriptor, config_descriptor_temp, sizeof(_config_descriptor));
        return _config_descriptor;
    } else {
        return NULL;
    }
}
