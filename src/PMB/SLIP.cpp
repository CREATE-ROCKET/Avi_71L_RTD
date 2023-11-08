#include <PMB/SLIP.h>

IRAM_ATTR void SLIP::begin(HardwareSerial *serial)
{
    this->ser = serial;
}

IRAM_ATTR bool SLIP::encode()
{
    uint8_t tx_buf[256];
    uint16_t k = 0;
    for (uint8_t i = 0; i < tx_len; i++)
    {
        if (tx_data[i] == END_CODE)
        {
            tx_buf[k] = ESC_CODE;
            k++;
            tx_buf[k] = ESC_END;
        }
        else if (tx_data[i] == ESC_CODE)
        {
            tx_buf[k] = ESC_CODE;
            k++;
            tx_buf[k] = ESC_ESC;
        }
        else
        {
            tx_buf[k] = tx_data[i];
        }
        k++;
    }
    if (k > 255)
    {
        return false;
    }
    for (uint8_t i = 0; i < k; i++)
    {
        tx_data[i] = tx_buf[i];
    }
    tx_data[k] = END_CODE;
    tx_len = k + 1;
    return true;
}

IRAM_ATTR bool SLIP::decode()
{
    uint8_t rx_buf[256];
    uint8_t i = 0;
    uint8_t k = 0;
    while (i < rx_len)
    {
        if (rx_data[i] == ESC_CODE)
        {
            i++;
            if (rx_data[i] == ESC_END)
            {
                rx_buf[k] = END_CODE;
            }
            else
            {
                rx_buf[k] = ESC_CODE;
            }
        }
        else if (rx_data[i] == END_CODE)
        {
            rx_buf[k] = END_CODE;
            k++;
            break;
        }
        else
        {
            rx_buf[k] = rx_data[i];
        }
        i++;
        k++;
    }
    if (k == 0)
    {
        return false;
    }
    rx_len = k;
    for (uint8_t j = 0; j < rx_len; j++)
    {
        rx_data[j] = rx_buf[j];
    }
    return true;
}

IRAM_ATTR bool SLIP::addtx(uint8_t data)
{
    if (128 - tx_len > 1)
    {
        tx_data[tx_len] = data;
        tx_len++;
        return true;
    }
    return false;
}

IRAM_ATTR bool SLIP::addtx(int32_t data)
{
    uint8_t tx_buf[4];
    tx_buf[0] = (uint8_t)(data >> 24);
    tx_buf[1] = (uint8_t)(data >> 16);
    tx_buf[2] = (uint8_t)(data >> 8);
    tx_buf[3] = (uint8_t)(data);
    return addtx(tx_buf, 4);
}

IRAM_ATTR bool SLIP::addtx(uint8_t *data, uint8_t size)
{
    if (128 - tx_len > size)
    {
        for (uint8_t i = 0; i < size; i++)
        {
            tx_data[tx_len + i] = data[i];
        }
        tx_len = tx_len + size;
        return true;
    }
    else
    {
        return false;
    }
}

IRAM_ATTR void SLIP::sendMsg()
{
    for (int16_t i = 0; i < tx_len; i++)
    {
        ser->write(tx_data[i]);
    }
    tx_len = 0;
}

IRAM_ATTR void SLIP::send()
{
    if (encode())
    {
        sendMsg();
    }
}

IRAM_ATTR void SLIP::send(uint8_t data)
{
    addtx((uint8_t)(data));
    if (encode())
    {
        sendMsg();
    }
}

IRAM_ATTR void SLIP::send(int32_t data)
{
    addtx((int32_t)(data));
    if (encode())
    {
        sendMsg();
    }
}

IRAM_ATTR void SLIP::send(uint8_t *data, uint8_t size)
{
    addtx(data, size);
    if (encode())
    {
        sendMsg();
    }
}

IRAM_ATTR bool SLIP::receive()
{
    uint16_t i = 0;
    if (ser->available())
    {
        while (ser->available() && (i < 256))
        {
            rx_data[i] = (uint8_t)(ser->read());
            i++;
            if (rx_data[i - 1] == END_CODE)
            {
                return true;
            }
        }
    }
    return false;
}

IRAM_ATTR bool SLIP::read()
{
    if (receive())
    {
        decode();
        return true;
    }
    return false;
}