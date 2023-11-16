#pragma once
#ifndef PMBCom_
#define PMBCom_

#include <Arduino.h>
#include <PMB/SLIP.h>

#define PMBCMD_Pb 0x01
#define PMBCMD_Li 0x02
#define PMBCMD_AC 0x04
#define PMBCMD_ALL 0x07
#define PMBCMD_OK 0x70
#define PMBVAL_OK 0x71

class PMBCOM
{
    SLIP *PMBSLIP;
    uint8_t Target = 0;
    uint8_t element_num = 0;

public:
    int32_t voltage[3];
    uint8_t flag = 0;
    void begin(SLIP *slip);
    bool check();
    void request(uint8_t target);
    bool Get();
};

IRAM_ATTR void PMBCOM::begin(SLIP *slip)
{
    PMBSLIP = slip;
}

IRAM_ATTR bool PMBCOM::check()
{
    uint8_t counter = 0;
    PMBSLIP->send((uint8_t)(PMBCMD_OK));
    while (!PMBSLIP->read())
    {
        if (counter > 200)
        {
            return false;
        }
        else
        {
            counter++;
        }
        delay(1);
    }
    if (PMBSLIP->rx_data[0] == PMBVAL_OK)
    {
        return true;
    }
    return false;
}

IRAM_ATTR void PMBCOM::request(uint8_t target)
{
    Target = target;
    flag++;
    PMBSLIP->send((uint8_t)(target));
}

IRAM_ATTR bool PMBCOM::Get()
{
    uint8_t i = 0;
    if (PMBSLIP->read())
    {
        if (Target & PMBCMD_Pb == PMBCMD_Pb)
        {
            voltage[i] = (int32_t)(PMBSLIP->rx_data[i * 4 + 0] << 24 | PMBSLIP->rx_data[i * 4 + 1] << 16 | PMBSLIP->rx_data[i * 4 + 2] << 8 | PMBSLIP->rx_data[i * 4 + 3]);
            i++;
        }
        if (Target & PMBCMD_Li == PMBCMD_Li)
        {
            voltage[i] = (int32_t)(PMBSLIP->rx_data[i * 4 + 0] << 24 | PMBSLIP->rx_data[i * 4 + 1] << 16 | PMBSLIP->rx_data[i * 4 + 2] << 8 | PMBSLIP->rx_data[i * 4 + 3]);
            i++;
        }
        if (Target & PMBCMD_AC == PMBCMD_AC)
        {
            voltage[i] = (int32_t)(PMBSLIP->rx_data[i * 4 + 0] << 24 | PMBSLIP->rx_data[i * 4 + 1] << 16 | PMBSLIP->rx_data[i * 4 + 2] << 8 | PMBSLIP->rx_data[i * 4 + 3]);
            i++;
        }
        element_num = i;
        flag = 0;
        return true;
    }
    return false;
}

#endif