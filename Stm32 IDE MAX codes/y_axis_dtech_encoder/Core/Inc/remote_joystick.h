/* =========================================================
   REMOTE JOYSTICK INPUT (USART1 RX via RS485, interrupt driven)

   One ASCII line per update:   "<0..4095>\n"   e.g. "2051\n"
   2048 = stop / centre.

   FAIL-SAFE: no valid line for REMOTE_TIMEOUT_MS -> returns centre.

   Include this header from main.c ONLY (functions are static).
   ========================================================= */
#ifndef REMOTE_JOYSTICK_H
#define REMOTE_JOYSTICK_H

#include <stdint.h>

#ifndef REMOTE_CENTER
#define REMOTE_CENTER      2048U
#endif
#define REMOTE_TIMEOUT_MS  250U

static volatile uint16_t remote_value = REMOTE_CENTER;
static volatile uint32_t remote_stamp = 0U;
static volatile uint8_t  remote_valid = 0U;

static char    remote_buf[6];
static uint8_t remote_len = 0U;
static uint8_t remote_bad = 0U;   /* current line is garbage -> skip to '\n' */

/* Feed one received byte. Hardware independent (host-testable). */
static void Remote_Feed(uint8_t c, uint32_t now_ms)
{
    if (c == '\r')
    {
        return;
    }

    if (c == '\n')
    {
        if ((!remote_bad) && (remote_len > 0U))
        {
            uint32_t v = 0U;
            for (uint8_t i = 0U; i < remote_len; i++)
            {
                v = (v * 10U) + (uint32_t)(remote_buf[i] - '0');
            }
            if (v <= 4095U)
            {
                remote_value = (uint16_t)v;
                remote_stamp = now_ms;
                remote_valid = 1U;
            }
        }
        remote_len = 0U;
        remote_bad = 0U;
        return;
    }

    if ((c >= '0') && (c <= '9') && (remote_len < 4U))
    {
        remote_buf[remote_len++] = (char)c;
    }
    else
    {
        remote_bad = 1U;   /* non-digit, or more than 4 digits */
    }
}

/* Latest value, or `fallback` if nothing fresh has arrived. */
static uint16_t Remote_Get(uint32_t now_ms, uint16_t fallback)
{
    if ((remote_valid != 0U) &&
        ((now_ms - remote_stamp) <= REMOTE_TIMEOUT_MS))
    {
        return remote_value;
    }
    return fallback;
}

#ifndef REMOTE_JOYSTICK_HOST_TEST
/* ---------------- STM32 HAL glue ---------------- */
#include "main.h"

#define REMOTE_UART_HANDLE   huart1
#define REMOTE_UART_IRQn     USART1_IRQn

extern UART_HandleTypeDef REMOTE_UART_HANDLE;

static void Remote_Init(void)
{
    __HAL_UART_ENABLE_IT(&REMOTE_UART_HANDLE, UART_IT_RXNE);
    HAL_NVIC_SetPriority(REMOTE_UART_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(REMOTE_UART_IRQn);
}

static uint16_t Remote_ReadOrCenter(void)
{
    return Remote_Get(HAL_GetTick(), REMOTE_CENTER);
}

/* Call from USART1_IRQHandler */
static void Remote_OnUartIRQ(void)
{
    uint32_t sr = REMOTE_UART_HANDLE.Instance->SR;
    uint8_t  c  = (uint8_t)(REMOTE_UART_HANDLE.Instance->DR & 0xFFU); /* reading DR clears ORE/FE/NE */

    if ((sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE)) != 0U)
    {
        remote_bad = 1U;   /* corrupted byte -> drop this line */
        if ((sr & USART_SR_RXNE) == 0U) { return; }
    }
    if ((sr & USART_SR_RXNE) != 0U)
    {
        Remote_Feed(c, HAL_GetTick());
    }
}
#endif /* REMOTE_JOYSTICK_HOST_TEST */

#endif /* REMOTE_JOYSTICK_H */