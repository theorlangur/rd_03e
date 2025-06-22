#include "lib_uart.h"
#include <functional>
#include <zephyr/drivers/uart.h>

namespace uart
{
    Channel::Channel(const struct device *pUART):
	m_pUART(pUART)
    {
    }

    Channel::~Channel()
    {
    }

    Channel::ExpectedResult Channel::Configure()
    {
	k_sem_init(&m_rx_sem, 0, 1);
	k_sem_init(&m_tx_sem, 1, 1);

	uart_irq_rx_disable(m_pUART);
	uart_irq_tx_disable(m_pUART);

	if (auto r = Flush(); !r) return r;

	uart_irq_callback_user_data_set(m_pUART, &uart_isr, (void *)this);

	return std::ref(*this);
    }

    Channel::ExpectedResult Channel::Open()
    {
	return std::ref(*this);
    }

    Channel::ExpectedResult Channel::Close()
    {
	return std::ref(*this);
    }

    void Channel::uart_isr(const struct device *uart_dev, void *user_data)
    {
	Channel *pC = (Channel *)user_data;
	if (!uart_dev) return;
	if (!uart_irq_update(uart_dev)) return;

	if (uart_irq_rx_ready(uart_dev))
	{
	    int b = uart_fifo_read(uart_dev, pC->m_pRecvBuf, pC->m_RecvLen);
	    if (b < 0)
	    {
		pC->m_RecvLen = b;
		uart_irq_rx_disable(uart_dev);
		k_sem_give(&pC->m_rx_sem);
	    }else
	    {
		pC->m_pRecvBuf += b;
		pC->m_RecvLen -= b;
		if (!pC->m_RecvLen)
		{
		    pC->m_pRecvBuf = nullptr;
		    uart_irq_rx_disable(uart_dev);
		    k_sem_give(&pC->m_rx_sem);
		}
	    }
	}

	if (uart_irq_tx_ready(uart_dev))
	{
	    if (pC->m_pSendBuf)
	    {
		int b = uart_fifo_fill(uart_dev, pC->m_pSendBuf, pC->m_SendLen);
		if (b < 0)
		{
		    pC->m_SendLen = b;
		    uart_irq_tx_disable(uart_dev);
		    k_sem_give(&pC->m_tx_sem);
		}else
		{
		    pC->m_pSendBuf += b;
		    pC->m_SendLen -= b;
		    if (!pC->m_SendLen)
		    {
			pC->m_pSendBuf = nullptr;
			uart_irq_tx_disable(uart_dev);
			k_sem_give(&pC->m_tx_sem);
		    }
		}
	    }else
	    {
		//disable
		uart_irq_tx_disable(uart_dev);
	    }
	}
    }

    Channel::ExpectedResult Channel::Send(const uint8_t *pData, size_t len)
    {
	CALL_WITH_EXPECTED("Channel::Send", k_sem_take(&m_tx_sem, Z_TIMEOUT_MS(m_DefaultWait)));
	m_pSendBuf = pData;
	m_SendLen = len;
	uart_irq_tx_enable(m_pUART);
	return std::ref(*this);
    }

    Channel::ExpectedValue<size_t> Channel::Read(uint8_t *pBuf, size_t len, duration_ms_t wait)
    {
	if (!len) return RetVal<size_t>{*this, size_t(0)};
	if (wait == kDefaultWait) wait = m_DefaultWait;
	bool addedPeek = m_HasPeekByte;
	if (m_HasPeekByte)
	{
	    pBuf[0] = m_PeekByte;
	    m_HasPeekByte = false;
	    --len;
	    ++pBuf;
	    if (!len) return RetVal<size_t>{*this, 1};
	}

	m_pRecvBuf = pBuf;
	m_RecvLen = len;
	uart_irq_rx_enable(m_pUART);
	CALL_WITH_EXPECTED("Channel::Read", k_sem_take(&m_rx_sem, Z_TIMEOUT_MS(wait)));
	return RetVal<size_t>{*this, len};
    }

    Channel::ExpectedResult Channel::Flush()
    {
	uart_irq_rx_disable(m_pUART);
	k_sem_reset(&m_rx_sem);
	uint8_t c;
	while(uart_fifo_read(m_pUART, &c, 1) == 1)
	    continue;
	return std::ref(*this);
    }

    Channel::ExpectedResult Channel::WaitAllSent()
    {
	CALL_WITH_EXPECTED("Channel::WaitAllSent", k_sem_take(&m_tx_sem, Z_TIMEOUT_MS(m_DefaultWait)));
	return std::ref(*this);
    }

    Channel::ExpectedValue<uint8_t> Channel::ReadByte(duration_ms_t wait)
    {
	if (wait == kDefaultWait) wait = m_DefaultWait;
	uint8_t b;
	if (auto e = Read(&b, 1, wait); !e)
	    return std::unexpected(e.error());
	else if (auto l = e.value().v; !l)
	    return std::unexpected(::Err{"Channel::ReadByte no data", 0});
	else
	    return RetVal<uint8_t>{std::ref(*this), b};
    }

    Channel::ExpectedValue<uint8_t> Channel::PeekByte(duration_ms_t wait)
    {
	if (m_HasPeekByte)
	    return RetVal<uint8_t>{std::ref(*this), m_PeekByte};
	if (wait == kDefaultWait) wait = m_DefaultWait;
	if (auto e = ReadByte(wait); !e) return e;
	else{
	    m_HasPeekByte = true;
	    m_PeekByte = e.value().v;
	    return RetVal<uint8_t>{std::ref(*this), m_PeekByte};
	}
	return RetVal<uint8_t>{*this, 0};
    }
}
