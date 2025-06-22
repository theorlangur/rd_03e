#include "lib_uart.h"

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
		return *this;
	}

	Channel::ExpectedResult Channel::Open()
	{
		return *this;
	}

	Channel::ExpectedResult Channel::Close()
	{
		return *this;
	}

	Channel::ExpectedValue<size_t> Channel::GetReadyToReadDataLen()
	{
		return RetVal<size_t>{*this, 0};
	}

	Channel::ExpectedValue<size_t> Channel::GetReadyToWriteDataLen()
	{
		return RetVal<size_t>{*this, 0};
	}

	Channel::ExpectedResult Channel::Send(const uint8_t *pData, size_t len)
	{
		return *this;
	}

	Channel::ExpectedResult Channel::SendWithBreak(const uint8_t *pData, size_t len, size_t breakLen)
	{
		return *this;
	}

	Channel::ExpectedValue<size_t> Channel::Read(uint8_t *pBuf, size_t len, duration_ms_t wait)
	{
		return RetVal<size_t>{*this, 0};
	}

	Channel::ExpectedResult Channel::Flush()
	{
		return *this;
	}
	Channel::ExpectedResult Channel::WaitAllSent()
	{
		return *this;
	}

	Channel::ExpectedValue<uint8_t> Channel::ReadByte(duration_ms_t wait)
	{
		return RetVal<uint8_t>{*this, 0};
	}

	Channel::ExpectedValue<uint8_t> Channel::PeekByte(duration_ms_t wait)
	{
		return RetVal<uint8_t>{*this, 0};
	}
}
