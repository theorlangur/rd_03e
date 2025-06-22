#ifndef LIB_UART_H_
#define LIB_UART_H_

#include <zephyr/kernel.h>
#include "lib_ret_err.h"
#include <expected>
#include <chrono>
namespace uart
{
    using duration_ms_t = int;
    class Channel
    {
    public:
        using Ref = std::reference_wrapper<Channel>;
        using ExpectedResult = std::expected<Ref, Err>;

        template<typename V>
        using RetVal = RetValT<Ref, V>;

        template<typename V>
        using ExpectedValue = std::expected<RetVal<V>, Err>;

        static const constexpr duration_ms_t kDefaultWait = duration_ms_t{-1};

        Channel(const struct device *pUART);
        ~Channel();

        ExpectedResult Configure();

        void SetDefaultWait(duration_ms_t w) { m_DefaultWait = w; }
        duration_ms_t GetDefaultWait() const { return m_DefaultWait; }

        ExpectedResult Open();
        ExpectedResult Close();

        ExpectedResult Send(const uint8_t *pData, size_t len);
        ExpectedValue<size_t> Read(uint8_t *pBuf, size_t len, duration_ms_t wait=kDefaultWait);
        ExpectedResult Flush();
        ExpectedResult WaitAllSent();

        ExpectedValue<uint8_t> ReadByte(duration_ms_t wait=kDefaultWait);
        ExpectedValue<uint8_t> PeekByte(duration_ms_t wait=kDefaultWait);

        //using EventCallback = GenericCallback<void(uart_event_type_t)>;
        //void SetEventCallback(EventCallback cb) { m_EventCallback = std::move(cb); }
        //bool HasEventCallback() const { return (bool)m_EventCallback; }

    private:
        static void uart_isr(const struct device *uart_dev, void *user_data);

        const struct device *m_pUART = nullptr;
        struct k_sem m_tx_sem;
        struct k_sem m_rx_sem;
        duration_ms_t m_DefaultWait{0};

        //receive buf
        uint8_t *m_pRecvBuf;
        int m_RecvLen;
        //transmitt buf
        const uint8_t *m_pSendBuf;
        int m_SendLen;

        bool m_HasPeekByte = false;
        uint8_t m_PeekByte = 0;

        //std::atomic<bool> m_DataReady={false};
        //EventCallback m_EventCallback;

        //bool m_DbgPrintSend = false;
    };
}
#endif
