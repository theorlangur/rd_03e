#ifndef LIB_DFR_C4001_H_
#define LIB_DFR_C4001_H_

#include "lib_misc_helpers.hpp"
#include "lib_uart.h"
#include <span>
#include "lib_uart_primitives.h"
#include <lib_type_traits.hpp>

namespace dfr
{
    template<class T>
    concept Invokable = requires (T const& t) { t(); };


    template<class T>
    struct Sendable
    {
        static constexpr const bool value = false;
    };

    template<class T, size_t N> requires 
        (std::is_same_v<T, uint8_t> || std::is_same_v<T, const uint8_t> 
         || std::is_same_v<T, char> || std::is_same_v<T, const char>
         )
    struct Sendable<const T(&)[N]>
    {
        static constexpr const bool value = true;
        static auto send(uart::Channel &c, const T (&a)[N])
        {
            return c.Send((const uint8_t*)a, N - 1/*skip 0*/);
        }
    };

    template<class T, size_t N>
    struct Sendable<T[N]>: Sendable<T(&)[N]>{};

    template<Invokable T>
    struct Sendable<T>
    {
        static constexpr const bool value = true;
        static auto send(uart::Channel &c, T const& a)
        {
            return a();
        }
    };

    class C4001: public uart::Channel
    {
        public:
            using duration_ms_t = uart::duration_ms_t;
            static const constexpr duration_ms_t kRestartTimeout{2000};
            static const constexpr duration_ms_t kDefaultWait{350};

            using Ref = std::reference_wrapper<C4001>;
            struct Err
            {
                ::Err uartErr;
                const char *pLocation;
            };

            using ExpectedResult = std::expected<Ref, Err>;
            struct CmdErr
            {
                Err e;
                uint16_t returnCode;
            };

            template<typename V>
                using RetVal = RetValT<Ref, V>;

            template<class V>
                using ExpectedValue = std::expected<RetVal<V>, Err>;

            struct Version
            {
                char m_Version[32];
            };

        public:

            /**********************************************************************/
            /* PresenceResult                                                     */
            /**********************************************************************/

            C4001(const struct device *pUART);

            ExpectedResult Init();
            ExpectedResult ReloadConfig();

            ExpectedResult Restart();
            ExpectedResult FactoryReset();

            const Version& GetHWVer() const { return m_HWVersion; }
            const Version& GetSWVer() const { return m_SWVersion; }
        private:

            constexpr static const uint8_t kCmdSensorStop[] = "sensorStop";
            constexpr static const uint8_t kCmdSensorStart[] = "sensorStart";
            constexpr static const uint8_t kCmdGetHWVersion[] = "getHWV";
            constexpr static const uint8_t kCmdGetSWVersion[] = "getSWV";

            template<class E>
            static ExpectedResult to_result(E &&e, const char* pLocation)
            {
                using PureE = std::remove_cvref_t<E>;
                if constexpr(is_expected_type_v<PureE>)
                {
                    if constexpr (std::is_same_v<PureE, ExpectedResult>)
                        return std::move(e);
                    else
                        return to_result(e.error(), pLocation);
                }else if constexpr (std::is_same_v<PureE,::Err>)
                    return ExpectedResult(std::unexpected(Err{e, pLocation}));
                else if constexpr (std::is_same_v<PureE,Err>)
                    return ExpectedResult(std::unexpected(e));
                else if constexpr (std::is_same_v<PureE,CmdErr>)
                    return ExpectedResult(std::unexpected(e.e));
                else
                {
                    static_assert(std::is_same_v<PureE,Err>, "Don't know how to convert passed type");
                    return ExpectedResult(std::unexpected(Err{}));
                }
            }

#define TRY_UART_COMM(f, location) \
            if (auto r = f; !r) \
            return to_result(std::move(r), location)

#define TRY_UART_COMM_CMD(f, location) \
            if (auto r = f; !r) \
            return to_cmd_result(std::move(r), location)

#define TRY_UART_COMM_CMD_WITH_RETRY(f, location) \
            if (auto r = f; !r) \
            {\
                if (retry) \
                {\
                    FMT_PRINTLN("Failed on " #f);\
                    continue;\
                }\
                return to_cmd_result(std::move(r), location);\
            }

            template<class... ToSend> static auto to_send(ToSend&&...args) { return std::forward_as_tuple(std::forward<ToSend>(args)...); }
            template<class... ToRecv> static auto to_recv(ToRecv&&...args) { return std::forward_as_tuple(std::forward<ToRecv>(args)...); }

            template<class ToSend> 
            ExpectedResult SendTpl(ToSend &&arg) 
            {
                TRY_UART_COMM(Sendable<ToSend>::send(*this, arg), "SendTpl");
                return std::ref(*this);
            }

            void DrainAndStop()
            {
                //[[maybe_unused]]auto r = uart::primitives::drain(*this, {.maxWait = 50});
                StopReading();
                //k_msleep(50);
            }

            template<class... ToSend> 
            ExpectedResult SendCmdNoResp(ToSend&&...args) 
            { 
                static_assert((Sendable<ToSend>::value && ... && true), "All arguments must be sendable");
                Channel::ExpectedResult _r(std::ref(*this));
                bool first = true;
                auto send_one = [&]<class SendArg>(SendArg &&a)
                {
                    if (!_r.has_value())
                        return;
                    if (first)
                        first = false;
                    else
                        _r = Sendable<decltype(" ")>::send(*this, " ");
                    _r = std::move(Sendable<SendArg>::send(*this, std::forward<SendArg>(a)));
                };
                (send_one(std::forward<ToSend>(args)),...);
                TRY_UART_COMM(_r, "SendArgs");
                TRY_UART_COMM(Sendable<decltype("\r\n")>::send(*this, "\r\n"), "<endl>");
                //TRY_UART_COMM(uart::primitives::drain(*this, {.maxWait = 50}), "SendCmdNoResp.drain");
                return std::ref(*this);
            }


            template<class... ToSend> 
            ExpectedResult SendCmd(ToSend&&...args) 
            { 
                static_assert((Sendable<ToSend>::value && ... && true), "All arguments must be sendable");
                Channel::ExpectedResult _r(std::ref(*this));
                bool first = true;
                auto send_one = [&]<class SendArg>(SendArg &&a)
                {
                    if (!_r.has_value())
                        return;
                    if (first)
                        first = false;
                    else
                        _r = Sendable<decltype(" ")>::send(*this, " ");
                    _r = std::move(Sendable<SendArg>::send(*this, std::forward<SendArg>(a)));
                };
                (send_one(std::forward<ToSend>(args)),...);
                TRY_UART_COMM(_r, "SendArgs");

                //ScopeExit onExitStopReading = [&]{ DrainAndStop(); };
                //AllowReadUpTo(m_recvBuf, sizeof(m_recvBuf));

                TRY_UART_COMM(Sendable<decltype("\r\n")>::send(*this, "\r\n"), "<endl>");

                //wait for an answer
                using namespace uart::primitives;
                if (auto r = find_any_str({}, *this, "Done\r\n", "Error\r\n"); !r)
                    return std::unexpected(Err{r.error()});
                else if (r->v != 0)//not 'Done', but 'Error'
                    return std::unexpected(Err{{"SendCmd Error"}});
                return std::ref(*this);
            }

            template<class... ToSend, class... ToRecv> 
            ExpectedResult SendCmdWithParams(std::tuple<ToSend...> tosend, std::tuple<ToRecv...> torecv, bool dbg = false) 
            { 
                static_assert((Sendable<ToSend>::value && ... && true), "All arguments must be sendable");
                Channel::ExpectedResult _r(std::ref(*this));
                bool first = true;
                auto send_one = [&]<class SendArg>(SendArg &&a)
                {
                    if (!_r.has_value())
                        return;
                    if (first)
                        first = false;
                    else
                        _r = Sendable<decltype(" ")>::send(*this, " ");
                    _r = std::move(Sendable<SendArg>::send(*this, std::forward<SendArg>(a)));
                };
                auto send_tuple = [&]<size_t... idx>(std::index_sequence<idx...>)
                {
                    (send_one(std::get<idx>(tosend)),...);
                };
                send_tuple(std::make_index_sequence<sizeof...(ToSend)>());
                TRY_UART_COMM(_r, "SendArgs");

                //ScopeExit onExitStopReading = [&]{ DrainAndStop(); };
                //AllowReadUpTo(m_recvBuf, sizeof(m_recvBuf));

                TRY_UART_COMM(SendTpl("\r\n"), "<endl>");

                auto recv_tuple = [&]<size_t... idx>(std::index_sequence<idx...>)
                {
                    return uart::primitives::read_any(*this, std::get<idx>(torecv)...);
                };

                FMT_PRINTLN("Receiving params");
                TRY_UART_COMM(recv_tuple(std::make_index_sequence<sizeof...(ToRecv)>()), "SendCmdWithParams");

                FMT_PRINTLN("Received. Receiving Done or Error");
                //wait for a final 'Done'
                using namespace uart::primitives;
                if (auto r = find_any_str({}, *this, "Done\r\n", "Error\r\n"); !r)
                    return std::unexpected(Err{r.error()});
                else if (r->v != 0)//not 'Done', but 'Error'
                    return std::unexpected(Err{{"SendCmd Error"}});
                FMT_PRINTLN("Done");
                return std::ref(*this);
            }

            ExpectedResult StopSensor();
            ExpectedResult StartSensor();

            ExpectedResult UpdateHWVersion();
            ExpectedResult UpdateSWVersion();

            ExpectedResult ReadFrame();
            //data
            //Version m_Version;
            //Configuration m_Configuration;
            Version m_HWVersion;
            Version m_SWVersion;

            class RxBlock: public Channel::RxBlock
            {
            public:
                RxBlock(C4001 &c):Channel::RxBlock(c, c.m_recvBuf, sizeof(c.m_recvBuf)){}
            private:
            };

            uint8_t m_recvBuf[128];
        public:
            bool m_dbg = false;
    };
}

template<>
struct ::tools::formatter_t<::Err>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, ::Err const& e)
    {
        return tools::format_to(std::forward<Dest>(dst), "E<{} at {}>", e.code, e.pLocation);
    }
};

template<>
struct ::tools::formatter_t<dfr::C4001::Err>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, dfr::C4001::Err const& e)
    {
        return tools::format_to(std::forward<Dest>(dst), "Err\\{uart=[{}] at {} }", e.uartErr, e.pLocation);
    }
};

#endif
