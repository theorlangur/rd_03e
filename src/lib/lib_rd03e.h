#ifndef LIB_RD03E_H_
#define LIB_RD03E_H_

#include "lib_misc_helpers.hpp"
#include "lib_uart.h"
#include <span>
#include "lib_uart_primitives.h"
#include <lib_type_traits.hpp>

namespace ai_thinker
{
class RD03E: public uart::Channel
{
public:
    using duration_ms_t = uart::duration_ms_t;
    static const constexpr duration_ms_t kRestartTimeout{2000};
    static const constexpr duration_ms_t kDefaultWait{350};
    static const constexpr bool kDebugFrame = false;
    static const constexpr bool kDebugCommands = false;
    enum class ErrorCode: uint8_t
    {
        Ok,
        Init,
        SendFrame,
        SendFrame_Incomplete,

        SendCommand_InvalidResponse,
        SendCommand_FailedWrite,
        SendCommand_FailedRead,
        SendCommand_WrongFormat,
        SendCommand_Failed,
        SendCommand_InsufficientSpace,

        RecvFrame_Malformed,
        RecvFrame_Incomplete,

        SimpleData_Malformed,
        SimpleData_Failure,

        FillBuffer_NoSpace,
        FillBuffer_ReadFailure,

        MatchError,
        RestartFailed,
        BTFailed,
        FactoryResetFailed,
    };
    static const char* err_to_str(ErrorCode e);

    enum class TargetType: uint8_t
    {
        Clear,
        Move,
        Still,
    };

    enum class Drain
    {
        No,
        Try,
        Only,
    };

    enum class BaudRate: uint16_t
    {
        _9600   = 0x0001,
        _19200  = 0x0002,
        _38400  = 0x0003,
        _57600  = 0x0004,
        _115200 = 0x0005,
        _230400 = 0x0006,
        _256000 = 0x0007,
        _460800 = 0x0008,
    };

    using Ref = std::reference_wrapper<RD03E>;
    struct Err
    {
        ::Err uartErr;
        const char *pLocation;
        ErrorCode code;
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

private:
#pragma pack(push,1)
    struct BaseConfigData
    {
        uint8_t m_MinDistanceGate;
        uint8_t m_MaxDistanceGate;
        uint16_t m_Duration;//seconds
        uint8_t m_OutputPinPolarity;//0 - high on presence; 1 - low on presence
    };
    struct Configuration
    {
        BaseConfigData m_Base;
        uint8_t m_MoveThreshold[14];
        uint8_t m_StillThreshold[14];
    };
#pragma pack(pop)
public:

    /**********************************************************************/
    /* PresenceResult                                                     */
    /**********************************************************************/
#pragma pack(push,1)
    struct PresenceData
    {
        TargetType m_Target;
        uint16_t   m_Distance;//cm
    };
    struct Version
    {
        uint8_t m_Minor;
        uint8_t m_Major;
        uint32_t m_Misc;
    };
#pragma pack(pop)


    RD03E(const struct device *pUART);

    ExpectedResult Init(int txPin, int rxPin);

    int GetMinDistance() const { return m_Configuration.m_Base.m_MinDistanceGate * 75 / 100; }
    uint8_t GetMinDistanceRaw() const { return m_Configuration.m_Base.m_MinDistanceGate; }

    int GetMaxDistance() const { return m_Configuration.m_Base.m_MaxDistanceGate * 75 / 100; }
    uint8_t GetMaxDistanceRaw() const { return m_Configuration.m_Base.m_MaxDistanceGate; }

    auto GetMoveThreshold(uint8_t gate) const { return m_Configuration.m_MoveThreshold[gate]; }
    auto GetStillThreshold(uint8_t gate) const { return m_Configuration.m_StillThreshold[gate]; }
    auto const& GetAllMoveThresholds() const { return m_Configuration.m_MoveThreshold; }
    auto const& GetAllStillThresholds() const { return m_Configuration.m_StillThreshold; }

    auto GetTimeout() const { return m_Configuration.m_Base.m_Duration; }//seconds
    bool GetOutPinPolarity() const { return m_Configuration.m_Base.m_OutputPinPolarity; }

    ExpectedResult UpdateDistanceRes();

    ExpectedResult ReloadConfig();

    auto const& GetVersion() const { return m_Version; }

    auto const& GetBluetoothMAC() const { return m_BluetoothMAC; }
    ExpectedResult SwitchBluetooth(bool on);

    ExpectedResult Restart();
    ExpectedResult FactoryReset();

    ExpectedResult TryReadFrame(int attempts = 3, bool flush = false, Drain drain = Drain::No);

    //using Channel::SetEventCallback;
    using Channel::Flush;
    //using Channel::GetReadyToReadDataLen;

    ExpectedResult RunDynamicBackgroundAnalysis();
    bool IsDynamicBackgroundAnalysisRunning();
private:
    enum class Cmd: uint16_t
    {
        ReadVer = 0x000a,
        WriteBaseParams = 0x0067,
        ReadBaseParams = 0x0073,

        SetSerialBaudRate = 0x00a1,

        FactoryReset = 0x00a2,
        Restart = 0x00a3,

        SwitchBluetooth = 0x00a4,
        GetMAC = 0x00a5,

        OpenCmd = 0x00ff,
        CloseCmd = 0x00fe,
    };

    friend Cmd operator|(Cmd r, uint16_t v)
    {
        return Cmd(uint16_t(r) | v);
    }

    struct OpenCmdModeResponse
    {
        uint16_t protocol_version;
        uint16_t buffer_size;
    };

#pragma pack(push,1)
    template<class ParamT>
    struct SetParam
    {
        static_assert(sizeof(ParamT) == 2, "Must be 2 bytes");
        ParamT param;
        uint32_t value;
    };
#pragma pack(pop)

    using OpenCmdModeRetVal = RetValT<Ref, OpenCmdModeResponse>;
    using ExpectedOpenCmdModeResult = std::expected<OpenCmdModeRetVal, CmdErr>;
    using ExpectedGenericCmdResult = std::expected<Ref, CmdErr>;

    constexpr static uint8_t kFrameHeader[] = {0xFD, 0xFC, 0xFB, 0xFA};
    constexpr static uint8_t kFrameFooter[] = {0x04, 0x03, 0x02, 0x01};
    constexpr static uint8_t kMotionDataFrameHeader[] = {0xaa, 0xaa};
    constexpr static uint8_t kMotionDataFrameFooter[] = {0x55, 0x55};

    template<class E>
    static ExpectedResult to_result(E &&e, const char* pLocation, ErrorCode ec)
    {
        using PureE = std::remove_cvref_t<E>;
        if constexpr(is_expected_type_v<PureE>)
        {
            if constexpr (std::is_same_v<PureE, ExpectedResult>)
                return std::move(e);
            else
                return to_result(e.error(), pLocation, ec);
        }else if constexpr (std::is_same_v<PureE,::Err>)
            return ExpectedResult(std::unexpected(Err{e, pLocation, ec}));
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

    template<class E>
    static ExpectedGenericCmdResult to_cmd_result(E &&e, const char* pLocation, ErrorCode ec)
    {
        using PureE = std::remove_cvref_t<E>;
        if constexpr(is_expected_type_v<PureE>)
        {
            if constexpr (std::is_same_v<PureE, ExpectedGenericCmdResult>)
                return std::move(e);
            else
                return to_cmd_result(e.error(), pLocation, ec);
        }else if constexpr (std::is_same_v<PureE,::Err>)
            return ExpectedGenericCmdResult(std::unexpected(CmdErr{Err{e, pLocation, ec}, 0}));
        else if constexpr (std::is_same_v<PureE,Err>)
            return ExpectedGenericCmdResult(std::unexpected(CmdErr{e, 0}));
        else if constexpr (std::is_same_v<PureE,CmdErr>)
            return ExpectedGenericCmdResult(std::unexpected(e));
        else
        {
            static_assert(std::is_same_v<PureE,Err>, "Don't know how to convert passed type");
            return ExpectedGenericCmdResult(std::unexpected(CmdErr{}));
        }
    }

#define TRY_UART_COMM(f, location, ec) \
    if (auto r = f; !r) \
        return to_result(std::move(r), location, ec)

#define TRY_UART_COMM_CMD(f, location, ec) \
    if (auto r = f; !r) \
        return to_cmd_result(std::move(r), location, ec)

#define TRY_UART_COMM_CMD_WITH_RETRY(f, location, ec) \
    if (auto r = f; !r) \
    {\
        if (retry) \
        {\
            FMT_PRINT("Failed on " #f);\
            continue;\
        }\
        return to_cmd_result(std::move(r), location, ec);\
    }

    template<class...T>
    ExpectedResult SendFrameV2(T&&... args)
    {
        using namespace uart::primitives;
        //1. header
        TRY_UART_COMM(Send(kFrameHeader, sizeof(kFrameHeader)), "SendFrameV2", ErrorCode::SendFrame);

        //2. length
        {
            uint16_t len = (sizeof(args) + ...);
            TRY_UART_COMM(Send((uint8_t const*)&len, sizeof(len)), "SendFrameV2", ErrorCode::SendFrame);
        }
        //3. data
        TRY_UART_COMM(uart::primitives::write_any(*this, std::forward<T>(args)...), "SendFrameV2", ErrorCode::SendFrame);

        //4. footer
        TRY_UART_COMM(Send(kFrameFooter, sizeof(kFrameFooter)), "SendFrameV2", ErrorCode::SendFrame);

        return std::ref(*this);
    }

    template<class...T>
    ExpectedResult RecvFrameV2(T&&... args)
    {
        constexpr const size_t arg_size = (uart::primitives::uart_sizeof<std::remove_cvref_t<T>>() + ...);
        if (m_dbg) m_Dbg = true;
        ScopeExit resetDbg = [&]{ if (m_dbg) m_Dbg = false; };
        TRY_UART_COMM(uart::primitives::match_bytes(*this, kFrameHeader), "RecvFrameV2", ErrorCode::RecvFrame_Malformed);
        if (m_dbg) FMT_PRINT("RecvFrameV2: matched header\n"); 
        uint16_t len;
        TRY_UART_COMM(uart::primitives::read_into(*this, len), "RecvFrameV2", ErrorCode::RecvFrame_Malformed);
        if (m_dbg) FMT_PRINT("RecvFrameV2: len: {}\n", len);
        if (arg_size > len)
            return std::unexpected(Err{{}, "RecvFrameV2 len invalid", ErrorCode::RecvFrame_Malformed}); 

        TRY_UART_COMM(uart::primitives::read_any_limited(*this, len, std::forward<T>(args)...), "RecvFrameV2", ErrorCode::RecvFrame_Malformed);
        if (len)
        {
            TRY_UART_COMM(uart::primitives::skip_bytes(*this, len), "RecvFrameV2", ErrorCode::RecvFrame_Malformed);
        }
        TRY_UART_COMM(uart::primitives::match_bytes(*this, kFrameFooter), "RecvFrameV2", ErrorCode::RecvFrame_Malformed);
        return std::ref(*this);
    }

    template<class... ToSend> static auto to_send(ToSend&&...args) { return std::forward_as_tuple(std::forward<ToSend>(args)...); }
    template<class... ToRecv> static auto to_recv(ToRecv&&...args) { return std::forward_as_tuple(std::forward<ToRecv>(args)...); }

    template<class CmdT, class... ToSend, class... ToRecv>
    ExpectedGenericCmdResult SendCommandV2(CmdT cmd, std::tuple<ToSend...> sendArgs, std::tuple<ToRecv...> recvArgs)
    {
        static_assert(sizeof(CmdT) == 2, "must be 2 bytes");
        if (GetDefaultWait() < kDefaultWait)
            SetDefaultWait(kDefaultWait);
        uint16_t status;
        auto SendFrameExpandArgs = [&]<size_t...idx>(std::index_sequence<idx...>){
            return SendFrameV2(cmd, std::get<idx>(sendArgs)...);
        };
        auto RecvFrameExpandArgs = [&]<size_t...idx>(std::index_sequence<idx...>){ 
            return RecvFrameV2(
                uart::primitives::match_t{uint16_t(cmd | 0x100)}, 
                status, 
                uart::primitives::callback_t{[&]()->Channel::ExpectedResult{
                    if (m_dbg) FMT_PRINT("Recv frame resp. Status {}\n", status);
                    if (status != 0)
                        return std::unexpected(::Err{"SendCommandV2 status", status});
                    return std::ref((Channel&)*this);
                }},
                std::get<idx>(recvArgs)...);
        };

        constexpr int kMaxRetry = 1;
        for(int retry=kMaxRetry; retry >= 0; --retry)
        {
            if (retry != kMaxRetry)
            {
                /*if (m_dbg)*/ FMT_PRINT("Sending command {:x} retry: {}\n", uint16_t(cmd), (kMaxRetry - retry));
                k_sleep(Z_TIMEOUT_MS(kDefaultWait));
            }
            TRY_UART_COMM_CMD_WITH_RETRY(Flush(), "SendCommandV2", ErrorCode::SendCommand_Failed);
            if (m_dbg) FMT_PRINT("Sent cmd {}\n", uint16_t(cmd));
            TRY_UART_COMM_CMD_WITH_RETRY(SendFrameExpandArgs(std::make_index_sequence<sizeof...(ToSend)>()), "SendCommandV2", ErrorCode::SendCommand_Failed);
            if (m_dbg) FMT_PRINT("Wait all\n");
            TRY_UART_COMM_CMD_WITH_RETRY(WaitAllSent(), "SendCommandV2", ErrorCode::SendCommand_Failed);
            if (m_dbg) FMT_PRINT("Receiving {} args\n", sizeof...(ToRecv));
            TRY_UART_COMM_CMD_WITH_RETRY(RecvFrameExpandArgs(std::make_index_sequence<sizeof...(ToRecv)>()), "SendCommandV2", ErrorCode::SendCommand_Failed);
            break;
        }
        return std::ref(*this);
    }

    ExpectedOpenCmdModeResult OpenCommandMode();
    ExpectedGenericCmdResult CloseCommandMode();

    ExpectedGenericCmdResult UpdateVersion();

    ExpectedResult QueryDynamicBackgroundAnalysisRunState();

    ExpectedResult ReadFrame();
    //data
    Version m_Version;
    Configuration m_Configuration;

    //the data will be read into as is
    PresenceData m_PresenceData;

    uint8_t m_BluetoothMAC[6] = {0};
    bool m_DynamicBackgroundAnalysis = false;
public:
    struct DbgNow
    {
        DbgNow(RD03E *pC): m_Dbg(pC->m_dbg), m_PrevDbg(pC->m_dbg) { m_Dbg = true; }
        ~DbgNow() { 
            printf("\n");
            m_Dbg = m_PrevDbg; 
        }

        bool &m_Dbg;
        bool m_PrevDbg;
    };
    bool m_dbg = false;
};


inline bool operator&(RD03E::TargetType s1, RD03E::TargetType s2)
{
    return (uint8_t(s1) & uint8_t(s2)) != 0;
}
}

template<>
struct tools::formatter_t<ai_thinker::RD03E::Err>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, ai_thinker::RD03E::Err const& e)
    {
        return tools::format_to(std::forward<Dest>(dst), "Err\\{uart=[{}] at {} with {} }", e.uartErr, e.pLocation, ai_thinker::RD03E::err_to_str(e.code));
    }
};

template<>
struct tools::formatter_t<ai_thinker::RD03E::CmdErr>
{
    template<FormatDestination Dest>
        static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, ai_thinker::RD03E::CmdErr const& e)
        {
            return tools::format_to(std::forward<Dest>(dst), "CmdErr\\{{Err=[{}]; return={} }", e.e, e.returnCode);
        }
};

template<>
struct tools::formatter_t<ai_thinker::RD03E::TargetType>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, ai_thinker::RD03E::TargetType const& p)
    {
        const char *pStr = "<unk>";
        switch(p)
        {
            using enum ai_thinker::RD03E::TargetType;
            case Clear: pStr = "Clear"; break;
            case Still: pStr = "Still"; break;
            case Move: pStr = "Move"; break;
        }
        return tools::format_to(std::forward<Dest>(dst), "{}", pStr);
    }
};

template<>
struct tools::formatter_t<ai_thinker::RD03E::Version>
{
    template<FormatDestination Dest>
    static std::expected<size_t, FormatError> format_to(Dest &&dst, std::string_view const& fmtStr, ai_thinker::RD03E::Version const& v)
    {
        return tools::format_to(std::forward<Dest>(dst), "v{}.{}.{}" , v.m_Major, v.m_Minor, v.m_Misc);
    }
};
#endif
