#include <charconv>
#define FORCE_FMT
#define PRINTF_FUNC(...) printk(__VA_ARGS__)

#include <algorithm>
#include <cstring>
#include "lib_dfr_c4001.h"
#include "lib_misc_helpers.hpp"

#define DBG_UART Channel::DbgNow _dbg_uart{this}; 
#define DBG_ME DbgNow _dbg_me{this}; 

namespace dfr
{
    C4001::C4001(const struct device *pUART):
        uart::Channel(pUART)
    {
    }

    C4001::ExpectedResult C4001::Init()
    {
        SetDefaultWait(kDefaultWait);
        TRY_UART_COMM(Configure(), "Init");
        TRY_UART_COMM(Open(), "Init");
        return ReloadConfig();
    }

    auto C4001::GetConfigurator() -> Configurator
    {
        return Configurator{*this};
    }

    C4001::ExpectedResult C4001::ReloadConfig()
    {
        auto cfg = GetConfigurator();
        cfg.UpdateHWVersion();
        cfg.UpdateSWVersion();
        cfg.UpdateInhibit();
        cfg.UpdateRange();
        auto r = cfg.End();
        if (!r)
            return result<ExpectedResult>::to(std::move(r.error()), "ReloadConfig");
        return std::ref(*this);
    }

    C4001::ExpectedResult C4001::Restart()
    {
        using namespace uart::primitives;
        return std::ref(*this);
    }

    C4001::ExpectedResult C4001::FactoryReset()
    {
        using namespace uart::primitives;
        SetDefaultWait(duration_ms_t(1000));
        return ReloadConfig();
    }

    C4001::Configurator::Configurator(C4001 &c):
        m_C(c),
        m_RxBlock(c),
        m_CtrResult(std::ref(*this))
    {
        if (auto r = StopSensor(); !r)
            m_CtrResult = result<ExpectedResult>::to(std::move(r),  "Configurator::Configurator");
    }

    struct read_float_cfg_t
    {
        float min = 0;
        float max = 0;
        size_t N = 16;
    };
    template<read_float_cfg_t cfg = {}>
    struct read_float_from_str_t
    {
        using functional_read_helper = void;
        float &dstVar;
        char until = ' ';
        char dstStr[cfg.N];
        bool consume_last = true;

        static constexpr size_t size() { return cfg.N; }
        size_t rt_size() const { return cfg.N; }
        auto run(uart::Channel &c) { 
            using ExpectedResult = std::expected<uart::Channel::Ref, ::Err>;
            auto r = uart::primitives::read_until_into(c, until, (uint8_t*)dstStr, cfg.N, consume_last, {}); 
            if (!r) return r;
            char *pEnd = dstStr + cfg.N;
            dstVar = strtof(dstStr, &pEnd);
            if (dstStr == pEnd)
            {
                //nothing was read to convert
                return ExpectedResult(std::unexpected(::Err{"failed to convert"}));
            }
            if (cfg.min != cfg.max)
            {
                if (dstVar < cfg.min || dstVar > cfg.max)
                    return ExpectedResult(std::unexpected(::Err{"failed validation"}));
            }
            return r;
        } 
    };

    auto C4001::Configurator::UpdateRange() noexcept -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        using namespace uart::primitives;
        read_float_from_str_t<{.min = 0.6, .max = 25}> readFrom{m_C.m_MinRange, ' '};
        read_float_from_str_t<{.min = 0.6, .max = 25}> readTo{m_C.m_MaxRange, '\r'};
        TRY_UART_CFG(m_C.SendCmdWithParamsStd(
                        to_send(kCmdGetRange), 
                        to_recv(readFrom, readTo)), "");
        return std::ref(*this);
    }

    auto C4001::Configurator::SetRange(float from, float to) noexcept -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        char buf[16]; 
        tools::format_to(tools::BufferFormatter(buf), "{:.1} {:.1}", from, to);
        TRY_UART_CFG(m_C.SendCmd(kCmdSetRange), (const char*)buf);

        return std::ref(*this);
    }

    auto C4001::Configurator::UpdateInhibit() noexcept -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        using namespace uart::primitives;
        read_float_from_str_t<{.min = 0, .max = 255}> readInhibit{m_C.m_Inhibit, '\r'};
        TRY_UART_CFG(m_C.SendCmdWithParamsStd(
                        to_send(kCmdGetInhibit), 
                        to_recv(readInhibit)), "");

        return std::ref(*this);
    }

    auto C4001::Configurator::SetInhibit(float v) noexcept -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        char buf[16]; 
        tools::format_to(tools::BufferFormatter(buf), "{:.1}", v);
        TRY_UART_CFG(m_C.SendCmd(kCmdSetInhibit), (const char*)buf);
        return std::ref(*this);
    }

    auto C4001::Configurator::SwitchToPresenceMode() noexcept -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        TRY_UART_CFG(m_C.SendCmdNoResp(kCmdSetRunApp, kCmdAppModePresence), "");
        k_msleep(500);
        return std::ref(*this);
    }

    auto C4001::Configurator::SwitchToSpeedDistanceMode() noexcept -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        TRY_UART_CFG(m_C.SendCmdNoResp(kCmdSetRunApp, kCmdAppModeSpeedDistance), "");
        k_msleep(500);
        return std::ref(*this);
    }

    auto C4001::Configurator::Restart() noexcept -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        TRY_UART_CFG(m_C.SendCmdNoResp(kCmdRestart, kCmdRestartParamNormal), "");
        k_msleep(500);
        return std::ref(*this);
    }

    auto C4001::Configurator::SaveConfig() noexcept -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        TRY_UART_CFG(m_C.SendCmd(kCmdSaveConfig), "");
        return std::ref(*this);
    }

    auto C4001::Configurator::ResetConfig() noexcept -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        TRY_UART_CFG(m_C.SendCmd(kCmdResetConfig), "");
        return std::ref(*this);
    }

    auto C4001::Configurator::End() -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        TRY_UART_CFG(StartSensor(), "Configurator::End");
        return std::ref(*this);
    }

    auto C4001::Configurator::StopSensor()->ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        TRY_UART_CFG(m_C.SendCmdNoResp(kCmdSensorStop), "");
        return std::ref(*this);
    }

    auto C4001::Configurator::StartSensor()->ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        //ChangeWait longerWait(*this, 300);
        TRY_UART_CFG(m_C.SendCmd(kCmdSensorStart), "");
        return std::ref(*this);
    }

    auto C4001::Configurator::UpdateHWVersion()->ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        using namespace uart::primitives;
        std::fill(std::begin(m_C.m_HWVersion.m_Version), std::end(m_C.m_HWVersion.m_Version), 0);
        TRY_UART_CFG(m_C.SendCmdWithParams(to_send(kCmdGetHWVersion), to_recv(
                        find_str_t{"HardwareVersion:"},
                        read_until_t{m_C.m_HWVersion.m_Version, '\r'}
                        )), "");
        *(std::end(m_C.m_HWVersion.m_Version) - 1) = 0;
        return std::ref(*this);
    }

    auto C4001::Configurator::UpdateSWVersion()->ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        using namespace uart::primitives;
        std::fill(std::begin(m_C.m_SWVersion.m_Version), std::end(m_C.m_SWVersion.m_Version), 0);
        TRY_UART_CFG(m_C.SendCmdWithParams(to_send(kCmdGetSWVersion), to_recv(
                        find_str_t{"SoftwareVersion:"},
                        read_until_t{m_C.m_SWVersion.m_Version, '\r'}
                        )), "");
        *(std::end(m_C.m_SWVersion.m_Version) - 1) = 0;
        return std::ref(*this);
    }
}
