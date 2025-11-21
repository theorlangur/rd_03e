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

    auto C4001::Configurator::UpdateInhibit() -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        using namespace uart::primitives;
        char valStr[16];
        TRY_UART_CFG(m_C.SendCmdWithParams(to_send(kCmdGetInhibit), to_recv(
                        read_until_t{valStr, '\r'}
                        )), "");
        std::from_chars(std::begin(valStr), std::end(valStr), m_C.m_Inhibit);
        return std::ref(*this);
    }

    auto C4001::Configurator::SetInhibit(float v) -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        char buf[16]; 
        tools::format_to(tools::BufferFormatter(buf), "{:.1}", v);
        TRY_UART_CFG(m_C.SendCmd(kCmdSetInhibit), (const char*)buf);
        return std::ref(*this);
    }

    auto C4001::Configurator::SwitchToPresenceMode() -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        TRY_UART_CFG(m_C.SendCmdNoResp(kCmdSetRunApp, kCmdAppModePresence), "");
        k_msleep(500);
        return std::ref(*this);
    }

    auto C4001::Configurator::SwitchToSpeedDistanceMode() -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        TRY_UART_CFG(m_C.SendCmdNoResp(kCmdSetRunApp, kCmdAppModeSpeedDistance), "");
        k_msleep(500);
        return std::ref(*this);
    }

    auto C4001::Configurator::Restart() -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        TRY_UART_CFG(m_C.SendCmdNoResp(kCmdRestart, kCmdRestartParamNormal), "");
        k_msleep(500);
        return std::ref(*this);
    }

    auto C4001::Configurator::SaveConfig() -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        TRY_UART_CFG(m_C.SendCmd(kCmdSaveConfig), "");
        return std::ref(*this);
    }

    auto C4001::Configurator::ResetConfig() -> ExpectedResult
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
