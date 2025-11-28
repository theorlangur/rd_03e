#define FORCE_FMT
#define PRINTF_FUNC(...) printk(__VA_ARGS__)

#include <algorithm>
#include <cstring>
#include <cstdlib>
#include "lib_dfr_c4001.h"

#define DBG_UART Channel::DbgNow _dbg_uart{this}; 
#define DBG_ME DbgNow _dbg_me{this}; 

#define TRY_CFG(f, location) \
            if (auto r = f; !r) \
                return result<ExpectedResult>::to(r.error(), location)

#define TRY_UART_CFG(f, location) \
            if (auto r = f; !r) \
                return result<Configurator::ExpectedResult>::to(std::move(r), location)

namespace dfr
{
    template<class T>
    struct read_cfg_t
    {
        T min = 0;
        T max = 0;
        size_t N = 16;
    };
    using read_float_cfg_t = read_cfg_t<float>;
    using read_uint8_cfg_t = read_cfg_t<uint8_t>;

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
            char *pEnd = dstStr;
            dstVar = strtof(dstStr, &pEnd);
            if (pEnd == dstStr)
            {
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

    template<read_uint8_cfg_t cfg = {}>
    struct read_uint8_from_str_t
    {
        using functional_read_helper = void;
        uint8_t &dstVar;
        char until = ' ';
        char dstStr[cfg.N];
        bool consume_last = true;

        static constexpr size_t size() { return cfg.N; }
        size_t rt_size() const { return cfg.N; }
        auto run(uart::Channel &c) { 
            using ExpectedResult = std::expected<uart::Channel::Ref, ::Err>;
            auto r = uart::primitives::read_until_into(c, until, (uint8_t*)dstStr, cfg.N, consume_last, {}); 
            if (!r) return r;
            char *pEnd = dstStr;
            dstVar = strtoul(dstStr, &pEnd, 10);
            if (pEnd == dstStr)
            {
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
        TRY_CFG(cfg.UpdateHWVersion(), "ReloadConfig.HW");
        TRY_CFG(cfg.UpdateSWVersion(), "ReloadConfig.SW");
        TRY_CFG(cfg.UpdateInhibit(), "ReloadConfig.Inhibit");
        TRY_CFG(cfg.UpdateRange(), "ReloadConfig.Range");
        TRY_CFG(cfg.UpdateTrigRange(), "ReloadConfig.TrigRange");
        TRY_CFG(cfg.UpdateSensitivity(), "ReloadConfig.Sensitivity");
        TRY_CFG(cfg.UpdateLatency(), "ReloadConfig.Latency");
        TRY_CFG(cfg.End(), "ReloadConfig.End");
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

    C4001::Configurator::~Configurator()
    {
        End().value_or(*this);
    }

    auto C4001::Configurator::UpdateLatency() -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        using namespace uart::primitives;
        read_float_from_str_t<{.min = 0, .max = 100}> readDetect{m_C.m_DetectLatency, ' '};
        read_float_from_str_t<{.min = 0, .max = 1500}> readClear{m_C.m_ClearLatency, '\r'};
        TRY_UART_CFG(m_C.SendCmdWithParamsStd(
                        to_send(kCmdGetLatency), 
                        to_recv(readDetect, readClear)), "");
        return std::ref(*this);
    }

    auto C4001::Configurator::SetLatency(float detect, float clear) -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        char buf[16]; 
        auto arg = tools::format_to_span(buf, "{:.1} {:.1}", detect, clear);
        if (arg.empty())
            return std::unexpected(Err{"Configurator::SetLatency fmt", 0});
        TRY_UART_CFG(m_C.SendCmd(kCmdSetLatency, arg), "Configurator::SetLatency");

        return std::ref(*this);
    }

    auto C4001::Configurator::UpdateSensitivity() -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        using namespace uart::primitives;
        read_uint8_from_str_t<{.min = 0, .max = 9}> readHold{m_C.m_SensitivityHold, ' '};
        read_uint8_from_str_t<{.min = 0, .max = 9}> readTrig{m_C.m_SensitivityTrigger, '\r'};
        TRY_UART_CFG(m_C.SendCmdWithParamsStd(
                        to_send(kCmdGetSensitivity), 
                        to_recv(readHold, readTrig)), "");
        return std::ref(*this);
    }

    auto C4001::Configurator::SetSensitivity(uint8_t trig, uint8_t hold) -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        char buf[16]; 
        auto arg = tools::format_to_span(buf, "{} {}", hold, trig);
        if (arg.empty()) return std::unexpected(Err{"Configurator::SetSensitivity fmt", 0});
        TRY_UART_CFG(m_C.SendCmd(kCmdSetSensitivity, arg), "Configurator.SetSensitivity");

        return std::ref(*this);
    }

    auto C4001::Configurator::SetSensitivityTrig(uint8_t val) -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        char buf[16]; 
        auto arg = tools::format_to_span(buf, "255 {}", val);
        if (arg.empty()) return std::unexpected(Err{"Configurator::SetSensitivityTrig fmt", 0});
        TRY_UART_CFG(m_C.SendCmd(kCmdSetSensitivity, arg), "Configurator.SetSensitivityTrig");

        return std::ref(*this);
    }

    auto C4001::Configurator::SetSensitivityHold(uint8_t val) -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        char buf[16]; 
        auto arg = tools::format_to_span(buf, "{} 255", val);
        if (arg.empty()) return std::unexpected(Err{"Configurator::SetSensitivityHold fmt", 0});
        TRY_UART_CFG(m_C.SendCmd(kCmdSetSensitivity, arg), "Configurator::SetSensitivityHold");

        return std::ref(*this);
    }

    auto C4001::Configurator::UpdateTrigRange() -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        using namespace uart::primitives;
        read_float_from_str_t<{.min = 0.6, .max = 25}> readTrig{m_C.m_TrigRange, '\r'};
        TRY_UART_CFG(m_C.SendCmdWithParamsStd(
                        to_send(kCmdGetTrigRange), 
                        to_recv(readTrig)), "");
        return std::ref(*this);
    }

    auto C4001::Configurator::SetTrigRange(float v) -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        char buf[8]; 
        auto arg = tools::format_to_span(buf, "{:.1}", v);
        if (arg.empty()) return std::unexpected(Err{"Configurator::SetTrigRange fmt", 0});
        TRY_UART_CFG(m_C.SendCmd(kCmdSetTrigRange, arg), "Configurator.SetTrigRange");
        return std::ref(*this);
    }

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
        auto arg = tools::format_to_span(buf, "{:.2} {:.2}", from, to);
        if (arg.empty()) return std::unexpected(Err{"Configurator::SetRange fmt", 0});
        TRY_UART_CFG(m_C.SendCmd(kCmdSetRange, arg), "Configurator.SetRange");

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
        char buf[8]; 
        auto arg = tools::format_to_span(buf, "{:.1}", v);
        if (arg.empty()) return std::unexpected(Err{"Configurator::SetInhibit fmt", 0});
        TRY_UART_CFG(m_C.SendCmd(kCmdSetInhibit, arg), "Configurator.SetInhibit");
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

    void C4001::Configurator::StartDbg()
    {
        m_C.m_Dbg = true;
    }

    void C4001::Configurator::StopDbg()
    {
        m_C.m_Dbg = false;
    }

    auto C4001::Configurator::ReloadConfig() noexcept -> ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        return UpdateInhibit()
            .and_then([](Configurator &cfg){ return cfg.UpdateSensitivity(); })
            .and_then([](Configurator &cfg){ return cfg.UpdateLatency(); })
            .and_then([](Configurator &cfg){ return cfg.UpdateRange(); })
            .and_then([](Configurator &cfg){ return cfg.UpdateTrigRange(); })
            .and_then([](Configurator &cfg){ return cfg.UpdateHWVersion(); })
            .and_then([](Configurator &cfg){ return cfg.UpdateSWVersion(); });
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
        if (m_Finished)
            return std::unexpected(Err{"Configurator::End unexpected finish"});
        m_Finished = true;
        if (!m_CtrResult) return m_CtrResult;
        TRY_UART_CFG(StartSensor(), "Configurator::End");
        return std::ref(*this);
    }

    auto C4001::Configurator::StopSensor()->ExpectedResult
    {
        if (!m_CtrResult) return m_CtrResult;
        TRY_UART_CFG(m_C.SendCmd(kCmdSensorStop), "");
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
