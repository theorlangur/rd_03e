/*
 * Copyright (c) 2023, Meta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define FORCE_FMT
#define PRINTF_FUNC(...) printk(__VA_ARGS__)

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/settings/settings.h>
#include "c4001_task.hpp"

/**********************************************************************/
/* Zigbee                                                             */
/**********************************************************************/
#include <nrfzbcpp/zb_main.hpp>
#include <nrfzbcpp/zb_std_cluster_desc.hpp>
#include <nrfzbcpp/zb_status_cluster_desc.hpp>
#include <nrfzbcpp/zb_occupancy_sensing_cluster_desc.hpp>
#include "zb/zb_c4001_cluster_desc.hpp"

/**********************************************************************/
/* Zigbee Declarations and Definitions                                */
/**********************************************************************/
static bool g_ZigbeeReady = false;

/* Manufacturer name (32 bytes). */
#define INIT_BASIC_MANUF_NAME      "SFINAE"

/* Model number assigned by manufacturer (32-bytes long string). */
#define INIT_BASIC_MODEL_ID        "C4001-NG"


/* Button used to enter the Bulb into the Identify mode. */
#define IDENTIFY_MODE_BUTTON            DK_BTN1_MSK

/* Button to start Factory Reset */
#define FACTORY_RESET_BUTTON IDENTIFY_MODE_BUTTON

/* Device endpoint, used to receive light controlling commands. */
constexpr uint8_t kMMW_EP = 1;
constexpr uint16_t kDEV_ID = 0xBAAD;

struct device_ctx_t{
    zb::zb_zcl_basic_names_t basic_attr;
    zb::zb_zcl_status_t status_attr;
    zb::zb_zcl_occupancy_ultrasonic_t occupancy;
    zb::zb_zcl_on_off_attrs_client_t on_off_client;
    zb::zb_zcl_c4001_t c4001;
};

//attribute shortcuts for template arguments

/**********************************************************************/
/* Status attribute shortcuts                                         */
/**********************************************************************/
constexpr auto kAttrStatus1 = &zb::zb_zcl_status_t::status1;
constexpr auto kAttrStatus2 = &zb::zb_zcl_status_t::status2;
constexpr auto kAttrStatus3 = &zb::zb_zcl_status_t::status3;

/**********************************************************************/
/* Occupancy attribute shortcuts                                      */
/**********************************************************************/
constexpr auto kAttrOccupancy = &zb::zb_zcl_occupancy_ultrasonic_t::occupancy;
constexpr auto kAttrDetectToClearDelay = &zb::zb_zcl_occupancy_ultrasonic_t::UltrasonicOccupiedToUnoccupiedDelay;
constexpr auto kAttrClearToDetectDelay = &zb::zb_zcl_occupancy_ultrasonic_t::UltrasonicUnoccupiedToOccupiedDelay;

constexpr auto kCmdOn = &zb::zb_zcl_on_off_attrs_client_t::on;
constexpr auto kCmdOff = &zb::zb_zcl_on_off_attrs_client_t::off;

//forward declare
/**********************************************************************/
/* Support for On/Off cluster client commands                         */
/**********************************************************************/
template<> struct zb::cluster_custom_handler_t<zb::zb_zcl_on_off_attrs_client_t, kMMW_EP>;
using custom_accel_handler_t = zb::cluster_custom_handler_t<zb::zb_zcl_on_off_attrs_client_t, kMMW_EP>;

/* Zigbee device application context storage. */
static constinit device_ctx_t dev_ctx{
    .basic_attr = {
	{
	    .zcl_version = ZB_ZCL_VERSION,
	    .power_source = zb::zb_zcl_basic_min_t::PowerSource::DC
	},
	/*.manufacturer =*/ INIT_BASIC_MANUF_NAME,
	/*.model =*/ INIT_BASIC_MODEL_ID,
    },
};

constinit static auto zb_ctx = zb::make_device(
	zb::make_ep_args<{.ep=kMMW_EP, .dev_id=kDEV_ID, .dev_ver=1}>(
	    dev_ctx.basic_attr
	    , dev_ctx.status_attr
	    , dev_ctx.occupancy
	    , dev_ctx.on_off_client
	    , dev_ctx.c4001
	    )
	);

//a shortcut for a convenient access
constinit static auto &zb_ep = zb_ctx.ep<kMMW_EP>();

/**********************************************************************/
/* Support for On/Off cluster client commands                         */
/**********************************************************************/
//magic handwaving to avoid otherwise necessary command handling boilerplate
//uses CRTP so that cluster_custom_handler_base_t would know the end type it needs to work with
template<> 
struct zb::cluster_custom_handler_t<zb::zb_zcl_on_off_attrs_client_t, kMMW_EP>: cluster_custom_handler_base_t<custom_accel_handler_t>
{
    //the rest will be done by cluster_custom_handler_base_t
    static auto& get_device() { return zb_ctx; }
};

/**********************************************************************/
/* Device defines                                                     */
/**********************************************************************/
/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
//constinit const struct device *rd_uart = DEVICE_DT_GET(DT_NODELABEL(uart30));

///* Get the node from the alias */
#define SENSOR_NODE DT_ALIAS(presence)

constinit static dfr::C4001 *pC4001 = nullptr;

/* Get the GPIO spec directly from the node */
/* Note: We look for the property "gpios" inside the node */
static const struct gpio_dt_spec presence = GPIO_DT_SPEC_GET(SENSOR_NODE, gpios);

gpio_callback g_cb;

void presence_triggered(const struct device *port,
					struct gpio_callback *cb,
					gpio_port_pins_t pins)
{
    int val = gpio_pin_get_dt(&presence);
    gpio_pin_set_dt(&led, val);
    //post to zigbee and shoot commands
}

void on_dev_cb_error(int err)
{
    printk("on_dev_cb_error: %d\r\n", err);
}

void on_c4001_error(c4001::err_t e)
{
    //post to zigbee thread
}

int configure_c4001_out_pin();

void on_zigbee_start()
{
    printk("on_zigbee_start\r\n");
    g_ZigbeeReady = true;
    //TODO: stuff...
}

/**@brief Zigbee stack event handler.
 *
 * @param[in]   bufid   Reference to the Zigbee stack buffer
 *                      used to pass signal.
 */
void zboss_signal_handler(zb_bufid_t bufid)
{
        zb_zdo_app_signal_hdr_t *pHdr;
        auto signalId = zb_get_app_signal(bufid, &pHdr);

	auto ret = zb::tpl_signal_handler<zb::sig_handlers_t{
	.on_leave = +[]{ 
	    k_sleep(K_MSEC(2100));
	    sys_reboot(SYS_REBOOT_COLD);
	},
	    //.on_error = []{ led::show_pattern(led::kPATTERN_3_BLIPS_NORMED, 1000); },
	    .on_dev_reboot = on_zigbee_start,
	    .on_steering = on_zigbee_start,
	   }>(bufid);
    const uint32_t LOCAL_ERR_CODE = (uint32_t) (-ret);	
    if (LOCAL_ERR_CODE != RET_OK) {				
	zb_osif_abort();				
    }							
}

int main(void)
{
    int err = settings_subsys_init();
    err = settings_load();

    pC4001 = c4001::setup(&on_c4001_error);
    if (pC4001)
    {
	dev_ctx.c4001.range_min = pC4001->GetRangeFrom();
	dev_ctx.c4001.range_max = pC4001->GetRangeTo();
	dev_ctx.c4001.range_trig = pC4001->GetTriggerDistance();
	dev_ctx.c4001.inhibit_duration = pC4001->GetInhibitDuration();
	dev_ctx.c4001.sensitivity_detect = pC4001->GetSensitivityTrig();
	dev_ctx.c4001.sensitivity_hold = pC4001->GetSensitivityHold();
	dev_ctx.c4001.sw_ver = pC4001->GetSWVer().m_Version;
	dev_ctx.c4001.hw_ver = pC4001->GetHWVer().m_Version;
    }

    /* Register callback for handling ZCL commands. */
    auto dev_cb = zb::tpl_device_cb<
		zb::dev_cb_handlers_desc{ .error_handler = on_dev_cb_error }
	//to_settings_handler<on_wake_sleep_settings_changed>(ZbSettingsEntries::wake_sleep_threshold)
    >;

    ZB_ZCL_REGISTER_DEVICE_CB(dev_cb);

    /* Register device context (endpoints). */
    ZB_AF_REGISTER_DEVICE_CTX(zb_ctx);

    zigbee_enable();
    if (int err = configure_c4001_out_pin(); err != 0)
    {
	printk("Failed to configure c4001 out pin\r\n");
    }

    printk("Main: sleep forever\r\n");
    //test_func();
    while (1) {
	k_sleep(K_FOREVER);
    }

    return 0;
}

int configure_c4001_out_pin()
{
    gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
    int err = gpio_pin_configure_dt(&presence, GPIO_INPUT);
    if (err != 0)
    {
	printk("gpio_pin_configure_dt: %d\r\n", err);
	return err;
    }
    gpio_pin_set_dt(&led, 0);

    err = gpio_pin_interrupt_configure_dt(&presence, GPIO_INT_EDGE_BOTH);
    if (err != 0)
    {
	printk("gpio_pin_interrupt_configure_dt: %d\r\n", err);
	return err;
    }
    gpio_init_callback(&g_cb, presence_triggered, BIT(presence.pin));
    return gpio_add_callback_dt(&presence, &g_cb);
}

