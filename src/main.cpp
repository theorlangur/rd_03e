/*
 * Copyright (c) 2023, Meta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define FORCE_FMT
#define PRINTF_FUNC(...) printk(__VA_ARGS__)

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/settings/settings.h>
#include "lib/lib_dfr_c4001.h"

/**********************************************************************/
/* Zigbee                                                             */
/**********************************************************************/
#include <nrfzbcpp/zb_main.hpp>
#include <nrfzbcpp/zb_std_cluster_desc.hpp>
#include <nrfzbcpp/zb_status_cluster_desc.hpp>

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
};

//attribute shortcuts for template arguments
constexpr auto kAttrStatus1 = &zb::zb_zcl_status_t::status1;
constexpr auto kAttrStatus2 = &zb::zb_zcl_status_t::status2;
constexpr auto kAttrStatus3 = &zb::zb_zcl_status_t::status3;

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
	    )
	);

//a shortcut for a convenient access
constinit static auto &zb_ep = zb_ctx.ep<kMMW_EP>();

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
constinit const struct device *rd_uart = DEVICE_DT_GET(DT_NODELABEL(uart30));

///* Get the node from the alias */
#define SENSOR_NODE DT_ALIAS(presence)

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
}

void on_dev_cb_error(int err)
{
    printk("on_dev_cb_error: %d\r\n", err);
}

void test_func();

int main(void)
{
    int err = settings_subsys_init();
    err = settings_load();

    /* Register callback for handling ZCL commands. */
    auto dev_cb = zb::tpl_device_cb<
		zb::dev_cb_handlers_desc{ .error_handler = on_dev_cb_error }
    >;

    ZB_ZCL_REGISTER_DEVICE_CB(dev_cb);

    /* Register dimmer switch device context (endpoints). */
    ZB_AF_REGISTER_DEVICE_CTX(zb_ctx);

    zigbee_enable();
    printk("Main: sleep forever\r\n");
	test_func();
    while (1) {
		k_sleep(K_FOREVER);
    }

	return 0;
}

void test_func()
{
	FMT_PRINTLN("main. start");
	k_msleep(1000);

	dfr::C4001 c4001(rd_uart);
	auto r = c4001.Init();
	if (!r)
	{
		FMT_PRINTLN("init err={}", r.error());
	}else
	{
		printk("getting version\r\n");
		FMT_PRINTLN("hwver={}", (const char*)c4001.GetHWVer().m_Version);
		FMT_PRINTLN("swver={}", (const char*)c4001.GetSWVer().m_Version);
		FMT_PRINTLN("inhibit={:.2}", c4001.GetInhibitDuration());
		FMT_PRINTLN("range={:.1}-{:.1}", c4001.GetRangeFrom(), c4001.GetRangeTo());
		FMT_PRINTLN("trig range={:.1}", c4001.GetTriggerDistance());
		FMT_PRINTLN("sens hold={:.1}; sens trig={:.1}", c4001.GetSensitivityHold(), c4001.GetSensitivityTrig());
		FMT_PRINTLN("latency detect={:.1}; clear={:.1}", c4001.GetDetectLatency(), c4001.GetClearLatency());

		auto cfg = c4001.GetConfigurator();
		auto r = 
			cfg.SetRange(0.6, 4)
			.and_then([&](dfr::C4001::Configurator &c){ return c.SetTrigRange(7.5); })
			.and_then([&](dfr::C4001::Configurator &c){ return c.UpdateRange(); })
			.and_then([&](dfr::C4001::Configurator &c){ return c.UpdateTrigRange(); });
		if (!r)
		{
			FMT_PRINTLN("failed to change ranges: err={}", r.error());
		}else
		{
			FMT_PRINTLN("new range={:.1}-{:.1}", c4001.GetRangeFrom(), c4001.GetRangeTo());
			FMT_PRINTLN("new trig range={:.1}", c4001.GetTriggerDistance());
		}
	}

	gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	int err = gpio_pin_configure_dt(&presence, GPIO_INPUT);
	if (err != 0)
		printk("gpio_pin_configure_dt: %d\r\n", err);
	gpio_pin_set_dt(&led, 0);

	err = gpio_pin_interrupt_configure_dt(&presence, GPIO_INT_EDGE_BOTH);
	if (err != 0)
		printk("gpio_pin_interrupt_configure_dt: %d\r\n", err);
	gpio_init_callback(&g_cb, presence_triggered, BIT(presence.pin));
	gpio_add_callback_dt(&presence, &g_cb);

	//printk("sleeping...\r\n");
	//while(true)
	//	k_msleep(100000);
}
