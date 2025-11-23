/*
 * Copyright (c) 2023, Meta
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <optional>
#define FORCE_FMT
#define PRINTF_FUNC(...) printk(__VA_ARGS__)

//#include <iostream>
#include <memory>

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>
#include "lib/lib_uart.h"
#include "lib/lib_dfr_c4001.h"

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   5000

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

int main(void)
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
		//auto ver = rd03e.GetVersion();
		FMT_PRINTLN("hwver={}", (const char*)c4001.GetHWVer().m_Version);
		FMT_PRINTLN("swver={}", (const char*)c4001.GetSWVer().m_Version);
		FMT_PRINTLN("inhibit={:.2}", c4001.GetInhibitDuration());
		FMT_PRINTLN("range={:.1}-{:.1}", c4001.GetRangeFrom(), c4001.GetRangeTo());
		FMT_PRINTLN("trig range={:.1}", c4001.GetTriggerDistance());
		FMT_PRINTLN("sens hold={:.1}; sens trig={:.1}", c4001.GetSensitivityHold(), c4001.GetSensitivityTrig());
		FMT_PRINTLN("latency detect={:.1}; clear={:.1}", c4001.GetDetectLatency(), c4001.GetClearLatency());

		auto cfg = c4001.GetConfigurator();
		auto r = cfg.SetRange(0.6, 3)
			.and_then([&](dfr::C4001::Configurator &c){ return c.SetTrigRange(3); })
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

	printk("sleeping...\r\n");
	while(true)
		k_msleep(100000);
	return 0;

	int ret;
	bool led_state = true;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}
	//ret = gpio_pin_configure();

	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return 0;
	}

	while (1) {
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) return 0;
		k_msleep(1000);
		ret = gpio_pin_toggle_dt(&led);
		if (ret < 0) return 0;

		led_state = !led_state;
		printf("LED state: %s\n", led_state ? "ON" : "OFF");
		k_msleep(SLEEP_TIME_MS);
	}

	return 0;
}

