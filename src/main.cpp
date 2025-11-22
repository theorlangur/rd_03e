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

//constinit const struct device *rd03e = DEVICE_DT_GET(DT_NODELABEL(rd03e));
//constinit const struct device *rd_uart = nullptr;
constinit const struct device *rd_uart = DEVICE_DT_GET(DT_NODELABEL(uart30));
constinit const struct device *uart22 = nullptr;//DEVICE_DT_GET(DT_NODELABEL(uart22));

uint8_t uart_buf[2][4];
int uart_buf_idx = 0;
constexpr static size_t rx_size = 1024;
uint8_t rx_buf[rx_size];
int rx_write = 0;
int rx_read = 0;
void uart_async_callback(const struct device *dev, uart_event *evt, void *user_data)
{
	switch(evt->type)
	{
	    case UART_TX_ABORTED:
		break;
	    case UART_TX_DONE:
	    break;
	    case UART_RX_BUF_REQUEST:
	    {
		if (uart_buf_idx != -1)
		{
		    uart_buf_idx ^= 1;
		    uart_rx_buf_rsp(dev
			    , uart_buf[uart_buf_idx]
			    , 4);
		}
	    }
	    break;
	    case UART_RX_BUF_RELEASED:
		//printk("rx buf rel\r\n");
		break;
	    case UART_RX_DISABLED:
		printk("rx buf dis\r\n");
		break;
	    case UART_RX_RDY:
		{
			//FMT_PRINTLN("rx: {}", std::span<const uint8_t>{evt->data.rx.buf + evt->data.rx.offset, (size_t)evt->data.rx.len});
			for(int i = 0; i < evt->data.rx.len; ++i)
			{
				rx_buf[rx_write] = (evt->data.rx.buf + evt->data.rx.offset)[i];
				rx_write = (rx_write + 1) % rx_size;
			}
		}
		break;
	    case UART_RX_STOPPED:
		printk("rx stopped\r\n");
		break;
	}
}

int main(void)
{
	FMT_PRINTLN("main. start");
	k_msleep(1000);

	//struct uart_config uart_cfg;
	//uart_config_get(rd_uart, &uart_cfg);
	//uart_cfg.baudrate = 9600;
	//int uart_err = uart_configure(rd_uart, &uart_cfg);
	//FMT_PRINTLN("uart config res={}; baudrate={:x}", uart_err, uart_cfg.baudrate);
	//k_msleep(1000);

	//uart_callback_set(rd_uart, uart_async_callback, nullptr);
	//printk("starting rx\r\n");
	//uart_rx_enable(rd_uart, uart_buf[0], 4, SYS_FOREVER_US);
	//while(true)
	//{
	//	k_msleep(1000);
	//	bool empty = true;
	//	while(rx_read != rx_write)
	//	{
	//		printk("%X ", rx_buf[rx_read++]);
	//		empty = false;
	//	}
	//	if (!empty)
	//		printk("\r\n");
	//}
	//
	//printk("attempt to talk uart\r\n");
	//k_msleep(1000);

	dfr::C4001 c4001(rd_uart);
	printk("before init\r\n");
	k_msleep(01*1000);
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
	}

	printk("sleeping...\r\n");
	k_msleep(100000);
	return 0;

	int ret;
	bool led_state = true;

	if (!gpio_is_ready_dt(&led)) {
		return 0;
	}

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

	//std::unique_ptr<int> ptr;
	//std::shared_ptr<char> ptr2;
	//std::cout << "Hello, C++ world! " << CONFIG_BOARD << std::endl;
	//return (int)ptr.get();
	return 0;
}
