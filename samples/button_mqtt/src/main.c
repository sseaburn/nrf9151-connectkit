/*
 * Copyright (c) 2016-2025, Makerdiary
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Publishes an MQTT message to CONFIG_BUTTON_MQTT_PUBLISH_TOPIC each time
 * the sw0 button is pressed.  Connect MQTTX (or any client) to the same
 * broker and subscribe to that topic to see the events.
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/conn_mgr_connectivity.h>
#include <zephyr/net/conn_mgr_monitor.h>
#include <net/mqtt_helper.h>

LOG_MODULE_REGISTER(button_mqtt, LOG_LEVEL_INF);

#define SW0_NODE DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS_OKAY(SW0_NODE)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

static const struct gpio_dt_spec button = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static struct gpio_callback button_cb_data;

/* Same pattern as the button sample — no-op if led0 is not defined. */
static struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led0), gpios, {0});

static K_SEM_DEFINE(button_sem, 0, 10);
static K_SEM_DEFINE(network_sem, 0, 1);

static volatile bool mqtt_connected;
static volatile bool lte_connected;
static uint32_t press_count;

/* Stored at module scope so the reconnect work handler can reach them. */
static const char broker[]    = CONFIG_BUTTON_MQTT_BROKER_HOSTNAME;
static const char client_id[] = CONFIG_BUTTON_MQTT_CLIENT_ID;

/* ---- Button -------------------------------------------------------------- */

static void button_pressed_cb(const struct device *dev, struct gpio_callback *cb,
			      uint32_t pins)
{
	k_sem_give(&button_sem);
}

/* ---- MQTT reconnection --------------------------------------------------- */

static void reconnect_work_fn(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(reconnect_work, reconnect_work_fn);

static void reconnect_work_fn(struct k_work *work)
{
	if (!lte_connected) {
		return;
	}

	struct mqtt_helper_conn_params conn = {
		.hostname.ptr	= broker,
		.hostname.size	= strlen(broker),
		.device_id.ptr	= client_id,
		.device_id.size	= strlen(client_id),
	};

	LOG_INF("Reconnecting to MQTT broker...");

	int err = mqtt_helper_connect(&conn);

	if (err) {
		LOG_ERR("Reconnect failed: %d — retrying in 30s", err);
		k_work_reschedule(&reconnect_work, K_SECONDS(30));
	}
}

/* ---- MQTT callbacks ------------------------------------------------------ */

static void on_mqtt_connack(enum mqtt_conn_return_code code, bool session_present)
{
	LOG_INF("MQTT connected to %s", broker);
	mqtt_connected = true;
	k_work_cancel_delayable(&reconnect_work);
}

static void on_mqtt_disconnect(int result)
{
	LOG_WRN("MQTT disconnected (%d) — reconnecting in 5s", result);
	mqtt_connected = false;
	k_work_reschedule(&reconnect_work, K_SECONDS(5));
}

static void on_mqtt_publish(struct mqtt_helper_buf topic, struct mqtt_helper_buf payload)
{
	/* Not subscribing — callback required by the API. */
}

/* ---- Publish ------------------------------------------------------------- */

static void publish_button_press(void)
{
	char buf[96];
	int len;

	press_count++;
	len = snprintk(buf, sizeof(buf),
		       "{\"event\":\"button_pressed\",\"count\":%u,\"uptime_ms\":%u}",
		       press_count, k_uptime_get_32());

	struct mqtt_publish_param param = {
		.message.payload.data		= buf,
		.message.payload.len		= len,
		.message.topic.qos		= MQTT_QOS_1_AT_LEAST_ONCE,
		.message_id			= mqtt_helper_msg_id_get(),
		.message.topic.topic.utf8	= CONFIG_BUTTON_MQTT_PUBLISH_TOPIC,
		.message.topic.topic.size	= strlen(CONFIG_BUTTON_MQTT_PUBLISH_TOPIC),
	};

	int err = mqtt_helper_publish(&param);

	if (err) {
		LOG_ERR("Publish failed: %d", err);
		return;
	}

	LOG_INF("Published (#%u): %s", press_count, buf);

	if (led.port) {
		gpio_pin_set_dt(&led, 1);
		k_msleep(150);
		gpio_pin_set_dt(&led, 0);
	}
}

/* ---- Network events ------------------------------------------------------ */

#define L4_EVENT_MASK (NET_EVENT_L4_CONNECTED | NET_EVENT_L4_DISCONNECTED)
static struct net_mgmt_event_callback l4_cb;

static void l4_event_handler(struct net_mgmt_event_callback *cb,
			     uint32_t event, struct net_if *iface)
{
	if (event == NET_EVENT_L4_CONNECTED) {
		LOG_INF("LTE connected");
		lte_connected = true;
		k_sem_give(&network_sem);
		if (!mqtt_connected) {
			k_work_reschedule(&reconnect_work, K_SECONDS(2));
		}
	} else if (event == NET_EVENT_L4_DISCONNECTED) {
		LOG_WRN("LTE disconnected");
		lte_connected = false;
		mqtt_connected = false;
		k_work_cancel_delayable(&reconnect_work);
	}
}

/* ---- Main ---------------------------------------------------------------- */

int main(void)
{
	int err;

	/* Button */
	if (!gpio_is_ready_dt(&button)) {
		LOG_ERR("Button device not ready");
		return -ENODEV;
	}
	gpio_pin_configure_dt(&button, GPIO_INPUT);
	gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
	gpio_init_callback(&button_cb_data, button_pressed_cb, BIT(button.pin));
	gpio_add_callback(button.port, &button_cb_data);

	/* LED (optional — same pattern as button sample) */
	if (led.port && !gpio_is_ready_dt(&led)) {
		LOG_WRN("LED device not ready, disabling");
		led.port = NULL;
	}
	if (led.port) {
		gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);
		LOG_INF("LED ready");
	}

	LOG_INF("Waiting for LTE...");

	/* LTE */
	net_mgmt_init_event_callback(&l4_cb, l4_event_handler, L4_EVENT_MASK);
	net_mgmt_add_event_callback(&l4_cb);
	conn_mgr_all_if_up(true);
	conn_mgr_all_if_connect(true);
	k_sem_take(&network_sem, K_FOREVER);

	/* MQTT init */
	struct mqtt_helper_cfg cfg = {
		.cb = {
			.on_connack    = on_mqtt_connack,
			.on_disconnect = on_mqtt_disconnect,
			.on_publish    = on_mqtt_publish,
		},
	};

	err = mqtt_helper_init(&cfg);
	if (err) {
		LOG_ERR("mqtt_helper_init: %d", err);
		return err;
	}

	/* MQTT connect */
	struct mqtt_helper_conn_params conn = {
		.hostname.ptr	= broker,
		.hostname.size	= strlen(broker),
		.device_id.ptr	= client_id,
		.device_id.size	= strlen(client_id),
	};

	LOG_INF("Connecting to MQTT broker: %s", broker);

	err = mqtt_helper_connect(&conn);
	if (err) {
		LOG_ERR("mqtt_helper_connect: %d", err);
		return err;
	}

	LOG_INF("Press the button to publish to: %s", CONFIG_BUTTON_MQTT_PUBLISH_TOPIC);

	while (true) {
		k_sem_take(&button_sem, K_FOREVER);

		if (mqtt_connected) {
			publish_button_press();
		} else {
			LOG_WRN("MQTT not connected — press discarded");
		}
	}

	return 0;
}
