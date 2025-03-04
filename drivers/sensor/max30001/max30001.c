/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT maxim_max30001

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(max30001, CONFIG_SENSOR_LOG_LEVEL);

struct max30001_data {
  const struct device *dev;

  struct spi_dt_spec bus;
  struct gpio_callback gpio_cb;
  sensor_trigger_handler_t drdy_handler;
  const struct sensor_trigger *drdy_trigger;

  struct k_work work;
};

struct max30001_config {
  struct spi_dt_spec bus;
  struct gpio_dt_spec interrupt_pin;
};

static void max30001_work_cb(struct k_work *work) {
  struct max30001_data *data = CONTAINER_OF(work, struct max30001_data, work);
  if (data->drdy_handler != NULL) {
    data->drdy_handler(data->dev, data->drdy_trigger);
  }
}

static void max30001_gpio_callback(const struct device *dev,
                                   struct gpio_callback *cb, uint32_t pins) {
  struct max30001_data *data = CONTAINER_OF(cb, struct max30001_data, gpio_cb);

  ARG_UNUSED(pins);
  const int ret = k_work_submit(&data->work);
  switch (ret) {
    case 0:
      LOG_WRN("MAX30001 read already submitted");
      break;
    case 2:
      LOG_WRN("MAX30001 read already running");
      break;
    case -EBUSY:
      LOG_WRN("MAX30001 queue busy");
      break;
    case -EINVAL:
      LOG_WRN("MAX30001 queue is null");
      break;
    case -ENODEV:
      LOG_WRN("MAX30001 queue has not been started");
      break;
  }
}

static int max30001_trigger_set(const struct device *dev,
                                const struct sensor_trigger *trig,
                                sensor_trigger_handler_t handler) {
  struct max30001_data *data = dev->data;

  if (trig->type != SENSOR_TRIG_DATA_READY) {
    return -ENOTSUP;
  }
  if (trig->chan != SENSOR_CHAN_ALL) {
    return -ENOTSUP;
  }

  data->drdy_handler = handler;
  data->drdy_trigger = trig;

  return 0;
}

static int max30001_sample_fetch(const struct device *dev,
                                 enum sensor_channel chan) {
//   const struct max30001_config *config = dev->config;
//   struct max30001_data *data = dev->data;

  //   data->state = gpio_pin_get_dt(&config->interrupt_pin);

  return 0;
}

static int max30001_channel_get(const struct device *dev,
                                enum sensor_channel chan,
                                struct sensor_value *val) {
//   struct max30001_data *data = dev->data;

  if (chan != SENSOR_CHAN_PROX) {
    return -ENOTSUP;
  }

  val->val1 = 0;

  return 0;
}

static DEVICE_API(sensor, max30001_api) = {
    .sample_fetch = &max30001_sample_fetch,
    .channel_get = &max30001_channel_get,
    .trigger_set = &max30001_trigger_set,
};

static int max30001_read_reg(const struct device *dev, uint8_t reg, uint8_t* buf, size_t buf_size) {
    struct max30001_data *data = dev->data;
    const size_t padding_bytes = buf_size % 3;
    
    uint8_t command = reg << 1;

    struct spi_buf tx_buf[2] = {
        {.buf = &command, .len = 1},
        {.len = buf_size + padding_bytes},
    };
    struct spi_buf_set tx_bufs = {.buffers = tx_buf, .count = 2};

    struct spi_buf rx_buf[3] = {
        {.len = 1},
        {.buf = buf, .len = buf_size},
        {.len = padding_bytes},
    };
    struct spi_buf_set rx_bufs = {.buffers = rx_buf, .count = 3};

    return spi_transceive_dt(&data->bus, &tx_bufs, &rx_bufs);
}

static int max30001_init(const struct device *dev) {
  const struct max30001_config *config = dev->config;
  struct max30001_data *data = dev->data;
  data->dev = dev;
  memcpy(&data->bus, &config->bus, sizeof(data->bus));

  int ret;

  if (!device_is_ready(data->bus.bus)) {
    LOG_ERR("SPI is not ready");
    return -ENODEV;
  }

  if (!spi_is_ready_dt(&data->bus)) {
    LOG_ERR("SPI is not ready");
    return -ENODEV;
  }

  if (!device_is_ready(config->interrupt_pin.port)) {
    LOG_ERR("GPIO is not ready");
    return -ENODEV;
  }

  uint8_t first_five_registers[15];

  k_msleep(100);

  ret = max30001_read_reg(dev, 0x00, first_five_registers, 15);
  if (ret) {
    LOG_ERR("Could not read initial register values (%d)", ret);
    return ret;
  }

  LOG_HEXDUMP_INF(first_five_registers, sizeof(first_five_registers), "Read registers from MAX30001");

  data->work.handler = max30001_work_cb;
  gpio_init_callback(&data->gpio_cb, max30001_gpio_callback,
                     BIT(config->interrupt_pin.pin));

  if (gpio_add_callback(config->interrupt_pin.port, &data->gpio_cb) < 0) {
    LOG_ERR("Could not add gpio callback");
    return -EIO;
  }
  if (gpio_pin_interrupt_configure_dt(&config->interrupt_pin,
                                      GPIO_INT_EDGE_TO_ACTIVE)) {
    LOG_ERR("Could not configure gpio callback");
    return -EIO;
  }

  ret = gpio_pin_configure_dt(&config->interrupt_pin, GPIO_INPUT);
  if (ret < 0) {
    LOG_ERR("Could not configure interrupt GPIO (%d)", ret);
    return ret;
  }

  return 0;
}

#define MAXIM_MAX30001_INIT(i)                                                 \
  static struct max30001_data max30001_data_##i;                               \
                                                                               \
  static const struct max30001_config max30001_config_##i = {                  \
      .bus = SPI_DT_SPEC_INST_GET(i, SPI_OP_MODE_MASTER | SPI_WORD_SET(8), 0), \
      .interrupt_pin = GPIO_DT_SPEC_INST_GET(i, interrupt_gpios),              \
  };                                                                           \
                                                                               \
  DEVICE_DT_INST_DEFINE(i, max30001_init, NULL, &max30001_data_##i,            \
                        &max30001_config_##i, POST_KERNEL,                     \
                        CONFIG_SENSOR_INIT_PRIORITY, &max30001_api);

DT_INST_FOREACH_STATUS_OKAY(MAXIM_MAX30001_INIT)
