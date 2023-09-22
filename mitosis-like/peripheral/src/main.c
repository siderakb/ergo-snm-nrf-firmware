/**
 * @file  main.c
 * @brief
 * @author ZiTe (honmonoh@gmail.com)
 * @note SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "main.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_DBG);

/* Key matrix GPIO. */
static const struct gpio_dt_spec row0_gpio = GPIO_DT_SPEC_GET_OR(DT_ALIAS(row0), gpios, {0});
static const struct gpio_dt_spec row1_gpio = GPIO_DT_SPEC_GET_OR(DT_ALIAS(row1), gpios, {0});
static const struct gpio_dt_spec row2_gpio = GPIO_DT_SPEC_GET_OR(DT_ALIAS(row2), gpios, {0});
static const struct gpio_dt_spec row3_gpio = GPIO_DT_SPEC_GET_OR(DT_ALIAS(row3), gpios, {0});
static struct gpio_dt_spec row_gpios[ROW_COUNT];

static const struct gpio_dt_spec col0_gpio = GPIO_DT_SPEC_GET_OR(DT_ALIAS(col0), gpios, {0});
static const struct gpio_dt_spec col1_gpio = GPIO_DT_SPEC_GET_OR(DT_ALIAS(col1), gpios, {0});
static const struct gpio_dt_spec col2_gpio = GPIO_DT_SPEC_GET_OR(DT_ALIAS(col2), gpios, {0});
static struct gpio_dt_spec col_gpios[COL_COUNT];

uint8_t raw_keymatrix[ROW_COUNT][COL_COUNT] = {0};

int16_t mouse_x, mouse_y, mouse_v;

/* Pipe 0 is used in this example. */
#define PIPE_NUMBER 0

#define TX_PAYLOAD_LENGTH 12

/* Maximum number of transmission attempts */
#define MAX_TX_ATTEMPTS 100

/* Gazell Link Layer TX result structure */
struct gzll_tx_result
{
  bool success;
  uint32_t pipe;
  nrf_gzll_device_tx_info_t info;
};

/* Payload to send to Host. */
static uint8_t data_payload[TX_PAYLOAD_LENGTH];

/* Placeholder for received ACK payloads from Host. */
static uint8_t ack_payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH];

#ifdef CONFIG_GZLL_TX_STATISTICS
/* Struct containing transmission statistics. */
static nrf_gzll_tx_statistics_t statistics;
static nrf_gzll_tx_statistics_t statistics_2;
#endif

/* Gazell Link Layer TX result message queue */
K_MSGQ_DEFINE(gzll_msgq,
              sizeof(struct gzll_tx_result),
              1,
              sizeof(uint32_t));

/* Main loop semaphore */
static K_SEM_DEFINE(main_sem, 0, 1);

static struct k_work gzll_work;

static void gzll_tx_result_handler(struct gzll_tx_result *tx_result);
static void gzll_work_handler(struct k_work *work);

void main(void)
{
  if (!keymatrix_init())
  {
    while (1) {}
  }

  if (!gzll_init())
  {
    while (1) {}
  }

  if (!pmw3360_init())
  {
    while (1) {}
  }

  LOG_INF("All ready.");
  while (true)
  {
    uint8_t curr_keymatrix[ROW_COUNT][COL_COUNT] = {0};
    keymatrix_scan(curr_keymatrix);

    bool keymatrix_changed = memcmp(raw_keymatrix, curr_keymatrix, sizeof(curr_keymatrix)) != 0;
    if (keymatrix_changed)
    {
      memcpy(raw_keymatrix, curr_keymatrix, sizeof(curr_keymatrix)); /* Update key matrix data. */
    }

    for (int r = 0; r < ROW_COUNT; r++)
    {
      for (int c = 0; c < COL_COUNT; c++)
      {
        if (raw_keymatrix[r][c] != 0)
        {
          LOG_DBG("Key R%d-C%d down", r, c);
        }
      }
    }
    LOG_DBG("------");

    /* Gazell. */
    if (k_sem_take(&main_sem, K_FOREVER))
    {
      continue;
    }

    pmw3360_read(&mouse_x, &mouse_y);

#ifdef CONFIG_GZLL_TX_STATISTICS
    if (statistics.packets_num >= 1000)
    {
      uint32_t key = irq_lock();

      statistics_2 = statistics;

      /* Reset statistics buffers. */
      nrf_gzll_reset_tx_statistics();

      irq_unlock(key);

      /* Print all transmission statistics. */
      LOG_INF("");
      LOG_INF("Total transmitted packets:   %4u", statistics_2.packets_num);
      LOG_INF("Total transmission time-outs: %03u", statistics_2.timeouts_num);
      LOG_INF("");

      for (uint8_t i = 0; i < nrf_gzll_get_channel_table_size(); i++)
      {
        LOG_INF(
            "Channel %u: %03u packets transmitted, %03u transmissions failed.",
            i,
            statistics_2.channel_packets[i],
            statistics_2.channel_timeouts[i]);
      }
    }
#endif

    k_msleep(10);
  }
}

/**
 * @brief Setup Gazell wireless protocol.
 * @return true if successful, otherwise error.
 */
bool gzll_init(void)
{
  bool ret;

  k_work_init(&gzll_work, gzll_work_handler);

  /* Initialize Gazell Link Layer glue */
  ret = gzll_glue_init();
  if (!ret)
  {
    LOG_ERR("Cannot initialize Gazell glue code");
    return false;
  }

  /* Initialize the Gazell Link Layer */
  ret = nrf_gzll_init(NRF_GZLL_MODE_DEVICE);
  if (!ret)
  {
    LOG_ERR("Cannot initialize Gazell");
    return false;
  }

  nrf_gzll_set_max_tx_attempts(MAX_TX_ATTEMPTS);

#ifdef CONFIG_GZLL_TX_STATISTICS
  /* Turn on transmission statistics gathering. */
  int result_value = nrf_gzll_tx_statistics_enable(&statistics);
  if (!result_value)
  {
    LOG_ERR("Cannot enable GZLL TX statistics");
    return false;
  }
#endif

  data_payload[0] = 0x00;
  ret = nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER, data_payload, TX_PAYLOAD_LENGTH);
  if (!ret)
  {
    LOG_ERR("Cannot add packet to Gazell TX FIFO");
    return;
  }

  ret = nrf_gzll_enable();
  if (!ret)
  {
    LOG_ERR("Cannot enable Gazell");
    return false;
  }

  LOG_INF("Gazell device started.");

  return true; /* Success. */
}

static void gzll_device_report_tx(bool success,
                                  uint32_t pipe,
                                  nrf_gzll_device_tx_info_t *tx_info)
{
  int err;
  struct gzll_tx_result tx_result;

  tx_result.success = success;
  tx_result.pipe = pipe;
  tx_result.info = *tx_info;
  err = k_msgq_put(&gzll_msgq, &tx_result, K_NO_WAIT);
  if (!err)
  {
    /* Get work handler to run */
    k_work_submit(&gzll_work);
  }
  else
  {
    LOG_ERR("Cannot put TX result to message queue");
  }
}

void nrf_gzll_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
  gzll_device_report_tx(true, pipe, &tx_info);
}

void nrf_gzll_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
  gzll_device_report_tx(false, pipe, &tx_info);
}

void nrf_gzll_disabled(void)
{
}

void nrf_gzll_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info)
{
}

static void gzll_work_handler(struct k_work *work)
{
  struct gzll_tx_result tx_result;

  /* Process message queue */
  while (!k_msgq_get(&gzll_msgq, &tx_result, K_NO_WAIT))
  {
    gzll_tx_result_handler(&tx_result);
  }

  /* Get main loop to run */
  k_sem_give(&main_sem);
}

static void gzll_tx_result_handler(struct gzll_tx_result *tx_result)
{
  int err;
  bool result_value;
  uint32_t ack_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

  if (tx_result->success)
  {
    if (tx_result->info.payload_received_in_ack)
    {
      /* Pop packet and write first byte of the payload to the GPIO port. */
      result_value = nrf_gzll_fetch_packet_from_rx_fifo(tx_result->pipe,
                                                        ack_payload,
                                                        &ack_payload_length);
      if (!result_value)
      {
        LOG_ERR("RX fifo error");
      }
      else if (ack_payload_length > 0)
      {
        // Do something.
      }
    }
  }
  else
  {
    LOG_ERR("Gazell transmission failed");
  }

  /* Load data payload into the TX queue. */
  // data_payload[0] = (raw_keymatrix[1][1] << 3) + (raw_keymatrix[1][0] << 2) + (raw_keymatrix[2][1] << 1) + (raw_keymatrix[2][0]);
  // data_payload[0] = (raw_keymatrix[0][0] << 4) + (raw_keymatrix[0][1] << 3) + (raw_keymatrix[0][2] << 2) + (raw_keymatrix[0][3] << 1) + (raw_keymatrix[0][4]);
  // data_payload[1] = (raw_keymatrix[0][5] << 7) + (raw_keymatrix[0][6] << 6) + (raw_keymatrix[1][0] << 5) + (raw_keymatrix[1][1] << 4) + (raw_keymatrix[1][2] << 3) + (raw_keymatrix[1][3] << 2) + (raw_keymatrix[1][4] << 1) + (raw_keymatrix[1][5]);
  // data_payload[2] = (raw_keymatrix[1][6] << 7) + (raw_keymatrix[2][0] << 6) + (raw_keymatrix[2][1] << 5) + (raw_keymatrix[2][2] << 4) + (raw_keymatrix[2][3] << 3) + (raw_keymatrix[2][4] << 2) + (raw_keymatrix[2][5] << 1) + (raw_keymatrix[2][6]);
  // data_payload[3] = (raw_keymatrix[3][0] << 7) + (raw_keymatrix[3][1] << 6) + (raw_keymatrix[3][2] << 5) + (raw_keymatrix[3][3] << 4) + (raw_keymatrix[3][4] << 3) + (raw_keymatrix[3][5] << 2) + (raw_keymatrix[3][6] << 1) + (raw_keymatrix[4][0]);
  // data_payload[4] = (raw_keymatrix[4][1] << 7) + (raw_keymatrix[4][2] << 6) + (raw_keymatrix[4][3] << 5) + (raw_keymatrix[4][4] << 4) + (raw_keymatrix[4][5] << 3) + (raw_keymatrix[3][5] << 2) + (raw_keymatrix[3][6] << 1) + (raw_keymatrix[4][0]);

  memcpy(data_payload[0], raw_keymatrix[0], 1);
  memcpy(data_payload[1], raw_keymatrix[1], 1);
  memcpy(data_payload[2], raw_keymatrix[2], 1);
  memcpy(data_payload[3], raw_keymatrix[3], 1);
  memcpy(data_payload[4], raw_keymatrix[4], 1);
  data_payload[5] = mouse_x >> 8;
  data_payload[6] = mouse_x & 0xFF;
  data_payload[7] = mouse_y >> 8;
  data_payload[8] = mouse_y & 0xFF;
  data_payload[9] = mouse_v >> 8;
  data_payload[10] = mouse_v & 0xFF;
  data_payload[11] = EOT;

  result_value = nrf_gzll_add_packet_to_tx_fifo(tx_result->pipe,
                                                data_payload,
                                                TX_PAYLOAD_LENGTH);
  if (!result_value)
  {
    LOG_ERR("Gazell TX fifo error");
  }
}

/**
 * @brief Setup key matrix.
 * @return true if successful, otherwise error.
 */
bool keymatrix_init(void)
{
  col_gpios[0] = col0_gpio;
  col_gpios[1] = col1_gpio;
  col_gpios[2] = col2_gpio;

  for (uint8_t i = 0; i < COL_COUNT; i++)
  {
    struct gpio_dt_spec gpio = col_gpios[i];

    if (!device_is_ready(gpio.port))
    {
      LOG_ERR("GPIO Device %s is not ready.", gpio.port->name);
      return false;
    }

    int ret = gpio_pin_configure_dt(&gpio, GPIO_INPUT);
    if (ret != 0)
    {
      LOG_ERR("Failed to configure GPIO %s pin %d, return: %d.", gpio.port->name, gpio.pin, ret);
      return false;
    }
  }

  row_gpios[0] = row0_gpio;
  row_gpios[1] = row1_gpio;
  row_gpios[2] = row2_gpio;
  row_gpios[3] = row3_gpio;

  for (uint8_t i = 0; i < ROW_COUNT; i++)
  {
    struct gpio_dt_spec gpio = row_gpios[i];

    if (!device_is_ready(gpio.port))
    {
      LOG_ERR("GPIO Device %s is not ready.", gpio.port->name);
      return false;
    }

    int ret = gpio_pin_configure_dt(&gpio, GPIO_OUTPUT);
    if (ret != 0)
    {
      LOG_ERR("Failed to configure GPIO %s pin %d, return: %d.", gpio.port->name, gpio.pin, ret);
      return false;
    }

    ret = gpio_pin_set_dt(&gpio, 1); /* Unselect row by output HIGH. */
    if (ret != 0)
    {
      LOG_ERR("Failed to set GPIO %s pin %d HIGH, return: %d.", gpio.port->name, gpio.pin, ret);
      return false;
    }
  }

  LOG_INF("Key matrix ready.");

  return true; /* Success. */
}

/**
 * @brief Select a row and read columns. (Diode direction: Col-to-Row)
 * @param row Selected row index.
 * @param col_states Readed column states output.
 */
void keymatrix_read_cols(uint8_t row, uint8_t col_states[])
{
  struct gpio_dt_spec row_pin = row_gpios[row];
  gpio_pin_set_dt(&row_pin, 0); /* Select row by output LOW. */

  /* Read each column. */
  for (uint8_t col_index = 0; col_index < COL_COUNT; col_index++)
  {
    struct gpio_dt_spec col_pin = col_gpios[col_index];
    int val = gpio_pin_get_dt(&col_pin); /* Read selected colume pin state. */
    col_states[col_index] = val;
  }

  gpio_pin_set_dt(&row_pin, 1); /* Unselect row by output HIGH. */
}

/**
 * @brief Key matrix scan.
 * @param matrix Key matrix[Row][Col] output.
 */
void keymatrix_scan(uint8_t matrix[][COL_COUNT])
{
  /* For each row. */
  for (uint8_t row_index = 0; row_index < ROW_COUNT; row_index++)
  {
    uint8_t col_states[COL_COUNT] = {0};
    keymatrix_read_cols(row_index, col_states);                /* Select a row and read columns. */
    memcpy(matrix[row_index], col_states, sizeof(col_states)); /* Copy data. */
  }
}

/**
 * @brief Setup PMW3360 mouse sensor.
 * @return true if successful, otherwise error.
 */
bool pmw3360_init(void)
{
  if (!device_is_ready(pmw3360_device))
  {
    LOG_ERR("Device %s is not ready.", pmw3360_device->name);
    return false;
  }

#if defined(ENABLE_INTERRUPT)
  /* Setup interrupt triger. */
  pmw3360_trigger = (struct sensor_trigger){
      .type = SENSOR_TRIG_DATA_READY,
      .chan = SENSOR_CHAN_ALL,
  };

  int ret = sensor_trigger_set(pmw3360_device, &pmw3360_trigger, pmw3360_callback);
  if (ret != 0)
  {
    LOG_ERR("PMW3360 cannot config trigger, error code: %d.", ret);
    return false;
  }
#endif

  LOG_INF("PMW3360 started.");

  return true; /* Success. */
}

/**
 * @brief Read mouse report data from PMW3360.
 * @param x X axis data.
 * @param y Y axis data.
 * @return 0 if successful, otherwise error code.
 */
int pmw3360_read(int16_t *x, int16_t *y)
{
  struct sensor_value sensor_x, sensor_y;
  int ret;

  /* Fetch all sensor channels. */
  ret = sensor_sample_fetch_chan(pmw3360_device, SENSOR_CHAN_ALL);
  if (ret < 0)
  {
    LOG_ERR("PMW3360 could not fetch data, error code: %d", ret);
    return ret;
  }

  /* Get X axis data. */
  ret = sensor_channel_get(pmw3360_device, SENSOR_CHAN_POS_DX, &sensor_x);
  if (ret < 0)
  {
    LOG_ERR("PMW3360 could not get value X, error code: %d", ret);
    return ret;
  }

  /* Get Y axis data. */
  ret = sensor_channel_get(pmw3360_device, SENSOR_CHAN_POS_DY, &sensor_y);
  if (ret < 0)
  {
    LOG_ERR("PMW3360 could not get value Y, error code: %d", ret);
    return ret;
  }

  /* Conver to 16 bit data. */
  *x = (int16_t)sensor_value_to_double(&sensor_x);
  *y = (int16_t)sensor_value_to_double(&sensor_y);

  LOG_DBG("PMW3360 report X: %5d, Y: %5d", (int)*x, (int)*y);

  return 0; /* Success. */
}

void pmw3360_callback(const struct device *dev, const struct sensor_trigger *trig)
{
  int16_t x, y;
  int ret = pmw3360_read(&x, &y);
  if (ret != 0)
  {
    LOG_ERR("PMW3360 could not read by trigger, error code: %d", ret);
    (void)sensor_trigger_set(dev, trig, NULL);
    return;
  }
}
