/**
 * @file  main.c
 * @brief
 * @author ZiTe (honmonoh@gmail.com)
 * @note SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include "main.h"

/* UART. */
static const struct device *qmk_uart_device = DEVICE_DT_GET(DT_NODELABEL(uart1));

#define MSG_SIZE 32

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

/* receive buffer used in UART ISR callback */
static char rx_buf[MSG_SIZE];
static int rx_buf_pos;

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

const struct device *pmw3360_device = DEVICE_DT_GET_ONE(pixart_pmw3360);
static struct sensor_trigger pmw3360_trigger;

LOG_MODULE_REGISTER(app, LOG_LEVEL_DBG);

/* Pipe 0 is used in this example. */
#define PIPE_NUMBER 0

#define TX_PAYLOAD_LENGTH 1

/* Gazell Link Layer RX result structure */
struct gzll_rx_result
{
  uint32_t pipe;
  nrf_gzll_host_rx_info_t info;
};

/* Placeholder for data payload received from host. */
static uint8_t data_payload[NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH];

/* Payload to attach to ACK sent to device. */
static uint8_t ack_payload[TX_PAYLOAD_LENGTH];

/* Gazell Link Layer RX result message queue */
K_MSGQ_DEFINE(gzll_msgq,
              sizeof(struct gzll_rx_result),
              1,
              sizeof(uint32_t));

/* Main loop semaphore */
static K_SEM_DEFINE(main_sem, 0, 1);

static struct k_work gzll_work;

void main(void)
{
  /* Init. */
  if (!qmk_uart_init())
  {
    while (1) {}
  }

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

  // qmk_uart_send_bytes("Hello!\n");
  LOG_INF("All ready.");
  while (1)
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

    /* Gazell. */
    if (k_sem_take(&main_sem, K_FOREVER))
    {
      continue;
    }

    int16_t mouse_x, mouse_y;
    pmw3360_read(&mouse_x, &mouse_y);

    qmk_uart_send(&raw_keymatrix, mouse_x, mouse_y);

    k_msleep(10);
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
 * @brief Setup the serial port for QMK.
 * @return true if successful, otherwise error.
 */
bool qmk_uart_init(void)
{
  if (!device_is_ready(qmk_uart_device))
  {
    LOG_ERR("Device %s is not ready.", qmk_uart_device->name);
    return false;
  }

  /* Setup interrupt. */
  uart_irq_callback_user_data_set(qmk_uart_device, qmk_uart_callback, NULL); /* Callback function. */
  uart_irq_rx_enable(qmk_uart_device);

  LOG_INF("QMK UART started.");

  return true; /* Success. */
}

void qmk_uart_callback(const struct device *dev, void *user_data)
{
  /* Do nothing. */
}

/**
 * @brief Send data bytes to QMK.
 * @param data Data packets.
 * @param len  Data packets length.
 */
void qmk_uart_send_bytes(uint8_t *data, uint16_t len)
{
  for (int i = 0; i < len; i++)
  {
    uart_poll_out(qmk_uart_device, data[i]);
  }
}

/**
 * @brief Send keymatrix and mouse data to QMK.
 * @param keymatrix Keymatrix[Row][Col].
 * @param mouse_x Mouse X axix data.
 * @param mouse_y Mouse Y axix data.
 */
void qmk_uart_send(uint8_t keymatrix[][COL_COUNT], int16_t mouse_x, int16_t mouse_y)
{
  /*
   * Send keymatrix data.
   *
   * Row-0 Col-0
   * Row-0 Col-1
   * Row-0 Col-2
   * ...
   * Row-1 Col-0
   * Row-1 Col-1
   */
  for (uint8_t row = 0; row < ROW_COUNT; row++)
  {
    qmk_uart_send_bytes(keymatrix[row], COL_COUNT);
  }

  /* Send mouse data and EOT. */
  uint8_t data[5];
  data[0] = ((uint16_t)mouse_x >> 8);   /* Upper 8 bits of mouse X. */
  data[1] = ((uint16_t)mouse_x & 0xFF); /* Lower 8 bits of mouse X. */
  data[2] = ((uint16_t)mouse_y >> 8);   /* Upper 8 bits of mouse Y. */
  data[3] = ((uint16_t)mouse_y & 0xFF); /* Lower 8 bits of mouse Y. */
  data[4] = 0x84;                       /* EOT (End of Transmission) character. */
  qmk_uart_send_bytes(data, 5);
}

/**
 * @brief Setup Gazell wireless protocol.
 * @return true if successful, otherwise error.
 */
bool gzll_init(void)
{
  bool ret;

  k_work_init(&gzll_work, gzll_work_callback);

  ret = gzll_glue_init();
  if (!ret)
  {
    LOG_ERR("Cannot initialize Gazell glue.");
    return false;
  }

  ret = nrf_gzll_init(NRF_GZLL_MODE_HOST);
  if (!ret)
  {
    LOG_ERR("Cannot initialize Gazell.");
    return false;
  }

  ret = nrf_gzll_enable();
  if (!ret)
  {
    LOG_ERR("Cannot enable Gazell.");
    return false;
  }

  LOG_INF("Gazell host started.");

  return true; /* Success. */
}

void nrf_gzll_device_tx_success(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
  /* Do nothing. */
}

void nrf_gzll_device_tx_failed(uint32_t pipe, nrf_gzll_device_tx_info_t tx_info)
{
  /* Do nothing. */
}

void nrf_gzll_disabled(void)
{
  /* Do nothing. */
}

void nrf_gzll_host_rx_data_ready(uint32_t pipe, nrf_gzll_host_rx_info_t rx_info)
{
  int err;
  struct gzll_rx_result rx_result;

  rx_result.pipe = pipe;
  rx_result.info = rx_info;
  err = k_msgq_put(&gzll_msgq, &rx_result, K_NO_WAIT);
  if (!err)
  {
    /* Get work handler to run */
    k_work_submit(&gzll_work);
  }
  else
  {
    LOG_ERR("Cannot put RX result to message queue");
  }
}

void gzll_work_callback(struct k_work *work)
{
  struct gzll_rx_result rx_result;

  /* Process message queue */
  while (!k_msgq_get(&gzll_msgq, &rx_result, K_NO_WAIT))
  {
    gzll_rx_result_handler(&rx_result);
  }

  /* Get main loop to run */
  k_sem_give(&main_sem);
}

void gzll_rx_result_handler(struct gzll_rx_result *rx_result)
{
  int err;
  uint32_t data_payload_length = NRF_GZLL_CONST_MAX_PAYLOAD_LENGTH;

  /* Pop packet and write first byte of the payload to the GPIO port. */
  bool result_value = nrf_gzll_fetch_packet_from_rx_fifo(rx_result->pipe,
                                                         data_payload,
                                                         &data_payload_length);
  if (!result_value)
  {
    LOG_ERR("Gazell RX fifo error");
  }
  else if (data_payload_length > 0)
  {
    LOG_DBG("Gazell received");
  }

  /* Send. */
  ack_payload[0] = (raw_keymatrix[1][1] << 3) + (raw_keymatrix[1][0] << 2) + (raw_keymatrix[2][1] << 1) + (raw_keymatrix[2][0]);
  result_value = nrf_gzll_add_packet_to_tx_fifo(rx_result->pipe,
                                                ack_payload,
                                                TX_PAYLOAD_LENGTH);
  if (!result_value)
  {
    LOG_ERR("Gazell TX fifo error");
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
