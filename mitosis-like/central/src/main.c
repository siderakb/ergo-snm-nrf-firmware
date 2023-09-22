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

int16_t mouse_x, mouse_y;

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

  if (!gzll_init())
  {
    while (1) {}
  }

  // qmk_uart_send_bytes("Hello!\n");
  LOG_INF("All ready.");
  while (1)
  {

    /* Gazell. */
    if (k_sem_take(&main_sem, K_FOREVER))
    {
      continue;
    }

    qmk_uart_send(&raw_keymatrix, mouse_x, mouse_y);

    k_msleep(10);
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

  ack_payload[0] = 0x00;
  ret = nrf_gzll_add_packet_to_tx_fifo(PIPE_NUMBER, ack_payload, TX_PAYLOAD_LENGTH);
  if (!ret)
  {
    LOG_ERR("Cannot add packet to Gazell TX FIFO");
    return;
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
    LOG_DBG("Gazell received%d", data_payload[0]);
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
