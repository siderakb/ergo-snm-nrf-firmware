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

uint8_t raw_keymatrix[ROW_COUNT * 2] = {0};
uint8_t raw_mouse[6] = {0};

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

  LOG_INF("Central all ready.");
  qmk_uart_send_bytes("QMK UART Ready\r\n", 18);
  while (1)
  {
    /* Gazell. */
    if (k_sem_take(&main_sem, K_FOREVER))
    {
      continue;
    }

    qmk_uart_send(raw_keymatrix, raw_mouse);

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
  for (uint16_t i = 0; i < len; i++)
  {
    uart_poll_out(qmk_uart_device, data[i]);
    // LOG_DBG("QMK UART send: %d", data[i]);
  }
}

/**
 * @brief Send keymatrix and mouse data to QMK.
 * @param keymatrix Keymatrix[Row][Col].
 * @param mouse_x Mouse X axix data.
 * @param mouse_y Mouse Y axix data.
 */
void qmk_uart_send(uint8_t *keymatrix, uint8_t *mouse)
{
  qmk_uart_send_bytes(keymatrix, ROW_COUNT * 2);

  qmk_uart_send_bytes(mouse, 6);
  // LOG_DBG("X: 0x%02X%02X. Y:0x%2X%2X", mouse[0], mouse[1], mouse[2], mouse[3]);

  uint8_t end[1] = {EOT};
  qmk_uart_send_bytes(end, 1);
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
    LOG_DBG("Gazell received, length: %d", data_payload_length);

    uint8_t offset = 0;
    if (data_payload_length == (ROW_COUNT + 6 + 1)) /* Right. */
    {
      offset = ROW_COUNT;
      memcpy(raw_mouse, data_payload + ROW_COUNT, 6);
      // LOG_DBG("L: %d, x: 0x%02X%02X, y: 0x%02X%02X, v: 0x%02X%02X, E: 0x%02X",
      //         data_payload_length,
      //         data_payload[ROW_COUNT],
      //         data_payload[ROW_COUNT + 1],
      //         data_payload[ROW_COUNT + 2],
      //         data_payload[ROW_COUNT + 3],
      //         data_payload[ROW_COUNT + 4],
      //         data_payload[ROW_COUNT + 5],
      //         data_payload[ROW_COUNT + 6]);
    }

    memcpy(raw_keymatrix + offset, data_payload, ROW_COUNT);
  }

  /* Send. */
  ack_payload[0] = ACK;
  result_value = nrf_gzll_add_packet_to_tx_fifo(rx_result->pipe,
                                                ack_payload,
                                                TX_PAYLOAD_LENGTH);
  if (!result_value)
  {
    LOG_ERR("Gazell TX fifo error");
  }
}
