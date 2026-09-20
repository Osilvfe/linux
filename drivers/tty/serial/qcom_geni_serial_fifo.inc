/* SPDX-License-Identifier: GPL-2.0-only */
/* Optional FIFO-mode support selected by a serial controller DT property. */

static const struct uart_ops qcom_geni_uart_fifo_pops = {
	.tx_empty = qcom_geni_serial_tx_empty,
	.stop_tx = qcom_geni_serial_stop_tx_fifo,
	.start_tx = qcom_geni_serial_start_tx_fifo,
	.stop_rx = qcom_geni_serial_stop_rx_fifo,
	.start_rx = qcom_geni_serial_start_rx_fifo,
	.set_termios = qcom_geni_serial_set_termios,
	.startup = qcom_geni_serial_startup,
	.request_port = qcom_geni_serial_request_port,
	.config_port = qcom_geni_serial_config_port,
	.shutdown = qcom_geni_serial_shutdown,
	.flush_buffer = qcom_geni_serial_flush_buffer_fifo,
	.type = qcom_geni_serial_get_type,
	.set_mctrl = qcom_geni_serial_set_mctrl,
	.get_mctrl = qcom_geni_serial_get_mctrl,
	.pm = qcom_geni_serial_pm,
};

static void qcom_geni_serial_select_fifo_mode(
		struct platform_device *pdev,
		const struct qcom_geni_device_data *data,
		struct qcom_geni_serial_port *port)
{
	if (data->console ||
	    !of_property_read_bool(pdev->dev.of_node, "qcom,force-fifo-mode"))
		return;

	port->fifo_mode = true;
	port->uport.ops = &qcom_geni_uart_fifo_pops;
	dev_info(&pdev->dev, "using FIFO transfer mode\n");
}
