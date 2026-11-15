static bool caihong_pd_fixed_test;
module_param_named(caihong_pd_fixed_test, caihong_pd_fixed_test, bool, 0444);
MODULE_PARM_DESC(caihong_pd_fixed_test,
		 "Expose bounded Caihong fixed 5/9V PD tests (default: disabled)");

#define BATTMGR_ONEPLUS_PD_FIXED_9V_MV	9000
#define BATTMGR_ONEPLUS_PD_TRANSITION_UA	500000
#define BATTMGR_ONEPLUS_PD_TEST_MAX_UA	2000000
#define BATTMGR_ONEPLUS_PD_TOLERANCE_UV	500000
#define BATTMGR_ONEPLUS_PD_VOLTAGE_CHECKS	30
#define BATTMGR_ONEPLUS_PD_VOLTAGE_DELAY_MS	100
#define BATTMGR_ONEPLUS_PD_SAMPLE_INTERVAL	45
#define BATTMGR_ONEPLUS_PD_HOLD_CHECKS	\
	(BATTMGR_ONEPLUS_PD_TEST_SAMPLES * BATTMGR_ONEPLUS_PD_SAMPLE_INTERVAL)
#define BATTMGR_ONEPLUS_PD_HOLD_DELAY_MS	250

static int qcom_battmgr_oneplus_pd_fixed_current(
	struct qcom_battmgr *battmgr, u32 voltage_mv)
{
	u32 size = le32_to_cpu(battmgr->oneplus_pps.data_size);
	u32 selected_ma = 0;
	u32 count;
	u32 index;

	if (voltage_mv != BATTMGR_ONEPLUS_FIXED_5V_MV &&
	    voltage_mv != BATTMGR_ONEPLUS_PD_FIXED_9V_MV)
		return -EINVAL;
	if (size < 3 * sizeof(__le32) ||
	    size > sizeof(battmgr->oneplus_pps.data) || size % sizeof(__le32))
		return -EMSGSIZE;
	count = min_t(u32, size / sizeof(__le32) - 2,
		      BATTMGR_ONEPLUS_PPS_PDO_MAX);
	for (index = 0; index < count; index++) {
		u32 pdo = le32_to_cpu(battmgr->oneplus_pps.data[index + 2]);
		u32 current_ma = (pdo & 0x3ff) * 10;

		if (pdo >> 30 || ((pdo >> 10) & 0x3ff) * 50 != voltage_mv)
			continue;
		if (!current_ma)
			return -EPROTO;
		selected_ma = selected_ma ? min(selected_ma, current_ma) : current_ma;
	}

	return selected_ma ? selected_ma : -EOPNOTSUPP;
}

static int qcom_battmgr_oneplus_pd_battery_check_locked(
	struct qcom_battmgr *battmgr, bool entry)
{
	u32 max_uv = entry ? BATTMGR_ONEPLUS_POLICY_AUTO_VBAT_MAX_UV :
		BATTMGR_ONEPLUS_POLICY_VBAT_MAX_UV;
	int ret;

	if (!battmgr->service_up)
		return -ENOTCONN;
	if (READ_ONCE(battmgr->oneplus_charge_inhibit))
		return -ECANCELED;
	ret = qcom_battmgr_oneplus_request_property(battmgr,
		BATTMGR_BAT_PROPERTY_GET, BATT_CAPACITY, 0);
	if (ret)
		return ret;
	if (battmgr->status.percent >= BATTMGR_ONEPLUS_CP_DUAL_FULL_CAPACITY_MIN)
		return -EALREADY;
	ret = qcom_battmgr_oneplus_request_property(battmgr,
		BATTMGR_BAT_PROPERTY_GET, BATT_STATUS, 0);
	if (ret)
		return ret;
	if (battmgr->status.status != POWER_SUPPLY_STATUS_CHARGING)
		return -EAGAIN;
	ret = qcom_battmgr_oneplus_request_property(battmgr,
		BATTMGR_BAT_PROPERTY_GET, BATT_TEMP, 0);
	if (ret)
		return ret;
	if (battmgr->status.temperature < BATTMGR_ONEPLUS_POLICY_TEMP_MIN_DECIC ||
	    battmgr->status.temperature > BATTMGR_ONEPLUS_POLICY_TEMP_MAX_DECIC)
		return -ERANGE;
	ret = qcom_battmgr_oneplus_request_property(battmgr,
		BATTMGR_BAT_PROPERTY_GET, BATT_VOLT_NOW, 0);
	if (ret)
		return ret;
	if (battmgr->status.voltage_now < BATTMGR_ONEPLUS_CP_2TO1_VBAT_MIN_UV ||
	    battmgr->status.voltage_now > max_uv)
		return -ERANGE;

	return 0;
}

static int qcom_battmgr_oneplus_pd_voltage_check_locked(
	struct qcom_battmgr *battmgr, u32 voltage_mv, u32 *measured_uv,
	bool allow_offline)
{
	u32 target_uv = voltage_mv * 1000;
	u32 stable = 0;
	u32 index;
	int ret;

	for (index = 0; index < BATTMGR_ONEPLUS_PD_VOLTAGE_CHECKS; index++) {
		ret = qcom_battmgr_oneplus_request_property(battmgr,
			BATTMGR_USB_PROPERTY_GET, USB_VOLT_NOW, 0);
		if (ret)
			return ret;
		*measured_uv = battmgr->usb.voltage_now;
		if (*measured_uv > BATTMGR_ONEPLUS_PD_FIXED_9V_MV * 1000 +
		    BATTMGR_ONEPLUS_PD_TOLERANCE_UV)
			return -EOVERFLOW;
		if (allow_offline && *measured_uv < 1000000) {
			ret = qcom_battmgr_oneplus_request_property(battmgr,
				BATTMGR_USB_PROPERTY_GET, USB_ONLINE, 0);
			if (ret)
				return ret;
			if (!battmgr->usb.online)
				return 0;
		}
		if (*measured_uv >= target_uv - BATTMGR_ONEPLUS_PD_TOLERANCE_UV &&
		    *measured_uv <= target_uv + BATTMGR_ONEPLUS_PD_TOLERANCE_UV) {
			if (++stable == 2)
				return 0;
		} else {
			stable = 0;
		}
		msleep(BATTMGR_ONEPLUS_PD_VOLTAGE_DELAY_MS);
	}

	return -ETIMEDOUT;
}

static int qcom_battmgr_oneplus_pd_request_locked(
	struct qcom_battmgr *battmgr, u32 voltage_mv)
{
	if (voltage_mv != BATTMGR_ONEPLUS_FIXED_5V_MV &&
	    voltage_mv != BATTMGR_ONEPLUS_PD_FIXED_9V_MV)
		return -EINVAL;

	return qcom_battmgr_oneplus_property_set_locked(battmgr,
		BATTMGR_BAT_PROPERTY_SET, BATTMGR_ONEPLUS_BATT_SET_PDO, voltage_mv);
}

static int qcom_battmgr_oneplus_pd_source_check_locked(
	struct qcom_battmgr *battmgr, u32 generation)
{
	int ret;

	ret = qcom_battmgr_oneplus_request_property(battmgr,
		BATTMGR_USB_PROPERTY_GET, BATTMGR_ONEPLUS_USB_IN_STATUS, 0);
	if (ret)
		return ret;
	if (!(battmgr->oneplus_usb_in_status & 0xff) ||
	    generation != qcom_battmgr_caihong_source_generation())
		return -ENOTCONN;

	return 0;
}

static int qcom_battmgr_oneplus_pd_fixed_run_locked(
	struct qcom_battmgr *battmgr, u32 voltage_mv)
{
	u32 previous_generation;
	u32 generation;
	u32 saved_icl;
	u32 fallback_ua;
	u32 limit_ua;
	u32 index;
	u32 sample;
	unsigned long hold_start;
	int current_ma;
	int cleanup_rc;
	int request_rc;
	int ret;

	if (!caihong_pd_fixed_test || !battmgr->oneplus_charge_control ||
	    !battmgr->oneplus_pps_read_buffer)
		return -EOPNOTSUPP;
	if (battmgr->oneplus_dual_auto_run || battmgr->oneplus_dual_running ||
	    battmgr->oneplus_secondary_running || battmgr->oneplus_policy_running ||
	    battmgr->oneplus_pps_5m_running)
		return -EBUSY;
	ret = qcom_battmgr_oneplus_pd_battery_check_locked(battmgr, true);
	if (ret)
		return ret;
	ret = qcom_battmgr_caihong_update_usb_property(battmgr,
		POWER_SUPPLY_PROP_USB_TYPE);
	if (ret)
		return ret;
	if (battmgr->usb.usb_type != POWER_SUPPLY_USB_TYPE_PD &&
	    battmgr->usb.usb_type != POWER_SUPPLY_USB_TYPE_PD_PPS)
		return -EOPNOTSUPP;
	ret = qcom_battmgr_oneplus_request_property(battmgr,
		BATTMGR_USB_PROPERTY_GET, USB_ONLINE, 0);
	if (ret)
		return ret;
	if (!battmgr->usb.online)
		return -ENOTCONN;
	ret = qcom_battmgr_oneplus_request_property(battmgr,
		BATTMGR_USB_PROPERTY_GET, BATTMGR_ONEPLUS_USB_IN_STATUS, 0);
	if (ret)
		return ret;
	if (!(battmgr->oneplus_usb_in_status & 0xff))
		return -ENOTCONN;
	previous_generation = qcom_battmgr_caihong_source_generation();
	ret = qcom_battmgr_oneplus_pps_request(battmgr);
	if (ret)
		return ret;
	current_ma = qcom_battmgr_oneplus_pd_fixed_current(battmgr, voltage_mv);
	if (current_ma < 0)
		return current_ma;
	if (current_ma * 1000 < BATTMGR_ONEPLUS_PD_TRANSITION_UA)
		return -EOPNOTSUPP;
	ret = qcom_battmgr_oneplus_pd_fixed_current(battmgr,
		BATTMGR_ONEPLUS_FIXED_5V_MV);
	if (ret < BATTMGR_ONEPLUS_PD_TRANSITION_UA / 1000)
		return ret < 0 ? ret : -EOPNOTSUPP;
	fallback_ua = ret * 1000;
	if (battmgr->oneplus_dual_auto_enabled &&
	    qcom_battmgr_oneplus_pps_current_limit(battmgr,
		BATTMGR_ONEPLUS_DUAL_AUTO_PPS_READY_MV,
		BATTMGR_ONEPLUS_CP_DUAL_VENDOR_PPS_MAX_MA))
		return -EBUSY;
	ret = qcom_battmgr_oneplus_request_property(battmgr,
		BATTMGR_USB_PROPERTY_GET, USB_INPUT_CURR_LIMIT, 0);
	if (ret)
		return ret;
	saved_icl = battmgr->usb.current_limit;
	if (!saved_icl || saved_icl > INT_MAX)
		return -EBUSY;
	fallback_ua = min(saved_icl, fallback_ua);
	ret = qcom_battmgr_caihong_protocol_claim(CAIHONG_CHARGE_PD, &generation);
	if (ret)
		return ret;
	if (generation != previous_generation) {
		ret = -EAGAIN;
		goto release;
	}
	ret = qcom_battmgr_oneplus_require_cp_off(BATTMGR_ONEPLUS_PRIMARY_CP_NAME);
	if (!ret)
		ret = qcom_battmgr_oneplus_require_cp_off(BATTMGR_ONEPLUS_SECONDARY_CP_NAME);
	if (ret)
		goto release;

	limit_ua = min_t(u32, current_ma * 1000, BATTMGR_ONEPLUS_PD_TEST_MAX_UA);
	limit_ua = min(limit_ua, saved_icl);
	battmgr->oneplus_pd_test_valid = true;
	battmgr->oneplus_pd_test_request_mv = voltage_mv;
	battmgr->oneplus_pd_test_advertised_ma = current_ma;
	battmgr->oneplus_pd_test_limit_ua = limit_ua;
	battmgr->oneplus_pd_test_measured_uv = 0;
	battmgr->oneplus_pd_test_final_uv = 0;
	battmgr->oneplus_pd_test_sample_count = 0;
	battmgr->oneplus_pd_test_checks = 0;
	battmgr->oneplus_pd_test_elapsed_ms = 0;
	ret = qcom_battmgr_oneplus_property_set_locked(battmgr,
		BATTMGR_USB_PROPERTY_SET, USB_INPUT_CURR_LIMIT,
		min_t(u32, saved_icl, BATTMGR_ONEPLUS_PD_TRANSITION_UA));
	if (!ret)
		ret = qcom_battmgr_oneplus_pd_request_locked(battmgr, voltage_mv);
	if (!ret)
		ret = qcom_battmgr_oneplus_pd_voltage_check_locked(battmgr, voltage_mv,
			&battmgr->oneplus_pd_test_measured_uv, false);
	if (!ret)
		ret = qcom_battmgr_oneplus_pd_battery_check_locked(battmgr, false);
	if (!ret)
		ret = qcom_battmgr_oneplus_pd_source_check_locked(battmgr, generation);
	if (!ret)
		ret = qcom_battmgr_oneplus_property_set_locked(battmgr,
			BATTMGR_USB_PROPERTY_SET, USB_INPUT_CURR_LIMIT, limit_ua);
	hold_start = jiffies;
	for (index = 0; !ret && index < BATTMGR_ONEPLUS_PD_HOLD_CHECKS; index++) {
		msleep(BATTMGR_ONEPLUS_PD_HOLD_DELAY_MS);
		ret = qcom_battmgr_oneplus_pd_source_check_locked(battmgr, generation);
		if (ret)
			break;
		ret = qcom_battmgr_oneplus_pd_battery_check_locked(battmgr, false);
		if (!ret)
			ret = qcom_battmgr_oneplus_pd_voltage_check_locked(battmgr, voltage_mv,
				&battmgr->oneplus_pd_test_measured_uv, false);
		if (!ret)
			ret = qcom_battmgr_oneplus_request_property(battmgr,
				BATTMGR_USB_PROPERTY_GET, USB_CURR_NOW, 0);
		if (!ret)
			ret = qcom_battmgr_oneplus_request_property(battmgr,
				BATTMGR_BAT_PROPERTY_GET, BATT_CURR_NOW, 0);
		if (ret)
			break;
		battmgr->oneplus_pd_test_checks++;
		if (index % BATTMGR_ONEPLUS_PD_SAMPLE_INTERVAL)
			continue;
		sample = battmgr->oneplus_pd_test_sample_count;
		battmgr->oneplus_pd_test_samples[sample].elapsed_ms =
			jiffies_to_msecs(jiffies - hold_start);
		battmgr->oneplus_pd_test_samples[sample].battery_temp_decic =
			battmgr->status.temperature;
		battmgr->oneplus_pd_test_samples[sample].usb_voltage_uv =
			battmgr->oneplus_pd_test_measured_uv;
		battmgr->oneplus_pd_test_samples[sample].usb_current_ua =
			battmgr->usb.current_now;
		battmgr->oneplus_pd_test_samples[sample].battery_voltage_uv =
			battmgr->status.voltage_now;
		battmgr->oneplus_pd_test_samples[sample].battery_current_ua =
			qcom_battmgr_oneplus_battery_current(battmgr, battmgr->status.current_now);
		battmgr->oneplus_pd_test_sample_count++;
	}
	battmgr->oneplus_pd_test_elapsed_ms = jiffies_to_msecs(jiffies - hold_start);

	cleanup_rc = qcom_battmgr_oneplus_property_set_locked(battmgr,
		BATTMGR_USB_PROPERTY_SET, USB_INPUT_CURR_LIMIT,
		min_t(u32, saved_icl, BATTMGR_ONEPLUS_PD_TRANSITION_UA));
	request_rc = qcom_battmgr_oneplus_pd_request_locked(battmgr,
		BATTMGR_ONEPLUS_FIXED_5V_MV);
	if (!cleanup_rc)
		cleanup_rc = request_rc;
	request_rc = qcom_battmgr_oneplus_pd_voltage_check_locked(battmgr,
		BATTMGR_ONEPLUS_FIXED_5V_MV, &battmgr->oneplus_pd_test_final_uv, true);
	if (!cleanup_rc)
		cleanup_rc = request_rc;
	if (!cleanup_rc)
		cleanup_rc = qcom_battmgr_oneplus_property_set_locked(battmgr,
			BATTMGR_USB_PROPERTY_SET, USB_INPUT_CURR_LIMIT, fallback_ua);
	if (cleanup_rc)
		qcom_battmgr_oneplus_property_set_locked(battmgr,
			BATTMGR_USB_PROPERTY_SET, USB_INPUT_CURR_LIMIT, 0);
	battmgr->oneplus_pd_test_result = ret;
	battmgr->oneplus_pd_test_cleanup_rc = cleanup_rc;
	qcom_battmgr_caihong_protocol_release(CAIHONG_CHARGE_PD, generation,
		false, !cleanup_rc);
	return ret ? ret : cleanup_rc;

release:
	qcom_battmgr_caihong_protocol_release(CAIHONG_CHARGE_PD, generation, false, true);
	return ret;
}

static ssize_t oneplus_pd_fixed_test_show(struct device *dev,
					struct device_attribute *attr, char *buf)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	u32 index;
	ssize_t ret;

	mutex_lock(&battmgr->oneplus_lock);
	ret = sysfs_emit(buf,
		"valid=%u requested_mv=%u advertised_ma=%u limit_ua=%u measured_uv=%u final_uv=%u rc=%d cleanup_rc=%d samples=%u checks=%u elapsed_ms=%u\n",
		battmgr->oneplus_pd_test_valid, battmgr->oneplus_pd_test_request_mv,
		battmgr->oneplus_pd_test_advertised_ma, battmgr->oneplus_pd_test_limit_ua,
		battmgr->oneplus_pd_test_measured_uv, battmgr->oneplus_pd_test_final_uv,
		battmgr->oneplus_pd_test_result, battmgr->oneplus_pd_test_cleanup_rc,
		battmgr->oneplus_pd_test_sample_count, battmgr->oneplus_pd_test_checks,
		battmgr->oneplus_pd_test_elapsed_ms);
	for (index = 0; index < battmgr->oneplus_pd_test_sample_count; index++)
		ret += sysfs_emit_at(buf, ret,
			"sample=%u usb_uv=%u usb_ua=%u battery_uv=%u battery_ua=%d elapsed_ms=%u temp_decic=%d\n",
			index, battmgr->oneplus_pd_test_samples[index].usb_voltage_uv,
			battmgr->oneplus_pd_test_samples[index].usb_current_ua,
			battmgr->oneplus_pd_test_samples[index].battery_voltage_uv,
			battmgr->oneplus_pd_test_samples[index].battery_current_ua,
			battmgr->oneplus_pd_test_samples[index].elapsed_ms,
			battmgr->oneplus_pd_test_samples[index].battery_temp_decic);
	mutex_unlock(&battmgr->oneplus_lock);

	return ret;
}

static ssize_t oneplus_pd_fixed_test_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	u32 voltage_mv;
	int ret;

	ret = kstrtou32(buf, 10, &voltage_mv);
	if (ret)
		return ret;
	if (voltage_mv != BATTMGR_ONEPLUS_FIXED_5V_MV &&
	    voltage_mv != BATTMGR_ONEPLUS_PD_FIXED_9V_MV)
		return -EINVAL;
	if (!mutex_trylock(&battmgr->oneplus_lock))
		return -EBUSY;
	battmgr->oneplus_pd_test_valid = false;
	battmgr->oneplus_pd_test_sample_count = 0;
	battmgr->oneplus_pd_test_checks = 0;
	battmgr->oneplus_pd_test_elapsed_ms = 0;
	battmgr->oneplus_pd_test_request_mv = voltage_mv;
	ret = qcom_battmgr_oneplus_pd_fixed_run_locked(battmgr, voltage_mv);
	battmgr->oneplus_pd_test_result = ret;
	mutex_unlock(&battmgr->oneplus_lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(oneplus_pd_fixed_test);

static ssize_t oneplus_pd_test_pps_auto_show(struct device *dev,
					  struct device_attribute *attr, char *buf)
{
	return oneplus_pps_dual_auto_once_show(dev, attr, buf);
}

static ssize_t oneplus_pd_test_pps_auto_store(struct device *dev,
					   struct device_attribute *attr,
					   const char *buf, size_t count)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	bool enable;
	int ret;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;
	return qcom_battmgr_oneplus_dual_auto_store(battmgr, enable, true, count);
}
static DEVICE_ATTR_RW(oneplus_pd_test_pps_auto);

static struct attribute *qcom_battmgr_oneplus_pd_fixed_test_attrs[] = {
	&dev_attr_oneplus_pd_fixed_test.attr,
	&dev_attr_oneplus_pd_test_pps_auto.attr,
	NULL,
};

static const struct attribute_group qcom_battmgr_oneplus_pd_fixed_test_group = {
	.attrs = qcom_battmgr_oneplus_pd_fixed_test_attrs,
};
