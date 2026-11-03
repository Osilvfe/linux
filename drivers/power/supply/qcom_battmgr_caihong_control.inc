#define CAIHONG_CHARGE_CHECK_MS 5000
#define CAIHONG_CHARGE_RETRY_MS 250

static bool qcom_battmgr_caihong_charge_latched(unsigned int start,
					      unsigned int end,
					      unsigned int capacity, bool latched)
{
	if (end == 100)
		return false;
	if (capacity >= end)
		return true;
	if (capacity <= start)
		return false;
	return latched;
}

bool qcom_battmgr_caihong_charge_is_inhibited(struct power_supply *psy)
{
	struct qcom_battmgr *battmgr;

	if (!psy || strcmp(psy->desc->name, "qcom-battmgr-bat"))
		return false;
	battmgr = power_supply_get_drvdata(psy);
	return battmgr && READ_ONCE(battmgr->oneplus_charge_inhibit);
}
EXPORT_SYMBOL_GPL(qcom_battmgr_caihong_charge_is_inhibited);

static bool qcom_battmgr_caihong_charge_requested(struct qcom_battmgr *battmgr)
{
	return battmgr->oneplus_charge_behaviour ==
		POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE ||
		battmgr->oneplus_charge_suspended || battmgr->oneplus_charge_latched;
}

static int qcom_battmgr_caihong_charge_apply_locked(struct qcom_battmgr *battmgr)
{
	static const char * const pumps[] = {
		BATTMGR_ONEPLUS_PRIMARY_CP_NAME,
		BATTMGR_ONEPLUS_SECONDARY_CP_NAME,
	};
	bool inhibit = qcom_battmgr_caihong_charge_requested(battmgr);
	bool active = false;
	bool enable;
	unsigned int index;
	int ret;

	WRITE_ONCE(battmgr->oneplus_charge_inhibit, inhibit);
	if (!battmgr->service_up)
		return -ENOTCONN;
	if (inhibit) {
		for (index = 0; index < ARRAY_SIZE(pumps); index++) {
			struct power_supply *pump = power_supply_get_by_name(pumps[index]);
			union power_supply_propval value;

			if (!pump)
				continue;
			ret = qcom_battmgr_oneplus_cancel_cp(pump);
			if (!ret)
				ret = power_supply_get_property(pump, POWER_SUPPLY_PROP_ONLINE,
							&value);
			power_supply_put(pump);
			if (ret)
				return ret;
			active |= value.intval != 0;
		}
		if (active || battmgr->oneplus_dual_running ||
		    battmgr->oneplus_policy_running || battmgr->oneplus_pps_5m_running ||
		    battmgr->oneplus_secondary_running)
			return -EAGAIN;
	}
	if (inhibit == battmgr->oneplus_charge_applied &&
	    !battmgr->oneplus_charge_dirty && !battmgr->oneplus_charge_verify_pending) {
		if (!inhibit)
			return 0;
		battmgr->oneplus_ffc_charge_enabled_valid = false;
		ret = qcom_battmgr_oneplus_request_property(battmgr,
			BATTMGR_BAT_PROPERTY_GET, BATT_CHG_CTRL_EN, 0);
		if (ret)
			return ret;
		if (!battmgr->oneplus_ffc_charge_enabled_valid)
			return -EIO;
		if (!battmgr->oneplus_ffc_charge_enabled)
			return 0;
	}
	if (!inhibit && !battmgr->oneplus_charge_applied &&
	    !battmgr->oneplus_charge_verify_pending) {
		battmgr->oneplus_charge_dirty = false;
		return 0;
	}
	enable = !inhibit && !battmgr->oneplus_ffc_full;
	battmgr->oneplus_charge_verify_pending = true;
	ret = qcom_battmgr_oneplus_property_set_locked(battmgr,
		BATTMGR_BAT_PROPERTY_SET, BATT_CHG_CTRL_EN, enable);
	if (ret)
		return ret;
	battmgr->oneplus_charge_applied = inhibit;
	battmgr->oneplus_charge_verify_pending = true;
	battmgr->oneplus_ffc_charge_enabled_valid = false;
	ret = qcom_battmgr_oneplus_request_property(battmgr,
		BATTMGR_BAT_PROPERTY_GET, BATT_CHG_CTRL_EN, 0);
	if (ret)
		return ret;
	if (!battmgr->oneplus_ffc_charge_enabled_valid ||
	    battmgr->oneplus_ffc_charge_enabled != enable)
		return -EIO;
	battmgr->oneplus_charge_applied = inhibit;
	battmgr->oneplus_charge_verify_pending = false;
	battmgr->oneplus_charge_dirty = false;
	return 0;
}

static void qcom_battmgr_caihong_charge_worker(struct work_struct *work)
{
	struct qcom_battmgr *battmgr = container_of(to_delayed_work(work),
		struct qcom_battmgr, oneplus_charge_work);
	unsigned int delay = CAIHONG_CHARGE_CHECK_MS;
	bool previous;
	bool was_applied;
	bool changed;
	bool reschedule;
	int ret;

	if (!READ_ONCE(battmgr->oneplus_charge_control))
		return;
	if (READ_ONCE(battmgr->oneplus_charge_inhibit)) {
		struct power_supply *pump;

		pump = power_supply_get_by_name(BATTMGR_ONEPLUS_PRIMARY_CP_NAME);
		if (pump) {
			qcom_battmgr_oneplus_cancel_cp(pump);
			power_supply_put(pump);
		}
		pump = power_supply_get_by_name(BATTMGR_ONEPLUS_SECONDARY_CP_NAME);
		if (pump) {
			qcom_battmgr_oneplus_cancel_cp(pump);
			power_supply_put(pump);
		}
	}
	mutex_lock(&battmgr->oneplus_charge_lock);
	if (!mutex_trylock(&battmgr->oneplus_lock)) {
		mutex_unlock(&battmgr->oneplus_charge_lock);
		mod_delayed_work(system_freezable_wq, &battmgr->oneplus_charge_work,
				 msecs_to_jiffies(CAIHONG_CHARGE_RETRY_MS));
		return;
	}
	previous = battmgr->oneplus_charge_inhibit;
	was_applied = battmgr->oneplus_charge_applied;
	if (battmgr->info.charge_ctrl_end < 100) {
		ret = qcom_battmgr_oneplus_request_property(battmgr,
			BATTMGR_BAT_PROPERTY_GET, BATT_CAPACITY, 0);
		if (ret || battmgr->status.percent > 100)
			battmgr->oneplus_charge_latched = true;
		else
			battmgr->oneplus_charge_latched = qcom_battmgr_caihong_charge_latched(
				battmgr->info.charge_ctrl_start, battmgr->info.charge_ctrl_end,
				battmgr->status.percent, battmgr->oneplus_charge_latched);
	}
	ret = qcom_battmgr_caihong_charge_apply_locked(battmgr);
	if (!ret && was_applied && !battmgr->oneplus_charge_inhibit) {
		battmgr->oneplus_dual_auto_armed = true;
		battmgr->oneplus_dual_auto_connected = false;
		battmgr->oneplus_policy_auto_armed = true;
		if (battmgr->oneplus_ffc_policy)
			mod_delayed_work(system_long_wq, &battmgr->oneplus_ffc_work, 0);
	}
	battmgr->oneplus_charge_last_rc = ret;
	changed = previous != battmgr->oneplus_charge_inhibit;
	reschedule = ret || battmgr->info.charge_ctrl_end < 100 ||
		battmgr->oneplus_charge_inhibit || battmgr->oneplus_charge_applied;
	if (ret)
		delay = ret == -EAGAIN ? CAIHONG_CHARGE_RETRY_MS : 5000;
	mutex_unlock(&battmgr->oneplus_lock);
	mutex_unlock(&battmgr->oneplus_charge_lock);
	if (changed)
		power_supply_changed(battmgr->bat_psy);
	if (reschedule && READ_ONCE(battmgr->oneplus_charge_control))
		mod_delayed_work(system_freezable_wq, &battmgr->oneplus_charge_work,
				 msecs_to_jiffies(delay));
}

static int qcom_battmgr_caihong_charge_get_property(
	struct qcom_battmgr *battmgr, enum power_supply_property psp,
	union power_supply_propval *val)
{
	if (!battmgr->oneplus_charge_control)
		return -EOPNOTSUPP;
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		val->intval = READ_ONCE(battmgr->info.charge_ctrl_start);
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		val->intval = READ_ONCE(battmgr->info.charge_ctrl_end);
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		val->intval = READ_ONCE(battmgr->oneplus_charge_behaviour);
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

static int qcom_battmgr_caihong_charge_set_property(
	struct qcom_battmgr *battmgr, enum power_supply_property psp, int value)
{
	int start;
	int end;
	int ret = 0;

	if (!battmgr->oneplus_charge_control)
		return -EOPNOTSUPP;
	if (psp != POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD &&
	    psp != POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD &&
	    psp != POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR)
		return -EOPNOTSUPP;
	mutex_lock(&battmgr->oneplus_charge_lock);
	start = battmgr->info.charge_ctrl_start;
	end = battmgr->info.charge_ctrl_end;
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_START_THRESHOLD:
		if (value < 0 || value >= end) {
			ret = -EINVAL;
			break;
		}
		start = value;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		if (value <= start || value > 100) {
			ret = -EINVAL;
			break;
		}
		end = value;
		break;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		if (value != POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO &&
		    value != POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE) {
			ret = -EINVAL;
			break;
		}
		battmgr->oneplus_charge_behaviour = value;
		break;
	default:
		break;
	}
	if (!ret) {
		battmgr->oneplus_charge_last_rc = -EINPROGRESS;
		battmgr->info.charge_ctrl_start = start;
		battmgr->info.charge_ctrl_end = end;
		battmgr->oneplus_charge_latched = qcom_battmgr_caihong_charge_latched(
			start, end, READ_ONCE(battmgr->status.percent),
			battmgr->oneplus_charge_latched);
		WRITE_ONCE(battmgr->oneplus_charge_inhibit,
			   qcom_battmgr_caihong_charge_requested(battmgr));
		battmgr->oneplus_charge_dirty = true;
	}
	mutex_unlock(&battmgr->oneplus_charge_lock);
	if (!ret) {
		power_supply_changed(battmgr->bat_psy);
		mod_delayed_work(system_freezable_wq, &battmgr->oneplus_charge_work, 0);
	}
	return ret;
}

static int qcom_battmgr_caihong_battery_get_property(struct power_supply *psy,
	enum power_supply_property psp, union power_supply_propval *val)
{
	struct qcom_battmgr *battmgr = power_supply_get_drvdata(psy);
	int ret;

	if (!READ_ONCE(battmgr->oneplus_charge_control))
		return -ENODEV;
	ret = qcom_battmgr_caihong_charge_get_property(battmgr, psp, val);
	if (ret != -EOPNOTSUPP)
		return ret;
	ret = battmgr->oneplus_battery_base_desc->get_property(psy, psp, val);
	if (!ret && psp == POWER_SUPPLY_PROP_STATE_OF_HEALTH &&
	    (val->intval < 0 || val->intval > 100))
		return -ENODATA;
	return ret;
}

static int qcom_battmgr_caihong_battery_set_property(struct power_supply *psy,
	enum power_supply_property psp, const union power_supply_propval *val)
{
	struct qcom_battmgr *battmgr = power_supply_get_drvdata(psy);
	int ret;

	if (!READ_ONCE(battmgr->oneplus_charge_control))
		return -ENODEV;
	ret = qcom_battmgr_caihong_charge_set_property(battmgr, psp, val->intval);
	if (ret != -EOPNOTSUPP)
		return ret;
	if (!battmgr->oneplus_battery_base_desc->set_property)
		return -EOPNOTSUPP;
	return battmgr->oneplus_battery_base_desc->set_property(psy, psp, val);
}

static const struct power_supply_desc *
qcom_battmgr_caihong_battery_desc(struct qcom_battmgr *battmgr,
				const struct power_supply_desc *desc)
{
	if (!battmgr->oneplus_charge_control || desc == &battmgr->oneplus_battery_desc)
		return desc;
	battmgr->oneplus_battery_base_desc = desc;
	battmgr->oneplus_battery_desc = *desc;
	battmgr->oneplus_battery_desc.get_property = qcom_battmgr_caihong_battery_get_property;
	battmgr->oneplus_battery_desc.set_property = qcom_battmgr_caihong_battery_set_property;
	return &battmgr->oneplus_battery_desc;
}

static int qcom_battmgr_caihong_behaviour_get(struct power_supply *psy,
	const struct power_supply_ext *ext, void *data,
	enum power_supply_property psp, union power_supply_propval *val)
{
	return qcom_battmgr_caihong_charge_get_property(data, psp, val);
}

static int qcom_battmgr_caihong_behaviour_set(struct power_supply *psy,
	const struct power_supply_ext *ext, void *data,
	enum power_supply_property psp, const union power_supply_propval *val)
{
	return qcom_battmgr_caihong_charge_set_property(data, psp, val->intval);
}

static int qcom_battmgr_caihong_behaviour_writeable(struct power_supply *psy,
	const struct power_supply_ext *ext, void *data, enum power_supply_property psp)
{
	return psp == POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR;
}

static const enum power_supply_property qcom_battmgr_caihong_charge_props[] = {
	POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
};

static const struct power_supply_ext qcom_battmgr_caihong_charge_ext = {
	.name = "caihong-charge-control",
	.charge_behaviours = BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) |
		BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE),
	.properties = qcom_battmgr_caihong_charge_props,
	.num_properties = ARRAY_SIZE(qcom_battmgr_caihong_charge_props),
	.get_property = qcom_battmgr_caihong_behaviour_get,
	.set_property = qcom_battmgr_caihong_behaviour_set,
	.property_is_writeable = qcom_battmgr_caihong_behaviour_writeable,
};

static ssize_t oneplus_charge_control_status_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);

	return sysfs_emit(buf, "inhibited=%u applied=%u pending=%u latched=%u suspended=%u raw_soc=%u last_rc=%d\n",
		READ_ONCE(battmgr->oneplus_charge_inhibit),
		READ_ONCE(battmgr->oneplus_charge_applied),
		READ_ONCE(battmgr->oneplus_charge_verify_pending),
		READ_ONCE(battmgr->oneplus_charge_latched),
		READ_ONCE(battmgr->oneplus_charge_suspended),
		READ_ONCE(battmgr->status.percent),
		READ_ONCE(battmgr->oneplus_charge_last_rc));
}
static DEVICE_ATTR_RO(oneplus_charge_control_status);

static struct attribute *qcom_battmgr_caihong_charge_attrs[] = {
	&dev_attr_oneplus_charge_control_status.attr,
	NULL,
};

static const struct attribute_group qcom_battmgr_caihong_charge_group = {
	.attrs = qcom_battmgr_caihong_charge_attrs,
};

static int qcom_battmgr_caihong_charge_suspend(struct qcom_battmgr *battmgr)
{
	int ret;

	if (!battmgr->oneplus_charge_control ||
	    (READ_ONCE(battmgr->info.charge_ctrl_end) == 100 &&
	     !READ_ONCE(battmgr->oneplus_charge_behaviour)))
		return 0;
	mutex_lock(&battmgr->oneplus_charge_lock);
	if (!mutex_trylock(&battmgr->oneplus_lock)) {
		mutex_unlock(&battmgr->oneplus_charge_lock);
		return -EBUSY;
	}
	battmgr->oneplus_charge_suspended = true;
	ret = qcom_battmgr_caihong_charge_apply_locked(battmgr);
	battmgr->oneplus_charge_last_rc = ret;
	if (ret)
		battmgr->oneplus_charge_suspended = false;
	mutex_unlock(&battmgr->oneplus_lock);
	mutex_unlock(&battmgr->oneplus_charge_lock);
	if (ret)
		mod_delayed_work(system_freezable_wq, &battmgr->oneplus_charge_work, 0);
	return ret;
}

static int qcom_battmgr_caihong_charge_resume(struct qcom_battmgr *battmgr)
{
	if (battmgr->oneplus_charge_control) {
		WRITE_ONCE(battmgr->oneplus_charge_suspended, false);
		mod_delayed_work(system_freezable_wq, &battmgr->oneplus_charge_work, 0);
	}
	return 0;
}

static int qcom_battmgr_caihong_charge_pm_event(struct notifier_block *nb,
					     unsigned long action, void *unused)
{
	struct qcom_battmgr *battmgr = container_of(nb, struct qcom_battmgr,
						 oneplus_charge_pm_nb);

	switch (action) {
	case PM_SUSPEND_PREPARE:
	case PM_HIBERNATION_PREPARE:
	case PM_RESTORE_PREPARE:
		return notifier_from_errno(qcom_battmgr_caihong_charge_suspend(battmgr));
	case PM_POST_SUSPEND:
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:
		qcom_battmgr_caihong_charge_resume(battmgr);
		return NOTIFY_OK;
	default:
		return NOTIFY_DONE;
	}
}

static int qcom_battmgr_caihong_charge_register(struct qcom_battmgr *battmgr)
{
	int ret;

	ret = devm_device_add_group(battmgr->dev, &qcom_battmgr_caihong_charge_group);
	if (ret)
		goto disable;
	ret = power_supply_register_extension(battmgr->bat_psy,
		&qcom_battmgr_caihong_charge_ext, battmgr->dev, battmgr);
	if (ret)
		goto disable;
	battmgr->oneplus_charge_pm_nb.notifier_call = qcom_battmgr_caihong_charge_pm_event;
	ret = register_pm_notifier(&battmgr->oneplus_charge_pm_nb);
	if (ret) {
		power_supply_unregister_extension(battmgr->bat_psy,
						 &qcom_battmgr_caihong_charge_ext);
		goto disable;
	}
	battmgr->oneplus_charge_registered = true;
	return 0;

disable:
	WRITE_ONCE(battmgr->oneplus_charge_control, false);
	cancel_delayed_work_sync(&battmgr->oneplus_charge_work);
	return ret;
}

static void qcom_battmgr_caihong_charge_unregister(void *data)
{
	struct qcom_battmgr *battmgr = data;

	WRITE_ONCE(battmgr->oneplus_charge_control, false);
	if (battmgr->oneplus_charge_registered)
		unregister_pm_notifier(&battmgr->oneplus_charge_pm_nb);
	cancel_delayed_work_sync(&battmgr->oneplus_charge_work);
	if (battmgr->oneplus_charge_registered) {
		power_supply_unregister_extension(battmgr->bat_psy,
						 &qcom_battmgr_caihong_charge_ext);
		battmgr->oneplus_charge_registered = false;
	}
}
