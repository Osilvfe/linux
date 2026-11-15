#include <linux/spinlock.h>
#include <linux/wait.h>

static DEFINE_SPINLOCK(caihong_protocol_lock);
static DECLARE_WAIT_QUEUE_HEAD(caihong_protocol_wait);
static enum caihong_charge_protocol caihong_protocol_owner;
static u32 caihong_protocol_generation = 1;
static u32 caihong_protocol_owner_generation;
static bool caihong_vooc_pending = true;
static bool caihong_protocol_fault;
static bool caihong_source_known;
static bool caihong_source_present;
static bool caihong_pd_svooc;
static bool caihong_source_otg;

static void caihong_protocol_input(bool present)
{
	unsigned long flags;

	spin_lock_irqsave(&caihong_protocol_lock, flags);
	if ((caihong_source_known && present != caihong_source_present) ||
	    (!present && caihong_pd_svooc)) {
		caihong_protocol_generation++;
		caihong_vooc_pending = true;
	}
	caihong_source_known = true;
	caihong_source_present = present;
	if (!present)
		caihong_pd_svooc = false;
	spin_unlock_irqrestore(&caihong_protocol_lock, flags);
	wake_up_all(&caihong_protocol_wait);
}

static void caihong_protocol_reset(bool otg)
{
	unsigned long flags;

	spin_lock_irqsave(&caihong_protocol_lock, flags);
	caihong_protocol_generation++;
	caihong_source_known = false;
	caihong_source_present = false;
	caihong_pd_svooc = false;
	caihong_source_otg = otg;
	caihong_vooc_pending = true;
	spin_unlock_irqrestore(&caihong_protocol_lock, flags);
	wake_up_all(&caihong_protocol_wait);
}

static void caihong_protocol_pd_svooc(void)
{
	unsigned long flags;

	spin_lock_irqsave(&caihong_protocol_lock, flags);
	if (!caihong_source_otg) {
		if (!caihong_pd_svooc && !caihong_vooc_pending &&
		    caihong_protocol_owner == CAIHONG_CHARGE_IDLE) {
			caihong_protocol_generation++;
			caihong_vooc_pending = true;
		}
		caihong_pd_svooc = true;
	}
	spin_unlock_irqrestore(&caihong_protocol_lock, flags);
	wake_up_all(&caihong_protocol_wait);
}

static int caihong_protocol_pd_state(u32 generation, bool finish)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&caihong_protocol_lock, flags);
	if (generation != caihong_protocol_generation)
		ret = -ESTALE;
	else if (caihong_protocol_fault)
		ret = -EIO;
	else if (caihong_source_otg ||
		 (caihong_source_known && !caihong_source_present))
		ret = -ENODEV;
	else if (caihong_pd_svooc)
		ret = 0;
	else if (finish) {
		if (caihong_protocol_owner != CAIHONG_CHARGE_VOOC)
			caihong_vooc_pending = false;
		ret = -EOPNOTSUPP;
	} else
		ret = caihong_vooc_pending ? -EAGAIN : -EOPNOTSUPP;
	spin_unlock_irqrestore(&caihong_protocol_lock, flags);

	return ret;
}

int qcom_battmgr_caihong_vooc_pd_allowed(void)
{
	u32 generation = qcom_battmgr_caihong_source_generation();
	int ret;

	ret = caihong_protocol_pd_state(generation, false);
	if (ret != -EAGAIN)
		return ret;
	wait_event_timeout(caihong_protocol_wait,
		caihong_protocol_pd_state(generation, false) != -EAGAIN,
		msecs_to_jiffies(BATTMGR_ONEPLUS_PD_SVOOC_WAIT_MS));
	return caihong_protocol_pd_state(generation, true);
}
EXPORT_SYMBOL_GPL(qcom_battmgr_caihong_vooc_pd_allowed);

u32 qcom_battmgr_caihong_source_generation(void)
{
	u32 generation;
	unsigned long flags;

	spin_lock_irqsave(&caihong_protocol_lock, flags);
	generation = caihong_protocol_generation;
	spin_unlock_irqrestore(&caihong_protocol_lock, flags);

	return generation;
}
EXPORT_SYMBOL_GPL(qcom_battmgr_caihong_source_generation);

u32 qcom_battmgr_caihong_source_changed(u32 previous_generation)
{
	unsigned long flags;
	u32 generation;

	spin_lock_irqsave(&caihong_protocol_lock, flags);
	if (previous_generation == caihong_protocol_generation) {
		caihong_protocol_generation++;
		caihong_vooc_pending = true;
	}
	generation = caihong_protocol_generation;
	spin_unlock_irqrestore(&caihong_protocol_lock, flags);
	wake_up_all(&caihong_protocol_wait);

	return generation;
}
EXPORT_SYMBOL_GPL(qcom_battmgr_caihong_source_changed);

bool qcom_battmgr_caihong_pps_allowed(void)
{
	unsigned long flags;
	bool allowed;

	spin_lock_irqsave(&caihong_protocol_lock, flags);
	allowed = !caihong_protocol_fault && !caihong_vooc_pending &&
		  !caihong_source_otg &&
		  (!caihong_source_known || caihong_source_present) &&
		  caihong_protocol_owner == CAIHONG_CHARGE_IDLE;
	spin_unlock_irqrestore(&caihong_protocol_lock, flags);

	return allowed;
}
EXPORT_SYMBOL_GPL(qcom_battmgr_caihong_pps_allowed);

void qcom_battmgr_caihong_vooc_skip(u32 generation)
{
	unsigned long flags;

	spin_lock_irqsave(&caihong_protocol_lock, flags);
	if (generation == caihong_protocol_generation &&
	    caihong_protocol_owner != CAIHONG_CHARGE_VOOC)
		caihong_vooc_pending = false;
	spin_unlock_irqrestore(&caihong_protocol_lock, flags);
}
EXPORT_SYMBOL_GPL(qcom_battmgr_caihong_vooc_skip);

int qcom_battmgr_caihong_protocol_claim(enum caihong_charge_protocol protocol,
				       u32 *generation)
{
	unsigned long flags;
	int ret = 0;

	if (protocol != CAIHONG_CHARGE_VOOC && protocol != CAIHONG_CHARGE_PPS &&
	    protocol != CAIHONG_CHARGE_PD)
		return -EINVAL;

	spin_lock_irqsave(&caihong_protocol_lock, flags);
	if (caihong_protocol_fault)
		ret = -EIO;
	else if (caihong_source_otg ||
		 (caihong_source_known && !caihong_source_present))
		ret = -ENODEV;
	else if (caihong_protocol_owner != CAIHONG_CHARGE_IDLE)
		ret = -EBUSY;
	else if (protocol != CAIHONG_CHARGE_VOOC && caihong_vooc_pending)
		ret = -EAGAIN;
	else if (protocol == CAIHONG_CHARGE_VOOC && !caihong_vooc_pending)
		ret = -EALREADY;
	else {
		caihong_protocol_owner = protocol;
		caihong_protocol_owner_generation = caihong_protocol_generation;
		*generation = caihong_protocol_generation;
	}
	spin_unlock_irqrestore(&caihong_protocol_lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(qcom_battmgr_caihong_protocol_claim);

void qcom_battmgr_caihong_protocol_release(enum caihong_charge_protocol protocol,
					  u32 generation, bool retry, bool safe)
{
	unsigned long flags;

	spin_lock_irqsave(&caihong_protocol_lock, flags);
	if (caihong_protocol_owner == protocol &&
	    caihong_protocol_owner_generation == generation) {
		caihong_protocol_owner = CAIHONG_CHARGE_IDLE;
		if (!safe)
			caihong_protocol_fault = true;
		if (protocol == CAIHONG_CHARGE_VOOC &&
		    generation == caihong_protocol_generation)
			caihong_vooc_pending = retry || !safe;
	}
	spin_unlock_irqrestore(&caihong_protocol_lock, flags);
}
EXPORT_SYMBOL_GPL(qcom_battmgr_caihong_protocol_release);

static ssize_t oneplus_fastchg_status_show(struct device *dev,
					 struct device_attribute *attr, char *buf)
{
	struct qcom_battmgr *battmgr = dev_get_drvdata(dev);
	enum caihong_charge_protocol owner;
	unsigned long flags;
	u32 generation;
	bool pending;
	bool fault;
	bool pd_svooc;
	bool present;
	bool known;

	spin_lock_irqsave(&caihong_protocol_lock, flags);
	owner = caihong_protocol_owner;
	generation = caihong_protocol_generation;
	pending = caihong_vooc_pending;
	fault = caihong_protocol_fault;
	pd_svooc = caihong_pd_svooc;
	present = caihong_source_present;
	known = caihong_source_known;
	spin_unlock_irqrestore(&caihong_protocol_lock, flags);

	return sysfs_emit(buf,
		"owner=%s generation=%u vooc_pending=%u fault=%u "
		"pd_svooc=%u input_known=%u input_present=%u "
		"pps_cap_known=%u pps_available=%u armed=%u auto_run=%u last_rc=%d\n",
		owner == CAIHONG_CHARGE_VOOC ? "vooc" :
		owner == CAIHONG_CHARGE_PPS ? "pps" :
		owner == CAIHONG_CHARGE_PD ? "pd" : "idle",
		generation, pending, fault, pd_svooc, known, present,
		READ_ONCE(battmgr->oneplus_dual_auto_cap_known),
		READ_ONCE(battmgr->oneplus_dual_auto_pps_available),
		READ_ONCE(battmgr->oneplus_dual_auto_armed),
		READ_ONCE(battmgr->oneplus_dual_auto_run),
		READ_ONCE(battmgr->oneplus_dual_auto_last_rc));
}
static DEVICE_ATTR_RO(oneplus_fastchg_status);
