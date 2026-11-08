.. SPDX-License-Identifier: GPL-2.0-only

Caihong experimental higher-current PPS profile
==============================================

This profile is opt-in and applies only to the OnePlus Pad Pro
(``oneplus,caihong``), with the existing OnePlus firmware ABI enabled, during
an automatic continuous dual-pump PPS session.  The default policy remains
unchanged.  Use this opt-in only with a known suitable PPS source and a
5 A-rated electronically marked cable, as in the historical device test.
Linux does not independently verify the cable rating or measure its total
current through this interface; the firmware owns PD contract negotiation.  It does not enable higher current for fixed-ratio PPS diagnostics,
independent bounded pump tests, VOOC, other boards, or ordinary fixed PD.

Enable the read-only-at-runtime parameter at boot::

    qcom_battmgr.caihong_pps_high_current=1

For a modular battmgr driver, the corresponding module option is
``caihong_pps_high_current=1``.  Remove the option and reboot to use the
original profile.  The parameter is mode 0444 and cannot change an active
session.  There is no additional writable charging sysfs interface.

The battmgr coordinator publishes a stable, pointer-free active-session gate.
Each vendor-current continuous SC8547 run snapshots that gate; it is cleared
only after the existing simultaneous pump cancellation and worker completion.
The exported function returns false when the Caihong driver is not reachable.

Profile and limits
------------------

Under 20 to less than 38 degrees C, the aggregate charge-pump input target is
at most 4.5 A for raw SOC below 75 percent, 3 A from 75 to 84 percent, and 2 A
from 85 percent.  Below 20 degrees C or at and above 40 degrees C, it is at most
2 A; from 38 to below 40 degrees C, it is at most 3 A.  The existing system
voltage and temperature admissibility checks still run.  The target ceiling
can only decrease within one continuous session.

The source must advertise the requested PPS voltage and current.  The pump
budget subtracts the latest measured ordinary buck input current, with a
500 mA minimum reserve, from the selected APDO current.  Unknown, zero or
implausible buck telemetry rejects this experimental operation.  Less than
1.8 A of remaining pump budget also rejects it.  A fresh system/buck sample
is required after pump startup and before the first target selection, and
again on each experimental boost iteration.  During hold, system samples
retain the existing two-second cadence; the current target uses the latest
such sample.  These asynchronous measurements and the minimum reserve are
not a calibrated measurement or guarantee of total cable current.

The profile widens the following software limits, solely in this active
experimental path:

========================== ============== ==============
Limit                      Default        Experimental
========================== ============== ==============
Aggregate boost target     2 A            up to 4.5 A
Per-pump normal IBUS        1.45 A         2.65 A
Per-pump soft IBUS          1.35 A         2.45 A
Per-pump emergency IBUS     1.65 A         2.85 A
Path-delta soft taper       150 mV         175 mV
Loaded source request max  9.4 V          10 V
Boost steps, 40 mV each     10             15
Balanced handoff minimum   1.9 A          1.8 A
========================== ============== ==============

The 10 V value is the negotiated source request, not measured pump VBUS.  The
SC8547 measured VBUS ceiling stays 9.4 V, and the existing entry/pre-start
window and loaded-target margin remain unchanged.  The 200 mV path hard stop,
cell/output voltage limits, gauge-voltage CV feedback, temperature and full
checks, source-capability checks, hardware register profiles, charging
inhibition and cancellation are retained.  An additional experimental-only
10 A charging battery-current stop is applied.  The mode therefore must not
be described as preserving every software protection threshold unchanged.

A five-second startup grace followed by four consecutive samples below
100 mA exits an experimental continuous pump run through its existing
normal disable/readback/restore path.  This handles residual pump-side
voltage and delayed firmware USB-online reporting after cable removal.
It does not change the default run's termination rules.

``oneplus_pps_dual_500ms`` status reports ``experimental_profile`` and the
last ``effective_target_ua``.  Existing limit fields above that line describe
the default profile; they are labelled accordingly.  The effective target
is an upper bound and is not the source's measured current.

Validation and dependencies
---------------------------

The implementation transfers the higher-current control shape tested on a
Caihong with a 5 A-capable source and cable in the local October 2026 series.
One historical high-target trace covered about 304 seconds and reported an
aggregate pump median near 4.00 A, maximum 4.15 A, and peak battery net power
about 37 W.  These are device telemetry, not an external power-meter result,
and do not prove actual adapter output of 5 A or 9 V times 5 A.

The original low-current policy's loaded-target margin, independent source
capability checks, voltage feedback and teardown remain prerequisites here.
The separate PPS/VOOC mutex-order fix must be applied first; see
``https://github.com/Osilvfe/linux/pull/3``.  This change does not duplicate
that lock fix or alter upstream VOOC recovery and charging-inhibition logic.

This is an experimental port to a newer upstream base.  Historical device
validation is retained as evidence; source/host checks on the rebased code
are recorded separately and are not claimed as a new device test.  Longer
thermal, full-charge and additional source/cable coverage remain useful
before making the profile the default.
