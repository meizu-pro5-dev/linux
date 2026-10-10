/* SPDX-License-Identifier: GPL-2.0 */
#ifndef M86_PANFROST_DOMAIN_H
#define M86_PANFROST_DOMAIN_H
struct device;
/* Experimental provider-only observation. Caller pins the bound Panfrost
 * private object against unbind/fini, serializes runtime resume/reset, and
 * proves external G3D CFG/STATUS=f plus CMU/bridge clocks before calling.
 * Only call after the consumer suspend callback completed (PM core may still
 * label it RPM_SUSPENDING). Not an arbitrary suspended-device MMIO API.
 * No PM get, reset or power writes. Zero means all internal READY/PWRTRANS
 * LO/HI observed zero; it is not a reservation preventing later activity.
 * Provider must not register without reachable built-in Panfrost and this API.
 */
int m86_panfrost_domain_idle(struct device *dev);
#endif
