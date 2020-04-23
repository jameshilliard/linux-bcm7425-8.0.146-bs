/* Copyright (C) 2016 National Instruments Corp.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
  * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/leds.h>
#include <linux/phy.h>
#include <linux/phy_led_triggers.h>
#include <linux/netdevice.h>

static void phy_led_trigger_no_link(struct phy_device *phy)
{
	if (phy->last_triggered) {
		led_trigger_event(&phy->last_triggered->trigger, LED_OFF);
		phy->last_triggered = NULL;
	}
}

void phy_led_trigger_change_speed(struct phy_device *phy)
{
	if (!phy->link)
		return phy_led_trigger_no_link(phy);

	if (phy->speed == 0)
		return;

	if (!phy->last_triggered) {
		led_trigger_event(&phy->led_link_trigger->trigger,
				  LED_FULL);
		phy->last_triggered = phy->led_link_trigger;
	}
}
EXPORT_SYMBOL_GPL(phy_led_trigger_change_speed);

static void phy_led_trigger_format_name(struct phy_device *phy, char *buf,
					size_t size, char *suffix)
{
	snprintf(buf, size, PHY_ID_FMT ":%s",
		 phy->bus->id, phy->addr, suffix);
}

static void phy_led_trigger_unregister(struct phy_led_trigger *plt)
{
	led_trigger_unregister(&plt->trigger);
}

int phy_led_triggers_register(struct phy_device *phy)
{
	int err;

	phy->led_link_trigger = devm_kzalloc(&phy->dev,
					     sizeof(*phy->led_link_trigger),
					     GFP_KERNEL);
	if (!phy->led_link_trigger) {
		err = -ENOMEM;
		goto out_clear;
	}

	phy_led_trigger_format_name(phy, phy->led_link_trigger->name,
				    sizeof(phy->led_link_trigger->name),
				    "link");
	phy->led_link_trigger->trigger.name = phy->led_link_trigger->name;

	err = led_trigger_register(&phy->led_link_trigger->trigger);
	if (err)
		goto out_free_link;

	phy->last_triggered = NULL;
	phy_led_trigger_change_speed(phy);

	return 0;

out_free_link:
	devm_kfree(&phy->dev, phy->led_link_trigger);
	phy->led_link_trigger = NULL;
out_clear:
	return err;
}
EXPORT_SYMBOL_GPL(phy_led_triggers_register);

void phy_led_triggers_unregister(struct phy_device *phy)
{
	int i;

	if (phy->led_link_trigger)
		phy_led_trigger_unregister(phy->led_link_trigger);
}
EXPORT_SYMBOL_GPL(phy_led_triggers_unregister);
