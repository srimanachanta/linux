// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * HID driver for 8BitDo devices
 *
 * Copyright (c) 2025 Sriman Achanta
 */
#include <linux/device.h>
#include <linux/hid.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>

#include "hid-ids.h"

#define EIGHTBITDO_RUMBLE_REPORT_ID 0x05

struct eightbitdo_device {
	struct hid_device *hdev;
	struct work_struct work;

	spinlock_t lock;
	bool removed;

	u8 magnitude[2];
};

static void eightbitdo_rumble_worker(struct work_struct *work)
{
	struct eightbitdo_device *ebd =
		container_of(work, struct eightbitdo_device, work);
	struct hid_device *hdev = ebd->hdev;
	u8 rumble_data[5];
	unsigned long flags;

	spin_lock_irqsave(&ebd->lock, flags);
	if (ebd->removed) {
		spin_unlock_irqrestore(&ebd->lock, flags);
		return;
	}

	rumble_data[0] = EIGHTBITDO_RUMBLE_REPORT_ID;
	rumble_data[1] = ebd->magnitude[0];
	rumble_data[2] = ebd->magnitude[1];
	rumble_data[3] = 0x00;
	rumble_data[4] = 0x00;
	spin_unlock_irqrestore(&ebd->lock, flags);

	hid_hw_output_report(hdev, rumble_data, sizeof(rumble_data));
}

static int eightbitdo_play_effect(struct input_dev *dev, void *data,
				  struct ff_effect *effect)
{
	struct hid_device *hdev = input_get_drvdata(dev);
	struct eightbitdo_device *ebd = hid_get_drvdata(hdev);
	unsigned long flags;

	if (effect->type != FF_RUMBLE)
		return 0;

	spin_lock_irqsave(&ebd->lock, flags);
	ebd->magnitude[0] = (effect->u.rumble.strong_magnitude * 100) / 0xFFFF;
	ebd->magnitude[1] = (effect->u.rumble.weak_magnitude * 100) / 0xFFFF;
	spin_unlock_irqrestore(&ebd->lock, flags);

	schedule_work(&ebd->work);

	return 0;
}

static int eightbitdo_probe(struct hid_device *hdev,
			    const struct hid_device_id *id)
{
	struct eightbitdo_device *ebd;
	struct hid_input *hidinput;
	struct input_dev *input_dev;
	int ret;

	ebd = devm_kzalloc(&hdev->dev, sizeof(*ebd), GFP_KERNEL);
	if (!ebd)
		return -ENOMEM;

	ebd->hdev = hdev;
	hid_set_drvdata(hdev, ebd);

	spin_lock_init(&ebd->lock);
	INIT_WORK(&ebd->work, eightbitdo_rumble_worker);

	ret = hid_parse(hdev);
	if (ret)
		return ret;

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret)
		return ret;

	hidinput =
		list_first_entry_or_null(&hdev->inputs, struct hid_input, list);
	if (!hidinput) {
		ret = -ENODEV;
		goto err_stop;
	}

	input_dev = hidinput->input;
	input_set_drvdata(input_dev, hdev);

	input_set_capability(input_dev, EV_FF, FF_RUMBLE);
	ret = input_ff_create_memless(input_dev, NULL, eightbitdo_play_effect);
	if (ret) {
		hid_err(hdev, "Failed to create FF device: %d\n", ret);
		goto err_stop;
	}

	hid_info(hdev, "8BitDo initialized\n");

	return 0;

err_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void eightbitdo_remove(struct hid_device *hdev)
{
	struct eightbitdo_device *ebd = hid_get_drvdata(hdev);
	unsigned long flags;

	spin_lock_irqsave(&ebd->lock, flags);
	ebd->removed = true;
	spin_unlock_irqrestore(&ebd->lock, flags);

	cancel_work_sync(&ebd->work);

	hid_hw_stop(hdev);
}

static const struct hid_device_id eightbitdo_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_8BITDO,
			 USB_DEVICE_ID_8BITDO_ULTIMATE_2_WIRELESS) },
	{}
};
MODULE_DEVICE_TABLE(hid, eightbitdo_devices);

static struct hid_driver eightbitdo_driver = {
	.name = "8bitdo",
	.id_table = eightbitdo_devices,
	.probe = eightbitdo_probe,
	.remove = eightbitdo_remove,
};
module_hid_driver(eightbitdo_driver);

MODULE_AUTHOR("Sriman Achanta <srimanachanta@gmail.com>");
MODULE_DESCRIPTION("HID driver for 8BitDo devices");
MODULE_LICENSE("GPL");
