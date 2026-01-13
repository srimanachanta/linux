// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  HID driver for Steelseries devices
 *
 *  Copyright (c) 2013 Simon Wood
 *  Copyright (c) 2023 Bastien Nocera
 *  Copyright (c) 2025 Sriman Achanta
 */

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/usb.h>
#include <linux/leds.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/spinlock.h>

#include "hid-ids.h"

#define SS_CAP_BATTERY BIT(0)
#define SS_CAP_MIC_MUTE_LED BIT(1)

/* Legacy quirk flag for SRW-S1 */
#define STEELSERIES_SRWS1 BIT(0)

struct steelseries_device_info {
	u16 product_id;
	const char *name;
	u8 interface_binding_mode; /* 0 = first enumerated, 1 = specific interface(s) */
	u16 valid_interfaces; /* Bitmask when mode = 1, ignored when mode = 0 */
	unsigned long capabilities;
};

struct steelseries_device {
	struct hid_device *hdev;
	const struct steelseries_device_info *info;

	/* Battery subsystem */
	struct power_supply_desc battery_desc;
	struct power_supply *battery;
	struct delayed_work battery_work;
	u8 battery_capacity;
	bool headset_connected;
	bool battery_charging;

	/* LED subsystem */
	struct led_classdev mute_led;

	/* Synchronization */
	spinlock_t lock;
	bool removed;
};

#if IS_BUILTIN(CONFIG_LEDS_CLASS) || \
	(IS_MODULE(CONFIG_LEDS_CLASS) && IS_MODULE(CONFIG_HID_STEELSERIES))
#define SRWS1_NUMBER_LEDS 15
struct steelseries_srws1_data {
	__u16 led_state;
	/* the last element is used for setting all leds simultaneously */
	struct led_classdev *led[SRWS1_NUMBER_LEDS + 1];
};
#endif

/* Fixed report descriptor for Steelseries SRW-S1 wheel controller
 *
 * The original descriptor hides the sensitivity and assists dials
 * a custom vendor usage page. This inserts a patch to make them
 * appear in the 'Generic Desktop' usage.
 */

static const __u8 steelseries_srws1_rdesc_fixed[] = {
0x05, 0x01,         /*  Usage Page (Desktop)                */
0x09, 0x08,         /*  Usage (MultiAxis), Changed          */
0xA1, 0x01,         /*  Collection (Application),           */
0xA1, 0x02,         /*      Collection (Logical),           */
0x95, 0x01,         /*          Report Count (1),           */
0x05, 0x01,         /* Changed  Usage Page (Desktop),       */
0x09, 0x30,         /* Changed  Usage (X),                  */
0x16, 0xF8, 0xF8,   /*          Logical Minimum (-1800),    */
0x26, 0x08, 0x07,   /*          Logical Maximum (1800),     */
0x65, 0x14,         /*          Unit (Degrees),             */
0x55, 0x0F,         /*          Unit Exponent (15),         */
0x75, 0x10,         /*          Report Size (16),           */
0x81, 0x02,         /*          Input (Variable),           */
0x09, 0x31,         /* Changed  Usage (Y),                  */
0x15, 0x00,         /*          Logical Minimum (0),        */
0x26, 0xFF, 0x03,   /*          Logical Maximum (1023),     */
0x75, 0x0C,         /*          Report Size (12),           */
0x81, 0x02,         /*          Input (Variable),           */
0x09, 0x32,         /* Changed  Usage (Z),                  */
0x15, 0x00,         /*          Logical Minimum (0),        */
0x26, 0xFF, 0x03,   /*          Logical Maximum (1023),     */
0x75, 0x0C,         /*          Report Size (12),           */
0x81, 0x02,         /*          Input (Variable),           */
0x05, 0x01,         /*          Usage Page (Desktop),       */
0x09, 0x39,         /*          Usage (Hat Switch),         */
0x25, 0x07,         /*          Logical Maximum (7),        */
0x35, 0x00,         /*          Physical Minimum (0),       */
0x46, 0x3B, 0x01,   /*          Physical Maximum (315),     */
0x65, 0x14,         /*          Unit (Degrees),             */
0x75, 0x04,         /*          Report Size (4),            */
0x95, 0x01,         /*          Report Count (1),           */
0x81, 0x02,         /*          Input (Variable),           */
0x25, 0x01,         /*          Logical Maximum (1),        */
0x45, 0x01,         /*          Physical Maximum (1),       */
0x65, 0x00,         /*          Unit,                       */
0x75, 0x01,         /*          Report Size (1),            */
0x95, 0x03,         /*          Report Count (3),           */
0x81, 0x01,         /*          Input (Constant),           */
0x05, 0x09,         /*          Usage Page (Button),        */
0x19, 0x01,         /*          Usage Minimum (01h),        */
0x29, 0x11,         /*          Usage Maximum (11h),        */
0x95, 0x11,         /*          Report Count (17),          */
0x81, 0x02,         /*          Input (Variable),           */
		    /*   ---- Dial patch starts here ----   */
0x05, 0x01,         /*          Usage Page (Desktop),       */
0x09, 0x33,         /*          Usage (RX),                 */
0x75, 0x04,         /*          Report Size (4),            */
0x95, 0x02,         /*          Report Count (2),           */
0x15, 0x00,         /*          Logical Minimum (0),        */
0x25, 0x0b,         /*          Logical Maximum (b),        */
0x81, 0x02,         /*          Input (Variable),           */
0x09, 0x35,         /*          Usage (RZ),                 */
0x75, 0x04,         /*          Report Size (4),            */
0x95, 0x01,         /*          Report Count (1),           */
0x25, 0x03,         /*          Logical Maximum (3),        */
0x81, 0x02,         /*          Input (Variable),           */
		    /*    ---- Dial patch ends here ----    */
0x06, 0x00, 0xFF,   /*          Usage Page (FF00h),         */
0x09, 0x01,         /*          Usage (01h),                */
0x75, 0x04,         /* Changed  Report Size (4),            */
0x95, 0x0D,         /* Changed  Report Count (13),          */
0x81, 0x02,         /*          Input (Variable),           */
0xC0,               /*      End Collection,                 */
0xA1, 0x02,         /*      Collection (Logical),           */
0x09, 0x02,         /*          Usage (02h),                */
0x75, 0x08,         /*          Report Size (8),            */
0x95, 0x10,         /*          Report Count (16),          */
0x91, 0x02,         /*          Output (Variable),          */
0xC0,               /*      End Collection,                 */
0xC0                /*  End Collection                      */
};

#if IS_BUILTIN(CONFIG_LEDS_CLASS) || \
	(IS_MODULE(CONFIG_LEDS_CLASS) && IS_MODULE(CONFIG_HID_STEELSERIES))
static void steelseries_srws1_set_leds(struct hid_device *hdev, __u16 leds)
{
	struct list_head *report_list = &hdev->report_enum[HID_OUTPUT_REPORT].report_list;
	struct hid_report *report = list_entry(report_list->next, struct hid_report, list);
	__s32 *value = report->field[0]->value;

	value[0] = 0x40;
	value[1] = leds & 0xFF;
	value[2] = leds >> 8;
	value[3] = 0x00;
	value[4] = 0x00;
	value[5] = 0x00;
	value[6] = 0x00;
	value[7] = 0x00;
	value[8] = 0x00;
	value[9] = 0x00;
	value[10] = 0x00;
	value[11] = 0x00;
	value[12] = 0x00;
	value[13] = 0x00;
	value[14] = 0x00;
	value[15] = 0x00;

	hid_hw_request(hdev, report, HID_REQ_SET_REPORT);

	/* Note: LED change does not show on device until the device is read/polled */
}

static void steelseries_srws1_led_all_set_brightness(struct led_classdev *led_cdev,
			enum led_brightness value)
{
	struct device *dev = led_cdev->dev->parent;
	struct hid_device *hid = to_hid_device(dev);
	struct steelseries_srws1_data *drv_data = hid_get_drvdata(hid);

	if (!drv_data) {
		hid_err(hid, "Device data not found.");
		return;
	}

	if (value == LED_OFF)
		drv_data->led_state = 0;
	else
		drv_data->led_state = (1 << (SRWS1_NUMBER_LEDS + 1)) - 1;

	steelseries_srws1_set_leds(hid, drv_data->led_state);
}

static enum led_brightness steelseries_srws1_led_all_get_brightness(struct led_classdev *led_cdev)
{
	struct device *dev = led_cdev->dev->parent;
	struct hid_device *hid = to_hid_device(dev);
	struct steelseries_srws1_data *drv_data;

	drv_data = hid_get_drvdata(hid);

	if (!drv_data) {
		hid_err(hid, "Device data not found.");
		return LED_OFF;
	}

	return (drv_data->led_state >> SRWS1_NUMBER_LEDS) ? LED_FULL : LED_OFF;
}

static void steelseries_srws1_led_set_brightness(struct led_classdev *led_cdev,
			enum led_brightness value)
{
	struct device *dev = led_cdev->dev->parent;
	struct hid_device *hid = to_hid_device(dev);
	struct steelseries_srws1_data *drv_data = hid_get_drvdata(hid);
	int i, state = 0;

	if (!drv_data) {
		hid_err(hid, "Device data not found.");
		return;
	}

	for (i = 0; i < SRWS1_NUMBER_LEDS; i++) {
		if (led_cdev != drv_data->led[i])
			continue;

		state = (drv_data->led_state >> i) & 1;
		if (value == LED_OFF && state) {
			drv_data->led_state &= ~(1 << i);
			steelseries_srws1_set_leds(hid, drv_data->led_state);
		} else if (value != LED_OFF && !state) {
			drv_data->led_state |= 1 << i;
			steelseries_srws1_set_leds(hid, drv_data->led_state);
		}
		break;
	}
}

static enum led_brightness steelseries_srws1_led_get_brightness(struct led_classdev *led_cdev)
{
	struct device *dev = led_cdev->dev->parent;
	struct hid_device *hid = to_hid_device(dev);
	struct steelseries_srws1_data *drv_data;
	int i, value = 0;

	drv_data = hid_get_drvdata(hid);

	if (!drv_data) {
		hid_err(hid, "Device data not found.");
		return LED_OFF;
	}

	for (i = 0; i < SRWS1_NUMBER_LEDS; i++)
		if (led_cdev == drv_data->led[i]) {
			value = (drv_data->led_state >> i) & 1;
			break;
		}

	return value ? LED_FULL : LED_OFF;
}

static int steelseries_srws1_probe(struct hid_device *hdev,
		const struct hid_device_id *id)
{
	int ret, i;
	struct led_classdev *led;
	struct steelseries_srws1_data *drv_data;
	size_t name_sz;
	char *name;

	drv_data = devm_kzalloc(&hdev->dev, sizeof(*drv_data), GFP_KERNEL);
	if (drv_data == NULL) {
		hid_err(hdev, "can't alloc SRW-S1 memory\n");
		return -ENOMEM;
	}

	hid_set_drvdata(hdev, drv_data);

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "parse failed\n");
		goto err;
	}

	if (!hid_validate_values(hdev, HID_OUTPUT_REPORT, 0, 0, 16)) {
		ret = -ENODEV;
		goto err;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret) {
		hid_err(hdev, "hw start failed\n");
		goto err;
	}

	/* register led subsystem */
	drv_data->led_state = 0;
	for (i = 0; i < SRWS1_NUMBER_LEDS + 1; i++)
		drv_data->led[i] = NULL;

	steelseries_srws1_set_leds(hdev, 0);

	name_sz = strlen(hdev->uniq) + 16;

	/* 'ALL', for setting all LEDs simultaneously */
	led = devm_kzalloc(&hdev->dev, sizeof(struct led_classdev)+name_sz, GFP_KERNEL);
	if (!led) {
		hid_err(hdev, "can't allocate memory for LED ALL\n");
		goto out;
	}

	name = (void *)(&led[1]);
	snprintf(name, name_sz, "SRWS1::%s::RPMALL", hdev->uniq);
	led->name = name;
	led->brightness = 0;
	led->max_brightness = 1;
	led->brightness_get = steelseries_srws1_led_all_get_brightness;
	led->brightness_set = steelseries_srws1_led_all_set_brightness;

	drv_data->led[SRWS1_NUMBER_LEDS] = led;
	ret = devm_led_classdev_register(&hdev->dev, led);
	if (ret) {
		hid_err(hdev, "failed to register LED %d. Aborting.\n", SRWS1_NUMBER_LEDS);
		goto out; /* let the driver continue without LEDs */
	}

	/* Each individual LED */
	for (i = 0; i < SRWS1_NUMBER_LEDS; i++) {
		led = devm_kzalloc(&hdev->dev, sizeof(struct led_classdev)+name_sz, GFP_KERNEL);
		if (!led) {
			hid_err(hdev, "can't allocate memory for LED %d\n", i);
			break;
		}

		name = (void *)(&led[1]);
		snprintf(name, name_sz, "SRWS1::%s::RPM%d", hdev->uniq, i+1);
		led->name = name;
		led->brightness = 0;
		led->max_brightness = 1;
		led->brightness_get = steelseries_srws1_led_get_brightness;
		led->brightness_set = steelseries_srws1_led_set_brightness;

		drv_data->led[i] = led;
		ret = devm_led_classdev_register(&hdev->dev, led);

		if (ret) {
			hid_err(hdev, "failed to register LED %d. Aborting.\n", i);
			break;	/* but let the driver continue without LEDs */
		}
	}
out:
	return 0;
err:
	return ret;
}
#endif

static const __u8 *steelseries_srws1_report_fixup(struct hid_device *hdev,
						  __u8 *rdesc,
						  unsigned int *rsize)
{
	if (hdev->vendor != USB_VENDOR_ID_STEELSERIES ||
	    hdev->product != USB_DEVICE_ID_STEELSERIES_SRWS1)
		return rdesc;

	if (*rsize >= 115 && rdesc[11] == 0x02 && rdesc[13] == 0xc8 &&
	    rdesc[29] == 0xbb && rdesc[40] == 0xc5) {
		hid_info(hdev,
			 "Fixing up Steelseries SRW-S1 report descriptor\n");
		*rsize = sizeof(steelseries_srws1_rdesc_fixed);
		return steelseries_srws1_rdesc_fixed;
	}
	return rdesc;
}

static const struct steelseries_device_info arctis_1_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_1,
	.name = "Arctis 1 Wireless",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(3),
	.capabilities = SS_CAP_BATTERY,
};

static const struct steelseries_device_info arctis_1_x_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_1_X,
	.name = "Arctis 1 Wireless for Xbox",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(3),
	.capabilities = SS_CAP_BATTERY,
};

static const struct steelseries_device_info arctis_7_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_7,
	.name = "Arctis 7",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(5),
	.capabilities = SS_CAP_BATTERY,
};

static const struct steelseries_device_info arctis_7_p_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_7_P,
	.name = "Arctis 7P",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(3),
	.capabilities = SS_CAP_BATTERY,
};

static const struct steelseries_device_info arctis_7_x_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_7_X,
	.name = "Arctis 7X",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(3),
	.capabilities = SS_CAP_BATTERY,
};

static const struct steelseries_device_info arctis_7_gen2_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_7_GEN2,
	.name = "Arctis 7 (2019 Edition)",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(5),
	.capabilities = SS_CAP_BATTERY,
};

static const struct steelseries_device_info arctis_7_plus_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_7_PLUS,
	.name = "Arctis 7+",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(3),
	.capabilities = SS_CAP_BATTERY,
};

static const struct steelseries_device_info arctis_7_plus_p_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_7_PLUS_P,
	.name = "Arctis 7+ (PlayStation)",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(3),
	.capabilities = SS_CAP_BATTERY,
};

static const struct steelseries_device_info arctis_7_plus_x_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_7_PLUS_X,
	.name = "Arctis 7+ (Xbox)",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(3),
	.capabilities = SS_CAP_BATTERY,
};

static const struct steelseries_device_info arctis_7_plus_destiny_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_7_PLUS_DESTINY,
	.name = "Arctis 7+ (Destiny Edition)",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(3),
	.capabilities = SS_CAP_BATTERY,
};

static const struct steelseries_device_info arctis_9_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_9,
	.name = "Arctis 9",
	.interface_binding_mode = 0,
	.valid_interfaces = 0,
	.capabilities = SS_CAP_BATTERY,
};

static const struct steelseries_device_info arctis_pro_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_PRO,
	.name = "Arctis Pro Wireless",
	.interface_binding_mode = 0,
	.valid_interfaces = 0,
	.capabilities = SS_CAP_BATTERY,
};

static const struct steelseries_device_info arctis_nova_3_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_3,
	.name = "Arctis Nova 3",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(4),
	.capabilities = SS_CAP_MIC_MUTE_LED,
};

static const struct steelseries_device_info arctis_nova_3_p_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_3_P,
	.name = "Arctis Nova 3 (PlayStation)",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(0),
	.capabilities = SS_CAP_BATTERY,
};

static const struct steelseries_device_info arctis_nova_3_x_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_3_X,
	.name = "Arctis Nova 3 (Xbox)",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(0),
	.capabilities = SS_CAP_BATTERY,
};

static const struct steelseries_device_info arctis_nova_5_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_5,
	.name = "Arctis Nova 5",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(3),
	.capabilities = SS_CAP_BATTERY | SS_CAP_MIC_MUTE_LED,
};

static const struct steelseries_device_info arctis_nova_5_x_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_5_X,
	.name = "Arctis Nova 5X",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(3),
	.capabilities = SS_CAP_BATTERY | SS_CAP_MIC_MUTE_LED,
};

static const struct steelseries_device_info arctis_nova_7_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_7,
	.name = "Arctis Nova 7",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(3),
	.capabilities = SS_CAP_BATTERY | SS_CAP_MIC_MUTE_LED,
};

static const struct steelseries_device_info arctis_nova_7_x_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_7_X,
	.name = "Arctis Nova 7X",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(3),
	.capabilities = SS_CAP_BATTERY | SS_CAP_MIC_MUTE_LED,
};

static const struct steelseries_device_info arctis_nova_7_p_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_7_P,
	.name = "Arctis Nova 7P",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(3),
	.capabilities = SS_CAP_BATTERY | SS_CAP_MIC_MUTE_LED,
};

static const struct steelseries_device_info arctis_nova_7_x_rev2_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_7_X_REV2,
	.name = "Arctis Nova 7X (Rev 2)",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(3),
	.capabilities = SS_CAP_BATTERY | SS_CAP_MIC_MUTE_LED,
};

static const struct steelseries_device_info arctis_nova_7_diablo_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_7_DIABLO,
	.name = "Arctis Nova 7 (Diablo IV Edition)",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(3),
	.capabilities = SS_CAP_BATTERY | SS_CAP_MIC_MUTE_LED,
};

static const struct steelseries_device_info arctis_nova_7_wow_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_7_WOW,
	.name = "Arctis Nova 7 (World of Warcraft Edition)",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(3),
	.capabilities = SS_CAP_BATTERY | SS_CAP_MIC_MUTE_LED,
};

static const struct steelseries_device_info arctis_nova_7_gen2_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_7_GEN2,
	.name = "Arctis Nova 7 (Gen 2)",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(3),
	.capabilities = SS_CAP_BATTERY | SS_CAP_MIC_MUTE_LED,
};

static const struct steelseries_device_info arctis_nova_7_x_gen2_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_7_X_GEN2,
	.name = "Arctis Nova 7X (Gen 2)",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(3),
	.capabilities = SS_CAP_BATTERY | SS_CAP_MIC_MUTE_LED,
};

static const struct steelseries_device_info arctis_nova_pro_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_PRO,
	.name = "Arctis Nova Pro Wireless",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(4),
	.capabilities = SS_CAP_BATTERY,
};

static const struct steelseries_device_info arctis_nova_pro_x_info = {
	.product_id = USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_PRO_X,
	.name = "Arctis Nova Pro Wireless (Xbox)",
	.interface_binding_mode = 1,
	.valid_interfaces = BIT(4),
	.capabilities = SS_CAP_BATTERY,
};

#define STEELSERIES_HEADSET_BATTERY_TIMEOUT_MS 3000

static int battery_capacity_to_level(int capacity)
{
	if (capacity >= 50)
		return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
	if (capacity >= 20)
		return POWER_SUPPLY_CAPACITY_LEVEL_LOW;
	return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
}

static u8 steelseries_map_battery(u8 capacity, u8 min_in, u8 max_in)
{
	if (capacity >= max_in)
		return 100;
	if (capacity <= min_in)
		return 0;
	return (capacity - min_in) * 100 / (max_in - min_in);
}

static void steelseries_headset_set_wireless_status(struct hid_device *hdev,
						    bool connected)
{
	struct usb_interface *intf;

	if (!hid_is_usb(hdev))
		return;

	intf = to_usb_interface(hdev->dev.parent);
	usb_set_wireless_status(intf, connected ?
					      USB_WIRELESS_STATUS_CONNECTED :
					      USB_WIRELESS_STATUS_DISCONNECTED);
}

#define STEELSERIES_PREFIX "SteelSeries "
#define STEELSERIES_PREFIX_LEN strlen(STEELSERIES_PREFIX)

static int steelseries_battery_get_property(struct power_supply *psy,
				enum power_supply_property psp,
				union power_supply_propval *val)
{
	struct steelseries_device *sd = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = sd->hdev->name;
		while (!strncmp(val->strval, STEELSERIES_PREFIX, STEELSERIES_PREFIX_LEN))
			val->strval += STEELSERIES_PREFIX_LEN;
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "SteelSeries";
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		break;
	case POWER_SUPPLY_PROP_STATUS:
		if (sd->headset_connected) {
			val->intval = sd->battery_charging ?
				POWER_SUPPLY_STATUS_CHARGING :
				POWER_SUPPLY_STATUS_DISCHARGING;
		} else
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_SCOPE:
		val->intval = POWER_SUPPLY_SCOPE_DEVICE;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = sd->battery_capacity;
		break;
	case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
		val->intval = battery_capacity_to_level(sd->battery_capacity);
		break;
	default:
		ret = -EINVAL;
		break;
	}
	return ret;
}

static enum power_supply_property steelseries_battery_props[] = {
	POWER_SUPPLY_PROP_MODEL_NAME,	  POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_PRESENT,	  POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_SCOPE,	  POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CAPACITY_LEVEL,
};

/* Forward declarations for battery request functions */
static int steelseries_arctis_1_request_battery(struct hid_device *hdev);
static int steelseries_arctis_7_plus_request_battery(struct hid_device *hdev);
static int steelseries_arctis_9_request_battery(struct hid_device *hdev);
static int steelseries_arctis_nova_request_battery(struct hid_device *hdev);
static int steelseries_arctis_nova_3p_request_battery(struct hid_device *hdev);
static int
steelseries_arctis_pro_wireless_request_battery(struct hid_device *hdev);

static int steelseries_request_battery(struct hid_device *hdev)
{
	u16 product = hdev->product;

	/* Route to device-specific battery request handler */
	if (product == USB_DEVICE_ID_STEELSERIES_ARCTIS_1 ||
	    product == USB_DEVICE_ID_STEELSERIES_ARCTIS_1_X ||
	    product == USB_DEVICE_ID_STEELSERIES_ARCTIS_7_P ||
	    product == USB_DEVICE_ID_STEELSERIES_ARCTIS_7_X)
		return steelseries_arctis_1_request_battery(hdev);

	if (product == USB_DEVICE_ID_STEELSERIES_ARCTIS_7_PLUS ||
	    product == USB_DEVICE_ID_STEELSERIES_ARCTIS_7_PLUS_P ||
	    product == USB_DEVICE_ID_STEELSERIES_ARCTIS_7_PLUS_X ||
	    product == USB_DEVICE_ID_STEELSERIES_ARCTIS_7_PLUS_DESTINY)
		return steelseries_arctis_7_plus_request_battery(hdev);

	if (product == USB_DEVICE_ID_STEELSERIES_ARCTIS_9)
		return steelseries_arctis_9_request_battery(hdev);

	if (product == USB_DEVICE_ID_STEELSERIES_ARCTIS_PRO)
		return steelseries_arctis_pro_wireless_request_battery(hdev);

	if (product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_3_P ||
	    product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_3_X)
		return steelseries_arctis_nova_3p_request_battery(hdev);

	/* All other Nova series use the same battery request */
	return steelseries_arctis_nova_request_battery(hdev);
}

static void steelseries_battery_timer_tick(struct work_struct *work)
{
	struct steelseries_device *sd = container_of(
		work, struct steelseries_device, battery_work.work);

	steelseries_request_battery(sd->hdev);
}

static int steelseries_battery_register(struct steelseries_device *sd)
{
	static atomic_t battery_no = ATOMIC_INIT(0);
	struct power_supply_config battery_cfg = { .drv_data = sd, };
	unsigned long n;
	int ret;

	sd->battery_desc.type = POWER_SUPPLY_TYPE_BATTERY;
	sd->battery_desc.properties = steelseries_battery_props;
	sd->battery_desc.num_properties = ARRAY_SIZE(steelseries_battery_props);
	sd->battery_desc.get_property = steelseries_battery_get_property;
	sd->battery_desc.use_for_apm = 0;
	n = atomic_inc_return(&battery_no) - 1;
	sd->battery_desc.name = devm_kasprintf(&sd->hdev->dev, GFP_KERNEL,
						    "steelseries_headset_battery_%ld", n);
	if (!sd->battery_desc.name)
		return -ENOMEM;

	/* avoid the warning of 0% battery while waiting for the first info */
	steelseries_headset_set_wireless_status(sd->hdev, false);
	sd->battery_capacity = 100;
	sd->battery_charging = false;
	sd->headset_connected = false;

	sd->battery = devm_power_supply_register(&sd->hdev->dev,
			&sd->battery_desc, &battery_cfg);
	if (IS_ERR(sd->battery)) {
		ret = PTR_ERR(sd->battery);
		hid_err(sd->hdev,
				"%s:power_supply_register failed with error %d\n",
				__func__, ret);
		return ret;
	}
	power_supply_powers(sd->battery, &sd->hdev->dev);

	INIT_DELAYED_WORK(&sd->battery_work, steelseries_battery_timer_tick);
	steelseries_request_battery(sd->hdev);

	if (sd->hdev->product == USB_DEVICE_ID_STEELSERIES_ARCTIS_9) {
		/* The first fetch_battery request can remain unanswered in some cases */
		schedule_delayed_work(&sd->battery_work,
				msecs_to_jiffies(STEELSERIES_HEADSET_BATTERY_TIMEOUT_MS));
	}

	return 0;
}

/* Helper function to send feature reports */
static int steelseries_send_feature_report(struct hid_device *hdev,
					   const u8 *data, size_t len)
{
	u8 *buf;
	int ret;

	buf = kmemdup(data, len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	ret = hid_hw_raw_request(hdev, data[0], buf, len, HID_FEATURE_REPORT,
				 HID_REQ_SET_REPORT);
	kfree(buf);

	if (ret < 0)
		return ret;
	if (ret < len)
		return -EIO;

	return 0;
}

/* Helper function to send output reports */
static int steelseries_send_output_report(struct hid_device *hdev,
					  const u8 *data, size_t len)
{
	u8 *buf;
	int ret;

	buf = kmemdup(data, len, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	/* Use raw_request with OUTPUT_REPORT type for devices without Interrupt OUT */
	ret = hid_hw_raw_request(hdev, data[0], buf, len, HID_OUTPUT_REPORT,
				 HID_REQ_SET_REPORT);
	kfree(buf);

	if (ret < 0)
		return ret;
	if (ret < len)
		return -EIO;

	return 0;
}

/* LED subsystem support */
#if IS_BUILTIN(CONFIG_LEDS_CLASS) || \
	(IS_MODULE(CONFIG_LEDS_CLASS) && IS_MODULE(CONFIG_HID_STEELSERIES))

static int steelseries_mute_led_brightness_set(struct led_classdev *led_cdev,
						enum led_brightness brightness)
{
	struct steelseries_device *sd = container_of(led_cdev,
						      struct steelseries_device,
						      mute_led);
	struct hid_device *hdev = sd->hdev;
	u16 product = hdev->product;
	u8 data[64] = { 0 };
	u8 value = brightness;
	int ret;

	/* Device-specific validation and mapping */
	if (product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_3) {
		if (value > 3)
			value = 3;
	} else if (product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_5 ||
		   product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_5_X) {
		if (value > 10)
			value = 10;
		/* Map special values */
		if (value == 2)
			value = 0x04;
		else if (value == 3)
			value = 0x0a;
	} else {
		/* Nova 7 series and others */
		if (value > 3)
			value = 3;
	}

	/* Send device-specific commands */
	if (product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_3) {
		data[0] = 0x06;
		data[1] = 0xae;
		data[2] = value;
		ret = steelseries_send_output_report(hdev, data, 64);
		if (ret >= 0) {
			memset(data, 0, sizeof(data));
			data[0] = 0x06;
			data[1] = 0x09;
			ret = steelseries_send_output_report(hdev, data, 64);
		}
	} else if (product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_5 ||
		   product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_5_X) {
		data[0] = 0x00;
		data[1] = 0xae;
		data[2] = value;
		ret = steelseries_send_output_report(hdev, data, 64);
		if (ret >= 0) {
			memset(data, 0, sizeof(data));
			data[0] = 0x00;
			data[1] = 0x09;
			ret = steelseries_send_output_report(hdev, data, 64);
			if (ret >= 0) {
				memset(data, 0, sizeof(data));
				data[0] = 0x00;
				data[1] = 0x35;
				data[2] = 0x01;
				ret = steelseries_send_output_report(hdev, data, 64);
			}
		}
	} else {
		/* Nova 7 series */
		data[0] = 0x00;
		data[1] = 0xae;
		data[2] = value;
		ret = steelseries_send_output_report(hdev, data, 64);
		if (ret >= 0) {
			memset(data, 0, sizeof(data));
			data[0] = 0x00;
			data[1] = 0x09;
			ret = steelseries_send_output_report(hdev, data, 64);
		}
	}

	return ret;
}

static int steelseries_mute_led_register(struct steelseries_device *sd)
{
	struct hid_device *hdev = sd->hdev;
	u16 product = hdev->product;
	int max_brightness;
	int ret;

	/* Determine max brightness based on device */
	if (product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_5 ||
	    product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_5_X) {
		max_brightness = 10;
	} else {
		max_brightness = 3;
	}

	sd->mute_led.name = devm_kasprintf(&hdev->dev, GFP_KERNEL,
					   "%s:mute", dev_name(&hdev->dev));
	if (!sd->mute_led.name)
		return -ENOMEM;

	sd->mute_led.brightness_set_blocking = steelseries_mute_led_brightness_set;
	sd->mute_led.max_brightness = max_brightness;
	sd->mute_led.flags = LED_RETAIN_AT_SHUTDOWN;

	ret = devm_led_classdev_register(&hdev->dev, &sd->mute_led);
	if (ret < 0) {
		hid_err(hdev, "Failed to register mute LED: %d\n", ret);
		return ret;
	}

	return 0;
}

#endif /* CONFIG_LEDS_CLASS */

static int steelseries_arctis_1_request_battery(struct hid_device *hdev)
{
	const u8 data[] = { 0x06, 0x12 };

	return steelseries_send_feature_report(hdev, data, sizeof(data));
}

static int steelseries_arctis_7_plus_request_battery(struct hid_device *hdev)
{
	const u8 data[] = { 0x00, 0xb0 };

	return steelseries_send_output_report(hdev, data, sizeof(data));
}

static int steelseries_arctis_9_request_battery(struct hid_device *hdev)
{
	const u8 data[] = { 0x00, 0x20 };

	return steelseries_send_feature_report(hdev, data, sizeof(data));
}

static int steelseries_arctis_nova_request_battery(struct hid_device *hdev)
{
	const u8 data[] = { 0x00, 0xb0 };

	return steelseries_send_output_report(hdev, data, sizeof(data));
}

static int steelseries_arctis_nova_3p_request_battery(struct hid_device *hdev)
{
	const u8 data[] = { 0xb0 };

	return steelseries_send_output_report(hdev, data, sizeof(data));
}

static int
steelseries_arctis_pro_wireless_request_battery(struct hid_device *hdev)
{
	/* Request battery - response will arrive asynchronously via raw_event */
	const u8 data[] = { 0x40, 0xAA };

	return steelseries_send_output_report(hdev, data, sizeof(data));
}

static int steelseries_raw_event(struct hid_device *hdev,
				 struct hid_report *report, u8 *data, int size)
{
	struct steelseries_device *sd = hid_get_drvdata(hdev);
	u16 product = hdev->product;
	int capacity = sd->battery_capacity;
	bool connected = sd->headset_connected;
	bool charging = sd->battery_charging;
	unsigned long flags;

	/* Skip SRW-S1 */
	if (product == USB_DEVICE_ID_STEELSERIES_SRWS1)
		return 0;

	/* Arctis 1 family (Arctis 1, 1X, 7P, 7X) */
	if (product == USB_DEVICE_ID_STEELSERIES_ARCTIS_1 ||
	    product == USB_DEVICE_ID_STEELSERIES_ARCTIS_1_X ||
	    product == USB_DEVICE_ID_STEELSERIES_ARCTIS_7_P ||
	    product == USB_DEVICE_ID_STEELSERIES_ARCTIS_7_X) {
		if (size < 8)
			goto schedule_work;

		if (data[2] == 0x01) {
			connected = false;
			capacity = 100;
		} else {
			connected = true;
			capacity = data[3];
			if (capacity > 100)
				capacity = 100;
		}
	}

	/* Arctis 7 (original and 2019) */
	else if (product == USB_DEVICE_ID_STEELSERIES_ARCTIS_7 ||
		 product == USB_DEVICE_ID_STEELSERIES_ARCTIS_7_GEN2) {
		/* Battery response is 8 bytes for Arctis 7 */
		if (size < 8)
			goto schedule_work;

		connected = true;
		charging = false;

		/* Battery level is in data[2] */
		capacity = data[2];
		if (capacity > 100)
			capacity = 100;
	}

	/* Arctis 7+ family */
	else if (product == USB_DEVICE_ID_STEELSERIES_ARCTIS_7_PLUS ||
		 product == USB_DEVICE_ID_STEELSERIES_ARCTIS_7_PLUS_P ||
		 product == USB_DEVICE_ID_STEELSERIES_ARCTIS_7_PLUS_X ||
		 product == USB_DEVICE_ID_STEELSERIES_ARCTIS_7_PLUS_DESTINY) {
		if (size < 6)
			goto schedule_work;

		/* data[1] == 0x01 means HEADSET_OFFLINE */
		if (data[1] == 0x01) {
			connected = false;
			capacity = 100;
		} else {
			connected = true;
			/* data[3] == 0x01 means charging */
			charging = (data[3] == 0x01);
			/* data[2] contains battery level (0x00-0x04 range) */
			capacity = steelseries_map_battery(data[2], 0x00, 0x04);
		}
	}

	/* Arctis 9 */
	else if (product == USB_DEVICE_ID_STEELSERIES_ARCTIS_9) {
		if (size < 12)
			goto schedule_work;

		connected = true;

		charging = (data[4] == 0x01);

		capacity = steelseries_map_battery(data[3], 0x64, 0x9A);
	}

	/* Arctis Pro Wireless */
	else if (product == USB_DEVICE_ID_STEELSERIES_ARCTIS_PRO) {
		if (size >= 2 && (data[0] == 0x02 || data[0] == 0x04)) {
			/* This is a connection status response */
			/* HEADSET_OFFLINE */
			if (data[0] == 0x02) {
				connected = false;
				capacity = 100;
				charging = false;
			}
			/* HEADSET_ONLINE (0x04) */
			else {
				connected = true;
				charging = false;
			}
		} else if (size >= 1 && sd->headset_connected) {
			/* This is a battery level response (only valid if headset connected) */
			/* Battery range is 0x00-0x04 */
			capacity = steelseries_map_battery(data[0], 0x00, 0x04);
		}
	}

	/* Arctis Nova 3 */
	else if (product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_3) {
		/* No battery monitoring for wired headset */
		goto schedule_work;
	}

	/* Arctis Nova 3P/3X Wireless */
	else if (product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_3_P ||
		 product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_3_X) {
		if (size < 4)
			goto schedule_work;

		/* data[1] == 0x02 means HEADSET_OFFLINE */
		if (data[1] == 0x02) {
			connected = false;
			capacity = 100;
		} else {
			connected = true;
			charging = false;
			/* data[3] contains battery level (0x00-0x64 range, 0-100) */
			capacity = steelseries_map_battery(data[3], 0x00, 0x64);
		}
	}

	/* Arctis Nova 5/5X */
	else if (product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_5 ||
		 product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_5_X) {
		if (size < 16)
			goto schedule_work;

		/* data[1] == 0x02 means HEADSET_OFFLINE */
		if (data[1] == 0x02) {
			connected = false;
			capacity = 100;
		} else {
			connected = true;
			/* data[4] == 0x01 means charging */
			charging = (data[4] == 0x01);
			/* data[3] contains battery level (0-100) */
			capacity = data[3];
			if (capacity > 100)
				capacity = 100;
		}
	}

	/* Arctis Nova 7 family */
	else if (product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_7 ||
		 product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_7_X ||
		 product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_7_P ||
		 product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_7_X_REV2 ||
		 product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_7_DIABLO ||
		 product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_7_WOW ||
		 product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_7_GEN2 ||
		 product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_7_X_GEN2) {
		if (size < 8)
			goto schedule_work;

		/* data[3] == 0x00 means HEADSET_OFFLINE */
		if (data[3] == 0x00) {
			connected = false;
			capacity = 100;
		} else {
			connected = true;
			/* data[3] == 0x01 means charging */
			charging = (data[3] == 0x01);
			/* data[2] contains battery level (0x00-0x04 range) */
			capacity = steelseries_map_battery(data[2], 0x00, 0x04);
		}
	}

	/* Arctis Nova Pro Wireless */
	else if (product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_PRO ||
		 product == USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_PRO_X) {
		if (size < 16)
			goto schedule_work;

		/* data[15] contains headset status */
		if (data[15] == 0x01) { /* HEADSET_OFFLINE */
			connected = false;
			capacity = 100;
		} else if (data[15] == 0x02) { /* HEADSET_CABLE_CHARGING */
			connected = true;
			charging = true;
			/* data[6] contains battery level (0x00-0x08 range) */
			capacity = steelseries_map_battery(data[6], 0x00, 0x08);
		} else if (data[15] == 0x08) { /* HEADSET_ONLINE */
			connected = true;
			charging = false;
			/* data[6] contains battery level (0x00-0x08 range) */
			capacity = steelseries_map_battery(data[6], 0x00, 0x08);
		} else {
			/* Unknown status */
			goto schedule_work;
		}
	}

	/* Update state if changed */
	if (connected != sd->headset_connected) {
		hid_dbg(hdev,
			"Connected status changed from %sconnected to %sconnected\n",
			sd->headset_connected ? "" : "not ",
			connected ? "" : "not ");
		sd->headset_connected = connected;
		steelseries_headset_set_wireless_status(hdev, connected);
	}

	if (capacity != sd->battery_capacity) {
		hid_dbg(hdev, "Battery capacity changed from %d%% to %d%%\n",
			sd->battery_capacity, capacity);
		sd->battery_capacity = capacity;
		power_supply_changed(sd->battery);
	}

	if (charging != sd->battery_charging) {
		hid_dbg(hdev,
			"Battery charging status changed from %scharging to %scharging\n",
			sd->battery_charging ? "" : "not ",
			charging ? "" : "not ");
		sd->battery_charging = charging;
		power_supply_changed(sd->battery);
	}

schedule_work:
	spin_lock_irqsave(&sd->lock, flags);
	if (!sd->removed)
		schedule_delayed_work(&sd->battery_work,
				msecs_to_jiffies(STEELSERIES_HEADSET_BATTERY_TIMEOUT_MS));
	spin_unlock_irqrestore(&sd->lock, flags);

	return 0;
}

static bool steelseries_is_vendor_usage_page(struct hid_device *hdev,
					     u8 usage_page)
{
	return hdev->rdesc[0] == 0x06 && hdev->rdesc[1] == usage_page &&
	       hdev->rdesc[2] == 0xff;
}

static int steelseries_probe(struct hid_device *hdev,
			     const struct hid_device_id *id)
{
	struct steelseries_device_info *info =
		(struct steelseries_device_info *)id->driver_data;
	struct steelseries_device *sd;
	struct usb_interface *intf;
	u8 interface_num;
	int ret;

	/* Legacy SRW-S1 handling */
	if (hdev->product == USB_DEVICE_ID_STEELSERIES_SRWS1) {
#if IS_BUILTIN(CONFIG_LEDS_CLASS) || \
	(IS_MODULE(CONFIG_LEDS_CLASS) && IS_MODULE(CONFIG_HID_STEELSERIES))
		return steelseries_srws1_probe(hdev, id);
#else
		return -ENODEV;
#endif
	}

	/* Get interface number for binding check */
	if (hid_is_usb(hdev)) {
		intf = to_usb_interface(hdev->dev.parent);
		interface_num = intf->cur_altsetting->desc.bInterfaceNumber;
	} else {
		/* Non-USB devices not supported for modern Arctis */
		return -ENODEV;
	}

	/* Interface binding logic */
	if (info->interface_binding_mode == 0) {
		/* Mode 0: First enumerated (interface 0) */
		if (interface_num != 0)
			return -ENODEV;
	} else {
		/* Mode 1: Check bitmask */
		if (!(info->valid_interfaces & BIT(interface_num)))
			return -ENODEV;
	}

	sd = devm_kzalloc(&hdev->dev, sizeof(*sd), GFP_KERNEL);
	if (!sd)
		return -ENOMEM;

	sd->hdev = hdev;
	sd->info = info;
	hid_set_drvdata(hdev, sd);

	ret = hid_parse(hdev);
	if (ret)
		return ret;

	/* Arctis 9 requires vendor usage page check */
	if (hdev->product == USB_DEVICE_ID_STEELSERIES_ARCTIS_9 &&
	    !steelseries_is_vendor_usage_page(hdev, 0xc0))
		return -ENODEV;

	spin_lock_init(&sd->lock);

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret)
		return ret;

	ret = hid_hw_open(hdev);
	if (ret)
		goto err_stop;

	/* Register battery if supported */
	if (info->capabilities & SS_CAP_BATTERY) {
		ret = steelseries_battery_register(sd);
		if (ret < 0)
			hid_warn(hdev, "Failed to register battery: %d\n", ret);
	}

	/* Register mute LED if supported */
#if IS_BUILTIN(CONFIG_LEDS_CLASS) || \
	(IS_MODULE(CONFIG_LEDS_CLASS) && IS_MODULE(CONFIG_HID_STEELSERIES))
	if (info->capabilities & SS_CAP_MIC_MUTE_LED) {
		ret = steelseries_mute_led_register(sd);
		if (ret < 0)
			hid_warn(hdev, "Failed to register mute LED: %d\n", ret);
	}
#endif

	hid_info(hdev, "SteelSeries %s initialized\n", info->name);

	return 0;

err_stop:
	hid_hw_stop(hdev);
	return ret;
}

static void steelseries_remove(struct hid_device *hdev)
{
	struct steelseries_device *sd;
	unsigned long flags;

	/* Legacy SRW-S1 */
	if (hdev->product == USB_DEVICE_ID_STEELSERIES_SRWS1) {
#if IS_BUILTIN(CONFIG_LEDS_CLASS) || \
	(IS_MODULE(CONFIG_LEDS_CLASS) && IS_MODULE(CONFIG_HID_STEELSERIES))
		hid_hw_stop(hdev);
#endif
		return;
	}

	sd = hid_get_drvdata(hdev);

	spin_lock_irqsave(&sd->lock, flags);
	sd->removed = true;
	spin_unlock_irqrestore(&sd->lock, flags);

	cancel_delayed_work_sync(&sd->battery_work);

	hid_hw_close(hdev);
	hid_hw_stop(hdev);
}

static const struct hid_device_id steelseries_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_SRWS1),
	  .driver_data = STEELSERIES_SRWS1 },

	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_1),
	  .driver_data = (unsigned long)&arctis_1_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_1_X),
	  .driver_data = (unsigned long)&arctis_1_x_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_7),
	  .driver_data = (unsigned long)&arctis_7_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_7_P),
	  .driver_data = (unsigned long)&arctis_7_p_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_7_X),
	  .driver_data = (unsigned long)&arctis_7_x_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_7_GEN2),
	  .driver_data = (unsigned long)&arctis_7_gen2_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_7_PLUS),
	  .driver_data = (unsigned long)&arctis_7_plus_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_7_PLUS_P),
	  .driver_data = (unsigned long)&arctis_7_plus_p_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_7_PLUS_X),
	  .driver_data = (unsigned long)&arctis_7_plus_x_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_7_PLUS_DESTINY),
	  .driver_data = (unsigned long)&arctis_7_plus_destiny_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_9),
	  .driver_data = (unsigned long)&arctis_9_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_PRO),
	  .driver_data = (unsigned long)&arctis_pro_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_3),
	  .driver_data = (unsigned long)&arctis_nova_3_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_3_P),
	  .driver_data = (unsigned long)&arctis_nova_3_p_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_3_X),
	  .driver_data = (unsigned long)&arctis_nova_3_x_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_5),
	  .driver_data = (unsigned long)&arctis_nova_5_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_5_X),
	  .driver_data = (unsigned long)&arctis_nova_5_x_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_7),
	  .driver_data = (unsigned long)&arctis_nova_7_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_7_X),
	  .driver_data = (unsigned long)&arctis_nova_7_x_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_7_P),
	  .driver_data = (unsigned long)&arctis_nova_7_p_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_7_X_REV2),
	  .driver_data = (unsigned long)&arctis_nova_7_x_rev2_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_7_DIABLO),
	  .driver_data = (unsigned long)&arctis_nova_7_diablo_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_7_WOW),
	  .driver_data = (unsigned long)&arctis_nova_7_wow_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_7_GEN2),
	  .driver_data = (unsigned long)&arctis_nova_7_gen2_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_7_X_GEN2),
	  .driver_data = (unsigned long)&arctis_nova_7_x_gen2_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_PRO),
	  .driver_data = (unsigned long)&arctis_nova_pro_info },
	{ HID_USB_DEVICE(USB_VENDOR_ID_STEELSERIES,
			 USB_DEVICE_ID_STEELSERIES_ARCTIS_NOVA_PRO_X),
	  .driver_data = (unsigned long)&arctis_nova_pro_x_info },
	{}
};
MODULE_DEVICE_TABLE(hid, steelseries_devices);

static struct hid_driver steelseries_driver = {
	.name = "steelseries",
	.id_table = steelseries_devices,
	.probe = steelseries_probe,
	.remove = steelseries_remove,
	.report_fixup = steelseries_srws1_report_fixup,
	.raw_event = steelseries_raw_event,
};

module_hid_driver(steelseries_driver);
MODULE_DESCRIPTION("HID driver for Steelseries devices");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Sriman Achanta <srimanachanta@gmail.com>");
MODULE_AUTHOR("Bastien Nocera <hadess@hadess.net>");
MODULE_AUTHOR("Simon Wood <simon@mungewell.org>");
MODULE_AUTHOR("Christian Mayer <git@mayer-bgk.de>");
