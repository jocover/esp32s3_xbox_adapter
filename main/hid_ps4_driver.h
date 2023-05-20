#ifndef _USB_HID_PS4_DRIVER_H_
#define _USB_HID_PS4_DRIVER_H_

#include <stdint.h>
#include "tusb.h"
#include "device/usbd_pvt.h"

#define USBD_PS4_NAME "Dualshock4 Emulation"

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct
    {
        uint8_t *data;
        uint16_t size;
    } usb_report_t;

    // https://www.psdevwiki.com/ps4/DS4-USB
 typedef struct __attribute((packed, aligned(1))) {
    uint8_t report_id;//0
    uint8_t lx;//1
    uint8_t ly;//2
    uint8_t rx;//3
    uint8_t ry;//4
    uint8_t buttons1;//5
    uint8_t buttons2;//6
    uint8_t buttons3;//7
    uint8_t lt;//8
    uint8_t rt;//9
    uint16_t timestamp;//10-11
    uint8_t battery;//12
    uint16_t gyrox;//13-14
    uint16_t gyroy;//15-16
    uint16_t gyroz;//17-18
    int16_t accelx;//19-20
    int16_t accely;//21-22
    int16_t accelz;//23-24
    uint8_t unknown1[5];//25-29
    uint8_t extension;//30
    uint8_t unknown2[2];//31-32
    uint8_t touchpad_event_active;//33
    uint8_t touchpad_counter;//34
    uint8_t touchpad1_touches;//35
    uint8_t touchpad1_position[3];//36-38
    uint8_t touchpad2_touches;//39
    uint8_t touchpad2_position[3];//40-42
    uint8_t unknown3[21];
} hid_ps4_report_t;

    // extern const tusb_desc_device_t ps4_divacon_desc_device;
    extern const char *ps4_string_descriptors[];
    extern const uint8_t ds4_desc_device[];
    extern const uint8_t ps4_desc_cfg[];
    extern const uint8_t ps4_desc_hid_report[];

    void ps4_driver_init(void);

    bool send_hid_ps4_report(hid_ps4_report_t* report);
    uint16_t hid_ps4_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer,
                                   uint16_t reqlen);
    void hid_ps4_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer,
                               uint16_t bufsize);

#ifdef __cplusplus
}
#endif

#endif // _USB_HID_PS4_DRIVER_H_