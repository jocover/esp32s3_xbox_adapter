
#include "hid_ps4_driver.h"
#include "class/hid/hid_device.h"
#include "esp_log.h"
#include "tusb.h"
#include "esp_rom_crc.h"
#include "esp_random.h"
#include "mbedtls/error.h"
#include "mbedtls/rsa.h"
#include "mbedtls/sha256.h"
#include "mbedtls/pk.h"
#include "esp_log.h"
#include "esp_timer.h"

#define LSB(n) (n & 255)
#define MSB(n) ((n >> 8) & 255)

enum
{
    USBD_STR_LANGUAGE,
    USBD_STR_MANUFACTURER,
    USBD_STR_PRODUCT,
    USBD_STR_SERIAL,
    USBD_STR_CDC,
    USBD_STR_SWITCH,
    USBD_STR_PS3,
    USBD_STR_PS4,
    USBD_STR_XINPUT,
    USBD_STR_MIDI,
    USBD_STR_RPI_RESET,
};

typedef enum
{
    no_nonce = 0,
    receiving_nonce = 1,
    nonce_ready = 2,
    signed_nonce_ready = 3
} PS4State;

const char *ps4_string_descriptors[] = {
    (const char[]){0x09, 0x04}, // 0: is supported language is English (0x0409)
    "Sony Computer Entertainment",                  // 1: Manufacturer
    "Wireless Controller",           // 2: Product
    "123456",                   // 3: Serials, should use chip ID
};

const tusb_desc_device_t ps4_divacon_desc_device = {
    .bLength = sizeof(tusb_desc_device_t),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_UNSPECIFIED,
    .bDeviceSubClass = 0x00,
    .bDeviceProtocol = 0x00,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x0F0D,  // HORI
    .idProduct = 0x013C, // PS4-161 aka Project Diva Arcade Controller
    .bcdDevice = 0x0100,
    .iManufacturer = USBD_STR_MANUFACTURER,
    .iProduct = USBD_STR_PRODUCT,
    .iSerialNumber = USBD_STR_SERIAL,
    .bNumConfigurations = 1,
};

/*
typedef struct __attribute((packed, aligned(1)))
{
    uint8_t content_flags; // 0x01: Rumble, 0x02: Color, 0x04: Flash
    uint8_t unknown1[2];
    uint8_t rumble_weak;
    uint8_t rumble_strong;
    uint8_t led_red;
    uint8_t led_green;
    uint8_t led_blue;
    uint8_t flash_bright_time;
    uint8_t flash_dark_time;
    uint8_t unknown2[21];
} hid_ps4_ouput_report_t;
*/

#define PS4_VENDOR_ID 0x054c
#define PS4_PRODUCT_ID 0x0ce6

const uint8_t ds4_desc_device[] =
    {
        18,                                       // bLength
        1,                                        // bDescriptorType
        0x00, 0x02,                               // bcdUSB
        0,                                        // bDeviceClass
        0,                                        // bDeviceSubClass
        0,                                        // bDeviceProtocol
        64,                                       // bMaxPacketSize0
        LSB(PS4_VENDOR_ID), MSB(PS4_VENDOR_ID),   // idVendor
        LSB(PS4_PRODUCT_ID), MSB(PS4_PRODUCT_ID), // idProduct
        0x00, 0x01,                               // bcdDevice
        1,                                        // iManufacturer
        2,                                        // iProduct
        0,                                        // iSerialNumber
        1                                         // bNumConfigurations
};

enum
{
    USBD_ITF_HID,
    USBD_ITF_MAX,
};

#define USBD_PS4_DESC_LEN (TUD_CONFIG_DESC_LEN + TUD_HID_INOUT_DESC_LEN)

const uint8_t ps4_desc_hid_report[] = {
    0x05, 0x01,       // Usage Page (Generic Desktop Ctrls)
    0x09, 0x05,       // Usage (Game Pad)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0x01,       //   Report ID (1)
    0x09, 0x30,       //   Usage (X)
    0x09, 0x31,       //   Usage (Y)
    0x09, 0x32,       //   Usage (Z)
    0x09, 0x35,       //   Usage (Rz)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x04,       //   Report Count (4)
    0x81, 0x02,       //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    0x09, 0x39,       //   Usage (Hat switch)
    0x15, 0x00,       //   Logical Minimum (0)
    0x25, 0x07,       //   Logical Maximum (7)
    0x35, 0x00,       //   Physical Minimum (0)
    0x46, 0x3B, 0x01, //   Physical Maximum (315)
    0x65, 0x14,       //   Unit (System: English Rotation, Length: Centimeter)
    0x75, 0x04,       //   Report Size (4)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x42,       //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,Null State)

    0x65, 0x00, //   Unit (None)
    0x05, 0x09, //   Usage Page (Button)
    0x19, 0x01, //   Usage Minimum (0x01)
    0x29, 0x0E, //   Usage Maximum (0x0E)
    0x15, 0x00, //   Logical Minimum (0)
    0x25, 0x01, //   Logical Maximum (1)
    0x75, 0x01, //   Report Size (1)
    0x95, 0x0E, //   Report Count (14)
    0x81, 0x02, //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    0x06, 0x00, 0xFF, //   Usage Page (Vendor Defined 0xFF00)
    0x09, 0x20,       //   Usage (0x20)
    0x75, 0x06,       //   Report Size (6)
    0x95, 0x01,       //   Report Count (1)
    0x81, 0x02,       //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    0x05, 0x01,       //   Usage Page (Generic Desktop Ctrls)
    0x09, 0x33,       //   Usage (Rx)
    0x09, 0x34,       //   Usage (Ry)
    0x15, 0x00,       //   Logical Minimum (0)
    0x26, 0xFF, 0x00, //   Logical Maximum (255)
    0x75, 0x08,       //   Report Size (8)
    0x95, 0x02,       //   Report Count (2)
    0x81, 0x02,       //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    0x06, 0x00, 0xFF, //   Usage Page (Vendor Defined 0xFF00)
    0x09, 0x21,       //   Usage (0x21)
    0x95, 0x36,       //   Report Count (54)
    0x81, 0x02,       //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)

    0x85, 0x05, //   Report ID (5)
    0x09, 0x22, //   Usage (0x22)
    0x95, 0x1F, //   Report Count (31)
    0x91, 0x02, //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)

    0x85, 0x03,       //   Report ID (3)
    0x0A, 0x21, 0x27, //   Usage (0x2721)
    0x95, 0x2F,       //   Report Count (47)
    0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0xC0,             // End Collection

    0x06, 0xF0, 0xFF, // Usage Page (Vendor Defined 0xFFF0)
    0x09, 0x40,       // Usage (0x40)
    0xA1, 0x01,       // Collection (Application)
    0x85, 0xF0,       //   Report ID (-16) AUTH F0
    0x09, 0x47,       //   Usage (0x47)
    0x95, 0x3F,       //   Report Count (63)
    0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xF1,       //   Report ID (-15) AUTH F1
    0x09, 0x48,       //   Usage (0x48)
    0x95, 0x3F,       //   Report Count (63)
    0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xF2,       //   Report ID (-14) AUTH F2
    0x09, 0x49,       //   Usage (0x49)
    0x95, 0x0F,       //   Report Count (15)
    0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0x85, 0xF3,       //   Report ID (-13) Auth F3 (Reset)
    0x0A, 0x01, 0x47, //   Usage (0x4701)
    0x95, 0x07,       //   Report Count (7)
    0xB1, 0x02,       //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
    0xC0,             // End Collection
};

#define CONFIG1_DESC_SIZE (9 + 9 + 9 + 7)
const uint8_t ps4_desc_cfg[] =
    {
        // configuration descriptor, USB spec 9.6.3, page 264-266, Table 9-10
        9,                      // bLength;
        2,                      // bDescriptorType;
        LSB(CONFIG1_DESC_SIZE), // wTotalLength
        MSB(CONFIG1_DESC_SIZE),
        1,                           // bNumInterfaces
        1,                           // bConfigurationValue
        0,                           // iConfiguration
        0x80,                        // bmAttributes
        50,                          // bMaxPower
                                     // interface descriptor, USB spec 9.6.5, page 267-269, Table 9-12
        9,                           // bLength
        4,                           // bDescriptorType
        0,                           // bInterfaceNumber
        0,                           // bAlternateSetting
        1,                           // bNumEndpoints
        0x03,                        // bInterfaceClass (0x03 = HID)
        0x00,                        // bInterfaceSubClass (0x00 = No Boot)
        0x00,                        // bInterfaceProtocol (0x00 = No Protocol)
        0,                           // iInterface
                                     // HID interface descriptor, HID 1.11 spec, section 6.2.1
        9,                           // bLength
        0x21,                        // bDescriptorType
        0x11, 0x01,                  // bcdHID
        0,                           // bCountryCode
        1,                           // bNumDescriptors
        0x22,                        // bDescriptorType
        sizeof(ps4_desc_hid_report), // wDescriptorLength
        0,
        // endpoint descriptor, USB spec 9.6.6, page 269-271, Table 9-13
        7,        // bLength
        5,        // bDescriptorType
        1 | 0x80, // bEndpointAddress
        0x03,     // bmAttributes (0x03=intr)
        64, 0,    // wMaxPacketSize
        1         // bInterval (1 ms)
};

/*
// MAC Address
static uint8_t ps4_0x81_report[] = {0x39, 0x39, 0x39, 0x68, 0x22, 0x00};

// Version Info
static const uint8_t ps4_0xa3_report[] = {0x4a, 0x75, 0x6c, 0x20, 0x31, 0x31, 0x20, 0x32, 0x30, 0x31, 0x36, 0x00,
                                          0x00, 0x00, 0x00, 0x00, 0x31, 0x32, 0x3a, 0x33, 0x33, 0x3a, 0x33, 0x38,
                                          0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x04, 0x64,
                                          0x01, 0x00, 0x00, 0x00, 0x09, 0x70, 0x00, 0x02, 0x00, 0x80, 0x03, 0x00};

// Calibration Data
static const uint8_t ps4_0x02_report[] = {0x04, 0x00, 0xf9, 0xff, 0x06, 0x00, 0x1d, 0x22, 0xec, 0xdd, 0x68, 0x22,
                                          0x88, 0xdd, 0xa9, 0x23, 0x62, 0xdc, 0x1c, 0x02, 0x1c, 0x02, 0x05, 0x20,
                                          0xfb, 0xdf, 0x49, 0x20, 0xb7, 0xdf, 0x0d, 0x20, 0xf4, 0xdf, 0x01, 0x00};
*/
static const uint8_t ps4_0x03_report[] = {
    0x21, 0x27, 0x04, 0xcf, 0x00, 0x2c, 0x56,
    0x08, 0x00, 0x3d, 0x00, 0xe8, 0x03, 0x04, 0x00,
    0xff, 0x7f, 0x0d, 0x0d, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00

};

static const uint8_t ps4_output_0xf3[] = {0x0, 0x38, 0x38, 0, 0, 0, 0};

static uint8_t cur_nonce_id = 1;
static uint8_t send_nonce_part = 0;



static PS4State ps4_auth_state;
static bool ps4_auth_authsent;
static uint8_t ps4_auth_buffer[1064];
static uint8_t ps4_auth_nonce_buffer[256];

extern const unsigned char key_pem_start[] asm("_binary_key_pem_start");
extern const unsigned char key_pem_end[] asm("_binary_key_pem_end");
extern const unsigned char ps4_serial_start[] asm("_binary_serial_txt_start");
extern const unsigned char ps4_serial_end[] asm("_binary_serial_txt_end");
extern const unsigned char ps4_signature_start[] asm("_binary_sig_bin_start");
extern const unsigned char ps4_signature_end[] asm("_binary_sig_bin_end");


mbedtls_pk_context pk;
static hid_ps4_report_t last_report = {};
static uint32_t last_report_timer = 0;
static uint8_t report_counter = 0;
static uint32_t axis_timing=0;

#define PS4_KEEPALIVE_TIMER 5

// uint8_t ps4_out_buffer[64] = {};

typedef enum
{
    PS4_UNKNOWN_0X03 = 0x03,        // Unknown (PS4 Report 0x03)
    PS4_SET_AUTH_PAYLOAD = 0xF0,    // Set Auth Payload
    PS4_GET_SIGNATURE_NONCE = 0xF1, // Get Signature Nonce
    PS4_GET_SIGNING_STATE = 0xF2,   // Get Signing State
    PS4_RESET_AUTH = 0xF3           // Unknown (PS4 Report 0xF3)
} PS4AuthReport;

bool send_hid_ps4_report(hid_ps4_report_t* report)
{
    bool result = false;

    // Wake up TinyUSB device
    if (tud_suspended())
        tud_remote_wakeup();

    uint32_t now = esp_timer_get_time()/1000;
    uint16_t report_size = sizeof(hid_ps4_report_t);

    report->buttons3 |= (report_counter << 2);
    report->timestamp = axis_timing;

    if (memcmp(&last_report, report, report_size) != 0)
    {

        // HID ready + report sent, copy previous report
        if (tud_hid_ready() && tud_hid_report(0, report, report_size) == true ) {
            memcpy(&last_report, report, report_size);
        }
        // keep track of our last successful report, for keepalive purposes
        last_report_timer = now;
        result=true;
    } else {

        if ((now - last_report_timer) > PS4_KEEPALIVE_TIMER) {
            report_counter = (report_counter+1) & 0x3F;
            axis_timing = now;		 		// axis counter is 16 bits
        }

    }

    return result;
}

uint16_t hid_ps4_get_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer,
                               uint16_t reqlen)
{
    (void)itf;
    (void)reqlen;

    if (report_type == HID_REPORT_TYPE_INPUT)
    {
        memcpy(&buffer, &last_report, sizeof(hid_ps4_report_t));
        return sizeof(hid_ps4_report_t);
    }
    else if (report_type == HID_REPORT_TYPE_FEATURE)
    {
        uint8_t data[64] = {};
        uint32_t crc32 = 0;
        // ps4_out_buffer[0] = report_id;
        switch (report_id)
        {
        case PS4_UNKNOWN_0X03:
            if (reqlen != sizeof(ps4_0x03_report))
            {
                return -1;
            }
            memcpy(buffer, ps4_0x03_report, reqlen);
            return reqlen;

        case PS4_GET_SIGNATURE_NONCE:
            data[0] = 0xF1;
            data[1] = cur_nonce_id;
            data[2] = send_nonce_part;
            data[3] = 0;

            memcpy(&data[4], &ps4_auth_buffer[send_nonce_part * 56], 56);
            crc32 = esp_rom_crc32_le(crc32, data, 60);
            memcpy(&data[60], &crc32, sizeof(uint32_t));
            memcpy(buffer, &data[1], 63);
            if ((++send_nonce_part) == 19)
            {
                ps4_auth_state = no_nonce;
                ps4_auth_authsent = true;
                send_nonce_part = 0;
            }
            return 63;

        case PS4_GET_SIGNING_STATE:
            data[0] = 0xF2;
            data[1] = cur_nonce_id;
            data[2] = ps4_auth_state == signed_nonce_ready ? 0 : 16;
            memset(&data[3], 0, 9);
            crc32 = esp_rom_crc32_le(crc32, data, 60);
            memcpy(&data[12], &crc32, sizeof(uint32_t));
            memcpy(buffer, &data[1], 15); // move data over to buffer
            return 15;
        case PS4_RESET_AUTH:
            if (reqlen != sizeof(ps4_output_0xf3))
            {
                return -1;
            }
            memcpy(buffer, ps4_output_0xf3, reqlen);
            ps4_auth_state = no_nonce;
            return reqlen;

        default:
            return -1;
        }
    }
    else
    {

        ESP_LOGI("PS4_DRV", "ignoring GetReport on output report");
    }

    return 0;
}



int rng(void *p_rng, unsigned char *p, size_t len)
{
    (void)p_rng;
    esp_fill_random(p, len);
    return 0;
};

void sign_nonce(void)
{

    int rss_error = 0;
    uint8_t hashed_nonce[32];
    if (ps4_auth_state == nonce_ready)
    {

        uint8_t nonce_signature[256];

        if (mbedtls_sha256(ps4_auth_nonce_buffer, 256, hashed_nonce, 0) < 0)
        {
            return;
        }

        mbedtls_rsa_context *rsa = mbedtls_pk_rsa(pk);
        rss_error = mbedtls_rsa_rsassa_pss_sign(rsa, rng, NULL, MBEDTLS_MD_SHA256,
                                                32, hashed_nonce,
                                                nonce_signature);

        if (rss_error < 0)
        {
            return;
        }

        int offset = 0;
        memcpy(&ps4_auth_buffer[offset], nonce_signature, 256);
        offset += 256;
        memcpy(&ps4_auth_buffer[offset], ps4_serial_start, 16);
        offset += 16;
        // mbedtls_mpi *mpi = static_cast<mbedtls_mpi *>(&rsa_context->N);
        mbedtls_mpi_write_binary(&rsa->MBEDTLS_PRIVATE(N), &ps4_auth_buffer[offset], 256);
        offset += 256;
        // mpi = static_cast<mbedtls_mpi *>(&rsa_context->E);
        mbedtls_mpi_write_binary(&rsa->MBEDTLS_PRIVATE(E), &ps4_auth_buffer[offset], 256);
        offset += 256;
        memcpy(&ps4_auth_buffer[offset], ps4_signature_start, 256);
        offset += 256;
        memset(&ps4_auth_buffer[offset], 0, 24);

        ps4_auth_state=signed_nonce_ready;
    }
}

void save_nonce(uint8_t nonce_id, uint8_t nonce_page, uint8_t *buffer, uint16_t buflen)
{

    if (nonce_page != 0 && nonce_id != cur_nonce_id)
    {
        ps4_auth_state = no_nonce;
        return; // setting nonce with mismatched id
    }

    memcpy(&ps4_auth_nonce_buffer[nonce_page * 56], buffer, buflen);
    if (nonce_page == 4)
    {

        ps4_auth_state = nonce_ready;

        sign_nonce();
    }
    else if (nonce_page == 0)
    {
        cur_nonce_id = nonce_id;
        ps4_auth_state = receiving_nonce;
    }
}

void hid_ps4_set_report_cb(uint8_t itf, uint8_t report_id, hid_report_type_t report_type, uint8_t const *data,
                           uint16_t bufsize)
{
    (void)itf;

    uint8_t nonce_id;
    uint8_t nonce_page;
    uint32_t crc32 = 0;
    uint8_t buffer[64];
    uint8_t nonce[56];
    uint16_t noncelen;
    uint16_t buflen;

    if (report_type == HID_REPORT_TYPE_FEATURE)
    {
        if (report_id == PS4_SET_AUTH_PAYLOAD)
        {

            if (bufsize != 63)
            {
                return;
            }

            buffer[0] = report_id;
            memcpy(&buffer[1], data, bufsize);
            buflen = bufsize + 1;
            nonce_id = data[0];
            nonce_page = data[1];
            crc32 = esp_rom_crc32_le(crc32, buffer, buflen - sizeof(uint32_t));
            if (crc32 != *((unsigned int *)&buffer[buflen - sizeof(uint32_t)]))
            {
                ESP_LOGI("PS4_DRV", "CRC32 failed on set report");
                return; // CRC32 failed on set report
            }
            // 256 byte nonce, with 56 byte packets leaves 24 extra bytes on the last packet?
            if (nonce_page == 4)
            {
                // Copy/append data from buffer[4:64-28] into our nonce
                noncelen = 32; // from 4 to 64 - 24 - 4
            }
            else
            {
                // Copy/append data from buffer[4:64-4] into our nonce
                noncelen = 56;
                // from 4 to 64 - 4
            }

            memcpy(nonce, &buffer[4], noncelen);
            save_nonce(nonce_id, nonce_page, nonce, noncelen);
        }
    }
}

void ps4_driver_init(void)
{

    ps4_auth_state = no_nonce;
    ps4_auth_authsent = false;
    memset(ps4_auth_buffer, 0, sizeof(ps4_auth_buffer));
    memset(ps4_auth_nonce_buffer, 0, sizeof(ps4_auth_nonce_buffer));

    int ret = 0;
    mbedtls_pk_init(&pk);
    ret = mbedtls_pk_parse_key(&pk, (uint8_t *)key_pem_start, key_pem_end - key_pem_start, NULL, 0, rng, NULL);

    if (ret)

    {
        ESP_LOGI("PS4_AUTH", " failed\n  ! mbedtls_pk_parse_public_key returned 0x%04x\n\n",
                 -ret);
    }

    mbedtls_rsa_context *rsa = mbedtls_pk_rsa(pk);
    if ((ret = mbedtls_rsa_complete(rsa)) != 0)
    {
        printf(" failed\n  ! mbedtls_rsa_complete returned %d\n\n",
               ret);
    }

    rsa->MBEDTLS_PRIVATE(padding) = MBEDTLS_RSA_PKCS_V21;
    rsa->MBEDTLS_PRIVATE(hash_id) = MBEDTLS_MD_SHA256;

    if ((ret = mbedtls_rsa_complete(rsa)) != 0)
    {
        printf(" failed\n  ! mbedtls_rsa_complete returned %d\n\n",
               ret);
    }

    if ((ret = mbedtls_rsa_check_privkey(rsa)) != 0)
    {
        printf(" failed\n  ! mbedtls_rsa_check_privkey failed with -0x%0x\n",
               (unsigned int)-ret);
    }
}