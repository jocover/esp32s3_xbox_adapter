# Xbox series Controller on PS4/PS5 or PC

Xbox Series Controlle only support BLE version.
Need to use BLE and USB OTG feature, so only supports esp32s3 now.

PS4 connection requires private key signature, you need to find key.pem, serial.txt, sig.bin files by yourself


# Reference

https://github.com/OpenStickCommunity/GP2040-CE
https://github.com/ravinrabbid/DivaCon2040
https://github.com/espressif/esp-idf/tree/master/examples/bluetooth/esp_hid_host
https://github.com/chrepl
https://github.com/asukiaaa/arduino-XboxControllerNotificationParser