set(srcs "esp_hid_host_main.c"
         "esp_hid_gap.c"
	 "hid_ps4_driver.c")
set(include_dirs ".")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES driver esp_hid mbedtls esp_timer
                       PRIV_REQUIRES nvs_flash
		       EMBED_TXTFILES "key.pem" "serial.txt" "sig.bin")
