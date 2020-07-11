Setup
plugin_names:
    {
        adis_imu_serial : hw_interface_plugin_adis_imu,
    }

adis_imu_serial:
    {
        adis_imu : "adis_imu",
    }

adis_imu:
    {
        deviceName         : "/dev/serial/by-path/pci-0000:00:14.0-usb-0:1.1:1.0-port0",
        baudrate           : 115200,
        subscribeToTopic   : "/example/subscribe",
        publishToTopic     : "/imu/data",
        publishToTopicAdis : "/imu/dataAdis",
    }

