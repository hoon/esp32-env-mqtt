esp32-env-mqtt
====================

This is a demo application that uses libraries that ship with ESP-IDF to periodically send data over MQTT. It obtains temperature, pressure, and humidity from a BME280 sensor module over I2C then sends the data over MQTT.

While the AWS IoT library that ships with ESP-IDF supports communicating with non-AWS MQTT servers, the example included in ESP-IDF does not explain how to do this. env32-env-mqtt demo application and the accompanying document will go into detail on communicating with non-AWS MQTT servers including how to generate certificates to facilitate encrypted MQTT communication.

This document assumes you will be using the [Mosquitto](https://mosquitto.org/) MQTT server.

## Obtaining a copy of esp32-env-mqtt

Because esp32-env-mqtt uses BME280 driver provided by Bosch Sensortec as a git submodule, you should use the following command to recursively clone the repository:

`git clone --recursive https://github.com/hoon/esp32-env-mqtt.git`

If you missed the `--recursive` option, you can do the following to get the submodule:

```
cd esp32-env-mqtt
git submodule update --init
```

## Generating TLS certificates

You can follow the instruction on Mosquitto's [documentation on TLS](https://mosquitto.org/man/mosquitto-tls-7.html), but there are some caveats with client accessing MQTT with the AWS IoT client library.

First, the certificates should be in Privacy Enhanced Mail (PEM) format (the first line should contain `BEGIN CERTIFICATE`), not OpenSSL Trusted Certificate format (which will have its first line contain `BEGIN TRUSTED CERTIFICATE` instead). OpenSSL Trusted Cerficiate format is not recognized by mbed TLS library used by the AWS IoT library.

Second, the client key should not be protected with password as passing password is not supported by the AWS IoT library. You probably shouldn't be using password with the Mosquitto server key either since this will require you enter password every time you start the Mosquitto server, preventing it from automatically running as a service.

So, to generate your Certificate Authority (CA) certificate and key:

`openssl req -new -x509 -days <duration> -extensions v3_ca -keyout ca.key -outform pem -out root-ca.pem`

Generate the server key:

`openssl genrsa -out server.key 2048`

Generate a certificate signing request (CSR) for the server:

`openssl req -out server.csr -key server.key -new`

Note that when prompted for Common Name, you must enter the domain name or the IP address the Mosquitto server will be accessible from (e.g. mqtt.example.com, 10.0.0.7).

Generate a server certificate with the CSR, the CA certificate, and the CA key:

`openssl x509 -req -in server.csr -CA root-ca.pem -CAkey ca.key -CAcreateserial -outform pem -out server.crt -days <duration>`

Generate a client key:

`openssl genrsa -out private.pem.key`

Generate a certificate signing requrest (CSR) for the client:

`openssl req -out client.csr -key private.pem.key -new`

Generate a client certificate with the CSR, the CA certificate, and the CA key:

`openssl x509 -req -in client.csr -CA root-ca.pem -CAkey ca.key -CAcreateserial -outform pem -out certificate.pem.crt -days <duration>`

`root-ca.pem`, `private.pem.key`, and `certificate.pem.crt` will be copied to the ESP32 module running this demo application. These file names follow the convention used by the AWS IoT framework.

## Configuring the Mosquitto MQTT server/broker

Install the Mosquitto server. On Debian (Stretch), you can install by typing the following:

`apt install mosquitto`

Assuming you're using Debian (Stretch), copy `root-ca.pem` to `/etc/mosquitto/ca_certificates`, and copy `server.key` and `server.crt` to `/etc/mosquitto/certs`.

Configure Mosquitto to use TLS by creating `tls.conf` file (it can be any name as long as it ends in `.conf`) in the `/etc/mosquitto/conf.d` directory with the following content.

```
listener 8883

cafile /etc/mosquitto/ca_certificates/root-ca.pem
certfile /etc/mosquitto/certs/server.crt
keyfile /etc/mosquitto/certs/server.key

require_certificate true
use_identity_as_username true

allow_anonymous false```

Restart the Mosquitto server:

`systemctl restart mosquitto`

Test TLS is enabled on port 8883:

`openssl s_client -CAfile /etc/mosquitto/ca_certificates/root-ca.pem -connect <server address>:8883 < /dev/null | head -10`

You may see an `sslv3` related warning message, but it's fine as long as `verify return` code is `1`.

Note that if you're running Mosquitto on your computer, `<server address>` should be the Common Name (CN) you entered while generating the server CSR, not `localhost`. If you don't use the CN, you will get an error message.

You might want to try connecting with client certificates using `mosquitto_sub` and `mosquitto_pub` commands provided by the `mosquitto-clients` apt package.

`mosquitto_sub --cafile /etc/mosquitto/ca_certificates/root-ca.pem -h <server address> -p 8883 --cert <certificate.pem.crt> --key <private.pem.key> -t <topic name>`

Open another terminal and send some message to the subscribed topic:

`mosquitto_pub --cafile <root-ca.pem> -h <server_address> -p 8883 --cert <certificate.pem.crt> --key <private.pem.key> -t <topic name> -m 'Hello World!'`

## Configuring the esp32-env-mqtt demo application

Copy the previously generated `root-ca.pem`, `private.pem.key`, and `certificate.pem.crt` files to the `main/certs` directory.

From the main project directory, run:

`make menuconfig`

Configure your Wi-Fi setting under the `esp32-env-mqtt configuration` section.

Ensure `Amazon Web Services IoT Platform` is selected under `Component config`.

Also ensure 'Tick rate (HZ)' is set to 1000 (100 is default) under `Component config` &rarr; `FreeRTOS`. There is an issue with reading BME280 module over I2C when the tick rate is set to 100 Hz.

Build and flash the project then start monitoring the ESP32 client output:

`make -j5 flash monitor`

In the monitor, you should be seeing output like this:

```
I (10236) envmon: Temp 22.53 C, Pres 981.53, Hum 36.99%
I (20238) envmon: Temp 22.69 C, Pres 981.53, Hum 36.88%
I (30238) envmon: Temp 22.67 C, Pres 981.56, Hum 37.00%
```

To verity the data is being sent to the Mosquitto server, you can subscribe to the topic the ESP32 module is sending sensor readings to using `mosquitto_sub`:

`mosquitto_sub --cafile <root-ca.pem> -h <server address> -p 8883 -cert <certificate.pem.crt> --key <private.pem.key> -t "env/room-01/sensor-01"`

You should see output like this:

```
{"temp_c": 22.53, "pressure_hpa": 981.53, "humidity_pct": 36.99}
{"temp_c": 22.69, "pressure_hpa": 981.53, "humidity_pct": 36.88}
{"temp_c": 22.67, "pressure_hpa": 981.56, "humidity_pct": 37.00}
```

Note `env/room-01/sensor-01` is the default topic for the demo application. If you changed it in the `menuconfig`, you should use what you changed it to instead.

## Credits
This demo application uses codes from several demos included with ESP-IDF, specifically the [I2C](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/i2c), [AWS IoT Subscribe/Publish](https://github.com/espressif/esp-idf/tree/master/examples/protocols/aws_iot/subscribe_publish), and [Timer](https://github.com/espressif/esp-idf/tree/master/examples/peripherals/timer_group) demos.

I've also peeked at yanbe's [bme280-esp-idf-i2c](https://github.com/yanbe/bme280-esp-idf-i2c) project to get some hints on using BME280 from ESP-IDF.
