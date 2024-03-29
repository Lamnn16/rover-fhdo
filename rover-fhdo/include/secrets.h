/**
 * ESP32 secrets file
 * 
 * Contains information required to connect to WiFi and in-turn, AWS IoT
 * 
 * Authors: Vipul Deshpande, Jaime Burbano
 */


/*
  Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
  Permission is hereby granted, free of charge, to any person obtaining a copy of this
  software and associated documentation files (the "Software"), to deal in the Software
  without restriction, including without limitation the rights to use, copy, modify,
  merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
  permit persons to whom the Software is furnished to do so.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
  HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
  OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/


#include <pgmspace.h>

#define SECRET
#define THINGNAME "PublisherG05"

const char WIFI_SSID[] = "nhatlam";
const char WIFI_PASSWORD[] = "12345678";
const char AWS_IOT_ENDPOINT[] = "a1ah0y2qxdql5g-ats.iot.eu-central-1.amazonaws.com";

/* Amazon Root CA 1 */
static const char AWS_CERT_CA[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF
ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6
b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL
MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv
b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj
ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM
9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw
IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6
VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L
93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm
jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC
AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA
A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI
U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs
N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv
o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU
5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy
rqXRfboQnoZsG4q5WTP468SQvvG5
-----END CERTIFICATE-----
)EOF";

/* Device Certificate */
static const char AWS_CERT_CRT[] PROGMEM = R"KEY(
-----BEGIN CERTIFICATE-----
MIIDWjCCAkKgAwIBAgIVAPFJwBLp6Ayn1i2HO1S0YdpneGO6MA0GCSqGSIb3DQEB
CwUAME0xSzBJBgNVBAsMQkFtYXpvbiBXZWIgU2VydmljZXMgTz1BbWF6b24uY29t
IEluYy4gTD1TZWF0dGxlIFNUPVdhc2hpbmd0b24gQz1VUzAeFw0yMzExMzAyMDI0
MjRaFw00OTEyMzEyMzU5NTlaMB4xHDAaBgNVBAMME0FXUyBJb1QgQ2VydGlmaWNh
dGUwggEiMA0GCSqGSIb3DQEBAQUAA4IBDwAwggEKAoIBAQDkVgmQwpAn3ohclsqr
+M7+unJb0RGT+cQsqcXm1xI1guOw95nYqzzLZy6luHHvMpEdMo7hHcShVwhwLfXG
TvikQl6t5nsYMFVvdIAEOWjQ5tb7wUUZxb/L4FX6QVWcOZhxChEheYbFTmGwmKsv
BJr7nV6CnuXiQSoctfPlkTWqkL/kbML4GpR3b0mM3VQ4eUIcxZx7oFi1neKTHLE3
iB1FPfqgs3rmf/Uo71M5aaC0g4ytYS0iMwUa0DR4cPtmuRVVWRFVgt96PHqMwCNT
PBnIL2Vgh+oKBDQCNMXnZcOrcPgX+kVpELRoP/KtRkFSmRj0Jlx1cr3c017BGm9Z
5MgbAgMBAAGjYDBeMB8GA1UdIwQYMBaAFBwjtk75orYzyJ7Jzs35x3WPb0GsMB0G
A1UdDgQWBBTlrLrhrLpCZm+hcyuYhoXSfqGGuTAMBgNVHRMBAf8EAjAAMA4GA1Ud
DwEB/wQEAwIHgDANBgkqhkiG9w0BAQsFAAOCAQEAVazpMFv/l1udpqO83BKS4c4W
3+NwmhsTu66lN67H0XVAYVs628m+u4ulAxN0ITzD2qoJ4VGFsAlsjGxUvNzQ052h
49idssG31ZSpz1hzwCXS9KoJj5ZOJ3M6awHxtDDLk/g2oWKuIiPq/6jyV6oHiXvs
2Olu+Hg6e/9rQaHpGP3wg3tm1k3HVToZfzXxE1gjhLLG5Cjlg1iylWES9T34g1N5
2jFPV59i0QyxxzWBqdVNlM5hnQvSTJXa5LHZ9m6zCzSQLwZQVQm9wRZIW7d5iiNY
iA2BHwP1p64nJdMtSbJ0yvNgoxkPffuWAZPsKDltecczIK2Sx7WE23UDMkGsOQ==
-----END CERTIFICATE-----
)KEY";

/* Device Private Key */
static const char AWS_CERT_PRIVATE[] PROGMEM = R"KEY(
-----BEGIN RSA PRIVATE KEY-----
MIIEowIBAAKCAQEA5FYJkMKQJ96IXJbKq/jO/rpyW9ERk/nELKnF5tcSNYLjsPeZ
2Ks8y2cupbhx7zKRHTKO4R3EoVcIcC31xk74pEJereZ7GDBVb3SABDlo0ObW+8FF
GcW/y+BV+kFVnDmYcQoRIXmGxU5hsJirLwSa+51egp7l4kEqHLXz5ZE1qpC/5GzC
+BqUd29JjN1UOHlCHMWce6BYtZ3ikxyxN4gdRT36oLN65n/1KO9TOWmgtIOMrWEt
IjMFGtA0eHD7ZrkVVVkRVYLfejx6jMAjUzwZyC9lYIfqCgQ0AjTF52XDq3D4F/pF
aRC0aD/yrUZBUpkY9CZcdXK93NNewRpvWeTIGwIDAQABAoIBAQCDEjBjfIwrlr5I
B/dHSIfqCTTkJ9sjSlimGr6TiBnXxc7WIwuZKJK5X/2S+5HLcQeD7yW9a80XoFIj
sv5GW8a247JduNQoSnaAiCuEQUA9yoTV91enN7ClY513DlIKX7V2DgVPZedLE3dY
XArH7qjYjrlV2ci0XBvEZ14eBYD86wyl0vjpYrFwSB8OrUKxfLl+iHduLLnXx+zJ
Qy7UwGHxVxLBWSaLovEa9X25qp4usBpShenb8qTTHhtcxkfSc1mbwRZiPCdngOTE
3/RVWm5yEtmnsBUB9q8x/thg8+TrbC7wLZPI38oRSPWwHZlv0EE3M57s3neWJhvo
iEEYLf4BAoGBAPyRE8GIdltowp89PMnVp05wfyHW6mSa+nvg48E3SUgsB5BYxJnk
ldkPevV8FL5FMcuQHNCs6nCSfqNgE18634bS8iihSqQajwzmskMTYHBz9T4rO/bW
1gWAKuSq+rMmg1U1pBpsvj0JQpDlpgpQt9YnpkLuU673YXwSW9YES2TbAoGBAOdw
o3WwPHhLm8Q0fK7eZqHNMphuTDuNTWIrbmxeGpW4OOswokGvnp/3HO6pimYyJDWK
sCXlwFdNv9f0j43LXLM8E1SUaPGcbfNP2Ey14/b7pVh8TPrHvzMae0Kn/PLaxXGy
J4qCdooXPuEUEC+DKU3aijz24Bii/Yj55YoBve3BAoGAFP3d3X0BEVHgsyl8zu2y
DSPufRqGurviy78CTwHQVm5KZqXqGTxVdAgEWTjp5HKN0/RzYKiuh5K1cdC/a2/S
Ojt4IwZc1MzcTN8zu711i4Mnw/YF8tsiSKzBY+YOFivCSKe/ru6Q1TPa034y0bKp
xOPgvrlePTGUaaKA/YW32ncCgYAFM6no3kItrf1dRpS61XLXFiaZ+HisKfvgRfug
YekVDmPxHVG4Sjs1ezdiQdtDesMHlQtqfqR5Ed1K/Esxs61std/1WbSWIZ7zBgtl
hf0Osw8/UuPkCxNIEcNNdzsfNj2T1Z4/5bjGV8lpA6ttdfQ6tLvArQfSZe9u4bcA
CBP9wQKBgBMyXApzU2HUcdhhThUCXE/lwXwYvZojxDutnLX02JIOWsxyv38Vtn+X
nj7OoKiuOr0N/QeQVd95P4jGtKPGl7MQGlFxREMG/3xLPJpakfR0SCh9Wg54vRB8
Kn95rXh6Qlv5EyKscobJfUIbQHvjEs2XXKh1Aj6aHtyM2VZqtZ/j
-----END RSA PRIVATE KEY-----
)KEY";