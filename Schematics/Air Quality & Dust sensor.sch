EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "MySensors Dust & air quality sensor"
Date "20181213"
Rev "1"
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L MCU_Module:Arduino_Pro_Mini_3V3_8Mhz A?
U 1 1 5E98BD9F
P 5900 3200
F 0 "A?" H 5900 2061 50  0001 C CNN
F 1 "Arduino_Pro_Mini_3V3_8Mhz" V 5900 3300 50  0000 C CNN
F 2 "Module:Arduino_Pro_Mini" H 5900 3200 50  0001 C CIN
F 3 "https://store.arduino.cc/arduino-pro-mini" H 5900 3200 50  0001 C CNN
	1    5900 3200
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR?
U 1 1 5E9A7E7D
P 6000 2200
F 0 "#PWR?" H 6000 2050 50  0001 C CNN
F 1 "+3V3" H 6015 2373 50  0000 C CNN
F 2 "" H 6000 2200 50  0001 C CNN
F 3 "" H 6000 2200 50  0001 C CNN
	1    6000 2200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E9AA06C
P 5900 4250
F 0 "#PWR?" H 5900 4000 50  0001 C CNN
F 1 "GND" H 5905 4077 50  0000 C CNN
F 2 "" H 5900 4250 50  0001 C CNN
F 3 "" H 5900 4250 50  0001 C CNN
	1    5900 4250
	1    0    0    -1  
$EndComp
$Comp
L pspice:INDUCTOR L
U 1 1 5E9B018B
P 3550 3050
F 0 "L" H 3550 3265 50  0001 C CNN
F 1 "220 μH" H 3550 3173 50  0000 C CNN
F 2 "" H 3550 3050 50  0001 C CNN
F 3 "~" H 3550 3050 50  0001 C CNN
	1    3550 3050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E9B3A0F
P 3550 3700
F 0 "#PWR?" H 3550 3450 50  0001 C CNN
F 1 "GND" H 3555 3527 50  0000 C CNN
F 2 "" H 3550 3700 50  0001 C CNN
F 3 "" H 3550 3700 50  0001 C CNN
	1    3550 3700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E9B78D8
P 4250 4600
F 0 "#PWR?" H 4250 4350 50  0001 C CNN
F 1 "GND" H 4255 4427 50  0000 C CNN
F 2 "" H 4250 4600 50  0001 C CNN
F 3 "" H 4250 4600 50  0001 C CNN
	1    4250 4600
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR?
U 1 1 5E9B41A8
P 3550 2700
F 0 "#PWR?" H 3550 2550 50  0001 C CNN
F 1 "+3V3" V 3565 2828 50  0000 L CNN
F 2 "" H 3550 2700 50  0001 C CNN
F 3 "" H 3550 2700 50  0001 C CNN
	1    3550 2700
	1    0    0    -1  
$EndComp
Wire Wire Line
	4750 3700 5400 3700
Wire Wire Line
	4750 3800 5400 3800
Wire Wire Line
	4750 3900 5400 3900
Wire Wire Line
	4750 4000 4900 4000
Wire Wire Line
	4900 4000 4900 3600
Wire Wire Line
	4900 3600 5400 3600
Wire Wire Line
	4750 4200 5000 4200
Wire Wire Line
	5000 4200 5000 3500
Wire Wire Line
	5000 3500 5400 3500
$Comp
L _BM_library:NRF24L01_Module U?
U 1 1 5E9CA4BC
P 4250 4000
F 0 "U?" H 3870 4046 50  0001 R CNN
F 1 "NRF24L01_Module" H 5000 4550 50  0000 R CNN
F 2 "RF_Module:nRF24L01_Module" H 4400 4600 50  0001 L CIN
F 3 "" H 4250 3900 50  0001 C CNN
	1    4250 4000
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C?
U 1 1 5E9CF6D2
P 3550 3550
F 0 "C?" H 3668 3596 50  0001 L CNN
F 1 "4.7μ" H 3668 3550 50  0000 L CNN
F 2 "" H 3588 3400 50  0001 C CNN
F 3 "~" H 3550 3550 50  0001 C CNN
	1    3550 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 3400 4250 3400
$Comp
L power:+5V #PWR?
U 1 1 5E9DE5AD
P 5800 2200
F 0 "#PWR?" H 5800 2050 50  0001 C CNN
F 1 "+5V" H 5815 2373 50  0000 C CNN
F 2 "" H 5800 2200 50  0001 C CNN
F 3 "" H 5800 2200 50  0001 C CNN
	1    5800 2200
	1    0    0    -1  
$EndComp
$Comp
L _BM_library:PMS1003_DUST U?
U 1 1 5E9DF033
P 4300 2550
F 0 "U?" H 4071 2596 50  0001 R CNN
F 1 "PMS1003_DUST_sensor" H 5200 2850 50  0000 R CNN
F 2 "" H 4300 2150 50  0001 C CNN
F 3 "" H 4450 2800 50  0001 C CNN
	1    4300 2550
	1    0    0    -1  
$EndComp
$Comp
L LED:WS2812B D?
U 1 1 5E9E0024
P 7650 3850
F 0 "D?" H 7994 3896 50  0001 L CNN
F 1 "WS2811_RGB" H 7800 3550 50  0000 L CNN
F 2 "LED_SMD:LED_WS2812B_PLCC4_5.0x5.0mm_P3.2mm" H 7700 3550 50  0001 L TNN
F 3 "https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf" H 7750 3475 50  0001 L TNN
	1    7650 3850
	1    0    0    -1  
$EndComp
$Comp
L LED:WS2812B D?
U 1 1 5E9E2072
P 8500 3850
F 0 "D?" H 8844 3896 50  0001 L CNN
F 1 "WS2811_RGB" H 8650 3550 50  0000 L CNN
F 2 "LED_SMD:LED_WS2812B_PLCC4_5.0x5.0mm_P3.2mm" H 8550 3550 50  0001 L TNN
F 3 "https://cdn-shop.adafruit.com/datasheets/WS2812B.pdf" H 8600 3475 50  0001 L TNN
	1    8500 3850
	1    0    0    -1  
$EndComp
$Comp
L _BM_library:MQ_135_Air_Quality_sensor U?
U 1 1 5E9E728C
P 8050 2650
F 0 "U?" H 7750 2950 79  0001 C CNN
F 1 "MQ_135_Air_Quality_sensor" H 9000 3000 79  0000 C CNN
F 2 "" H 8050 2650 79  0001 C CNN
F 3 "https://www.olimex.com/Products/Components/Sensors/Gas/SNS-MQ135/resources/SNS-MQ135.pdf" H 8050 2650 79  0001 C CNN
	1    8050 2650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R?
U 1 1 5E9E7B25
P 7400 3050
F 0 "R?" H 7470 3096 50  0001 L CNN
F 1 "10k" H 7470 3050 50  0000 L CNN
F 2 "" V 7330 3050 50  0001 C CNN
F 3 "~" H 7400 3050 50  0001 C CNN
	1    7400 3050
	1    0    0    -1  
$EndComp
$Comp
L Device:CP C?
U 1 1 5E9E85DC
P 9150 1500
F 0 "C?" H 9268 1546 50  0001 L CNN
F 1 "100μ" H 9268 1500 50  0000 L CNN
F 2 "" H 9188 1350 50  0001 C CNN
F 3 "~" H 9150 1500 50  0001 C CNN
	1    9150 1500
	1    0    0    -1  
$EndComp
$Comp
L Connector:Conn_Coaxial_Power J?
U 1 1 5E9E9EE4
P 9650 1350
F 0 "J?" V 9875 1300 50  0001 C CNN
F 1 "Power 5V" H 9738 1300 50  0000 L CNN
F 2 "" H 9650 1300 50  0001 C CNN
F 3 "~" H 9650 1300 50  0001 C CNN
	1    9650 1350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E9EBACE
P 9150 1650
F 0 "#PWR?" H 9150 1400 50  0001 C CNN
F 1 "GND" H 9155 1477 50  0000 C CNN
F 2 "" H 9150 1650 50  0001 C CNN
F 3 "" H 9150 1650 50  0001 C CNN
	1    9150 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E9ECAE0
P 9650 1650
F 0 "#PWR?" H 9650 1400 50  0001 C CNN
F 1 "GND" H 9655 1477 50  0000 C CNN
F 2 "" H 9650 1650 50  0001 C CNN
F 3 "" H 9650 1650 50  0001 C CNN
	1    9650 1650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E9EDCF6
P 8050 3000
F 0 "#PWR?" H 8050 2750 50  0001 C CNN
F 1 "GND" H 8055 2827 50  0000 C CNN
F 2 "" H 8050 3000 50  0001 C CNN
F 3 "" H 8050 3000 50  0001 C CNN
	1    8050 3000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E9EE9CE
P 7400 3200
F 0 "#PWR?" H 7400 2950 50  0001 C CNN
F 1 "GND" H 7405 3027 50  0000 C CNN
F 2 "" H 7400 3200 50  0001 C CNN
F 3 "" H 7400 3200 50  0001 C CNN
	1    7400 3200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E9EEFA5
P 7650 4150
F 0 "#PWR?" H 7650 3900 50  0001 C CNN
F 1 "GND" H 7655 3977 50  0000 C CNN
F 2 "" H 7650 4150 50  0001 C CNN
F 3 "" H 7650 4150 50  0001 C CNN
	1    7650 4150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E9EFC76
P 8500 4150
F 0 "#PWR?" H 8500 3900 50  0001 C CNN
F 1 "GND" H 8505 3977 50  0000 C CNN
F 2 "" H 8500 4150 50  0001 C CNN
F 3 "" H 8500 4150 50  0001 C CNN
	1    8500 4150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5E9F02E7
P 4300 2850
F 0 "#PWR?" H 4300 2600 50  0001 C CNN
F 1 "GND" H 4305 2677 50  0000 C CNN
F 2 "" H 4300 2850 50  0001 C CNN
F 3 "" H 4300 2850 50  0001 C CNN
	1    4300 2850
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 5E9F0CCE
P 8050 2300
F 0 "#PWR?" H 8050 2150 50  0001 C CNN
F 1 "+5V" H 8065 2473 50  0000 C CNN
F 2 "" H 8050 2300 50  0001 C CNN
F 3 "" H 8050 2300 50  0001 C CNN
	1    8050 2300
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 5E9F2861
P 8700 1250
F 0 "#PWR?" H 8700 1100 50  0001 C CNN
F 1 "+5V" V 8715 1378 50  0000 L CNN
F 2 "" H 8700 1250 50  0001 C CNN
F 3 "" H 8700 1250 50  0001 C CNN
	1    8700 1250
	0    -1   -1   0   
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 5E9F4181
P 4300 2150
F 0 "#PWR?" H 4300 2000 50  0001 C CNN
F 1 "+5V" H 4315 2323 50  0000 C CNN
F 2 "" H 4300 2150 50  0001 C CNN
F 3 "" H 4300 2150 50  0001 C CNN
	1    4300 2150
	1    0    0    -1  
$EndComp
$Comp
L power:+3V3 #PWR?
U 1 1 5E9F476F
P 7600 2300
F 0 "#PWR?" H 7600 2150 50  0001 C CNN
F 1 "+3V3" H 7615 2473 50  0000 C CNN
F 2 "" H 7600 2300 50  0001 C CNN
F 3 "" H 7600 2300 50  0001 C CNN
	1    7600 2300
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 5E9F4EEA
P 7650 3550
F 0 "#PWR?" H 7650 3400 50  0001 C CNN
F 1 "+5V" H 7665 3723 50  0000 C CNN
F 2 "" H 7650 3550 50  0001 C CNN
F 3 "" H 7650 3550 50  0001 C CNN
	1    7650 3550
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 5E9F6093
P 8500 3550
F 0 "#PWR?" H 8500 3400 50  0001 C CNN
F 1 "+5V" H 8515 3723 50  0000 C CNN
F 2 "" H 8500 3550 50  0001 C CNN
F 3 "" H 8500 3550 50  0001 C CNN
	1    8500 3550
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 2300 7600 2550
Wire Wire Line
	7600 2550 7700 2550
Wire Wire Line
	7700 2750 7400 2750
Wire Wire Line
	7400 2750 7400 2900
Wire Wire Line
	6400 3200 6850 3200
Wire Wire Line
	6850 3200 6850 2750
Wire Wire Line
	6850 2750 7400 2750
Connection ~ 7400 2750
Wire Wire Line
	7950 3850 8200 3850
Wire Wire Line
	5400 3200 5100 3200
Wire Wire Line
	5100 3200 5100 4550
Wire Wire Line
	5100 4550 7300 4550
Wire Wire Line
	7300 4550 7300 3850
Wire Wire Line
	7300 3850 7350 3850
Wire Wire Line
	8700 1250 9150 1250
Wire Wire Line
	9150 1350 9150 1250
Connection ~ 9150 1250
Wire Wire Line
	9150 1250 9650 1250
Wire Wire Line
	4300 2150 4300 2250
Wire Wire Line
	3550 2700 3550 2800
Wire Wire Line
	3550 3300 3550 3400
Connection ~ 3550 3400
Wire Wire Line
	4600 2500 5100 2500
Wire Wire Line
	5100 2500 5100 2900
Wire Wire Line
	5100 2900 5400 2900
Wire Wire Line
	9650 1550 9650 1650
Text Notes 9075 2575 0    39   ~ 0
MQ135 R value
Text Notes 8625 2575 0    39   ~ 0
Air quality
Text Notes 9625 2575 0    39   ~ 0
ADC perc
Text Notes 9075 2675 0    39   ~ 0
230 kΩ
Text Notes 8625 2675 0    39   ~ 0
Clean
Text Notes 9625 2675 0    39   ~ 0
4%
Text Notes 9075 2775 0    39   ~ 0
50 kΩ
Text Notes 8625 2775 0    39   ~ 0
Ok
Text Notes 9625 2775 0    39   ~ 0
17%
Text Notes 9075 2875 0    39   ~ 0
1 kΩ
Text Notes 8625 2875 0    39   ~ 0
Poluted
Text Notes 9625 2875 0    39   ~ 0
91%
Wire Notes Line
	8575 2475 8575 2925
Wire Notes Line
	8575 2925 9975 2925
Wire Notes Line
	9975 2925 9975 2475
Wire Notes Line
	9975 2475 8575 2475
Wire Notes Line style solid
	8575 2600 9975 2600
$EndSCHEMATC
