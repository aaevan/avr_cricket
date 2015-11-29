EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:newcricket-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L ATTINY85-P IC1
U 1 1 565B5313
P 5300 3600
F 0 "IC1" H 4150 4000 50  0000 C CNN
F 1 "ATTINY85-P" H 6300 3200 50  0000 C CNN
F 2 "Housings_DIP:DIP-8_W7.62mm" H 6300 3600 50  0000 C CIN
F 3 "" H 5300 3600 50  0000 C CNN
	1    5300 3600
	1    0    0    -1  
$EndComp
$Comp
L Battery BT1
U 1 1 565B6896
P 4650 2500
F 0 "BT1" H 4750 2550 50  0000 L CNN
F 1 "Battery" H 4750 2450 50  0000 L CNN
F 2 "AAAAAGGHGGHGG:SMD_cr2032_holder" V 4650 2540 50  0001 C CNN
F 3 "" V 4650 2540 50  0000 C CNN
	1    4650 2500
	0    -1   -1   0   
$EndComp
$Comp
L Battery BT2
U 1 1 565B68E7
P 5150 2500
F 0 "BT2" H 5250 2550 50  0000 L CNN
F 1 "Battery" H 5250 2450 50  0000 L CNN
F 2 "AAAAAGGHGGHGG:SMD_cr2032_holder" V 5150 2540 50  0001 C CNN
F 3 "" V 5150 2540 50  0000 C CNN
	1    5150 2500
	0    -1   -1   0   
$EndComp
$Comp
L SWITCH_INV SW3
U 1 1 565B692F
P 4900 1750
F 0 "SW3" H 4700 1900 50  0000 C CNN
F 1 "SWITCH_INV" H 4750 1600 50  0000 C CNN
F 2 "Buttons_Switches_SMD:SW_SPDT_PCM12" H 4900 1750 50  0001 C CNN
F 3 "" H 4900 1750 50  0000 C CNN
	1    4900 1750
	1    0    0    -1  
$EndComp
$Comp
L 78L05 U2
U 1 1 565B695E
P 6150 1900
F 0 "U2" H 6300 1704 50  0000 C CNN
F 1 "78L05" H 6150 2100 50  0000 C CNN
F 2 "AAAAAGGHGGHGG:78L05_handsoldering" H 6150 1900 50  0001 C CNN
F 3 "" H 6150 1900 50  0000 C CNN
	1    6150 1900
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 565B6A08
P 4900 2850
F 0 "C2" H 4925 2950 50  0000 L CNN
F 1 "22uF" H 4925 2750 50  0000 L CNN
F 2 "Capacitors_SMD:C_1210_HandSoldering" H 4938 2700 50  0001 C CNN
F 3 "" H 4900 2850 50  0000 C CNN
	1    4900 2850
	0    1    1    0   
$EndComp
$Comp
L VCC #PWR01
U 1 1 565B6B02
P 7050 3050
F 0 "#PWR01" H 7050 2900 50  0001 C CNN
F 1 "VCC" H 7050 3200 50  0000 C CNN
F 2 "" H 7050 3050 50  0000 C CNN
F 3 "" H 7050 3050 50  0000 C CNN
	1    7050 3050
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 565B6B3A
P 7050 3600
F 0 "C3" H 7075 3700 50  0000 L CNN
F 1 ".01uF" H 7075 3500 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206" H 7088 3450 50  0001 C CNN
F 3 "" H 7050 3600 50  0000 C CNN
	1    7050 3600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR02
U 1 1 565B6BF3
P 7050 4050
F 0 "#PWR02" H 7050 3800 50  0001 C CNN
F 1 "GND" H 7050 3900 50  0000 C CNN
F 2 "" H 7050 4050 50  0000 C CNN
F 3 "" H 7050 4050 50  0000 C CNN
	1    7050 4050
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 565B6C97
P 2750 2950
F 0 "R2" V 2830 2950 50  0000 C CNN
F 1 "10k" V 2750 2950 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 2680 2950 50  0001 C CNN
F 3 "" H 2750 2950 50  0000 C CNN
	1    2750 2950
	1    0    0    -1  
$EndComp
$Comp
L R R4
U 1 1 565B6D0C
P 3050 2950
F 0 "R4" V 3130 2950 50  0000 C CNN
F 1 "1k" V 3050 2950 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 2980 2950 50  0001 C CNN
F 3 "" H 3050 2950 50  0000 C CNN
	1    3050 2950
	1    0    0    -1  
$EndComp
$Comp
L R R5
U 1 1 565B6D47
P 3550 2950
F 0 "R5" V 3630 2950 50  0000 C CNN
F 1 "10k" V 3550 2950 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 3480 2950 50  0001 C CNN
F 3 "" H 3550 2950 50  0000 C CNN
	1    3550 2950
	1    0    0    -1  
$EndComp
$Comp
L R R1
U 1 1 565B6E0D
P 2350 3800
F 0 "R1" V 2430 3800 50  0000 C CNN
F 1 "10k" V 2350 3800 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 2280 3800 50  0001 C CNN
F 3 "" H 2350 3800 50  0000 C CNN
	1    2350 3800
	1    0    0    -1  
$EndComp
$Comp
L Q_NPN_BCE Q1
U 1 1 565B6E87
P 1950 5200
F 0 "Q1" H 2250 5250 50  0000 R CNN
F 1 "Q_NPN_BCE" H 2550 5150 50  0000 R CNN
F 2 "TO_SOT_Packages_SMD:SOT-23_Handsoldering" H 2150 5300 50  0001 C CNN
F 3 "" H 1950 5200 50  0000 C CNN
	1    1950 5200
	-1   0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 565B70B3
P 2350 5700
F 0 "C1" H 2375 5800 50  0000 L CNN
F 1 ".01uF" H 2375 5600 50  0000 L CNN
F 2 "Capacitors_SMD:C_1206_HandSoldering" H 2388 5550 50  0001 C CNN
F 3 "" H 2350 5700 50  0000 C CNN
	1    2350 5700
	1    0    0    -1  
$EndComp
$Comp
L SPEAKER SP1
U 1 1 565B7200
P 1300 4250
F 0 "SP1" H 1200 4500 50  0000 C CNN
F 1 "SPEAKER" H 1200 4000 50  0000 C CNN
F 2 "Buzzers_Beepers:MagneticBuzzer_ProSignal_ABT-410-RC" H 1300 4250 50  0001 C CNN
F 3 "" H 1300 4250 50  0000 C CNN
	1    1300 4250
	-1   0    0    -1  
$EndComp
$Comp
L D D1
U 1 1 565B72E6
P 2100 4250
F 0 "D1" H 2100 4350 50  0000 C CNN
F 1 "DIODE" H 2100 4150 50  0000 C CNN
F 2 "Diodes_SMD:SOD-323" H 2100 4250 50  0001 C CNN
F 3 "" H 2100 4250 50  0000 C CNN
	1    2100 4250
	0    1    1    0   
$EndComp
$Comp
L VCC #PWR03
U 1 1 565B739F
P 1850 3650
F 0 "#PWR03" H 1850 3500 50  0001 C CNN
F 1 "VCC" H 1850 3800 50  0000 C CNN
F 2 "" H 1850 3650 50  0000 C CNN
F 3 "" H 1850 3650 50  0000 C CNN
	1    1850 3650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR04
U 1 1 565B7414
P 1850 6150
F 0 "#PWR04" H 1850 5900 50  0001 C CNN
F 1 "GND" H 1850 6000 50  0000 C CNN
F 2 "" H 1850 6150 50  0000 C CNN
F 3 "" H 1850 6150 50  0000 C CNN
	1    1850 6150
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW1
U 1 1 565B74BF
P 2750 4850
F 0 "SW1" H 2900 4960 50  0000 C CNN
F 1 "SW_PUSH" H 2750 4770 50  0000 C CNN
F 2 "Buttons_Switches_SMD:SW_SPST_EVPBF" H 2750 4850 50  0001 C CNN
F 3 "" H 2750 4850 50  0000 C CNN
	1    2750 4850
	0    1    1    0   
$EndComp
$Comp
L VCC #PWR05
U 1 1 565B7581
P 2750 2600
F 0 "#PWR05" H 2750 2450 50  0001 C CNN
F 1 "VCC" H 2750 2750 50  0000 C CNN
F 2 "" H 2750 2600 50  0000 C CNN
F 3 "" H 2750 2600 50  0000 C CNN
	1    2750 2600
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR07
U 1 1 565B75FE
P 3550 2600
F 0 "#PWR07" H 3550 2450 50  0001 C CNN
F 1 "VCC" H 3550 2750 50  0000 C CNN
F 2 "" H 3550 2600 50  0000 C CNN
F 3 "" H 3550 2600 50  0000 C CNN
	1    3550 2600
	1    0    0    -1  
$EndComp
$Comp
L R R3
U 1 1 565B77AA
P 2900 4200
F 0 "R3" V 2980 4200 50  0000 C CNN
F 1 "10k" V 2900 4200 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 2830 4200 50  0001 C CNN
F 3 "" H 2900 4200 50  0000 C CNN
	1    2900 4200
	1    0    0    -1  
$EndComp
$Comp
L POT RV1
U 1 1 565B7836
P 3200 4650
F 0 "RV1" H 3200 4550 50  0000 C CNN
F 1 "POT" H 3200 4650 50  0000 C CNN
F 2 "bourns:BOURNS-3362P" H 3200 4650 50  0001 C CNN
F 3 "" H 3200 4650 50  0000 C CNN
	1    3200 4650
	0    -1   -1   0   
$EndComp
$Comp
L VCC #PWR08
U 1 1 565B791C
P 3200 4100
F 0 "#PWR08" H 3200 3950 50  0001 C CNN
F 1 "VCC" H 3200 4250 50  0000 C CNN
F 2 "" H 3200 4100 50  0000 C CNN
F 3 "" H 3200 4100 50  0000 C CNN
	1    3200 4100
	1    0    0    -1  
$EndComp
$Comp
L LM335 U1
U 1 1 565B8030
P 3050 6000
F 0 "U1" H 3200 6300 60  0000 C CNN
F 1 "LM335" H 2550 6050 60  0000 C CNN
F 2 "TO_SOT_Packages_THT:TO-92_Inline_Narrow_Oval" H 3050 6000 60  0001 C CNN
F 3 "" H 3050 6000 60  0000 C CNN
	1    3050 6000
	1    0    0    -1  
$EndComp
$Comp
L POT RV2
U 1 1 565B8194
P 3750 6000
F 0 "RV2" H 3750 5900 50  0000 C CNN
F 1 "POT" H 3750 6000 50  0000 C CNN
F 2 "bourns:BOURNS-3362P" H 3750 6000 50  0001 C CNN
F 3 "" H 3750 6000 50  0000 C CNN
	1    3750 6000
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR09
U 1 1 565B8297
P 3050 6950
F 0 "#PWR09" H 3050 6700 50  0001 C CNN
F 1 "GND" H 3050 6800 50  0000 C CNN
F 2 "" H 3050 6950 50  0000 C CNN
F 3 "" H 3050 6950 50  0000 C CNN
	1    3050 6950
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW2
U 1 1 565B8389
P 4450 5400
F 0 "SW2" H 4600 5510 50  0000 C CNN
F 1 "SW_PUSH" H 4450 5320 50  0000 C CNN
F 2 "Buttons_Switches_SMD:SW_SPST_EVPBF" H 4450 5400 50  0001 C CNN
F 3 "" H 4450 5400 50  0000 C CNN
	1    4450 5400
	0    1    1    0   
$EndComp
$Comp
L GND #PWR010
U 1 1 565B840E
P 2750 5300
F 0 "#PWR010" H 2750 5050 50  0001 C CNN
F 1 "GND" H 2750 5150 50  0000 C CNN
F 2 "" H 2750 5300 50  0000 C CNN
F 3 "" H 2750 5300 50  0000 C CNN
	1    2750 5300
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR011
U 1 1 565B84B6
P 4450 5900
F 0 "#PWR011" H 4450 5650 50  0001 C CNN
F 1 "GND" H 4450 5750 50  0000 C CNN
F 2 "" H 4450 5900 50  0000 C CNN
F 3 "" H 4450 5900 50  0000 C CNN
	1    4450 5900
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 565B858A
P 4450 4550
F 0 "R7" V 4530 4550 50  0000 C CNN
F 1 "10k" V 4450 4550 50  0000 C CNN
F 2 "Resistors_SMD:R_1206_HandSoldering" V 4380 4550 50  0001 C CNN
F 3 "" H 4450 4550 50  0000 C CNN
	1    4450 4550
	1    0    0    -1  
$EndComp
$Comp
L VCC #PWR012
U 1 1 565B85F5
P 4450 4200
F 0 "#PWR012" H 4450 4050 50  0001 C CNN
F 1 "VCC" H 4450 4350 50  0000 C CNN
F 2 "" H 4450 4200 50  0000 C CNN
F 3 "" H 4450 4200 50  0000 C CNN
	1    4450 4200
	1    0    0    -1  
$EndComp
$Comp
L LDR R6
U 1 1 565B875B
P 3550 4650
F 0 "R6" H 3700 4900 60  0000 C CNN
F 1 "LDR" H 3750 4800 60  0000 C CNN
F 2 "AAAAAGGHGGHGG:LDR" H 3550 4650 60  0001 C CNN
F 3 "" H 3550 4650 60  0000 C CNN
	1    3550 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	4800 2500 5000 2500
Wire Wire Line
	4400 1750 4250 1750
Wire Wire Line
	4250 1750 4250 2850
Wire Wire Line
	4250 2500 4500 2500
Wire Wire Line
	4250 2850 4750 2850
Connection ~ 4250 2500
Wire Wire Line
	5300 2500 7250 2500
Wire Wire Line
	5500 2500 5500 2850
Wire Wire Line
	5500 2850 5050 2850
Wire Wire Line
	5400 1850 5750 1850
Wire Wire Line
	6150 2500 6150 2150
Connection ~ 5500 2500
Wire Wire Line
	6650 3350 7050 3350
Wire Wire Line
	6850 3350 6850 1850
Wire Wire Line
	6850 1850 6550 1850
Wire Wire Line
	7250 2500 7250 3850
Wire Wire Line
	7250 3850 6650 3850
Connection ~ 6150 2500
Wire Wire Line
	7050 3050 7050 3450
Connection ~ 6850 3350
Connection ~ 7050 3350
Wire Wire Line
	7050 3750 7050 4050
Connection ~ 7050 3850
Wire Wire Line
	3950 3350 2350 3350
Wire Wire Line
	2350 3350 2350 3650
Wire Wire Line
	2350 3950 2350 5550
Wire Wire Line
	2150 5200 2350 5200
Connection ~ 2350 5200
Wire Wire Line
	1850 4150 1600 4150
Wire Wire Line
	1850 5400 1850 6150
Wire Wire Line
	2350 5850 2350 6000
Wire Wire Line
	2350 6000 1850 6000
Connection ~ 1850 6000
Wire Wire Line
	2750 3100 2750 4550
Wire Wire Line
	2750 2600 2750 2800
Wire Wire Line
	3050 2600 3050 2800
Wire Wire Line
	3550 2600 3550 2800
Wire Wire Line
	3050 3100 3050 5700
Wire Wire Line
	3050 3450 3950 3450
Wire Wire Line
	3550 3100 3550 4350
Wire Wire Line
	3950 3650 2750 3650
Connection ~ 2750 3650
Wire Wire Line
	2900 4350 2900 4650
Wire Wire Line
	2900 4650 3050 4650
Wire Wire Line
	3200 4100 3200 4400
Wire Wire Line
	3950 3550 2900 3550
Wire Wire Line
	2900 3550 2900 4050
Connection ~ 3050 3450
Wire Wire Line
	3400 6000 3600 6000
Wire Wire Line
	3050 5450 3750 5450
Wire Wire Line
	3750 5450 3750 5750
Connection ~ 3050 5450
Wire Wire Line
	3050 6300 3050 6950
Wire Wire Line
	3750 6250 3750 6550
Wire Wire Line
	3750 6550 3050 6550
Connection ~ 3050 6550
Wire Wire Line
	2750 5150 2750 5300
Wire Wire Line
	4450 5700 4450 5900
Wire Wire Line
	4450 4200 4450 4400
Wire Wire Line
	4450 4700 4450 5100
Wire Wire Line
	4450 4900 3850 4900
Wire Wire Line
	3850 4900 3850 3850
Wire Wire Line
	3850 3850 3950 3850
Connection ~ 4450 4900
Wire Wire Line
	3950 3750 3550 3750
Connection ~ 3550 3750
$Comp
L GND #PWR013
U 1 1 565B89B1
P 3550 5150
F 0 "#PWR013" H 3550 4900 50  0001 C CNN
F 1 "GND" H 3550 5000 50  0000 C CNN
F 2 "" H 3550 5150 50  0000 C CNN
F 3 "" H 3550 5150 50  0000 C CNN
	1    3550 5150
	1    0    0    -1  
$EndComp
Wire Wire Line
	3550 4900 3550 5150
Wire Wire Line
	1850 4450 2100 4450
Connection ~ 1850 4450
Wire Wire Line
	1850 4350 1850 5000
Wire Wire Line
	1850 4350 1600 4350
Wire Wire Line
	1850 3650 1850 4150
Wire Wire Line
	1850 4050 2100 4050
Connection ~ 1850 4050
Wire Wire Line
	2100 4450 2100 4400
Wire Wire Line
	2100 4050 2100 4100
Wire Wire Line
	3200 4900 3200 5050
Wire Wire Line
	3200 5050 3550 5050
Connection ~ 3550 5050
$Comp
L VCC #PWR06
U 1 1 565B75C3
P 3050 2600
F 0 "#PWR06" H 3050 2450 50  0001 C CNN
F 1 "VCC" H 3050 2750 50  0000 C CNN
F 2 "" H 3050 2600 50  0000 C CNN
F 3 "" H 3050 2600 50  0000 C CNN
	1    3050 2600
	1    0    0    -1  
$EndComp
$EndSCHEMATC
