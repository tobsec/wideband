EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr USLetter 11000 8500
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
L power:GND #PWR0101
U 1 1 5FB6223C
P 7325 3725
F 0 "#PWR0101" H 7325 3475 50  0001 C CNN
F 1 "GND" H 7330 3552 50  0001 C CNN
F 2 "" H 7325 3725 50  0001 C CNN
F 3 "" H 7325 3725 50  0001 C CNN
	1    7325 3725
	1    0    0    -1  
$EndComp
$Comp
L Device:D_Schottky D2
U 1 1 5FB62745
P 3050 3000
F 0 "D2" H 3050 2784 50  0000 C CNN
F 1 "D_Schottky" H 3050 2875 50  0000 C CNN
F 2 "Diode_SMD:D_SMA" H 3050 3000 50  0001 C CNN
F 3 "~" H 3050 3000 50  0001 C CNN
F 4 "C22452" H 3050 3000 50  0001 C CNN "LCSC"
	1    3050 3000
	-1   0    0    1   
$EndComp
$Comp
L Device:D_TVS D1
U 1 1 5FB63423
P 1500 3275
F 0 "D1" V 1454 3354 50  0000 L CNN
F 1 "D_TVS" V 1545 3354 50  0000 L CNN
F 2 "Diode_SMD:D_SMC" H 1500 3275 50  0001 C CNN
F 3 "~" H 1500 3275 50  0001 C CNN
F 4 "C133648" H 1500 3275 50  0001 C CNN "LCSC"
	1    1500 3275
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5FB63A0D
P 1500 3425
F 0 "#PWR0102" H 1500 3175 50  0001 C CNN
F 1 "GND" H 1505 3252 50  0000 C CNN
F 2 "" H 1500 3425 50  0001 C CNN
F 3 "" H 1500 3425 50  0001 C CNN
	1    1500 3425
	1    0    0    -1  
$EndComp
Wire Wire Line
	1500 3000 1500 3125
Text GLabel 1375 3000 0    50   Input ~ 0
12V_RAW
Wire Wire Line
	1375 3000 1500 3000
Connection ~ 1500 3000
$Comp
L Device:C C1
U 1 1 5FB64467
P 3725 3250
F 0 "C1" H 3840 3296 50  0000 L CNN
F 1 "1u" H 3840 3205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 3763 3100 50  0001 C CNN
F 3 "~" H 3725 3250 50  0001 C CNN
F 4 "C15849" H 3725 3250 50  0001 C CNN "LCSC"
	1    3725 3250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5FB64D47
P 3725 3400
F 0 "#PWR0103" H 3725 3150 50  0001 C CNN
F 1 "GND" H 3730 3227 50  0000 C CNN
F 2 "" H 3725 3400 50  0001 C CNN
F 3 "" H 3725 3400 50  0001 C CNN
	1    3725 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3725 3100 3725 3000
Wire Wire Line
	3950 3000 3725 3000
Connection ~ 3725 3000
$Comp
L Regulator_Linear:LM78M05_TO252 U2
U 1 1 5FB67A82
P 4250 3000
F 0 "U2" H 4250 3242 50  0000 C CNN
F 1 "LM78M05_TO252" H 4250 3151 50  0000 C CNN
F 2 "Package_TO_SOT_SMD:TO-252-2" H 4250 3225 50  0001 C CIN
F 3 "http://www.fairchildsemi.com/ds/LM/LM78M05.pdf" H 4250 2950 50  0001 C CNN
F 4 "C58069" H 4250 3000 50  0001 C CNN "LCSC"
	1    4250 3000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0104
U 1 1 5FB6926D
P 4250 3300
F 0 "#PWR0104" H 4250 3050 50  0001 C CNN
F 1 "GND" H 4255 3127 50  0000 C CNN
F 2 "" H 4250 3300 50  0001 C CNN
F 3 "" H 4250 3300 50  0001 C CNN
	1    4250 3300
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0105
U 1 1 5FB69902
P 4775 2900
F 0 "#PWR0105" H 4775 2750 50  0001 C CNN
F 1 "+5V" H 4790 3073 50  0000 C CNN
F 2 "" H 4775 2900 50  0001 C CNN
F 3 "" H 4775 2900 50  0001 C CNN
	1    4775 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 3000 4775 3000
Wire Wire Line
	4775 3000 4775 2900
$Comp
L Interface_CAN_LIN:TJA1051T-3 U1
U 1 1 5FB6A2CB
P 7325 3325
F 0 "U1" H 7000 3700 50  0000 C CNN
F 1 "TJA1051T-3" H 7600 3700 50  0000 C CNN
F 2 "Package_SO:SOIC-8_3.9x4.9mm_P1.27mm" H 7325 2825 50  0001 C CIN
F 3 "http://www.nxp.com/documents/data_sheet/TJA1051.pdf" H 7325 3325 50  0001 C CNN
F 4 "C38695" H 7325 3325 50  0001 C CNN "LCSC"
	1    7325 3325
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0106
U 1 1 5FB6B936
P 7325 2925
F 0 "#PWR0106" H 7325 2775 50  0001 C CNN
F 1 "+5V" H 7340 3098 50  0000 C CNN
F 2 "" H 7325 2925 50  0001 C CNN
F 3 "" H 7325 2925 50  0001 C CNN
	1    7325 2925
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0107
U 1 1 5FB6BFE5
P 6825 3525
F 0 "#PWR0107" H 6825 3275 50  0001 C CNN
F 1 "GND" V 6830 3397 50  0000 R CNN
F 2 "" H 6825 3525 50  0001 C CNN
F 3 "" H 6825 3525 50  0001 C CNN
	1    6825 3525
	0    1    1    0   
$EndComp
$Comp
L Device:C C2
U 1 1 5FB6D1DC
P 4775 3250
F 0 "C2" H 4890 3296 50  0000 L CNN
F 1 "1u" H 4890 3205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 4813 3100 50  0001 C CNN
F 3 "~" H 4775 3250 50  0001 C CNN
F 4 "C15849" H 4775 3250 50  0001 C CNN "LCSC"
	1    4775 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	4775 3000 4775 3100
Connection ~ 4775 3000
$Comp
L power:GND #PWR0109
U 1 1 5FB6E5F5
P 4775 3400
F 0 "#PWR0109" H 4775 3150 50  0001 C CNN
F 1 "GND" H 4780 3227 50  0000 C CNN
F 2 "" H 4775 3400 50  0001 C CNN
F 3 "" H 4775 3400 50  0001 C CNN
	1    4775 3400
	1    0    0    -1  
$EndComp
Text Label 6500 3125 0    50   ~ 0
CAN_TX
Wire Wire Line
	6500 3125 6825 3125
Wire Wire Line
	6825 3225 6500 3225
Text Label 6500 3225 0    50   ~ 0
CAN_RX
Text Label 8400 3225 2    50   ~ 0
CAN_H
Text Label 8400 3425 2    50   ~ 0
CAN_L
Wire Wire Line
	8400 3425 8050 3425
Wire Wire Line
	8400 3225 8050 3225
$Comp
L Device:R R1
U 1 1 5FB83149
P 5700 3225
F 0 "R1" H 5770 3271 50  0000 L CNN
F 1 "220" H 5770 3180 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5630 3225 50  0001 C CNN
F 3 "~" H 5700 3225 50  0001 C CNN
F 4 "C22962" H 5700 3225 50  0001 C CNN "LCSC"
	1    5700 3225
	1    0    0    -1  
$EndComp
$Comp
L Device:R R2
U 1 1 5FB834DF
P 5700 3650
F 0 "R2" H 5770 3696 50  0000 L CNN
F 1 "470" H 5770 3605 50  0000 L CNN
F 2 "Resistor_SMD:R_0603_1608Metric" V 5630 3650 50  0001 C CNN
F 3 "~" H 5700 3650 50  0001 C CNN
F 4 "C23179" H 5700 3650 50  0001 C CNN "LCSC"
	1    5700 3650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0108
U 1 1 5FB83A8E
P 5700 3800
F 0 "#PWR0108" H 5700 3550 50  0001 C CNN
F 1 "GND" H 5705 3627 50  0000 C CNN
F 2 "" H 5700 3800 50  0001 C CNN
F 3 "" H 5700 3800 50  0001 C CNN
	1    5700 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 3075 5700 3000
Wire Wire Line
	5700 3000 5225 3000
$Comp
L Device:C C3
U 1 1 5FB842B4
P 6050 3650
F 0 "C3" H 6165 3696 50  0000 L CNN
F 1 "1u" H 6165 3605 50  0000 L CNN
F 2 "Capacitor_SMD:C_0603_1608Metric" H 6088 3500 50  0001 C CNN
F 3 "~" H 6050 3650 50  0001 C CNN
F 4 "C15849" H 6050 3650 50  0001 C CNN "LCSC"
	1    6050 3650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0110
U 1 1 5FB8494A
P 6050 3800
F 0 "#PWR0110" H 6050 3550 50  0001 C CNN
F 1 "GND" H 6055 3627 50  0000 C CNN
F 2 "" H 6050 3800 50  0001 C CNN
F 3 "" H 6050 3800 50  0001 C CNN
	1    6050 3800
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 3375 5700 3425
Wire Wire Line
	5700 3425 6050 3425
Connection ~ 5700 3425
Wire Wire Line
	5700 3425 5700 3500
Wire Wire Line
	6050 3500 6050 3425
Connection ~ 6050 3425
Wire Wire Line
	6050 3425 6825 3425
Text Notes 5600 4275 0    50   ~ 0
VIO pin only draws 500uA max so\na divider will work fine
$Comp
L Connector_Generic:Conn_01x08 J2
U 1 1 5FB94B65
P 4125 4425
F 0 "J2" H 4250 4375 50  0000 C CNN
F 1 "Conn_01x08" H 4043 4851 50  0001 C CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x08_P2.54mm_Vertical" H 4125 4425 50  0001 C CNN
F 3 "~" H 4125 4425 50  0001 C CNN
	1    4125 4425
	-1   0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x08 J1
U 1 1 5FB96561
P 3750 4425
F 0 "J1" H 3825 4375 50  0000 L CNN
F 1 "Conn_01x08" H 3830 4326 50  0001 L CNN
F 2 "Connector_PinSocket_2.54mm:PinSocket_1x08_P2.54mm_Vertical" H 3750 4425 50  0001 C CNN
F 3 "~" H 3750 4425 50  0001 C CNN
	1    3750 4425
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR0111
U 1 1 5FB97B42
P 3550 4125
F 0 "#PWR0111" H 3550 3975 50  0001 C CNN
F 1 "+5V" V 3565 4253 50  0000 L CNN
F 2 "" H 3550 4125 50  0001 C CNN
F 3 "" H 3550 4125 50  0001 C CNN
	1    3550 4125
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0112
U 1 1 5FB987A8
P 3550 4225
F 0 "#PWR0112" H 3550 3975 50  0001 C CNN
F 1 "GND" V 3555 4097 50  0000 R CNN
F 2 "" H 3550 4225 50  0001 C CNN
F 3 "" H 3550 4225 50  0001 C CNN
	1    3550 4225
	0    1    1    0   
$EndComp
Text Label 2975 4325 0    50   ~ 0
UART_TX
Wire Wire Line
	2975 4325 3550 4325
Text Label 2975 4425 0    50   ~ 0
UART_RX
Wire Wire Line
	2975 4425 3550 4425
Text Label 2975 4625 0    50   ~ 0
CAN_TX
Text Label 2975 4525 0    50   ~ 0
CAN_RX
Wire Wire Line
	2975 4625 3550 4625
Wire Wire Line
	2975 4525 3550 4525
Text Label 3100 4725 0    50   ~ 0
nRESET
Wire Wire Line
	3100 4725 3550 4725
Text Label 3100 4825 0    50   ~ 0
Boot0
Wire Wire Line
	3100 4825 3550 4825
Text Label 4700 4525 2    50   ~ 0
LSU_Ip
Wire Wire Line
	4700 4525 4325 4525
Wire Wire Line
	4325 4625 4700 4625
Wire Wire Line
	4325 4725 4700 4725
Wire Wire Line
	4325 4825 4700 4825
Text Label 4700 4625 2    50   ~ 0
LSU_Vm
Text Label 4700 4725 2    50   ~ 0
LSU_Rtrim
Text Label 4700 4825 2    50   ~ 0
LSU_Un
$Comp
L power:GND #PWR0113
U 1 1 5FB9D2BA
P 4450 4175
F 0 "#PWR0113" H 4450 3925 50  0001 C CNN
F 1 "GND" V 4455 4047 50  0000 R CNN
F 2 "" H 4450 4175 50  0001 C CNN
F 3 "" H 4450 4175 50  0001 C CNN
	1    4450 4175
	0    -1   -1   0   
$EndComp
Wire Wire Line
	4325 4125 4375 4125
Wire Wire Line
	4375 4125 4375 4175
Wire Wire Line
	4375 4225 4325 4225
Wire Wire Line
	4375 4175 4450 4175
Connection ~ 4375 4175
Wire Wire Line
	4375 4175 4375 4225
Wire Wire Line
	4325 4325 4375 4325
Wire Wire Line
	4375 4325 4375 4375
Wire Wire Line
	4375 4425 4325 4425
Wire Wire Line
	4950 4375 4375 4375
Connection ~ 4375 4375
Wire Wire Line
	4375 4375 4375 4425
Text Label 4950 4375 2    50   ~ 0
LSU_Heater-
Text Label 3775 5825 0    50   ~ 0
LSU_Heater-
Text Label 3775 5725 0    50   ~ 0
LSU_Ip
Text Label 3725 2600 0    50   ~ 0
LSU_Heater+
Wire Wire Line
	3725 2600 3725 3000
Text Label 3775 5925 0    50   ~ 0
LSU_Heater+
Wire Wire Line
	4325 5925 3775 5925
Wire Wire Line
	3775 5825 4325 5825
Wire Wire Line
	3775 5725 4325 5725
Text Notes 3700 5725 2    50   ~ 0
red
Text Notes 3700 5875 2    50   ~ 0
white
Text Notes 3700 5975 2    50   ~ 0
grey
Text GLabel 4250 6125 0    50   Input ~ 0
12V_RAW
Wire Wire Line
	4250 6125 4325 6125
Text Label 5125 6025 2    50   ~ 0
CAN_H
Text Label 5125 6125 2    50   ~ 0
CAN_L
Wire Wire Line
	5125 6125 4825 6125
Wire Wire Line
	5125 6025 4825 6025
$Comp
L Connector_Generic:Conn_02x06_Top_Bottom J3
U 1 1 5FD3D185
P 4525 5825
F 0 "J3" H 4575 6242 50  0000 C CNN
F 1 "Conn_02x06_Top_Bottom" H 4575 6151 50  0000 C CNN
F 2 "mx120:mx120g-12" H 4525 5825 50  0001 C CNN
F 3 "~" H 4525 5825 50  0001 C CNN
	1    4525 5825
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 5FD4C273
P 4200 5625
F 0 "#PWR0115" H 4200 5375 50  0001 C CNN
F 1 "GND" V 4205 5497 50  0000 R CNN
F 2 "" H 4200 5625 50  0001 C CNN
F 3 "" H 4200 5625 50  0001 C CNN
	1    4200 5625
	0    1    1    0   
$EndComp
Wire Wire Line
	4200 5625 4325 5625
NoConn ~ 4325 6025
Text Notes 5450 5925 0    50   ~ 0
black
Text Notes 5450 5825 0    50   ~ 0
green
Text Notes 5450 5725 0    50   ~ 0
yellow
Wire Wire Line
	4825 5925 5375 5925
Wire Wire Line
	4825 5825 5375 5825
Wire Wire Line
	4825 5725 5375 5725
Text Label 5375 5925 2    50   ~ 0
LSU_Un
Text Label 5375 5825 2    50   ~ 0
LSU_Rtrim
Text Label 5375 5725 2    50   ~ 0
LSU_Vm
NoConn ~ 4825 5625
$Comp
L Device:C C4
U 1 1 5FC74FFA
P 8050 2975
F 0 "C4" H 8165 3021 50  0000 L CNN
F 1 "47p" H 8165 2930 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 8088 2825 50  0001 C CNN
F 3 "~" H 8050 2975 50  0001 C CNN
F 4 "C1567" H 8050 2975 50  0001 C CNN "LCSC"
	1    8050 2975
	1    0    0    -1  
$EndComp
$Comp
L Device:C C5
U 1 1 5FC7558C
P 8050 3675
F 0 "C5" H 8165 3721 50  0000 L CNN
F 1 "47p" H 8165 3630 50  0000 L CNN
F 2 "Capacitor_SMD:C_0402_1005Metric" H 8088 3525 50  0001 C CNN
F 3 "~" H 8050 3675 50  0001 C CNN
F 4 "C1567" H 8050 3675 50  0001 C CNN "LCSC"
	1    8050 3675
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0114
U 1 1 5FC75D4C
P 8050 3825
F 0 "#PWR0114" H 8050 3575 50  0001 C CNN
F 1 "GND" H 8055 3652 50  0001 C CNN
F 2 "" H 8050 3825 50  0001 C CNN
F 3 "" H 8050 3825 50  0001 C CNN
	1    8050 3825
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0116
U 1 1 5FC761A0
P 8050 2825
F 0 "#PWR0116" H 8050 2575 50  0001 C CNN
F 1 "GND" H 8055 2652 50  0001 C CNN
F 2 "" H 8050 2825 50  0001 C CNN
F 3 "" H 8050 2825 50  0001 C CNN
	1    8050 2825
	-1   0    0    1   
$EndComp
Wire Wire Line
	8050 3125 8050 3225
Connection ~ 8050 3225
Wire Wire Line
	8050 3225 7825 3225
Wire Wire Line
	8050 3425 8050 3525
Connection ~ 8050 3425
Wire Wire Line
	8050 3425 7825 3425
$Comp
L Device:C C6
U 1 1 601110B8
P 5225 3250
F 0 "C6" H 5340 3296 50  0000 L CNN
F 1 "22u" H 5340 3205 50  0000 L CNN
F 2 "Capacitor_SMD:C_0805_2012Metric" H 5263 3100 50  0001 C CNN
F 3 "~" H 5225 3250 50  0001 C CNN
F 4 "C45783" H 5225 3250 50  0001 C CNN "LCSC"
	1    5225 3250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0117
U 1 1 60111490
P 5225 3400
F 0 "#PWR0117" H 5225 3150 50  0001 C CNN
F 1 "GND" H 5230 3227 50  0000 C CNN
F 2 "" H 5225 3400 50  0001 C CNN
F 3 "" H 5225 3400 50  0001 C CNN
	1    5225 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	5225 3100 5225 3000
Connection ~ 5225 3000
Wire Wire Line
	5225 3000 4775 3000
$Comp
L Device:C C7
U 1 1 6011E2FA
P 1975 3250
F 0 "C7" H 2090 3296 50  0000 L CNN
F 1 "10u" H 2090 3205 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 2013 3100 50  0001 C CNN
F 3 "~" H 1975 3250 50  0001 C CNN
F 4 "C13585" H 1975 3250 50  0001 C CNN "LCSC"
	1    1975 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C9
U 1 1 6011FBBD
P 2775 3250
F 0 "C9" H 2890 3296 50  0000 L CNN
F 1 "10u" H 2890 3205 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 2813 3100 50  0001 C CNN
F 3 "~" H 2775 3250 50  0001 C CNN
F 4 "C13585" H 2775 3250 50  0001 C CNN "LCSC"
	1    2775 3250
	1    0    0    -1  
$EndComp
$Comp
L Device:C C8
U 1 1 6012041C
P 2375 3250
F 0 "C8" H 2490 3296 50  0000 L CNN
F 1 "10u" H 2490 3205 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 2413 3100 50  0001 C CNN
F 3 "~" H 2375 3250 50  0001 C CNN
F 4 "C13585" H 2375 3250 50  0001 C CNN "LCSC"
	1    2375 3250
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR0118
U 1 1 60125DEB
P 2775 3400
F 0 "#PWR0118" H 2775 3150 50  0001 C CNN
F 1 "GND" H 2780 3227 50  0000 C CNN
F 2 "" H 2775 3400 50  0001 C CNN
F 3 "" H 2775 3400 50  0001 C CNN
	1    2775 3400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0119
U 1 1 6012611E
P 2375 3400
F 0 "#PWR0119" H 2375 3150 50  0001 C CNN
F 1 "GND" H 2380 3227 50  0000 C CNN
F 2 "" H 2375 3400 50  0001 C CNN
F 3 "" H 2375 3400 50  0001 C CNN
	1    2375 3400
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0120
U 1 1 601265C7
P 1975 3400
F 0 "#PWR0120" H 1975 3150 50  0001 C CNN
F 1 "GND" H 1980 3227 50  0000 C CNN
F 2 "" H 1975 3400 50  0001 C CNN
F 3 "" H 1975 3400 50  0001 C CNN
	1    1975 3400
	1    0    0    -1  
$EndComp
$Comp
L Device:C C10
U 1 1 6013DA65
P 3325 3250
F 0 "C10" H 3440 3296 50  0000 L CNN
F 1 "10u" H 3440 3205 50  0000 L CNN
F 2 "Capacitor_SMD:C_1206_3216Metric" H 3363 3100 50  0001 C CNN
F 3 "~" H 3325 3250 50  0001 C CNN
F 4 "C13585" H 3325 3250 50  0001 C CNN "LCSC"
	1    3325 3250
	1    0    0    -1  
$EndComp
Wire Wire Line
	3325 3100 3325 3000
Wire Wire Line
	3325 3000 3725 3000
Wire Wire Line
	3200 3000 3325 3000
Connection ~ 3325 3000
$Comp
L power:GND #PWR0121
U 1 1 60146DF9
P 3325 3400
F 0 "#PWR0121" H 3325 3150 50  0001 C CNN
F 1 "GND" H 3330 3227 50  0000 C CNN
F 2 "" H 3325 3400 50  0001 C CNN
F 3 "" H 3325 3400 50  0001 C CNN
	1    3325 3400
	1    0    0    -1  
$EndComp
Wire Wire Line
	1975 3000 2375 3000
Wire Wire Line
	2775 3100 2775 3000
Connection ~ 2775 3000
Wire Wire Line
	2775 3000 2900 3000
Wire Wire Line
	2375 3000 2375 3100
Connection ~ 2375 3000
Wire Wire Line
	2375 3000 2775 3000
Wire Wire Line
	1975 3100 1975 3000
Wire Wire Line
	1500 3000 1975 3000
Connection ~ 1975 3000
$EndSCHEMATC
