EESchema Schematic File Version 4
EELAYER 30 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 2 2
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
Text GLabel 2700 1300 0    50   Input ~ 0
GND
Text GLabel 2700 1400 0    50   Input ~ 0
4V
Text GLabel 2700 1500 0    50   Input ~ 0
3.3V
Text GLabel 2700 2400 0    50   Input ~ 0
GND
Wire Wire Line
	2800 1300 2700 1300
Wire Wire Line
	2700 1400 2800 1400
Wire Wire Line
	2800 1500 2700 1500
Wire Wire Line
	2800 2400 2700 2400
Text GLabel 1250 2250 2    50   Input ~ 0
GND
$Comp
L power:GND #PWR0101
U 1 1 5F9B059C
P 1200 2300
F 0 "#PWR0101" H 1200 2050 50  0001 C CNN
F 1 "GND" H 1205 2127 50  0000 C CNN
F 2 "" H 1200 2300 50  0001 C CNN
F 3 "" H 1200 2300 50  0001 C CNN
	1    1200 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 2300 1200 2250
Wire Wire Line
	1200 2250 1250 2250
Text GLabel 1250 1650 2    50   Input ~ 0
4V
Text GLabel 1250 2100 2    50   Input ~ 0
3.3V
$Comp
L power:+3.3V #PWR0102
U 1 1 5F9B30ED
P 1200 2050
F 0 "#PWR0102" H 1200 1900 50  0001 C CNN
F 1 "+3.3V" H 1215 2223 50  0000 C CNN
F 2 "" H 1200 2050 50  0001 C CNN
F 3 "" H 1200 2050 50  0001 C CNN
	1    1200 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 2100 1200 2100
Wire Wire Line
	1200 2100 1200 2050
$Comp
L power:+4V #PWR0103
U 1 1 5F9B57A8
P 1200 1600
F 0 "#PWR0103" H 1200 1450 50  0001 C CNN
F 1 "+4V" H 1215 1773 50  0000 C CNN
F 2 "" H 1200 1600 50  0001 C CNN
F 3 "" H 1200 1600 50  0001 C CNN
	1    1200 1600
	1    0    0    -1  
$EndComp
Wire Wire Line
	1200 1600 1200 1650
Wire Wire Line
	1200 1650 1250 1650
$Comp
L Mechanical:MountingHole H1
U 1 1 5F9BE6DF
P 1250 3250
F 0 "H1" H 1350 3296 50  0000 L CNN
F 1 "MountingHole" H 1350 3205 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 1250 3250 50  0001 C CNN
F 3 "~" H 1250 3250 50  0001 C CNN
	1    1250 3250
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H2
U 1 1 5F9C2A4E
P 1250 3500
F 0 "H2" H 1350 3546 50  0000 L CNN
F 1 "MountingHole" H 1350 3455 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 1250 3500 50  0001 C CNN
F 3 "~" H 1250 3500 50  0001 C CNN
	1    1250 3500
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H3
U 1 1 5F9C3083
P 1250 3750
F 0 "H3" H 1350 3796 50  0000 L CNN
F 1 "MountingHole" H 1350 3705 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 1250 3750 50  0001 C CNN
F 3 "~" H 1250 3750 50  0001 C CNN
	1    1250 3750
	1    0    0    -1  
$EndComp
$Comp
L Mechanical:MountingHole H4
U 1 1 5F9C36A4
P 1250 4000
F 0 "H4" H 1350 4046 50  0000 L CNN
F 1 "MountingHole" H 1350 3955 50  0000 L CNN
F 2 "MountingHole:MountingHole_3.2mm_M3" H 1250 4000 50  0001 C CNN
F 3 "~" H 1250 4000 50  0001 C CNN
	1    1250 4000
	1    0    0    -1  
$EndComp
$Comp
L power:+12V #PWR01
U 1 1 5FA98C2C
P 1200 1100
F 0 "#PWR01" H 1200 950 50  0001 C CNN
F 1 "+12V" H 1215 1273 50  0000 C CNN
F 2 "" H 1200 1100 50  0001 C CNN
F 3 "" H 1200 1100 50  0001 C CNN
	1    1200 1100
	1    0    0    -1  
$EndComp
Text GLabel 1250 1150 2    50   Input ~ 0
12V
Wire Wire Line
	1200 1100 1200 1150
Wire Wire Line
	1200 1150 1250 1150
$Comp
L Connector_Generic:Conn_02x12_Top_Bottom J1
U 1 1 5FA354D6
P 3000 1800
F 0 "J1" H 3050 2517 50  0000 C CNN
F 1 "02x12" H 3050 2426 50  0000 C CNN
F 2 "LOCAL:SAMTEC-SSW-112-02-X-D-RA" H 3000 1800 50  0001 C CNN
F 3 "~" H 3000 1800 50  0001 C CNN
	1    3000 1800
	1    0    0    -1  
$EndComp
$EndSCHEMATC
