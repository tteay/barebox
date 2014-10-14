/* Setup VIO18 DC/DC */
#if defined(CONFIG_VIO18_1750)
	#define VIO18_VAL	0x0
#elif defined(CONFIG_VIO18_1800)
	#define VIO18_VAL	0x1
#elif defined(CONFIG_VIO18_1850)
	#define VIO18_VAL	0x2
#elif defined(CONFIG_VIO18_1900)
	#define VIO18_VAL	0x3
#endif

#if defined(CONFIG_MARCH_2504D)

/* Setup VSYS DC/DC */
#if defined(CONFIG_VSYS_1000)
	#define VSYS_VAL	0x6
#elif defined(CONFIG_VSYS_1050)
	#define VSYS_VAL	0x7
#elif defined(CONFIG_VSYS_1100)
	#define VSYS_VAL	0x8
#elif defined(CONFIG_VSYS_1150)
	#define VSYS_VAL	0x9
#elif defined(CONFIG_VSYS_1200)
	#define VSYS_VAL	0xa
#elif defined(CONFIG_VSYS_1250)
	#define VSYS_VAL	0xb
#elif defined(CONFIG_VSYS_1300)
	#define VSYS_VAL	0xc
#elif defined(CONFIG_VSYS_1350)
	#define VSYS_VAL	0xd
#endif

/* Setup VARM9 normal mode */
#if defined(CONFIG_VARM9_NORMAL_1000)
	#define VARM9_NORMAL_VAL	0x6
#elif defined(CONFIG_VARM9_NORMAL_1050)
	#define VARM9_NORMAL_VAL	0x7
#elif defined(CONFIG_VARM9_NORMAL_1100)
	#define VARM9_NORMAL_VAL	0x8
#elif defined(CONFIG_VARM9_NORMAL_1150)
	#define VARM9_NORMAL_VAL	0x9
#elif defined(CONFIG_VARM9_NORMAL_1200)
	#define VARM9_NORMAL_VAL	0xa
#elif defined(CONFIG_VARM9_NORMAL_1250)
	#define VARM9_NORMAL_VAL	0xb
#elif defined(CONFIG_VARM9_NORMAL_1300)
	#define VARM9_NORMAL_VAL	0xc
#elif defined(CONFIG_VARM9_NORMAL_1350)
	#define VARM9_NORMAL_VAL	0xd
#elif defined(CONFIG_VARM9_NORMAL_1400)
	#define VARM9_NORMAL_VAL	0xe
#elif defined(CONFIG_VARM9_NORMAL_1450)
	#define VARM9_NORMAL_VAL	0xf
#endif

/* Setup VARM9 sleep mode */
#if defined(CONFIG_VARM9_SLEEP_1000)
	#define VARM9_SLEEP_VAL		0x6
#elif defined(CONFIG_VARM9_SLEEP_1050)
	#define VARM9_SLEEP_VAL		0x7
#elif defined(CONFIG_VARM9_SLEEP_1100)
	#define VARM9_SLEEP_VAL		0x8
#elif defined(CONFIG_VARM9_SLEEP_1150)
	#define VARM9_SLEEP_VAL		0x9
#elif defined(CONFIG_VARM9_SLEEP_1200)
	#define VARM9_SLEEP_VAL		0xa
#elif defined(CONFIG_VARM9_SLEEP_1250)
	#define VARM9_SLEEP_VAL		0xb
#elif defined(CONFIG_VARM9_SLEEP_1300)
	#define VARM9_SLEEP_VAL		0xc
#elif defined(CONFIG_VARM9_SLEEP_1350)
	#define VARM9_SLEEP_VAL		0xd
#endif

#elif defined(CONFIG_MARCH_2504E)

/* Setup VSYS DC/DC */
#if defined(CONFIG_VSYS_0950)
	#define VSYS_VAL	0x0
#elif defined(CONFIG_VSYS_0975)
	#define VSYS_VAL	0x1
#elif defined(CONFIG_VSYS_1000)
	#define VSYS_VAL	0x2
#elif defined(CONFIG_VSYS_1025)
	#define VSYS_VAL	0x3
#elif defined(CONFIG_VSYS_1050)
	#define VSYS_VAL	0x4
#elif defined(CONFIG_VSYS_1075)
	#define VSYS_VAL	0x5
#elif defined(CONFIG_VSYS_1100)
	#define VSYS_VAL	0x6
#elif defined(CONFIG_VSYS_1125)
	#define VSYS_VAL	0x7
#elif defined(CONFIG_VSYS_1150)
	#define VSYS_VAL	0x8
#elif defined(CONFIG_VSYS_1175)
	#define VSYS_VAL	0x9
#elif defined(CONFIG_VSYS_1200)
	#define VSYS_VAL	0xa
#elif defined(CONFIG_VSYS_1225)
	#define VSYS_VAL	0xb
#elif defined(CONFIG_VSYS_1250)
	#define VSYS_VAL	0xc
#elif defined(CONFIG_VSYS_1275)
	#define VSYS_VAL	0xd
#elif defined(CONFIG_VSYS_1300)
	#define VSYS_VAL	0xe
#elif defined(CONFIG_VSYS_1325)
	#define VSYS_VAL	0xf
#endif

/* Setup VARM9 normal mode */
#if defined(CONFIG_VARM9_NORMAL_1400)
	#define VARM9_NORMAL_VAL	0x8
#elif defined(CONFIG_VARM9_NORMAL_1430)
	#define VARM9_NORMAL_VAL	0x9
#elif defined(CONFIG_VARM9_NORMAL_1460)
	#define VARM9_NORMAL_VAL	0xa
#elif defined(CONFIG_VARM9_NORMAL_1490)
	#define VARM9_NORMAL_VAL	0xb
#elif defined(CONFIG_VARM9_NORMAL_1520)
	#define VARM9_NORMAL_VAL	0xc
#elif defined(CONFIG_VARM9_NORMAL_1550)
	#define VARM9_NORMAL_VAL	0xd
#elif defined(CONFIG_VARM9_NORMAL_1580)
	#define VARM9_NORMAL_VAL	0xe
#elif defined(CONFIG_VARM9_NORMAL_1610)
	#define VARM9_NORMAL_VAL	0xf
#endif

/* Setup VARM9 sleep mode */
#if defined(CONFIG_VARM9_SLEEP_0920)
	#define VARM9_SLEEP_VAL		0x0
#elif defined(CONFIG_VARM9_SLEEP_0950)
	#define VARM9_SLEEP_VAL		0x1
#elif defined(CONFIG_VARM9_SLEEP_0980)
	#define VARM9_SLEEP_VAL		0x2
#elif defined(CONFIG_VARM9_SLEEP_1010)
	#define VARM9_SLEEP_VAL		0x3
#elif defined(CONFIG_VARM9_SLEEP_1040)
	#define VARM9_SLEEP_VAL		0x4
#elif defined(CONFIG_VARM9_SLEEP_1070)
	#define VARM9_SLEEP_VAL		0x5
#elif defined(CONFIG_VARM9_SLEEP_1100)
	#define VARM9_SLEEP_VAL		0x6
#elif defined(CONFIG_VARM9_SLEEP_1130)
	#define VARM9_SLEEP_VAL		0x7
#endif

#endif

#if defined(CONFIG_MACH_IM98XXV1)
unsigned int reg_pmu_ldovs4_flag = 1;
unsigned int reg_pmu_ldovs4_value = 0;
unsigned int reg_pmu_a9psc_flag = 1;
unsigned int reg_pmu_a9psc_value = 0;
unsigned int reg_pmu_modpsc_flag = 1;
unsigned int reg_pmu_modpsc_value = 0;
#endif

