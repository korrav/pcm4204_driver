/*
 * pcm4204_driver.h
 *
 *  Created on: 23.04.2012
 *      Author: korrav
 */

#ifndef PCM4204_DRIVER_H_
#define PCM4204_DRIVER_H_
#include <linux/ioctl.h>

//СТРУКТУРА ФЛАГОВ, СИГНАЛИЗИРУЮЩИХ ОБ ОШИБКЕ В ДРАЙВЕРЕ АЦП
union error_adc
{
	int error;
	struct {
		unsigned is_fragmentation:1;    //сигнализирует о непрочитанном до конца буфере (обрыв данных)
		unsigned is_passed_sync:1;	    //пропущенная синхронизация
		unsigned is_error_cc_edma:1;	//ошибка контроллера каналов EDMA (данный флаг проверяется, если АЦП остановлен pcm4204_status.mode = STOP_MODE)
		unsigned is_error_tc_edma:1;	//ошибка контроллера передачи EDMA	(данный флаг проверяется, если АЦП остановлен pcm4204_status.mode = STOP_MODE)
	};
};
//РЕЖИМЫ РАБОТЫ АЦП
enum pcm4204_mode {
	STOP_MODE,
	RUN_MODE,
};
//СТРУКТУРА БЛОКА ДАННЫХ, ПЕРЕДАВАЕМОГО ДЛЯ ОБРАБОТКИ В АЦП
#define NOT_SYNC -1		//в течение заполнения данного блока сигнал синхронизации не получен
struct dataUnit_ADC
{
	union error_adc error;		//ошибки в драйвере АЦП
	enum pcm4204_mode mode;     //режим работы АЦП
	unsigned int amountCount;	//количество отсчётов в блоке данных (1 отс = 4 x 4 байт)
	unsigned int count;			//порядковый номер первого отсчёта в блоке данных
	int* pUnit;					//указатель на блок данных
};

//КОМАНДЫ IOCTL
#define ID_IO_PCM4204 200	//идентификатор для команд ioctl
//запуск АЦП
#define IOCTL_ADC_START _IO(ID_IO_PCM4204, 0)
//синхронизация для АЦП
#define IOCTL_ADC_SYNC _IO(ID_IO_PCM4204, 1)
//остановка АЦП
#define IOCTL_ADC_STOP _IO(ID_IO_PCM4204, 2)
//приём данных от АЦП
#define IOCTL_ADC_GET_MESSAGE _IOWR(ID_IO_PCM4204, 0, struct dataUnit_ADC*)

//АДРЕСА РЕГИСТРОВ MCASP
#define BASE_OF_MCASP 					0x01d00000

#define DAVINCI_MCASP_PID_REG			0x00
#define DAVINCI_MCASP_PWREMUMGT_REG		0x04

#define DAVINCI_MCASP_PFUNC_REG			0x10
#define DAVINCI_MCASP_PDIR_REG			0x14
#define DAVINCI_MCASP_PDOUT_REG			0x18
#define DAVINCI_MCASP_PDSET_REG			0x1c

#define DAVINCI_MCASP_PDCLR_REG			0x20

#define DAVINCI_MCASP_TLGC_REG			0x30
#define DAVINCI_MCASP_TLMR_REG			0x34

#define DAVINCI_MCASP_GBLCTL_REG		0x44
#define DAVINCI_MCASP_AMUTE_REG			0x48
#define DAVINCI_MCASP_LBCTL_REG			0x4c

#define DAVINCI_MCASP_TXDITCTL_REG		0x50

#define DAVINCI_MCASP_GBLCTLR_REG		0x60
#define DAVINCI_MCASP_RXMASK_REG		0x64
#define DAVINCI_MCASP_RXFMT_REG			0x68
#define DAVINCI_MCASP_RXFMCTL_REG		0x6c

#define DAVINCI_MCASP_ACLKRCTL_REG		0x70
#define DAVINCI_MCASP_AHCLKRCTL_REG		0x74
#define DAVINCI_MCASP_RXTDM_REG			0x78
#define DAVINCI_MCASP_EVTCTLR_REG		0x7c

#define DAVINCI_MCASP_RXSTAT_REG		0x80
#define DAVINCI_MCASP_RXTDMSLOT_REG		0x84
#define DAVINCI_MCASP_RXCLKCHK_REG		0x88
#define DAVINCI_MCASP_REVTCTL_REG		0x8c

#define DAVINCI_MCASP_GBLCTLX_REG		0xa0
#define DAVINCI_MCASP_TXMASK_REG		0xa4
#define DAVINCI_MCASP_TXFMT_REG			0xa8
#define DAVINCI_MCASP_TXFMCTL_REG		0xac

#define DAVINCI_MCASP_ACLKXCTL_REG		0xb0
#define DAVINCI_MCASP_AHCLKXCTL_REG		0xb4
#define DAVINCI_MCASP_TXTDM_REG			0xb8
#define DAVINCI_MCASP_EVTCTLX_REG		0xbc

#define DAVINCI_MCASP_TXSTAT_REG		0xc0
#define DAVINCI_MCASP_TXTDMSLOT_REG		0xc4
#define DAVINCI_MCASP_TXCLKCHK_REG		0xc8
#define DAVINCI_MCASP_XEVTCTL_REG		0xcc

#define DAVINCI_MCASP_DITCSRA0_REG		0x100
#define DAVINCI_MCASP_DITCSRA1_REG		0x104
#define DAVINCI_MCASP_DITCSRA2_REG		0x108
#define DAVINCI_MCASP_DITCSRA3_REG		0x10c
#define DAVINCI_MCASP_DITCSRA4_REG		0x110
#define DAVINCI_MCASP_DITCSRA5_REG		0x114

#define DAVINCI_MCASP_DITCSRB0_REG		0x118
#define DAVINCI_MCASP_DITCSRB1_REG		0x11c
#define DAVINCI_MCASP_DITCSRB2_REG		0x120
#define DAVINCI_MCASP_DITCSRB3_REG		0x124
#define DAVINCI_MCASP_DITCSRB4_REG		0x128
#define DAVINCI_MCASP_DITCSRB5_REG		0x12c

#define DAVINCI_MCASP_DITUDRA0_REG		0x130
#define DAVINCI_MCASP_DITUDRA1_REG		0x134
#define DAVINCI_MCASP_DITUDRA2_REG		0x138
#define DAVINCI_MCASP_DITUDRA3_REG		0x13c
#define DAVINCI_MCASP_DITUDRA4_REG		0x140
#define DAVINCI_MCASP_DITUDRA5_REG		0x144

#define DAVINCI_MCASP_DITUDRB0_REG		0x148
#define DAVINCI_MCASP_DITUDRB1_REG		0x14c
#define DAVINCI_MCASP_DITUDRB2_REG		0x150
#define DAVINCI_MCASP_DITUDRB3_REG		0x154
#define DAVINCI_MCASP_DITUDRB4_REG		0x158
#define DAVINCI_MCASP_DITUDRB5_REG		0x15c

#define DAVINCI_MCASP_XRSRCTL0_REG		0x180
#define DAVINCI_MCASP_XRSRCTL1_REG		0x184
#define DAVINCI_MCASP_XRSRCTL2_REG		0x188
#define DAVINCI_MCASP_XRSRCTL3_REG		0x18c
#define DAVINCI_MCASP_XRSRCTL4_REG		0x190
#define DAVINCI_MCASP_XRSRCTL5_REG		0x194
#define DAVINCI_MCASP_XRSRCTL6_REG		0x198
#define DAVINCI_MCASP_XRSRCTL7_REG		0x19c
#define DAVINCI_MCASP_XRSRCTL8_REG		0x1a0
#define DAVINCI_MCASP_XRSRCTL9_REG		0x1a4
#define DAVINCI_MCASP_XRSRCTL10_REG		0x1a8
#define DAVINCI_MCASP_XRSRCTL11_REG		0x1ac
#define DAVINCI_MCASP_XRSRCTL12_REG		0x1b0
#define DAVINCI_MCASP_XRSRCTL13_REG		0x1b4
#define DAVINCI_MCASP_XRSRCTL14_REG		0x1b8
#define DAVINCI_MCASP_XRSRCTL15_REG	    0x1bc

#define DAVINCI_MCASP_TXBUF0_REG		0x200
#define DAVINCI_MCASP_TXBUF1_REG		0x204
#define DAVINCI_MCASP_TXBUF2_REG		0x208
#define DAVINCI_MCASP_TXBUF3_REG		0x20c
#define DAVINCI_MCASP_TXBUF4_REG		0x210
#define DAVINCI_MCASP_TXBUF5_REG		0x214
#define DAVINCI_MCASP_TXBUF6_REG		0x218
#define DAVINCI_MCASP_TXBUF7_REG		0x21c
#define DAVINCI_MCASP_TXBUF8_REG		0x220
#define DAVINCI_MCASP_TXBUF9_REG		0x224
#define DAVINCI_MCASP_TXBUF10_REG		0x228
#define DAVINCI_MCASP_TXBUF11_REG		0x22c
#define DAVINCI_MCASP_TXBUF12_REG		0x230
#define DAVINCI_MCASP_TXBUF13_REG		0x234
#define DAVINCI_MCASP_TXBUF14_REG		0x238
#define DAVINCI_MCASP_TXBUF15_REG		0x23c

#define DAVINCI_MCASP_RXBUF0_REG		0x280
#define DAVINCI_MCASP_RXBUF1_REG		0x284
#define DAVINCI_MCASP_RXBUF2_REG		0x288
#define DAVINCI_MCASP_RXBUF3_REG		0x28c
#define DAVINCI_MCASP_RXBUF4_REG		0x290
#define DAVINCI_MCASP_RXBUF5_REG		0x294
#define DAVINCI_MCASP_RXBUF6_REG		0x298
#define DAVINCI_MCASP_RXBUF7_REG		0x29c
#define DAVINCI_MCASP_RXBUF8_REG		0x2a0
#define DAVINCI_MCASP_RXBUF9_REG		0x2a4
#define DAVINCI_MCASP_RXBUF10_REG		0x2a8
#define DAVINCI_MCASP_RXBUF11_REG		0x2ac
#define DAVINCI_MCASP_RXBUF12_REG		0x2b0
#define DAVINCI_MCASP_RXBUF13_REG		0x2b4
#define DAVINCI_MCASP_RXBUF14_REG		0x2b8
#define DAVINCI_MCASP_RXBUF15_REG		0x2bc

#define DAVINCI_MCASP_AFIFOREV          0x1000
#define DAVINCI_MCASP_WFIFOCTL		    0x1010
#define DAVINCI_MCASP_WFIFOSTS		    0x1014
#define DAVINCI_MCASP_RFIFOCTL		    0x1018
#define DAVINCI_MCASP_RFIFOSTS		    0x101C

#define DAVINCI_MCASP_RBUF 				0x2000


//ПОЛЯ И БИТЫ ОТДЕЛЬНЫХ РЕГИСТРОВ, А ТАКЖЕ ОПЕРАЦИИ С НИМИ

//регистр  DAVINCI_MCASP_PWREMUMGT_REG - управление питанием и эмуляцией
#define MCASP_FREE	BIT(0)	//выключение модуля
#define MCASP_SOFT	BIT(1)	//включение модуля

//регистр DAVINCI_MCASP_RFIFOCTL - управление аппаратной очередью FIFO буфера чтения
#define FIFO_ENABLE	BIT(16)	//включение FIFO очереди
#define NUMEVT_MASK	(0xFF << 8)	//маска поля "количество слов на одно DMA событие"
#define NUMDMA_MASK	(0xFF)	//маска поля "количество слов на одну передачу"

//регистр  DAVINCI_MCASP_RXFMT_REG - форматирование потока принятых данных
#define RXROT(val)	(val)	    //смещение вправо принятых данных
#define RXSEL		BIT(3)      //данные считываются либо от DMA порта (0), либо от периферийной шины (1)
#define RXSSZ(val)	(val<<4)	//размер слота
#define RXPBIT(val)	(val<<8)	//бит, используемый для заполнение маски
#define RXPAD(val)	(val<<13)	//значение, которым заполняются замаскированные биты
#define RXORD		BIT(15)		//порядок следования битов
#define FSRDLY(val)	(val<<16)	//задержка

//регистр DAVINCI_MCASP_RXFMCTL_REG -управление приёмом фрейма
#define FSRPOL		BIT(0)	//полярность
#define AFSRE		BIT(1)	//внутренний или внешний источник
#define FSRDUR		BIT(4)	//длительность сигнала во время активного периода
#define FSRMOD(val)	(val<<7)	//количество слотов во фрейме

//регистр DAVINCI_MCASP_ACLKRCTL_REG - управление тактовым сигналом приёмника
#define ACLKRDIV(val)	(val)	//коэффициент делителя частоты
#define ACLKRE		    BIT(5)	//внутренний или внешний источник
#define ACLKRPOL	    BIT(7)	//полярность

//регистр DAVINCI_MCASP_ACLKXCTL_REG - управление тактовым сигналом передатчика
#define TX_ASYNC	BIT(6) //асинхронное тактирование передатчика и приёмника

// регистр DAVINCI_MCASP_AHCLKRCTL_REG - управление высокочастотным сигналом приёмника
#define AHCLKRDIV(val)	(val)	//коэффициент делителя частоты
#define AHCLKRPOL	BIT(14)	//полярность
#define AHCLKRE		BIT(15)	//внутренний или внешний источник

//регистр DAVINCI_MCASP_XRSRCTLn_REG - управление сериализатором с порядковым номером n
#define MODE(val)	(val)	    //режим работы
#define DISMOD(val) (val<<2)	//конфигурация вывода в неактивном состоянии
#define TXSTATE		BIT(4)		//состояние буфера передачи
#define RXSTATE		BIT(5)		//состояние буфера приёма

//регистр DAVINCI_MCASP_PFUNC_REG - управляет функциональностью выводов
#define AXR(n)		(1<<n) 		//конфигурирование функциональности вывода данных n
#define PFUNC_AMUTE	BIT(25)		//конфигурирование функциональности вывода AMUTE
#define ACLKX		BIT(26)		//конфигурирование функциональности вывода ACLKX
#define AHCLKX		BIT(27)		//конфигурирование функциональности вывода AHCLKX
#define AFSX		BIT(28)		//конфигурирование функциональности вывода AFSX
#define ACLKR		BIT(29)		//конфигурирование функциональности вывода ACLKR
#define AHCLKR		BIT(30)		//конфигурирование функциональности вывода AHCLKR
#define AFSR		BIT(31)		//конфигурирование функциональности вывода AFSR

//регистр DAVINCI_MCASP_PDIR_REG - управление направлением выводов
#define AXR(n)		(1<<n)	//конфигурирование направления вывода данных n
#define PDIR_AMUTE	BIT(25)	//конфигурирование направления вывода AMUTE
#define ACLKX		BIT(26)	//конфигурирование направления вывода ACLKX
#define AHCLKX		BIT(27)	//конфигурирование направления вывода AHCLKX
#define AFSX		BIT(28)	//конфигурирование направления вывода AFSX
#define ACLKR		BIT(29)	//конфигурирование направления вывода ACLKR
#define AHCLKR		BIT(30)	//конфигурирование направления вывода AHCLKR
#define AFSR		BIT(31)	//конфигурирование направления вывода AFSR

//регистр DAVINCI_MCASP_GBLCTL_REG - глобальное управление модулем
#define RXCLKRST	BIT(0)	//сброс делителя битового тактирования приёмника
#define RXHCLKRST	BIT(1)	//сброс делителя высокоскоростного тактирования приёмника
#define RXSERCLR	BIT(2)	//обнуление содержимого буферов сериализаторов приёмника
#define RXSMRST		BIT(3)	//сброс машины состояния приёмника
#define RXFSRST		BIT(4)	//сброс генератора фреймовой синхронизации приёмника
#define TXCLKRST	BIT(8)	//сброс делителя битового тактирования передатчика
#define TXHCLKRST	BIT(9)	//сброс делителя высокоскоростного тактирования передатчика
#define TXSERCLR	BIT(10)	//обнуление содержимого буферов сериализаторов передатчика
#define TXSMRST		BIT(11)	//сброс машины состояния передатчика
#define TXFSRST		BIT(12)	//сброс генератора фреймовой синхронизации передатчика

//ПОЛЯ РЕГИСТРА OPT
#define STATIC_SHIFT                3
#define TCCMODE_SHIFT               11
#define TCINTEN_SHIFT               20
#define ITCINTEN_SHIFT              21
#define TCCHEN_SHIFT                22
#define ITCCHEN_SHIFT               23

//ПОЛЬЗОВАТЕЛЬСКИЕ НАСТРОЙКИ
#define ADC_NAME "pcm4204"	//имя узла АЦП устройства
#define DEV_CLASS_ADC "MAD - adc"	//имя класса, к которому принадлежат устройства акустического модуля
#define SIZE_FIFO_READ 4 //размер аппаратного FIFO буфера чтения (обязательно должен быть кратен 4 и не превышать 64)
#define BUF_SIZE 800000	//размер DMA буфера (ACNT = 4, BCNT = 4, CCNT = 50 000

//РАЗНОЕ

#define COMPL_READ_BUFFER -1	//данный буффер полностью прочитан
#define SAMPL_SIZE 16			//размер одного отсчёта (по всем 4 каналам)
#define NUM_CHANNEL_MCASP_RX 0	//номер EDMA канала приёмника MCASP
#define MCASP_IRQ 11	//номер прерывания для MCASP EDMA
#endif /* PCM4204_DRIVER_H_ */
