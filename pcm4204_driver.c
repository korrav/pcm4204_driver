/*
 * pcm4204_driver.cpp
 *
 *  Created on: 23.04.2012
 *      Author: korrav
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <asm/io.h>
#include <linux/fs.h>
#include <mach/clock.h>
#include <mach/mux.h>
#include <mach/da8xx.h>
#include <mach/gpio.h>	//перед компиляцией изменить на <asm/gpio.h>
#include <linux/dma-mapping.h>
#include <linux/poll.h>
#include <mach/edma.h>
#include <mach/irqs.h>
#include "pcm4204_driver.h"
#include <linux/moduleparam.h>
MODULE_LICENSE( "GPL");
MODULE_AUTHOR( "Andrej Korobchenko <korrav@yandex.ru>");
MODULE_VERSION( "0:1.0");
/*Параметр rate представляет собой скорость АЦП. Всего существует 10 скоростей: 1 - 187.5 кГц, 2 - 125 кГц, 3 - 93.75 кГц,
 * 4 -75 кГц, 5 - 62.5 кГц, 6 - 53.571, 7 - 46.875, 8 - 41.666 кГц, 9 - 37.5 кГц, 10 - 34,091 кГц.
 * Параметр normal отвечает за номировку значений отсчётов АЦП в соответствие с действующим коэффициентом усиления: y - разрешена,
 * n - запрещена
 */
static char *normal = "n"; //параметр ядра определяет используется нормировка сигнала("y") или нет ("n")
static int rate = 1; //параметр ядра определяет скорость дискретизации АЦП. Возможные значения: 100 и 200
module_param(normal, charp, 0444);
module_param(rate, int, 0444);

extern struct davinci_soc_info davinci_soc_info; //общая структура чипа
//адреса
static void __iomem *base = NULL; //начальный адрес io пространства McAsp

//переменные модели устройство
static char device_name[] = ADC_NAME; //имя устройства
dev_t dev_pcm4204; //содержит старший и младший номер устройства
static struct cdev cdev_pcm4204;
static struct class* devclass; //класс устройств, к которому принадлежит pga2500

//DMA
struct edmacc_param param_set_buf[2]; //PARAM для буферов
int acnt = 4, bcnt = 4, ccnt = 50000, dst_bindex = 4, dst_cindex = 16; //параметры канала McAsp0
int num_channel = NUM_CHANNEL_MCASP_RX; //номер канала McAsp0

//локальные переменные драйвера
DECLARE_COMPLETION(done_PingPong);
//условная переменная, указывающая на то, что произошёл очередной ping-pong
DECLARE_WAIT_QUEUE_HEAD(qwait);
//очередь ожидания для операции чтения
static struct {
	enum pcm4204_mode mode; //текущий режим работы
	short num_buf_write; //номер заполняемого в данный момент буфера
	short num_buf_read; //номер считываемого в данный момент буфера
	unsigned int users; //количество пользователей в данный момент, использующих АЦП
	union error_adc error; //ошибки в драйвере АЦП
	unsigned int gain; //текущий коэффициент усиления
	struct mutex mutex_lock; //мьютекс
	int count; //текущий отсчёт (обновляется при ping-ponge)

} pcm4204_status; //состояние драйвера

struct gain_MAD { //структура, содержащая номер отсчёта и новое значение, которое приобрёл коэффициент усиления, начиная с данного отсчёта
	struct list_head list;
	int gain; //величина коэффициента усиления
	int sampl; //номер отсчёта
};
struct sync_marks {
	unsigned int count_of_first; //значение count для первого отсчёта в буфере
	unsigned int number_of_first; //номер отсчёта в буфере,  указывающий на новую синхронизацию (если нет, то =NOT_SYNC)
};

static struct buffer_adc { //структура состояния буфера
	bool compl_fill; //состояние заполненности буфера (полностью или нет)
	struct sync_marks sync_m; //метки синхронизации для данного буфера
	struct list_head gain; //список моментов изменения коэффициента усиления
	int loc; //текущая позиция, учитываемая при считывании буфера (=COMPL_READ_BUFFER, когда буфер полностью прочитан) 1 ед = 4 x 4 байта;
	char* virt_loc; //виртуальный адрес буфера
	dma_addr_t bus_loc; //шинный адрес буфера
	int size; //размер буфера (в байтах)
	int num_slot; //номер слота буфера
} buf[2];

//ФУНКЦИИ ВЗАИМОДЕЙСТВИЯ С РЕГИСТРАМИ

static inline void mcasp_set_bits(void __iomem *reg, u32 val) //установка битов val в регистр по адресу reg
{
	__raw_writel(__raw_readl(reg) | val, reg);
}

static inline void mcasp_clr_bits(void __iomem *reg, u32 val) //обнуление битов val в регистре по адресу reg
{
	__raw_writel((__raw_readl(reg) & ~(val)), reg);
}

static inline void mcasp_mod_bits(void __iomem *reg, u32 val, u32 mask) //установка битов val в регистре по адресу reg, в котором предварительно обнулены биты mask
{
	__raw_writel((__raw_readl(reg) & ~mask) | val, reg);
}

static inline void mcasp_set_reg(void __iomem *reg, u32 val) //запись значения val в регистр по адресу reg
{
	__raw_writel(val, reg);
}

static inline u32 mcasp_get_reg(void __iomem *reg) //чтение содержимого регистра reg
{
	return ((unsigned int) __raw_readl(reg) );
}

static inline void mcasp_set_ctl_reg(void __iomem *regs, u32 val) //установка битов val в управляющем регистре reg типа GBLCTL
{
	int i = 0;

	mcasp_set_bits(regs, val);
	for (i = 0; i < 1000; i++) {
		if ((mcasp_get_reg(regs) & val) == val)
			break;
	}

	if (i == 1000 && ((mcasp_get_reg(regs) & val) != val))
		printk(KERN_ERR "GBLCTL write error\n");
}

//ЛОКАЛЬНЫЕ ФУНКЦИИ ДРАЙВЕРА

static void print_reg_mcasp() {
	printk(
			KERN_INFO "register GBLCTL: address is equal to %x value is equal to %x\n",
			(char*) base + DAVINCI_MCASP_GBLCTL_REG,
			mcasp_get_reg((char*) base + DAVINCI_MCASP_GBLCTL_REG));
	printk(
			KERN_INFO "register RFIFOCTL: address is equal to %x value is equal to %x\n",
			(char*) base + DAVINCI_MCASP_RFIFOCTL,
			mcasp_get_reg((char*) base + DAVINCI_MCASP_RFIFOCTL));
	printk(
			KERN_INFO "register RMASK: address is equal to %x value is equal to %x\n",
			(char*) base + DAVINCI_MCASP_RXMASK_REG,
			mcasp_get_reg((char*) base + DAVINCI_MCASP_RXMASK_REG));
	printk(
			KERN_INFO "register RFMT: address is equal to %x value is equal to %x\n",
			(char*) base + DAVINCI_MCASP_RXFMT_REG,
			mcasp_get_reg((char*) base + DAVINCI_MCASP_RXFMT_REG));
	printk(
			KERN_INFO "register AFSRCTL: address is equal to %x value is equal to %x\n",
			(char*) base + DAVINCI_MCASP_RXFMCTL_REG,
			mcasp_get_reg((char*) base + DAVINCI_MCASP_RXFMCTL_REG));
	printk(
			KERN_INFO "register ACLKXCTL: address is equal to %x value is equal to %x\n",
			(char*) base + DAVINCI_MCASP_ACLKXCTL_REG,
			mcasp_get_reg((char*) base + DAVINCI_MCASP_ACLKXCTL_REG));
	printk(
			KERN_INFO "register ACLKRCTL: address is equal to %x value is equal to %x\n",
			(char*) base + DAVINCI_MCASP_ACLKRCTL_REG,
			mcasp_get_reg((char*) base + DAVINCI_MCASP_ACLKRCTL_REG));
	printk(
			KERN_INFO "register AHCLKRCTL: address is equal to %x value is equal to %x\n",
			(char*) base + DAVINCI_MCASP_AHCLKRCTL_REG,
			mcasp_get_reg((char*) base + DAVINCI_MCASP_AHCLKRCTL_REG));
	printk(
			KERN_INFO "register RTDM: address is equal to %x value is equal to %x\n",
			(char*) base + DAVINCI_MCASP_RXTDM_REG,
			mcasp_get_reg((char*) base + DAVINCI_MCASP_RXTDM_REG));
	printk(
			KERN_INFO "register SRCTL8: address is equal to %x value is equal to %x\n",
			(char*) base + DAVINCI_MCASP_XRSRCTL8_REG,
			mcasp_get_reg((char*) base + DAVINCI_MCASP_XRSRCTL8_REG));
	printk(
			KERN_INFO "register SRCTL10: address is equal to %x value is equal to %x\n",
			(char*) base + DAVINCI_MCASP_XRSRCTL10_REG,
			mcasp_get_reg((char*) base + DAVINCI_MCASP_XRSRCTL10_REG));
	printk(
			KERN_INFO "register PFUNC: address is equal to %x value is equal to %x\n",
			(char*) base + DAVINCI_MCASP_PFUNC_REG,
			mcasp_get_reg((char*) base + DAVINCI_MCASP_PFUNC_REG));
	printk(
			KERN_INFO "register PDIR: address is equal to %x value is equal to %x\n",
			(char*) base + DAVINCI_MCASP_PDIR_REG,
			mcasp_get_reg((char*) base + DAVINCI_MCASP_PDIR_REG));
	return;
}

static void start_adc(void) //запуск АЦП
{
	if (pcm4204_status.mode == STOP_MODE) {
		pcm4204_status.error.error = 0;
		gpio_set_value(143, 1); //вывод АЦП из сброса
		//printk(KERN_INFO "Value registers McAspi before modification\n");
		//print_reg_mcasp();
		mcasp_set_ctl_reg((char*) base + DAVINCI_MCASP_GBLCTLR_REG, RXHCLKRST); //вывод из сброса делителя высокочастотного сигнала приёмника
		mcasp_set_ctl_reg((char*) base + DAVINCI_MCASP_GBLCTLR_REG, RXCLKRST); //вывод из сброса делителя битового тактирования приёмника
		mcasp_set_reg((char*) base + DAVINCI_MCASP_RXSTAT_REG, 0xFFFFFFFF); //снятие всех флагов-ошибок
		mcasp_set_ctl_reg((char*) base + DAVINCI_MCASP_GBLCTLR_REG, RXSERCLR); //вывод из сброса всех сериализаторов приёмника
		mcasp_set_ctl_reg((char*) base + DAVINCI_MCASP_GBLCTLR_REG, RXSMRST); //вывод из сброса машины состояния приёмника
		mcasp_set_ctl_reg((char*) base + DAVINCI_MCASP_GBLCTLR_REG, RXFSRST); //вывод из сброса генератора тактирования фреймов
		pcm4204_status.mode = RUN_MODE;
		//printk(KERN_INFO "Started to perform ADC\n");
		//printk(KERN_INFO "Value registers McAspi after modification\n");
		//print_reg_mcasp();
	}
	return;
}

static void stop_adc(void) //остановка АЦП
{
	if (pcm4204_status.mode == RUN_MODE) {
		gpio_set_value(143, 0); //помещение АЦП в режим сброса
		mcasp_set_reg((char*) base + DAVINCI_MCASP_GBLCTLR_REG, 0);
		mcasp_set_reg((char*) base + DAVINCI_MCASP_RXSTAT_REG, 0xFFFFFFFF);
		pcm4204_status.mode = STOP_MODE;
		pcm4204_status.error.is_fragmentation = 1;
		pcm4204_status.error.is_passed_sync = 1;
		//printk(KERN_INFO "Stop to perform ADC\n");
	}
	return;
}

//ФУНКЦИИ СТРУКТУРЫ FILE_OPERATIONS
int pcm4204_open(struct inode *inode, struct file *filp) {
	pcm4204_status.users++;
	nonseekable_open(inode, filp); //сообщение ядру, что данное устройство не поддерживает произвольный доступ к данным
	//printk(KERN_INFO "Open file ADC\n");
	return (0);
}

static int pcm4204_release(struct inode *inode, struct file *filp) {
	pcm4204_status.users--;
	stop_adc();
	return (0);
}
ssize_t pcm4204_read(struct file *filp, char __user *bufu, size_t count,
		loff_t *fpos) {
	int buf_cur_size; //размер текущего буфера в единицах SAMPL_SIZE
	int num_trans_sampl = 0; // количество отсчётов, которое будет передано на данном запросе
	int i = 0;
	int cur_loc = 0;
	struct gain_MAD* change_gain = NULL; // элемент в списке gain текущего считываемого буфера
	struct list_head* p, *next; //указатель на его структуру list_head, а также на следующий элемент в списке
	if (count % SAMPL_SIZE)
		return (-EPROTO);
	count /= SAMPL_SIZE;
	//printk(KERN_INFO "You must supply %d samples\n", count);
	mutex_lock(&pcm4204_status.mutex_lock);
	disable_irq(MCASP_IRQ);
	buf_cur_size = buf[pcm4204_status.num_buf_read].size / SAMPL_SIZE;
	//проверка наблюдается ли фрагментация данных
	if (pcm4204_status.error.is_fragmentation == 1) {
		pcm4204_status.error.is_fragmentation = 0;
		//printk(KERN_INFO "There is fragmentation of data\n");
	} else
	//printk(KERN_INFO "There has been no data fragmentation\n");
	//возможно ли сейчас передать очередной блок данных АЦП?
	if (pcm4204_status.mode == STOP_MODE) { //проверка на наличие фатальных ошибок
		enable_irq(MCASP_IRQ);
		mutex_unlock(&pcm4204_status.mutex_lock);
		return (-EIO);
	} else if (buf[pcm4204_status.num_buf_read].compl_fill
			== false|| buf[pcm4204_status.num_buf_read].loc == COMPL_READ_BUFFER) {
	//ожидание, когда произойдёт ротация буферов АЦП (ping-pong)
//printk	(KERN_INFO "At the moment it is impossible to transmit data\n");
INIT_COMPLETION	(done_PingPong);
	disable_irq(MCASP_IRQ);
	wait_for_completion(&done_PingPong);
	//printk(KERN_INFO "It is now possible to transfer data\n");
	enable_irq(MCASP_IRQ);
}
	//вычисление сколько данных будет передано
	if (count > (buf_cur_size - buf[pcm4204_status.num_buf_read].loc)) //возможно ли передать полностью запрошенный объём данных?
		num_trans_sampl = buf_cur_size - buf[pcm4204_status.num_buf_read].loc;
	else
		num_trans_sampl = count;
	//printk(KERN_INFO "Will be transferred to %d sampls\n", num_trans_sampl);
	//корректировка значений отсчётов АЦП в соответствие с изменением с течением времени коэффициента усиления
	if (normal[0] == 'y') {
		cur_loc = buf[pcm4204_status.num_buf_read].loc;
		if (!list_empty(&buf[pcm4204_status.num_buf_read].gain)) { //в текущем считываемом буфере данных АЦП имеются моменты изменения коэффициента усиления?
			list_for_each_safe(p, next, &buf[pcm4204_status.num_buf_read].gain)
			{
				change_gain = list_entry(p, struct gain_MAD, list);
				if (change_gain->sampl >= cur_loc
						&& change_gain->sampl <= cur_loc + num_trans_sampl) { //в текущий запрошенный блок данных входят моменты времени изменения коэффициента усиления?
					for (i = 0; i < (change_gain->sampl - cur_loc) * 4; i++) { //нормировка
						*((int*) (buf[pcm4204_status.num_buf_read].virt_loc
								+ cur_loc * SAMPL_SIZE) + i) *=
								pcm4204_status.gain;
					}
					cur_loc = change_gain->sampl;
					pcm4204_status.gain = change_gain->gain;
					list_del(&change_gain->list); //удаление элемента из списка
					kfree(change_gain); //освобождение элемента
				} else
					break;
			}
		}
		for (i = 0;
				i
						< (buf[pcm4204_status.num_buf_read].loc
								+ num_trans_sampl - cur_loc) * 4; i++) { //нормировка
			*((int*) (buf[pcm4204_status.num_buf_read].virt_loc
					+ cur_loc * SAMPL_SIZE) + i) *= pcm4204_status.gain;
		}
	}
	//заполнение буфера пользовательского пространства отсчётами АЦП
	if (copy_to_user(bufu,
			buf[pcm4204_status.num_buf_read].virt_loc
					+ buf[pcm4204_status.num_buf_read].loc * SAMPL_SIZE,
			num_trans_sampl * SAMPL_SIZE))
		return (-EFAULT);
	/*else
	 printk(KERN_INFO "In custom yanked %d bytes\n",
	 num_trans_sampl * SAMPL_SIZE);*/
	//обновление текущей позиции в считываемом буфере
	buf[pcm4204_status.num_buf_read].loc += num_trans_sampl;
	if (buf[pcm4204_status.num_buf_read].loc >= buf_cur_size - 1) //считан ли до конца текущий буфер?
		buf[pcm4204_status.num_buf_read].loc = COMPL_READ_BUFFER;
	//printk(KERN_INFO "Current position in buffer %d was equal to %d readings\n",
	//pcm4204_status.num_buf_read, buf[pcm4204_status.num_buf_read].loc);
	//разблокирование мьютекса
	pcm4204_status.error.is_fragmentation = 1;
	enable_irq(MCASP_IRQ);
	mutex_unlock(&pcm4204_status.mutex_lock);
	return (num_trans_sampl * SAMPL_SIZE);
}

int pcm4204_ioctl(struct file *filp, unsigned int cmd, unsigned long arg) {
	struct dataUnit_ADC temp_Unit;
	int buf_cur_size; //размер текущего буфера в единицах SAMPL_SIZE
	dma_addr_t cur_position; //физический адрес текущей позиции заполняемого DMA буфера
	int num_trans_sampl = 0; // количество отсчётов, которое будет передано на данном запросе
	struct gain_MAD* change_gain = NULL; // элемент в списке gain текущего считываемого буфера
	struct list_head* p, *next; //указатель на его структуру list_head, а также на следующий элемент в списке
	int i = 0;
	int cur_loc = 0;
	switch (cmd) {
	case IOCTL_ADC_START: //СТАРТ АЦП
		start_adc();
		//printk(KERN_INFO "Was launched by ADC\n");
		break;
	case IOCTL_ADC_SYNC: //ПОЛУЧЕНИЕ СИНХРОСИГНАЛА
		//вычисление номера текущего отсчёта
		disable_irq(MCASP_IRQ);
		edma_get_position(num_channel, NULL, &cur_position);
		buf[pcm4204_status.num_buf_write].sync_m.number_of_first = (cur_position
				- buf[pcm4204_status.num_buf_write].bus_loc) / SAMPL_SIZE;
		pcm4204_status.error.is_passed_sync = 0;
		//printk(KERN_INFO "numer first sampl = %u\n",
		//		buf[pcm4204_status.num_buf_write].sync_m.number_of_first);
		enable_irq(MCASP_IRQ);
		break;
	case IOCTL_ADC_STOP:
		stop_adc();
		//printk(KERN_INFO "Was stopped by the ADC\n");
		break;
	case IOCTL_ADC_GET_MESSAGE: //ПОЛУЧЕНИЕ БЛОКА ДАННЫХ АЦП
		//получение буфера пользовательского пространства
		if (copy_from_user(&temp_Unit, (void*) arg,
				sizeof(struct dataUnit_ADC)))
			return (-EFAULT);
		temp_Unit.error.error = 0;
		temp_Unit.mode = pcm4204_status.mode;
		//заполнение поля первого отсчёта в пользовательском буфере
		mutex_lock(&pcm4204_status.mutex_lock);
		disable_irq(MCASP_IRQ);
		if (buf[pcm4204_status.num_buf_read].sync_m.number_of_first == NOT_SYNC
				|| buf[pcm4204_status.num_buf_read].loc
						< buf[pcm4204_status.num_buf_read].sync_m.number_of_first)
			temp_Unit.count =
					buf[pcm4204_status.num_buf_read].sync_m.count_of_first
							+ buf[pcm4204_status.num_buf_read].loc;
		else
			temp_Unit.count = buf[pcm4204_status.num_buf_read].loc
					- buf[pcm4204_status.num_buf_read].sync_m.number_of_first;
//		printk(
//				KERN_INFO "Number of the first frame in the buffer is equal to %d\n",
//				temp_Unit.count);
		buf_cur_size = buf[pcm4204_status.num_buf_read].size / SAMPL_SIZE;
		//проверка наблюдается ли фрагментация данных
		if (pcm4204_status.error.is_fragmentation == 1) {
			//printk(KERN_INFO "There is fragmentation of data\n");
			temp_Unit.error.is_fragmentation = 1;
			pcm4204_status.error.is_fragmentation = 0;
		} else
//			printk(KERN_INFO "There has been no data fragmentation\n");
		//проверка наблюдается ли пропущенная синхронизация данных
		if (pcm4204_status.error.is_passed_sync == 1) {
			//printk(KERN_INFO "There is a missed sync\n");
			temp_Unit.error.is_passed_sync = 1;
		} else
//			printk(KERN_INFO "Nor is there a missed sync\n");
		//возможно ли сейчас передать очередной блок данных АЦП?
		if (temp_Unit.mode == STOP_MODE) { //проверка на наличие фатальных ошибок
			//printk(KERN_INFO "ADC now stopped\n");
			temp_Unit.error.is_error_cc_edma =
					pcm4204_status.error.is_error_cc_edma;
			temp_Unit.error.is_error_tc_edma =
					pcm4204_status.error.is_error_tc_edma;
			temp_Unit.amountCount = 0;
			mutex_unlock(&pcm4204_status.mutex_lock);
			enable_irq(MCASP_IRQ);
			return (0);
		} else if (buf[pcm4204_status.num_buf_read].compl_fill
				== false|| buf[pcm4204_status.num_buf_read].loc == COMPL_READ_BUFFER) {
		//ожидание, когда произойдёт ротация буферов АЦП (ping-pong)
//printk(KERN_INFO "At the moment it is impossible to transmit data\n");
INIT_COMPLETION		(done_PingPong);
		enable_irq(MCASP_IRQ);
		wait_for_completion(&done_PingPong);
		//printk(KERN_INFO "It is now possible to transfer data\n");
		disable_irq(MCASP_IRQ);
	}
	//вычисление сколько данных будет передано
	if (temp_Unit.amountCount > (buf_cur_size - buf[pcm4204_status.num_buf_read].loc)) { //возможно ли передать полностью запрошенный объём данных?
		num_trans_sampl = buf_cur_size - buf[pcm4204_status.num_buf_read].loc;
		temp_Unit.amountCount = num_trans_sampl;
	}
	else
	num_trans_sampl = temp_Unit.amountCount;
//	printk(KERN_INFO "Will be transferred to %d sampls\n", num_trans_sampl);
	//проверка на наличие отметки синхронизации на текущей транзакции
	if(buf[pcm4204_status.num_buf_read].sync_m.number_of_first != NOT_SYNC && buf[pcm4204_status.num_buf_read].sync_m.number_of_first > buf[pcm4204_status.num_buf_read].loc
			&& buf[pcm4204_status.num_buf_read].sync_m.number_of_first <= buf[pcm4204_status.num_buf_read].loc + num_trans_sampl) {
		num_trans_sampl = buf[pcm4204_status.num_buf_read].sync_m.number_of_first - buf[pcm4204_status.num_buf_read].loc -1;
		//printk(KERN_INFO "At the current transaction is synchronized\n");
	}
	//корректировка значений отсчётов АЦП в соответствие с изменением с течением времени коэффициента усиления
	if (normal[0] == 'y') {
		cur_loc = buf[pcm4204_status.num_buf_read].loc;
		if (!list_empty(&buf[pcm4204_status.num_buf_read].gain)) { //в текущем считываемом буфере данных АЦП имеются моменты изменения коэффициента усиления?
			list_for_each_safe(p, next, &buf[pcm4204_status.num_buf_read].gain) {
				change_gain = list_entry(p, struct gain_MAD, list);
				if (change_gain->sampl >=cur_loc && change_gain->sampl <= cur_loc + num_trans_sampl) { //в текущий запрошенный блок данных входят моменты времени изменения коэффициента усиления?
					for(i = 0; i < (change_gain->sampl - cur_loc) * 4; i++) { //нормировка
						*((int*)(buf[pcm4204_status.num_buf_read].virt_loc +cur_loc * SAMPL_SIZE) + i) *= pcm4204_status.gain;
					}
					cur_loc = change_gain->sampl;
					pcm4204_status.gain = change_gain->gain;
					list_del(&change_gain->list); //удаление элемента из списка
					kfree(change_gain);//освобождение элемента
				}
				else
				break;
			}
		}
		for(i = 0; i < (buf[pcm4204_status.num_buf_read].loc + num_trans_sampl - cur_loc) * 4; i++) { //нормировка
			*((int*)(buf[pcm4204_status.num_buf_read].virt_loc +cur_loc * SAMPL_SIZE) + i) *= pcm4204_status.gain;
		}
	}
	//заполнение буфера пользовательского пространства отсчётами АЦП
	if (copy_to_user(temp_Unit.pUnit, buf[pcm4204_status.num_buf_read].virt_loc + buf[pcm4204_status.num_buf_read].loc * SAMPL_SIZE, num_trans_sampl * SAMPL_SIZE))
	return(-EFAULT);
//	printk(KERN_INFO "In custom yanked %d bytes\n", num_trans_sampl * SAMPL_SIZE);
	//обновление текущей позиции в считываемом буфере
	buf[pcm4204_status.num_buf_read].loc += num_trans_sampl;
	if (buf[pcm4204_status.num_buf_read].loc >= (buf_cur_size - 1))//считан ли до конца текущий буфер?
	buf[pcm4204_status.num_buf_read].loc = COMPL_READ_BUFFER;
//	printk(KERN_INFO "Current position in buffer %d was equal to %d readings\n",
//			pcm4204_status.num_buf_read, buf[pcm4204_status.num_buf_read].loc);
	//передача заполненного буфера АЦП процессу пользовательского пространства
	enable_irq(MCASP_IRQ);
	mutex_unlock(&pcm4204_status.mutex_lock);
	if (copy_to_user( (void*)arg, &temp_Unit, sizeof(struct dataUnit_ADC)))
	return(-EFAULT);
	return (num_trans_sampl);
	break;
}
	return (0);
}

unsigned int pcm4204_poll(struct file *filp, struct poll_table *wait) {
//	printk(KERN_INFO "Function is called poll\n");
	unsigned int mask = 0;
	mutex_lock(&pcm4204_status.mutex_lock);
	disable_irq(MCASP_IRQ);
	poll_wait(filp, &qwait, wait);
	if (pcm4204_status.mode == STOP_MODE) {
		//printk(KERN_INFO "ADC now stopped\n");
		mask = POLLHUP;
	} else if (buf[pcm4204_status.num_buf_read].compl_fill
			== false|| buf[pcm4204_status.num_buf_read].loc == COMPL_READ_BUFFER)mask = 0;
	else {
//		//printk(KERN_INFO "Now you can read data\n");
		mask = POLLIN | POLLRDNORM;
	}
	enable_irq(MCASP_IRQ);
	mutex_unlock(&pcm4204_status.mutex_lock);
	return (mask);
}
//СТРУКТУРА FILE_OPERATIONS
static const struct file_operations pcm4204_fops = { .owner = THIS_MODULE,
		.open = pcm4204_open, .release = pcm4204_release, .unlocked_ioctl =
				pcm4204_ioctl, .poll = pcm4204_poll, };
//ФУНКЦИЯ ОБРАТНОГО ВЫЗОВА (вызывается после полного заполнения буфера Dma)
static void callback_dma(unsigned lch, u16 ch_status, void *data) {
	int buf_temp = pcm4204_status.num_buf_read;
	struct gain_MAD* unit_gain = NULL; //указатель на последний узел в списке gain только что считываемого буфера
	struct list_head* p, *next; //указатель на его структуру list_head, а также на следующий элемент в списке
	switch (ch_status) {
	case DMA_COMPLETE: //нормальное завершение передачи EDMA
		/*		if(in_interrupt())
		 printk(KERN_INFO "Performing is in interrupt context\n");
		 else
		 printk(KERN_INFO "Performing is in the context of the process\n");
		 if(in_irq())
		 printk(KERN_INFO "This is - the top-level handler\n");
		 else
		 printk(KERN_INFO "This is - the bottom-level handler\n"); */

		//фиксирование факта, что только что записанный буфер полностью заполнен
		buf[pcm4204_status.num_buf_write].compl_fill = true;
		//установка текущей позиции считывания в начало только что записанного буфера
		buf[pcm4204_status.num_buf_write].loc = 0;
		//успелся ли считаться предыдущий буфер ?
		if (buf[pcm4204_status.num_buf_read].loc != COMPL_READ_BUFFER) {
			//printk(KERN_INFO "Previous buffer does not have time to fully read\n");
			pcm4204_status.error.is_fragmentation = 1; //предыдущий буфер не был полностью считан
			if (!list_empty(&buf[pcm4204_status.num_buf_read].gain)) { //были ли моменты смены коэффициента усиления в непрочитанных данных?
				//нахождение в списке gain только что прочитанного буфера последнего элемента, перемещение его в начало списка следующего буфера
				//и освобождение оставшихся элементов из списка
				list_for_each_safe(p, next, &buf[pcm4204_status.num_buf_read].gain)
				{
					unit_gain = list_entry(p, struct gain_MAD, list);
					if (!list_is_last(p,
							&buf[pcm4204_status.num_buf_read].gain)) { //элемент последний в списке?
						list_del(p); //удаление элемента из списка
						kfree(unit_gain); //освобождение элемента
					}
				}
				unit_gain->sampl = 0;
				list_move(p, &buf[pcm4204_status.num_buf_write].gain); //перемещение в начало списка только что заполненного буфера

			}
		}
		//обновление счётчика
		if (buf[pcm4204_status.num_buf_read].sync_m.number_of_first == NOT_SYNC)
			pcm4204_status.count += buf[pcm4204_status.num_buf_read].size
					/ SAMPL_SIZE;
		else
			pcm4204_status.count = (buf[pcm4204_status.num_buf_read].size
					/ SAMPL_SIZE)
					- buf[pcm4204_status.num_buf_read].sync_m.number_of_first;
		buf[pcm4204_status.num_buf_write].sync_m.count_of_first =
				pcm4204_status.count;
		buf[pcm4204_status.num_buf_read].sync_m.number_of_first = NOT_SYNC;
//		printk(KERN_INFO "Now the count is %d\n", pcm4204_status.count);
		//фиксирование факта, что только что считываемый буфер пуст
		buf[pcm4204_status.num_buf_read].compl_fill = false;
		//изменение индексов считываемого и записываемого буфера
		pcm4204_status.num_buf_read = pcm4204_status.num_buf_write;
		pcm4204_status.num_buf_write = buf_temp;
		//printk(KERN_INFO "Only that there was a ping-pong\n");
		complete(&done_PingPong); //сигнализирование о произошедшей ротации буферов (ping-pong)
		wake_up_interruptible(&qwait);
		//сигнализирование о доступности данных для чтения
		break;
	case DMA_CC_ERROR: //ошибка контроллера каналов
		printk(KERN_ALERT "there was an error of the controler of channels\n");
		stop_adc(); //остановка АЦП
		pcm4204_status.error.is_error_cc_edma = 1;
		complete(&done_PingPong); //сигнализирование об ошибке
		break;
	case DMA_TC1_ERROR:
	case DMA_TC2_ERROR:
		printk(
				KERN_ALERT "there was an error of the controler of transmissions\n");
		stop_adc(); //остановка АЦП
		pcm4204_status.error.is_error_tc_edma = 1;
		complete(&done_PingPong); //сигнализирование об ошибке
		break;
	}
}

//ЭКСПОРТИРУЕМЫЕ ФУНКЦИИ

//функция, сигнализирующая об изменении коэффициента усиления
int change_gain_MAD(short gain) {
	struct gain_MAD* unit_gain; //указатель на новый узел, который присоединится к списку gain текущего буфера
	int loc = 0; //смещение текущего отсчёта относительно стартового адреса буфера и номер текущего отсчёта
	dma_addr_t cur_position = NULL; //физический адрес текущей позиции заполняемого DMA буфера
	switch (gain) {
	case 0:
		gain = 100;
		break;
	case 20:
		gain = 10;
		break;
	case 100:
		gain = 1;
		break;
	default:
		return (-1);
	}
	mutex_lock(&pcm4204_status.mutex_lock);
	//вычисление номера текущего отсчёта
	edma_get_position(num_channel, NULL, &cur_position);
	loc = (cur_position - buf[pcm4204_status.num_buf_write].bus_loc)
			/ SAMPL_SIZE;
	//создание и инициализация нового элемента списка gain текущего буфера
	unit_gain = kzalloc(sizeof(struct gain_MAD), GFP_KERNEL);
	if (!unit_gain)
		return (-ENOMEM);
	unit_gain->gain = gain;
	unit_gain->sampl = loc;
	//добавление нового элемента в конец списка gain текущего буфера
	list_add_tail(&unit_gain->list, &buf[pcm4204_status.num_buf_write].gain);
	mutex_unlock(&pcm4204_status.mutex_lock);
	return (0);
}
EXPORT_SYMBOL_GPL(change_gain_MAD);

//ФУНКЦИИ ИНИЦИАЛИЗАЦИИ И ВЫКЛЮЧЕНИЯ МОДУЛЯ ДРАЙВЕРА

static int __init pcm4204_init(void) {
	int status_pin_func, dev;
	int gpio_num; //хранит номер вывода
	int result; //хранит результат различных операций
	if (rate > 0 && rate <= 10)
		printk(KERN_INFO "With %d speed ADC\n", rate);
	else {
		printk(
				KERN_INFO "Used during driver loading rate parameter is incorrect. With 1 speed ADC\n");
		rate = 1;
	}

	//конфигурирование gpio выводов
	short da850_pcm4204_pins[] = { DA850_PCM_RST, DA850_PCM_FS0, DA850_PCM_FS1,
			DA850_PCM_FS2, DA850_PCM__SM, DA850_PCM_FMT0, DA850_PCM_FMT1,
			DA850_PCM_FMT2, DA850_PCM_CLIP1, DA850_PCM_CLIP2, DA850_PCM_CLIP3,
			DA850_PCM_CLIP4, DA850_PCM_HPFD, DA850_PCM_SUB, -1 };
	status_pin_func = davinci_cfg_reg_list(da850_pcm4204_pins); //конфигурирование мультиплексора функциональности выводов
	if (status_pin_func < 0) {
		printk("pin could not be muxed for GPIO functionality");
		return (status_pin_func);
	}

	gpio_num = 143; //конфигурирование RST
	status_pin_func = gpio_request(gpio_num, "RST");
	if (status_pin_func < 0) {
		printk("ERROR can not open GPIO %d\n", gpio_num);
		return (status_pin_func);
	}
	status_pin_func = gpio_direction_output(gpio_num, 1);
	if (status_pin_func < 0) {
		printk("ERROR in case of assignment of level of an GPIO %d\n",
				gpio_num);
		return (status_pin_func);
	}

	gpio_num = 142; //конфигурирование FS0
	status_pin_func = gpio_request(gpio_num, "FS0");
	if (status_pin_func < 0) {
		printk("ERROR can not open GPIO %d\n", gpio_num);
		return (status_pin_func);
	}
	if (rate >= 4 && rate <= 5 /*второй скоростной режим АЦП*/)
		status_pin_func = gpio_direction_output(gpio_num, 1);
	else
		status_pin_func = gpio_direction_output(gpio_num, 0);
	if (status_pin_func < 0) {
		printk("ERROR in case of assignment of level of an GPIO %d\n",
				gpio_num);
		return (status_pin_func);
	}

	gpio_num = 141; //конфигурирование FS1
	status_pin_func = gpio_request(gpio_num, "FS1");
	if (status_pin_func < 0) {
		printk("ERROR can not open GPIO %d\n", gpio_num);
		return (status_pin_func);
	}
	if (rate <= 2 /*четвёртый скоростной режим АЦП*/)
		status_pin_func = gpio_direction_output(gpio_num, 1);
	else
		status_pin_func = gpio_direction_output(gpio_num, 0);
	if (status_pin_func < 0) {
		printk("ERROR in case of assignment of level of an GPIO %d\n",
				gpio_num);
		return (status_pin_func);
	}

	gpio_num = 140; //конфигурирование FS2
	status_pin_func = gpio_request(gpio_num, "FS2");
	if (status_pin_func < 0) {
		printk("ERROR can not open GPIO %d\n", gpio_num);
		return (status_pin_func);
	}
	status_pin_func = gpio_direction_output(gpio_num, 0);
	if (status_pin_func < 0) {
		printk("ERROR in case of assignment of level of an GPIO %d\n",
				gpio_num);
		return (status_pin_func);
	}

	gpio_num = 139; //конфигурирование S/M
	status_pin_func = gpio_request(gpio_num, "S/M");
	if (status_pin_func < 0) {
		printk("ERROR can not open GPIO %d\n", gpio_num);
		return (status_pin_func);
	}
	status_pin_func = gpio_direction_output(gpio_num, 1);
	if (status_pin_func < 0) {
		printk("ERROR in case of assignment of level of an GPIO %d\n",
				gpio_num);
		return (status_pin_func);
	}

	gpio_num = 138; //конфигурирование FMT0
	status_pin_func = gpio_request(gpio_num, "FMT0");
	if (status_pin_func < 0) {
		printk("ERROR can not open GPIO %d\n", gpio_num);
		return (status_pin_func);
	}
	status_pin_func = gpio_direction_output(gpio_num, 0);
	if (status_pin_func < 0) {
		printk("ERROR in case of assignment of level of an GPIO %d\n",
				gpio_num);
		return (status_pin_func);
	}

	gpio_num = 137; //конфигурирование FMT1
	status_pin_func = gpio_request(gpio_num, "FMT1");
	if (status_pin_func < 0) {
		printk("ERROR can not open GPIO %d\n", gpio_num);
		return (status_pin_func);
	}
	status_pin_func = gpio_direction_output(gpio_num, 1);
	if (status_pin_func < 0) {
		printk("ERROR in case of assignment of level of an GPIO %d\n",
				gpio_num);
		return (status_pin_func);
	}

	gpio_num = 136; //конфигурирование FMT2
	status_pin_func = gpio_request(gpio_num, "FMT2");
	if (status_pin_func < 0) {
		printk("ERROR can not open GPIO %d\n", gpio_num);
		return (status_pin_func);
	}
	status_pin_func = gpio_direction_output(gpio_num, 0);
	if (status_pin_func < 0) {
		printk("ERROR in case of assignment of level of an GPIO %d\n",
				gpio_num);
		return (status_pin_func);
	}

	gpio_num = 100; //конфигурирование CLIP1
	status_pin_func = gpio_request(gpio_num, "CLIP1");
	if (status_pin_func < 0) {
		printk("ERROR can not open GPIO %d\n", gpio_num);
		return (status_pin_func);
	}
	status_pin_func = gpio_direction_input(gpio_num);
	if (status_pin_func < 0) {
		printk("ERROR in case of assignment of level of an GPIO %d\n",
				gpio_num);
		return (status_pin_func);
	}

	gpio_num = 99; //конфигурирование CLIP2
	status_pin_func = gpio_request(gpio_num, "CLIP2");
	if (status_pin_func < 0) {
		printk("ERROR can not open GPIO %d\n", gpio_num);
		return (status_pin_func);
	}
	status_pin_func = gpio_direction_input(gpio_num);
	if (status_pin_func < 0) {
		printk("ERROR in case of assignment of level of an GPIO %d\n",
				gpio_num);
		return (status_pin_func);
	}

	gpio_num = 98; //конфигурирование CLIP3
	status_pin_func = gpio_request(gpio_num, "CLIP3");
	if (status_pin_func < 0) {
		printk("ERROR can not open GPIO %d\n", gpio_num);
		return (status_pin_func);
	}
	status_pin_func = gpio_direction_input(gpio_num);
	if (status_pin_func < 0) {
		printk("ERROR in case of assignment of level of an GPIO %d\n",
				gpio_num);
		return (status_pin_func);
	}

	gpio_num = 97; //конфигурирование CLIP4
	status_pin_func = gpio_request(gpio_num, "CLIP4");
	if (status_pin_func < 0) {
		printk("ERROR can not open GPIO %d\n", gpio_num);
		return (status_pin_func);
	}
	status_pin_func = gpio_direction_input(gpio_num);
	if (status_pin_func < 0) {
		printk("ERROR in case of assignment of level of an GPIO %d\n",
				gpio_num);
		return (status_pin_func);
	}

	gpio_num = 96; //конфигурирование HPFD
	status_pin_func = gpio_request(gpio_num, "HPFD");
	if (status_pin_func < 0) {
		printk("ERROR can not open GPIO %d\n", gpio_num);
		return (status_pin_func);
	}
	status_pin_func = gpio_direction_output(gpio_num, 0);
	if (status_pin_func < 0) {
		printk("ERROR in case of assignment of level of an GPIO %d\n",
				gpio_num);
		return (status_pin_func);
	}

	gpio_num = 120; //конфигурирование SUB
	status_pin_func = gpio_request(gpio_num, "SUB");
	if (status_pin_func < 0) {
		printk("ERROR can not open GPIO %d\n", gpio_num);
		return (status_pin_func);
	}
	status_pin_func = gpio_direction_output(gpio_num, 0);
	if (status_pin_func < 0) {
		printk("ERROR in case of assignment of level of an GPIO %d\n",
				gpio_num);
		return (status_pin_func);
	}

	//выделение памяти под регистры интерфейса McAsp
	/*	if (!request_mem_region(0x01D00000L, 0x3000L, device_name)) {
	 printk(KERN_ALERT "The request_mem_region function failed\n");
	 return (1);
	 } */
	//включение домена PSC для McAsp
	if (clk_enable(davinci_soc_info.cpu_clks[32].clk)) {
		printk(KERN_ERR "clk_enable() FAIL\n");
		return -EINVAL;
	}
//	printk(KERN_INFO "including domain %s", davinci_soc_info.cpu_clks[32].clk->name);
	base = ioremap(0x01D00000L, 0x3000L);
	//printk(KERN_INFO "address base = 0x%x\n", (int)base);

	//ИНИЦИАЛИЗАЦИЯ MCASP
	mcasp_set_bits((char*) base + DAVINCI_MCASP_PWREMUMGT_REG, MCASP_SOFT); //включение McAsp
	/*printk(KERN_INFO "For mcasp_set_bits()value DAVINCI_MCASP_PWREMUMGT_REG = 0x%x\n", mcasp_get_reg((char*)base + DAVINCI_MCASP_PWREMUMGT_REG));*/
	iowrite32(MCASP_SOFT, (char*)base + DAVINCI_MCASP_PWREMUMGT_REG);
	/*printk(KERN_INFO "For iowrite32()value DAVINCI_MCASP_PWREMUMGT_REG = 0x%x\n", ioread32((char*)base + DAVINCI_MCASP_PWREMUMGT_REG));  */
	mcasp_set_reg(base + DAVINCI_MCASP_GBLCTL_REG, 0); //глобальный сброс системы McAsp
	{
		int i = 0;
		for (i = 0; i < 1000; i++) {
			if (mcasp_get_reg((char*) base + DAVINCI_MCASP_GBLCTL_REG) == 0)
				break;
		}
		if (i == 1000
				&& (mcasp_get_reg((char*) base + DAVINCI_MCASP_GBLCTL_REG))
						!= 0) {
			printk(KERN_ALERT "The request_mem_region function failed\n");
			return (1);
		}
	}

	mcasp_mod_bits((char*) base + DAVINCI_MCASP_RFIFOCTL, 2, NUMDMA_MASK); //количество слов на одну передачу
	mcasp_mod_bits((char*) base + DAVINCI_MCASP_RFIFOCTL, SIZE_FIFO_READ << 8,
			NUMEVT_MASK); //количество слов на одно DMA событие
	mcasp_set_bits((char*) base + DAVINCI_MCASP_RFIFOCTL, FIFO_ENABLE); //позволение FIFO буфера чтения

	mcasp_set_reg((char*) base + DAVINCI_MCASP_RXMASK_REG, 0x00FFFFFF); //замаскированы биты 24-31
	mcasp_mod_bits((char*) base + DAVINCI_MCASP_RXFMT_REG, RXSSZ(0xF),
			RXSSZ(0xF)); //размер слота равен 32 бита
	mcasp_mod_bits((char*) base + DAVINCI_MCASP_RXFMT_REG, RXPBIT(0x17),
			RXPBIT(0x1F)); //замаскированные биты замещаются
	mcasp_mod_bits((char*) base + DAVINCI_MCASP_RXFMT_REG, RXPAD(0x2),
			RXPAD(0x3)); //битом 23
	mcasp_set_bits((char*) base + DAVINCI_MCASP_RXFMT_REG, RXORD); //MSB первый
	mcasp_clr_bits((char*) base + DAVINCI_MCASP_RXFMT_REG, RXSEL); //данные считываются с DMA порта

	mcasp_clr_bits((char*) base + DAVINCI_MCASP_RXFMCTL_REG, FSRPOL); //возрастающий фронт
	mcasp_set_bits((char*) base + DAVINCI_MCASP_RXFMCTL_REG, AFSRE); //внутренний источник
	mcasp_set_bits((char*) base + DAVINCI_MCASP_RXFMCTL_REG, FSRDUR); //длина активного периода - слово
	mcasp_mod_bits((char*) base + DAVINCI_MCASP_RXFMCTL_REG, FSRMOD(0x2),
			FSRMOD(0x1FF)); //2 слота во фрейме

	mcasp_set_bits((char*) base + DAVINCI_MCASP_ACLKXCTL_REG, TX_ASYNC); //асинхронный режим тактирования передатчика и приёмника
	mcasp_mod_bits((char*) base + DAVINCI_MCASP_ACLKRCTL_REG, ACLKRDIV(rate),
			ACLKRDIV(0x1F)); //частота 12 МГц
	mcasp_set_bits((char*) base + DAVINCI_MCASP_ACLKRCTL_REG, ACLKRE); //сигнал тактирования битов данных генерируется внутренне
	mcasp_set_bits((char*) base + DAVINCI_MCASP_ACLKRCTL_REG, ACLKRPOL); //стробирование осуществляется по возрастающему фронту

	mcasp_mod_bits((char*) base + DAVINCI_MCASP_AHCLKRCTL_REG, AHCLKRDIV(0),
			AHCLKRDIV(0xFFF)); //частота 24 МГц
	mcasp_clr_bits((char*) base + DAVINCI_MCASP_AHCLKRCTL_REG, AHCLKRPOL); //однофазен сигналу стробирования данных
	mcasp_set_bits((char*) base + DAVINCI_MCASP_AHCLKRCTL_REG, AHCLKRE); //сигнал системного тактирования - внутренний

	mcasp_set_reg((char*) base + DAVINCI_MCASP_RXTDM_REG, 0x3); //активны 0 и 1 слот

	mcasp_mod_bits((char*) base + DAVINCI_MCASP_XRSRCTL8_REG, MODE(0x2),
			MODE(0x3)); //сериализатор конфигурируется как приёмник
	mcasp_mod_bits((char*) base + DAVINCI_MCASP_XRSRCTL10_REG, MODE(0x2),
			MODE(0x3)); //сериализатор конфигурируется как приёмник

	mcasp_set_reg((char*) base + DAVINCI_MCASP_PFUNC_REG, 0xFFFFFFFF); //конфигурирование всех выводов как GPIO
	mcasp_clr_bits((char*) base + DAVINCI_MCASP_PFUNC_REG, AXR(8)); //конфигурирование вывода AXR8 в режиме McAsp
	mcasp_clr_bits((char*) base + DAVINCI_MCASP_PFUNC_REG, AXR(10)); //конфигурирование вывода AXR10 в режиме McAsp
	mcasp_clr_bits((char*) base + DAVINCI_MCASP_PFUNC_REG, ACLKR); //конфигурирование вывода ACLKR в режиме McAsp
	mcasp_clr_bits((char*) base + DAVINCI_MCASP_PFUNC_REG, AHCLKR); //конфигурирование вывода AHCLKR в режиме McAsp
	mcasp_clr_bits((char*) base + DAVINCI_MCASP_PFUNC_REG, AFSR); //конфигурирование вывода AHCLKR в режиме McAsp

	mcasp_set_bits((char*) base + DAVINCI_MCASP_PDIR_REG, ACLKR); //конфигурирование контакта ACLKR в режиме вывода
	mcasp_set_bits((char*) base + DAVINCI_MCASP_PDIR_REG, AHCLKR); //конфигурирование контакта AHCLKR в режиме вывода
	mcasp_set_bits((char*) base + DAVINCI_MCASP_PDIR_REG, AFSR); //конфигурирование контакта AHCLKR в режиме вывода

	//printk(KERN_INFO "McAsp initialized\n");

	//ИНИЦИАЛИЗАЦИЯ EDMA
	buf[0].compl_fill = false;
	buf[0].loc = 0;
	buf[0].size = BUF_SIZE;
	INIT_LIST_HEAD(&buf[0].gain);
	buf[0].sync_m.count_of_first = 0;
	buf[0].sync_m.number_of_first = NOT_SYNC;
	buf[0].virt_loc = dma_alloc_coherent(NULL, buf[0].size, &buf[0].bus_loc,
			GFP_KERNEL | GFP_DMA ); //размещение и отображение для буфера buf[0]
	if (!buf[0].virt_loc) {
		printk(KERN_ALERT "dma_alloc_coherent failed for buf[0]\n");
		return (-ENOMEM);
	} else
//		printk(
//				KERN_INFO "The function dma_alloc_coherent to execute buffer buf[0] successfully\n");
		buf[1].compl_fill = false;
	buf[1].loc = 0;
	buf[1].size = BUF_SIZE;
	INIT_LIST_HEAD(&buf[1].gain);
	buf[1].sync_m.count_of_first = 0;
	buf[1].sync_m.number_of_first = NOT_SYNC;
	buf[1].virt_loc = dma_alloc_coherent(NULL, buf[1].size, &buf[1].bus_loc,
			GFP_KERNEL | GFP_DMA ); //размещение и отображение для буфера buf[1]
	if (!buf[1].virt_loc) {
		printk(KERN_ALERT "dma_alloc_coherent failed for buf[1]\n");
		dma_free_coherent(NULL, buf[0].size, buf[0].virt_loc, buf[0].bus_loc);
		return (-ENOMEM);
	} else
//		printk(
//				KERN_INFO "The function dma_alloc_coherent to execute buffer buf[1] successfully\n");
		result = edma_alloc_channel(num_channel, callback_dma, NULL,
				EVENTQ_DEFAULT); //размещение DMA канала MCASP0
	if (result < 0) {
		printk(KERN_ALERT "edma_alloc_channel failed\n");
		dma_free_coherent(NULL, buf[0].size, buf[0].virt_loc, buf[0].bus_loc);
		dma_free_coherent(NULL, buf[1].size, buf[1].virt_loc, buf[1].bus_loc);
		return (result);
	} else
//		printk(KERN_INFO "The function edma_alloc_channel successfully\n");
		//заполнение PARAMSET для buf[0]
		edma_set_src(num_channel, (unsigned long) (0x01D02000), INCR, W32BIT); //установка адреса источника и его адресного режима
	edma_set_dest(num_channel, (unsigned long) (buf[0].bus_loc), INCR, W32BIT); //установка адреса приёмника и его адресного режима
	edma_set_src_index(num_channel, 0, 0); //установка параметров индексации адреса источника
	edma_set_dest_index(num_channel, dst_bindex, dst_cindex); //установка параметров индексации адреса приёмника
	edma_set_transfer_params(num_channel, acnt, bcnt, ccnt, bcnt, ABSYNC); //установка счётчиков и режима синхронизации
	edma_read_slot(num_channel, &param_set_buf[0]);
	__clear_bit(ITCCHEN_SHIFT, (unsigned long*) &param_set_buf[0].opt); //отключение функции интрацепочной передачи
	__clear_bit(TCCHEN_SHIFT, (unsigned long*) &param_set_buf[0].opt); //отключение функции цепочной передачи
	__clear_bit(ITCINTEN_SHIFT, (unsigned long*) &param_set_buf[0].opt); //запрещение промежуточного прерывания
	__set_bit(TCINTEN_SHIFT, (unsigned long*) &param_set_buf[0].opt); //разрешение прерывания по окончанию передачи
	param_set_buf[0].opt |= EDMA_TCC(EDMA_CHAN_SLOT(num_channel)); //идентифицирование кода завершения передачи
	__clear_bit(TCCMODE_SHIFT, (unsigned long*) &param_set_buf[0].opt); //установка режима нормального завершения
	__clear_bit(STATIC_SHIFT, (unsigned long*) &param_set_buf[0].opt); //PARAMSET не статичный
	//установка PARAMSET для канала
	edma_write_slot(num_channel, &param_set_buf[0]);
	//заполнение неотображённого слота для buf[0]
	result = edma_alloc_slot(0, EDMA_SLOT_ANY);
	if (result < 0) {
		printk(KERN_ALERT "edma_alloc_slot failed for buf[0]\n");
		edma_free_channel(num_channel);
		dma_free_coherent(NULL, buf[0].size, buf[0].virt_loc, buf[0].bus_loc);
		dma_free_coherent(NULL, buf[1].size, buf[1].virt_loc, buf[1].bus_loc);
		return (result);
	} else
//		printk(
//				KERN_INFO "The function edma_alloc_slot to execute buffer buf[0] successfully\n");
		buf[0].num_slot = result;
	edma_write_slot(buf[0].num_slot, &param_set_buf[0]);
	//заполнение неотображённого слота для buf[1]
	result = edma_alloc_slot(0, EDMA_SLOT_ANY);
	if (result < 0) {
		printk(KERN_ALERT "edma_alloc_slot failed for buffer2\n");
		edma_free_channel(num_channel);
		edma_free_slot(buf[0].num_slot);
		dma_free_coherent(NULL, buf[0].size, buf[0].virt_loc, buf[0].bus_loc);
		dma_free_coherent(NULL, buf[1].size, buf[1].virt_loc, buf[1].bus_loc);
		return (result);
	} else
//		printk(
//				KERN_INFO "The function edma_alloc_slot to execute buffer buf[1] successfully\n");
		buf[1].num_slot = result;
	param_set_buf[1] = param_set_buf[0];
	edma_write_slot(buf[1].num_slot, &param_set_buf[1]);
	edma_set_dest(buf[1].num_slot, (unsigned long) (buf[1].bus_loc), INCR,
			W32BIT); //установка адреса приёмника и его адресного режима
	edma_read_slot(buf[1].num_slot, &param_set_buf[1]);
	//связывание канала и слотов между собой - организация пинг-понга
	edma_link(num_channel, buf[1].num_slot);
	edma_link(buf[1].num_slot, buf[0].num_slot);
	edma_link(buf[0].num_slot, buf[1].num_slot);
	//запуск EDMA
	result = edma_start(num_channel);
	if (result != 0) {
		printk(KERN_ALERT "edma_start failed\n");
		edma_free_channel(num_channel);
		edma_free_slot(buf[0].num_slot);
		edma_free_slot(buf[1].num_slot);
		dma_free_coherent(NULL, buf[0].size, buf[0].virt_loc, buf[0].bus_loc);
		dma_free_coherent(NULL, buf[1].size, buf[1].virt_loc, buf[1].bus_loc);
		return (result);
	} else
//		printk(KERN_INFO "The function edma_start successfully\n");
		//ИНИЦИАЛИЗАЦИЯ ПЕРЕМЕННОЙ СОСТОЯНИЯ ДРАЙВЕРА
		pcm4204_status.mode = STOP_MODE;
	pcm4204_status.num_buf_write = 0;
	pcm4204_status.num_buf_read = 1;
	pcm4204_status.count = -buf[pcm4204_status.num_buf_read].size / SAMPL_SIZE;
	pcm4204_status.error.error = 0;
	pcm4204_status.users = 0;
	pcm4204_status.gain = 1;
	sema_init(&pcm4204_status.mutex_lock, 1);
//	init_MUTEX(&pcm4204_status.mutex_lock);
	//получение идентификатора для устройства
	if (alloc_chrdev_region(&dev_pcm4204, 0, 1, device_name)) {
		printk(KERN_ALERT "The request_mem_region function failed\n");
		edma_free_channel(num_channel);
		edma_free_slot(buf[0].num_slot);
		edma_free_slot(buf[1].num_slot);
		dma_free_coherent(NULL, buf[0].size, buf[0].virt_loc, buf[0].bus_loc);
		dma_free_coherent(NULL, buf[1].size, buf[1].virt_loc, buf[1].bus_loc);
		return (1);
	} else
//		printk(KERN_INFO "The function alloc_chrdev_region successfully\n");
		//регистрация символьного устройства
		cdev_init(&cdev_pcm4204, &pcm4204_fops);
	cdev_pcm4204.owner = THIS_MODULE;
	if (cdev_add(&cdev_pcm4204, dev_pcm4204, 1)) {
		printk(KERN_ERR "The cdev_add function failed\n");
		unregister_chrdev_region(dev_pcm4204, 1);
		edma_free_channel(num_channel);
		edma_free_slot(buf[0].num_slot);
		edma_free_slot(buf[1].num_slot);
		dma_free_coherent(NULL, buf[0].size, buf[0].virt_loc, buf[0].bus_loc);
		dma_free_coherent(NULL, buf[1].size, buf[1].virt_loc, buf[1].bus_loc);
		return (1);
	} else
//		printk(KERN_INFO "The function cdev_add successfully\n");
		//регистрация класса устройств
		devclass = class_create( THIS_MODULE, DEV_CLASS_ADC); //создание класса
	if (IS_ERR(devclass)) {
		printk(KERN_ERR "The class_create function failed\n");
		cdev_del(&cdev_pcm4204);
		unregister_chrdev_region(dev_pcm4204, 1);
		edma_free_channel(num_channel);
		edma_free_slot(buf[0].num_slot);
		edma_free_slot(buf[1].num_slot);
		dma_free_coherent(NULL, buf[0].size, buf[0].virt_loc, buf[0].bus_loc);
		dma_free_coherent(NULL, buf[1].size, buf[1].virt_loc, buf[1].bus_loc);
		return (PTR_ERR(devclass));
	} else
//		printk(KERN_INFO "The function class_create successfully\n");
		//регистрация устройства
		dev = device_create(devclass, NULL, dev_pcm4204, NULL, ADC_NAME); //создание устройства
	result = IS_ERR(dev) ? PTR_ERR(dev) : 0;
	if (result != 0) {
		printk(KERN_ERR "The device_create function failed\n");
		class_destroy(devclass);
		cdev_del(&cdev_pcm4204);
		unregister_chrdev_region(dev_pcm4204, 1);
		edma_free_channel(num_channel);
		edma_free_slot(buf[0].num_slot);
		edma_free_slot(buf[1].num_slot);
		dma_free_coherent(NULL, buf[0].size, buf[0].virt_loc, buf[0].bus_loc);
		dma_free_coherent(NULL, buf[1].size, buf[1].virt_loc, buf[1].bus_loc);
		return (result);
	} else
		//printk(KERN_INFO "The device_create function succesfull\n");
		//printk(KERN_INFO "Value registers McAspi after driver initialization\n");
		//print_reg_mcasp();
		return (0);
}

module_init(pcm4204_init);

static void __exit pcm4204_exit(void) {
	int i = 0;
	struct list_head* p, *next; //указатель на его структуру list_head, а также на следующий элемент в списке
	struct gain_MAD* change_gain = NULL; // элемент в списке gain текущего считываемого буфера
	//остановка АЦП
	stop_adc();
	//освобождение памяти, выделенной под списки моментов изменения коэффициента усиления буферов
	for (; i < 2; i++) {
		list_for_each_safe(p, next, &buf[i].gain)
		{
			change_gain = list_entry(p, struct gain_MAD, list);
			list_del(&change_gain->list); //удаление элемента из списка
			kfree(change_gain); //освобождение элемента
		}
	}
	//освобождение ресурсов DMA
	edma_free_channel(num_channel);
	edma_free_slot(buf[0].num_slot);
	edma_free_slot(buf[1].num_slot);
	dma_free_coherent(NULL, buf[0].size, buf[0].virt_loc, buf[0].bus_loc);
	dma_free_coherent(NULL, buf[1].size, buf[1].virt_loc, buf[1].bus_loc);
	//освобождение ресурсов модели устройств драйвера
	class_destroy(devclass);
	cdev_del(&cdev_pcm4204);
	unregister_chrdev_region(dev_pcm4204, 1);
	printk(KERN_ERR "Driver pcm4204 removed\n");
	print_reg_mcasp();
}

module_exit(pcm4204_exit);
