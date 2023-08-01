// SPDX-License-Identifier: GPL-2.0-only
/*
 * SPI-Engine SPI controller driver
 * Copyright 2015 Analog Devices Inc.
 * Author: Jorge Marques <jorge.marques@analog.com>
 */

#include <linux/bitops.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/i3c/master.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/of_device.h>

#define CURRENT_VERSION			0x12345678
#define MAX_DEVS			16
#define PARAM_MAX_DEVS(x)		((x) & GENMASK(3,0))

#define REG_VERSION			0x000
#define REG_SCRATCH			0x008
#define REG_PARAMETERS			0x00c
#define REG_ENABLE			0x040
#define REG_IRQ_MASK			0x080
#define REG_IRQ_PENDING			0x084
#define REG_IRQ_SOURCE			0x088
#define REG_CMD_FIFO			0x0d4
#define REG_CMDR_FIFO			0x0d8
#define REG_SDO_FIFO			0x0dc
#define REG_SDI_FIFO			0x0e0
#define REG_IBI_FIFO			0x0e4
#define REG_FIFO_STATUS			0x0e8
#define REG_RESET			0x040
#define REG_IBI_CONFIG			0x140
#define REG_DEVS_CTRL			0x280

#define REG_DEV_CHAR_0(d)		((0x60 + d) << 2)
#define REG_DEV_CHAR_1(d)		((0x70 + d) << 2)
#define REG_DEV_CHAR_2(d)		((0x80 + d) << 2)

#define DEV_CHAR_0_GET_DEV_ADDR(x)	((x >> 1) & GENMASK(6, 0))
#define DEV_CHAR_0_IS_I3C		BIT(8)

#define CMD0_FIFO_BCH			BIT(29)
#define CMD0_FIFO_PRIV_XMIT_MODE(m)	((m) << 27)
#define CMD0_FIFO_RSBC			BIT(25)
#define CMD0_FIFO_IS_10B		BIT(24)
#define CMD0_FIFO_PL_LEN(l)		((l) << 12)
#define CMD0_FIFO_PL_LEN_MAX		4095
#define CMD0_FIFO_DEV_ADDR(a)		((a) << 1)
#define CMD0_FIFO_RNW			BIT(0)

#define XMIT_BURST_WITHOUT_SUBADDR	3

#define CMD0_FIFO_IS_CCC		BIT(30)
#define CMD1_FIFO_CCC(id)		(id)

#define CMDR_NO_ERROR			0
#define CMDR_CE0_ERROR			1
#define CMDR_CE1_ERROR			2
#define CMDR_CE2_ERROR			3
#define CMDR_NACK_RESP			9
#define CMDR_ERROR(x)			(((x) & GENMASK(27, 24)) >> 24)
#define CMDR_XFER_BYTES(x)		(((x) & GENMASK(19, 8)) >> 8)
#define CMDR_SYNC(x)			((x) & GENMASK(7, 0))

#define IRQ_PENDING_CMDR_PENDING	BIT(5)
#define IRQ_PENDING_IBI_PENDING		BIT(6)

#define FIFO_STATUS_CMDR_EMPTY		BIT(0)
#define FIFO_STATUS_IBI_EMPTY		BIT(1)
#define FIFO_STATUS_SDI_EMPTY   	BIT(2)

#define DEVS_CTRL_DEV_CLR_SHIFT		16
#define DEVS_CTRL_DEV_CLR_ALL		GENMASK(31, 16)
#define DEVS_CTRL_DEV_CLR(dev)		BIT(16 + (dev))
#define DEVS_CTRL_DEV_ACTIVE(dev)	BIT(dev)
#define DEVS_CTRL_DEVS_ACTIVE_MASK	GENMASK(15, 0)

struct adi_i3c_cmd {
	u32 cmd0;
	u32 cmd1;
	u32 tx_len;
	const void *tx_buf;
	u32 rx_len;
	void *rx_buf;
	u32 error;
};

struct adi_i3c_xfer {
	struct list_head node;
	struct completion comp;
	int ret;
	unsigned int ncmds;
	struct adi_i3c_cmd cmds[];
};

struct adi_i3c_controller {
	struct i3c_master_controller base;
	u32 free_rr_slots;
	unsigned int maxdevs;
	struct {
		unsigned int num_slots;
		struct i3c_dev_desc **slots;
		spinlock_t lock;
	} ibi;
	struct {
		struct list_head list;
		struct adi_i3c_xfer *cur;
		spinlock_t lock;
	} xferqueue;
	void __iomem *regs;
	struct clk *clk;
	unsigned long i3c_scl_lim;
};

static inline struct adi_i3c_controller *
to_adi_i3c_controller(struct i3c_master_controller *master)
{
	return container_of(master, struct adi_i3c_controller, base);
}

static void adi_i3c_controller_wr_to_tx_fifo(struct adi_i3c_controller *controller,
					  const u8 *bytes, int nbytes)
{
	writesl(controller->regs + REG_SDO_FIFO, bytes, nbytes / 4);
	if (nbytes & 3) {
		u32 tmp = 0;
		memcpy(&tmp, bytes + (nbytes & ~3), nbytes & 3);
		writesl(controller->regs + REG_SDO_FIFO, &tmp, 1);
	}
}

static void adi_i3c_controller_rd_from_rx_fifo(struct adi_i3c_controller *controller,
					    u8 *bytes, int nbytes)
{
	readsl(controller->regs + REG_SDI_FIFO, bytes, nbytes / 4);
	if (nbytes & 3) {
		u32 tmp;
		readsl(controller->regs + REG_SDI_FIFO, &tmp, 1);
		memcpy(bytes + (nbytes & ~3), &tmp, nbytes & 3);
	}
}

static bool adi_i3c_controller_supports_ccc_cmd(struct i3c_master_controller *m,
					     const struct i3c_ccc_cmd *cmd)
{
	if (cmd->ndests > 1)
		return false;

	switch (cmd->id) {
	case I3C_CCC_ENEC(true):
	case I3C_CCC_ENEC(false):
	case I3C_CCC_DISEC(true):
	case I3C_CCC_DISEC(false):
	case I3C_CCC_RSTDAA(true):
	case I3C_CCC_RSTDAA(false):
	case I3C_CCC_ENTDAA:
		return true;
	default:
		break;
	}

	return false;
}

static int adi_i3c_controller_disable(struct adi_i3c_controller *master)
{
	return 0;
}

static struct adi_i3c_xfer *
adi_i3c_controller_alloc_xfer(struct adi_i3c_controller *controller, unsigned int ncmds)
{
	struct adi_i3c_xfer *xfer;

	xfer = kzalloc(struct_size(xfer, cmds, ncmds), GFP_KERNEL);
	if (!xfer)
		return NULL;

	INIT_LIST_HEAD(&xfer->node);
	xfer->ncmds = ncmds;
	xfer->ret = -ETIMEDOUT;

	return xfer;
}

static void adi_i3c_controller_start_xfer_locked(struct adi_i3c_controller *controller)
{
	struct adi_i3c_xfer *xfer = controller->xferqueue.cur;
	unsigned int i;

	if (!xfer)
		return;

	for (i = 0; i < xfer->ncmds; i++) {
		struct adi_i3c_cmd *cmd = &xfer->cmds[i];

		adi_i3c_controller_wr_to_tx_fifo(controller, cmd->tx_buf,
					      cmd->tx_len);
	}

	for (i = 0; i < xfer->ncmds; i++) {
		struct adi_i3c_cmd *cmd = &xfer->cmds[i];

		writel(cmd->cmd0, controller->regs + REG_CMD_FIFO);
		writel(cmd->cmd1, controller->regs + REG_CMD_FIFO);
	}
}

static void adi_i3c_controller_end_xfer_locked(struct adi_i3c_controller *controller,
						u32 pending)
{
	struct adi_i3c_xfer *xfer = controller->xferqueue.cur;
	int i, ret = 0;
	u32 status0;

	if (!(pending & IRQ_PENDING_CMDR_PENDING))
		return;

	if (!xfer)
		return;

	for (status0 = readl(controller->regs + REG_FIFO_STATUS);
	     !(status0 & FIFO_STATUS_CMDR_EMPTY);
	     status0 = readl(controller->regs + REG_FIFO_STATUS)) {
		struct adi_i3c_cmd *cmd;
		u32 cmdr, rx_len, id;

		cmdr = readl(controller->regs + REG_CMDR_FIFO);
		id = CMDR_SYNC(cmdr);

		cmd = &xfer->cmds[CMDR_SYNC(cmdr)];
		rx_len = min_t(u32, CMDR_XFER_BYTES(cmdr), cmd->rx_len);
		adi_i3c_controller_rd_from_rx_fifo(controller, cmd->rx_buf, rx_len);
		cmd->error = CMDR_ERROR(cmdr);
	}

	writel(IRQ_PENDING_CMDR_PENDING, controller->regs + REG_IRQ_PENDING);

	for (i = 0; i < xfer->ncmds; i++) {
		switch (xfer->cmds[i].error) {
		case CMDR_NO_ERROR:
			break;

		case CMDR_CE0_ERROR:
		case CMDR_CE1_ERROR:
		case CMDR_CE2_ERROR:
		case CMDR_NACK_RESP:
			ret = -EIO;
			break;

		default:
			ret = -EINVAL;
			break;
		}
	}

	xfer->ret = ret;
	complete(&xfer->comp);

	xfer = list_first_entry_or_null(&controller->xferqueue.list,
					struct adi_i3c_xfer, node);
	if (xfer)
		list_del_init(&xfer->node);

	controller->xferqueue.cur = xfer;
	adi_i3c_controller_start_xfer_locked(controller);
}

static void adi_i3c_controller_queue_xfer(struct adi_i3c_controller *controller,
				       struct adi_i3c_xfer *xfer)
{
	unsigned long flags;

	init_completion(&xfer->comp);
	spin_lock_irqsave(&controller->xferqueue.lock, flags);
	if (controller->xferqueue.cur) {
		list_add_tail(&xfer->node, &controller->xferqueue.list);
	} else {
		controller->xferqueue.cur = xfer;
		adi_i3c_controller_start_xfer_locked(controller);
	}
	spin_unlock_irqrestore(&controller->xferqueue.lock, flags);
}

static void adi_i3c_controller_unqueue_xfer(struct adi_i3c_controller *controller,
					 struct adi_i3c_xfer *xfer)
{
	unsigned long flags;

	spin_lock_irqsave(&controller->xferqueue.lock, flags);
	if (controller->xferqueue.cur == xfer) {
		controller->xferqueue.cur = NULL;
	} else {
		list_del_init(&xfer->node);
	}

	spin_unlock_irqrestore(&controller->xferqueue.lock, flags);
}

static enum i3c_error_code adi_i3c_cmd_get_err(struct adi_i3c_cmd *cmd)
{
	switch (cmd->error) {
	case CMDR_CE0_ERROR:
		return I3C_ERROR_M0;

	case CMDR_CE1_ERROR:
		return I3C_ERROR_M1;

	case CMDR_CE2_ERROR:
	case CMDR_NACK_RESP:
		return I3C_ERROR_M2;

	default:
		break;
	}

	return I3C_ERROR_UNKNOWN;
}

static int adi_i3c_controller_send_ccc_cmd(struct i3c_master_controller *m,
					struct i3c_ccc_cmd *cmd)
{
	struct adi_i3c_controller *controller = to_adi_i3c_controller(m);
	struct adi_i3c_xfer *xfer;
	struct adi_i3c_cmd *ccmd;
	int ret;

	xfer = adi_i3c_controller_alloc_xfer(controller, 1);
	if (!xfer)
		return -ENOMEM;

	ccmd = xfer->cmds;
	ccmd->cmd1 = CMD1_FIFO_CCC(cmd->id);
	ccmd->cmd0 = CMD0_FIFO_IS_CCC |
		     CMD0_FIFO_PL_LEN(cmd->dests[0].payload.len);

	if (cmd->id & I3C_CCC_DIRECT)
		ccmd->cmd0 |= CMD0_FIFO_DEV_ADDR(cmd->dests[0].addr);

	if (cmd->rnw) {
		ccmd->cmd0 |= CMD0_FIFO_RNW;
		ccmd->rx_buf = cmd->dests[0].payload.data;
		ccmd->rx_len = cmd->dests[0].payload.len;
	} else {
		ccmd->tx_buf = cmd->dests[0].payload.data;
		ccmd->tx_len = cmd->dests[0].payload.len;
	}

	adi_i3c_controller_queue_xfer(controller, xfer);
	if (!wait_for_completion_timeout(&xfer->comp, msecs_to_jiffies(1000)))
		adi_i3c_controller_unqueue_xfer(controller, xfer);

	ret = xfer->ret;
	cmd->err = adi_i3c_cmd_get_err(&xfer->cmds[0]);
	kfree(xfer);

	return 0;
}

static int adi_i3c_controller_priv_xfers(struct i3c_dev_desc *dev,
				      struct i3c_priv_xfer *xfers,
				      int nxfers)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct adi_i3c_controller *controller = to_adi_i3c_controller(m);
	int txslots = 0, rxslots = 0, i, ret;
	struct adi_i3c_xfer *xfer;

	for (i = 0; i < nxfers; i++) {
		if (xfers[i].len > CMD0_FIFO_PL_LEN_MAX)
			return -ENOTSUPP;
	}

	if (!nxfers)
		return 0;

	/*
	 * First make sure that all transactions (block of transfers separated
	 * by a STOP marker) fit.
	 */
	for (i = 0; i < nxfers; i++) {
		if (xfers[i].rnw)
			rxslots += DIV_ROUND_UP(xfers[i].len, 4);
		else
			txslots += DIV_ROUND_UP(xfers[i].len, 4);
	}

	xfer = adi_i3c_controller_alloc_xfer(controller, nxfers);
	if (!xfer)
		return -ENOMEM;

	for (i = 0; i < nxfers; i++) {
		struct adi_i3c_cmd *ccmd = &xfer->cmds[i];
		u32 pl_len = xfers[i].len;

		ccmd->cmd0 = CMD0_FIFO_DEV_ADDR(dev->info.dyn_addr) |
			CMD0_FIFO_PRIV_XMIT_MODE(XMIT_BURST_WITHOUT_SUBADDR);

		if (xfers[i].rnw) {
			ccmd->cmd0 |= CMD0_FIFO_RNW;
			ccmd->rx_buf = xfers[i].data.in;
			ccmd->rx_len = xfers[i].len;
			pl_len++;
		} else {
			ccmd->tx_buf = xfers[i].data.out;
			ccmd->tx_len = xfers[i].len;
		}

		ccmd->cmd0 |= CMD0_FIFO_PL_LEN(pl_len);

		if (i < nxfers - 1)
			ccmd->cmd0 |= CMD0_FIFO_RSBC;

		if (!i)
			ccmd->cmd0 |= CMD0_FIFO_BCH;
	}

	adi_i3c_controller_queue_xfer(controller, xfer);
	if (!wait_for_completion_timeout(&xfer->comp,
					 msecs_to_jiffies(1000)))
		adi_i3c_controller_unqueue_xfer(controller, xfer);

	ret = xfer->ret;

	for (i = 0; i < nxfers; i++)
		xfers[i].err = adi_i3c_cmd_get_err(&xfer->cmds[i]);

	kfree(xfer);

	return ret;
}

struct adi_i3c_i2c_dev_data {
	u16 id;
	s16 ibi;
	struct i3c_generic_ibi_pool *ibi_pool;
};

static u32 prepare_dev_char_0_address(u32 addr)
{
	u32 ret = (addr << 1) & 0xff;

	/* DEV_CHAR_0[7:1] = addr[6:0] */
	ret |= (addr & GENMASK(6, 0)) << 1;

	/* DEV_CHAR_[0] = ~XOR(addr[6:0]) */
	if (!(hweight8(addr & 0x7f) & 1))
		ret |= 1;

	return ret;
}

static void adi_i3c_controller_upd_i3c_addr(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct adi_i3c_controller *controller = to_adi_i3c_controller(m);
	struct adi_i3c_i2c_dev_data *data = i3c_dev_get_master_data(dev);
	u32 dev_char;

	dev_char = prepare_dev_char_0_address(dev->info.dyn_addr ?
						dev->info.dyn_addr :
						dev->info.static_addr);
	writel(DEV_CHAR_0_IS_I3C | dev_char, controller->regs + REG_DEV_CHAR_0(data->id));
}

static int adi_i3c_controller_get_dev_char_slot(struct adi_i3c_controller *controller,
						u8 dyn_addr)
{
	unsigned long activedevs;
	u32 dev_char;
	int i;

	if (!dyn_addr) {
		if (!controller->free_rr_slots)
			return -ENOSPC;

		return ffs(controller->free_rr_slots) - 1;
	}

	activedevs = readl(controller->regs + REG_DEVS_CTRL) & DEVS_CTRL_DEVS_ACTIVE_MASK;
	activedevs &= ~BIT(0);

	for_each_set_bit(i, &activedevs, controller->maxdevs + 1) {
		dev_char = readl(controller->regs + REG_DEV_CHAR_0(i));
		if (!(dev_char & DEV_CHAR_0_IS_I3C) ||
		    DEV_CHAR_0_GET_DEV_ADDR(dev_char) != dyn_addr)
			continue;

		return i;
	}

	return -EINVAL;
}

static int adi_i3c_controller_reattach_i3c_dev(struct i3c_dev_desc *dev,
					    u8 old_dyn_addr)
{
	adi_i3c_controller_upd_i3c_addr(dev);

	return 0;
}

static int adi_i3c_controller_attach_i3c_dev(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct adi_i3c_controller *controller = to_adi_i3c_controller(m);
	struct adi_i3c_i2c_dev_data *data;
	int slot;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	slot = adi_i3c_controller_get_dev_char_slot(controller, dev->info.dyn_addr);
	if (slot < 0) {
		kfree(data);
		return slot;
	}

	data->ibi = -1;
	data->id = slot;
	i3c_dev_set_master_data(dev, data);
	controller->free_rr_slots &= ~BIT(slot);

	if (!dev->info.dyn_addr) {
		adi_i3c_controller_upd_i3c_addr(dev);
		writel(readl(controller->regs + REG_DEVS_CTRL) |
		       DEVS_CTRL_DEV_ACTIVE(data->id),
		       controller->regs + REG_DEVS_CTRL);
	}

	return 0;
}

static int adi_i3c_controller_attach_i2c_dev(struct i2c_dev_desc *dev)
{
	struct i3c_master_controller *m = i2c_dev_get_master(dev);
	struct adi_i3c_controller *controller = to_adi_i3c_controller(m);
	struct adi_i3c_i2c_dev_data *data;
	int slot;

	slot = adi_i3c_controller_get_dev_char_slot(controller, 0);
	if (slot < 0)
		return slot;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->id = slot;
	controller->free_rr_slots &= ~BIT(slot);
	i2c_dev_set_master_data(dev, data);

	writel(prepare_dev_char_0_address(dev->addr),
	       controller->regs + REG_DEV_CHAR_0(data->id));
	writel(dev->lvr, controller->regs + REG_DEV_CHAR_2(data->id));
	writel(readl(controller->regs + REG_DEVS_CTRL) |
	       DEVS_CTRL_DEV_ACTIVE(data->id),
	       controller->regs + REG_DEVS_CTRL);

	return 0;
}

static void adi_i3c_controller_detach_i2c_dev(struct i2c_dev_desc *dev)
{
	struct i3c_master_controller *m = i2c_dev_get_master(dev);
	struct adi_i3c_controller *controller = to_adi_i3c_controller(m);
	struct adi_i3c_i2c_dev_data *data = i2c_dev_get_master_data(dev);

	writel(readl(controller->regs + REG_DEVS_CTRL) |
	       DEVS_CTRL_DEV_CLR(data->id),
	       controller->regs + REG_DEVS_CTRL);
	controller->free_rr_slots |= BIT(data->id);

	i2c_dev_set_master_data(dev, NULL);
	kfree(data);
}

static void adi_i3c_controller_detach_i3c_dev(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct adi_i3c_controller *controller = to_adi_i3c_controller(m);
	struct adi_i3c_i2c_dev_data *data = i3c_dev_get_master_data(dev);

	writel(readl(controller->regs + REG_DEVS_CTRL) |
	       DEVS_CTRL_DEV_CLR(data->id),
	       controller->regs + REG_DEVS_CTRL);

	i3c_dev_set_master_data(dev, NULL);
	controller->free_rr_slots |= BIT(data->id);
	kfree(data);
}

static void adi_i3c_controller_bus_cleanup(struct i3c_master_controller *m)
{
	struct adi_i3c_controller *master = to_adi_i3c_controller(m);

	adi_i3c_controller_disable(master);
}

static void adi_i3c_controller_dev_char_to_info(struct adi_i3c_controller *controller,
					   unsigned int slot,
					   struct i3c_device_info *info)
{
	u32 charc;

	memset(info, 0, sizeof(*info));
	charc = readl(controller->regs + REG_DEV_CHAR_0(slot));
	info->dyn_addr = DEV_CHAR_0_GET_DEV_ADDR(charc);
	charc = readl(controller->regs + REG_DEV_CHAR_2(slot));
	info->dcr = charc;
	info->bcr = charc >> 8;
	info->pid = charc >> 16;
	info->pid |= (u64)readl(controller->regs + REG_DEV_CHAR_1(slot)) << 16;
}



static int adi_i3c_controller_do_daa(struct i3c_master_controller *m)
{
	struct adi_i3c_controller *controller = to_adi_i3c_controller(m);
	unsigned long olddevs, newdevs;
	int ret, slot;
	u8 addrs[MAX_DEVS] = { };
	u8 last_addr = 0;

	olddevs = readl(controller->regs + REG_DEVS_CTRL)
		& DEVS_CTRL_DEVS_ACTIVE_MASK;
	olddevs |= BIT(0);

	/* Prepare DEV_CHAR slots before launching DAA. */
	for_each_clear_bit(slot, &olddevs, controller->maxdevs + 1) {
		ret = i3c_master_get_free_addr(m, last_addr + 1);
		if (ret < 0)
			return -ENOSPC;

		last_addr = ret;
		addrs[slot] = last_addr;
		writel(prepare_dev_char_0_address(last_addr) | DEV_CHAR_0_IS_I3C,
		       controller->regs + REG_DEV_CHAR_0(slot));
		writel(0, controller->regs + REG_DEV_CHAR_1(slot));
		writel(0, controller->regs + REG_DEV_CHAR_2(slot));
	}

	/* Disable IBIs during DAA */
	writel(0, controller->regs + REG_IBI_CONFIG);

	ret = i3c_master_entdaa_locked(&controller->base);
	if (ret && ret != I3C_ERROR_M2)
		return ret;

	/* Enable IBIs after DAA */
	writel(GENMASK(1,0), controller->regs + REG_IBI_CONFIG);

	newdevs = readl(controller->regs + REG_DEVS_CTRL) & DEVS_CTRL_DEVS_ACTIVE_MASK;
	newdevs &= ~olddevs;

	/*
	 * Clear all retaining registers filled during DAA. We already
	 * have the addressed assigned to them in the addrs array.
	 */
	for_each_set_bit(slot, &newdevs, controller->maxdevs + 1)
		i3c_master_add_i3c_dev_locked(m, addrs[slot]);

	/*
	 * Clear slots that ended up not being used. Can be caused by I3C
	 * device creation failure or when the I3C device was already known
	 * by the system but with a different address (in this case the device
	 * already has a slot and does not need a new one).
	 */
	writel(readl(controller->regs + REG_DEVS_CTRL) |
	       controller->free_rr_slots << DEVS_CTRL_DEV_CLR_SHIFT,
	       controller->regs + REG_DEVS_CTRL);

	i3c_master_defslvs_locked(&controller->base);

	return 0;
}

static int adi_i3c_controller_bus_init(struct i3c_master_controller *m)
{
	struct adi_i3c_controller *controller = to_adi_i3c_controller(m);
	struct i3c_device_info info = { };
	int ret;

	ret = i3c_master_get_free_addr(m, 0);
	if (ret < 0)
		return ret;

	writel(prepare_dev_char_0_address(ret) | DEV_CHAR_0_IS_I3C,
	       controller->regs + REG_DEV_CHAR_0(0));

	adi_i3c_controller_dev_char_to_info(controller, 0, &info);
	ret = i3c_master_set_info(&controller->base, &info);

	if (ret)
		return ret;

	return 0;
}

static void adi_i3c_controller_handle_ibi(struct adi_i3c_controller *controller,
				       u32 ibi)
{
	struct adi_i3c_i2c_dev_data *data;
	bool data_consumed = false;
	struct i3c_ibi_slot *slot;
	u32 id;
	struct i3c_dev_desc *dev;
	u8 *buf;

	id = adi_i3c_controller_get_dev_char_slot(controller, (ibi >> 17) & GENMASK(6,0));
	for (id = 0; id < controller->ibi.num_slots; id++) {
		if (controller->ibi.slots[id]) {
			if (controller->ibi.slots[id]->info.dyn_addr == ((ibi >> 17) & GENMASK(6,0))){
				break;
			}
		}
	}

	if (id >= controller->ibi.num_slots)
		return;

	dev = controller->ibi.slots[id];
	spin_lock(&controller->ibi.lock);

	data = i3c_dev_get_master_data(dev);
	slot = i3c_generic_ibi_get_free_slot(data->ibi_pool);
	if (!slot)
		goto out_unlock;

	buf = slot->data;
	buf[0] = (ibi >> 8) & GENMASK(7, 0);

	slot->len = 1;
	i3c_master_queue_ibi(dev, slot);
	data_consumed = true;

out_unlock:
	spin_unlock(&controller->ibi.lock);

}

static void adi_i3c_controller_demux_ibis(struct adi_i3c_controller *controller)
{
	u32 status0;

	for (status0 = readl(controller->regs + REG_FIFO_STATUS);
	     !(status0 & FIFO_STATUS_IBI_EMPTY);
	     status0 = readl(controller->regs + REG_FIFO_STATUS)) {
		u32 ibi = readl(controller->regs + REG_IBI_FIFO);

		adi_i3c_controller_handle_ibi(controller, ibi);
	}
}


static irqreturn_t adi_i3c_controller_interrupt(int irq, void *data)
{
	struct adi_i3c_controller *controller = data;
	u32 pending;

	pending = readl(controller->regs + REG_IRQ_PENDING);

	spin_lock(&controller->xferqueue.lock);
	adi_i3c_controller_end_xfer_locked(controller, pending);
	spin_unlock(&controller->xferqueue.lock);

	if (pending & IRQ_PENDING_IBI_PENDING)
		adi_i3c_controller_demux_ibis(controller);

	return IRQ_HANDLED;
}


static int adi_i3c_controller_i2c_xfers(struct i2c_dev_desc *dev,
				     const struct i2c_msg *xfers, int nxfers)
{
	struct i3c_master_controller *m = i2c_dev_get_master(dev);
	struct adi_i3c_controller *controller = to_adi_i3c_controller(m);
	unsigned int nrxwords = 0, ntxwords = 0;
	struct adi_i3c_xfer *xfer;
	int i, ret = 0;

	for (i = 0; i < nxfers; i++) {
		if (xfers[i].len > CMD0_FIFO_PL_LEN_MAX)
			return -ENOTSUPP;
		if (xfers[i].flags & I2C_M_TEN){
			return -ENOTSUPP;
		}

		if (xfers[i].flags & I2C_M_RD)
			nrxwords += DIV_ROUND_UP(xfers[i].len, 4);
		else
			ntxwords += DIV_ROUND_UP(xfers[i].len, 4);
	}

	xfer = adi_i3c_controller_alloc_xfer(controller, nxfers);
	if (!xfer)
		return -ENOMEM;

	for (i = 0; i < nxfers; i++) {
		struct adi_i3c_cmd *ccmd = &xfer->cmds[i];

		ccmd->cmd0 = CMD0_FIFO_DEV_ADDR(xfers[i].addr) |
			CMD0_FIFO_PL_LEN(xfers[i].len) |
			CMD0_FIFO_PRIV_XMIT_MODE(XMIT_BURST_WITHOUT_SUBADDR);

		if (xfers[i].flags & I2C_M_TEN)
			ccmd->cmd0 |= CMD0_FIFO_IS_10B;

		if (xfers[i].flags & I2C_M_RD) {
			ccmd->cmd0 |= CMD0_FIFO_RNW;
			ccmd->rx_buf = xfers[i].buf;
			ccmd->rx_len = xfers[i].len;
		} else {
			ccmd->tx_buf = xfers[i].buf;
			ccmd->tx_len = xfers[i].len;
		}
	}

	adi_i3c_controller_queue_xfer(controller, xfer);
	if (!wait_for_completion_timeout(&xfer->comp, msecs_to_jiffies(1000)))
		adi_i3c_controller_unqueue_xfer(controller, xfer);

	ret = xfer->ret;
	kfree(xfer);

	return ret;

}

static int adi_i3c_controller_disable_ibi(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	int ret;

	ret = i3c_master_disec_locked(m, dev->info.dyn_addr,
				      I3C_CCC_EVENT_SIR);

	return ret;
}

static int adi_i3c_controller_enable_ibi(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	int ret;

	ret = i3c_master_enec_locked(m, dev->info.dyn_addr,
				     I3C_CCC_EVENT_SIR);
	return ret;
}

static int adi_i3c_controller_request_ibi(struct i3c_dev_desc *dev,
				       const struct i3c_ibi_setup *req)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct adi_i3c_controller *controller = to_adi_i3c_controller(m);
	struct adi_i3c_i2c_dev_data *data = i3c_dev_get_master_data(dev);
	unsigned long flags;
	unsigned int i;

	data->ibi_pool = i3c_generic_ibi_alloc_pool(dev, req);
	if (IS_ERR(data->ibi_pool))
		return PTR_ERR(data->ibi_pool);

	spin_lock_irqsave(&controller->ibi.lock, flags);
	for (i = 0; i < controller->ibi.num_slots; i++) {
		if (!controller->ibi.slots[i]) {
			data->ibi = i;
			controller->ibi.slots[i] = dev;
			break;
		}
	}
	spin_unlock_irqrestore(&controller->ibi.lock, flags);

	if (i < controller->ibi.num_slots)
		return 0;

	i3c_generic_ibi_free_pool(data->ibi_pool);
	data->ibi_pool = NULL;

	return -ENOSPC;
}

static void adi_i3c_controller_free_ibi(struct i3c_dev_desc *dev)
{
	struct i3c_master_controller *m = i3c_dev_get_master(dev);
	struct adi_i3c_controller *controller = to_adi_i3c_controller(m);
	struct adi_i3c_i2c_dev_data *data = i3c_dev_get_master_data(dev);
	unsigned long flags;

	spin_lock_irqsave(&controller->ibi.lock, flags);
	controller->ibi.slots[data->ibi] = NULL;
	data->ibi = -1;
	spin_unlock_irqrestore(&controller->ibi.lock, flags);

	i3c_generic_ibi_free_pool(data->ibi_pool);

}

static void adi_i3c_controller_recycle_ibi_slot(struct i3c_dev_desc *dev,
					     struct i3c_ibi_slot *slot)
{
	struct adi_i3c_i2c_dev_data *data = i3c_dev_get_master_data(dev);

	i3c_generic_ibi_recycle_slot(data->ibi_pool, slot);
}

static const struct i3c_master_controller_ops adi_i3c_controller_ops = {
	.bus_init = adi_i3c_controller_bus_init,
	.bus_cleanup = adi_i3c_controller_bus_cleanup,
	.attach_i3c_dev = adi_i3c_controller_attach_i3c_dev,
	.reattach_i3c_dev = adi_i3c_controller_reattach_i3c_dev,
	.detach_i3c_dev = adi_i3c_controller_detach_i3c_dev,
	.attach_i2c_dev = adi_i3c_controller_attach_i2c_dev,
	.detach_i2c_dev = adi_i3c_controller_detach_i2c_dev,
	.do_daa = adi_i3c_controller_do_daa,
	.supports_ccc_cmd = adi_i3c_controller_supports_ccc_cmd,
	.send_ccc_cmd = adi_i3c_controller_send_ccc_cmd,
	.priv_xfers = adi_i3c_controller_priv_xfers,
	.i2c_xfers = adi_i3c_controller_i2c_xfers,
	.request_ibi = adi_i3c_controller_request_ibi,
	.enable_ibi = adi_i3c_controller_enable_ibi,
	.disable_ibi = adi_i3c_controller_disable_ibi,
	.free_ibi = adi_i3c_controller_free_ibi,
	.recycle_ibi_slot = adi_i3c_controller_recycle_ibi_slot,
};

static const struct of_device_id adi_i3c_controller_of_match[] = {
	{ .compatible = "adi,i3c-controller" },
	{},
};

static int adi_i3c_controller_probe(struct platform_device *pdev)
{
	struct adi_i3c_controller *controller;
	int ret, irq;

	controller = devm_kzalloc(&pdev->dev, sizeof(*controller), GFP_KERNEL);
	if (!controller)
		return -ENOMEM;

	controller->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(controller->regs))
		return PTR_ERR(controller->regs);

	controller->clk = devm_clk_get(&pdev->dev, "s_axi_aclk");
	if (IS_ERR(controller->clk))
		return PTR_ERR(controller->clk);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = clk_prepare_enable(controller->clk);
	if (ret)
		goto err_clk_disable;

	if (readl(controller->regs + REG_VERSION) != CURRENT_VERSION) {
		ret = -EINVAL;
		goto err_clk_disable;
	}

	writel(0x00, controller->regs + REG_ENABLE);

	ret = devm_request_irq(&pdev->dev, irq, adi_i3c_controller_interrupt, 0,
			       dev_name(&pdev->dev), controller);
	if (ret)
		goto err_clk_disable;

	platform_set_drvdata(pdev, controller);

	controller->maxdevs = PARAM_MAX_DEVS(readl(controller->regs + REG_PARAMETERS));
	writel(0x20, controller->regs + REG_IRQ_MASK);
	writel(DEVS_CTRL_DEV_CLR_ALL, controller->regs + REG_DEVS_CTRL);

	ret = i3c_master_register(&controller->base, &pdev->dev,
				  &adi_i3c_controller_ops, false);
	if (ret)
		goto err_clk_disable;

	return 0;

err_clk_disable:
	clk_disable_unprepare(controller->clk);

	return ret;
}

static int adi_i3c_controller_remove(struct platform_device *pdev)
{
	struct adi_i3c_controller *controller = platform_get_drvdata(pdev);
	int ret;

	ret = i3c_master_unregister(&controller->base);
	if (ret)
		return ret;

	clk_disable_unprepare(controller->clk);

	return 0;
}

static struct platform_driver adi_i3c_controller = {
	.probe = adi_i3c_controller_probe,
	.remove = adi_i3c_controller_remove,
	.driver = {
		.name = "adi-i3c-controller",
		.of_match_table = adi_i3c_controller_of_match,
	},
};
module_platform_driver(adi_i3c_controller);

MODULE_AUTHOR("Jorge Marques <jorge.marques@analog.com>");
MODULE_DESCRIPTION("Analog Devices I3C controller driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:adi-i3c-controller");
