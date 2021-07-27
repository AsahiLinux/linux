// SPDX-License-Identifier: GPL-2.0-only

#include "apple-asc-mailbox.h"

#define CREATE_TRACE_POINTS
#include "apple-asc-mailbox-trace.h"

static bool apple_mbox_hw_can_send(struct apple_mbox *apple_mbox)
{
	u32 mbox_ctrl = readl(apple_mbox->regs + APPLE_IOP_A2I_CONTROL);
	return !(mbox_ctrl & APPLE_IOP_A2I_CONTROL_FULL);
}

static void apple_mbox_hw_send(struct apple_mbox *apple_mbox,
			       struct apple_mbox_msg *msg)
{
	trace_apple_mbox_hw_send(apple_mbox, msg->msg, msg->info);
	WARN_ON(!apple_mbox_hw_can_send(apple_mbox));
	writeq(msg->msg, apple_mbox->regs + APPLE_IOP_A2I_MBOX_DATA);
	writeq(msg->info, apple_mbox->regs + APPLE_IOP_A2I_MBOX_INFO);
}

static bool apple_mbox_hw_can_recv(struct apple_mbox *apple_mbox)
{
	u32 mbox_ctrl = readl(apple_mbox->regs + APPLE_IOP_I2A_CONTROL);
	return !(mbox_ctrl & APPLE_IOP_I2A_CONTROL_EMPTY);
}

static void apple_mbox_hw_recv(struct apple_mbox *apple_mbox,
			       struct apple_mbox_msg *msg)
{
	WARN_ON(!apple_mbox_hw_can_recv(apple_mbox));
	msg->msg = readq(apple_mbox->regs + APPLE_IOP_I2A_MBOX_DATA);
	msg->info = readq(apple_mbox->regs + APPLE_IOP_I2A_MBOX_INFO);
	trace_apple_mbox_hw_recv(apple_mbox, msg->msg, msg->info);
}

static void apple_mbox_can_recv_irq_enable(struct apple_mbox *mbox, bool enable)
{
	trace_apple_mbox_can_recv_irq_enable(mbox, enable);
	if (enable)
		enable_irq(mbox->irq_can_recv);
	else
		disable_irq_nosync(mbox->irq_can_recv);
}

static void apple_mbox_can_send_irq_enable(struct apple_mbox *mbox, bool enable)
{
	trace_apple_mbox_can_send_irq_enable(mbox, enable);
	if (enable)
		enable_irq(mbox->irq_can_send);
	else
		disable_irq_nosync(mbox->irq_can_send);
}

static irqreturn_t apple_mbox_can_send_irq_handler(int irq, void *data)
{
	struct apple_mbox *apple_mbox = data;
	struct apple_chan_priv *chan_priv;
	bool keep_irq_enabled = false;
	unsigned long flags;
	int i;

	for (i = 0; i < APPLE_IOP_MAX_ACTIVE_CHANS; ++i) {
		chan_priv = apple_mbox->chans[i].con_priv;
		if (!chan_priv)
			continue;
		if (!chan_priv->needs_irq)
			continue;
		if (!apple_mbox_hw_can_send(apple_mbox))
			break;

		chan_priv->needs_irq = false;
		mbox_chan_txdone(&apple_mbox->chans[i], 0);
	}

	spin_lock_irqsave(&apple_mbox->lock, flags);
	for (i = 0; i < APPLE_IOP_MAX_ACTIVE_CHANS; ++i) {
		chan_priv = apple_mbox->chans[i].con_priv;
		if (!chan_priv)
			continue;
		if (chan_priv->needs_irq) {
			keep_irq_enabled = true;
			break;
		}
	}

	if (keep_irq_enabled) {
		apple_mbox->allow_can_send_irq_enable = false;
		apple_mbox_can_send_irq_enable(apple_mbox, true);
	} else {
		apple_mbox->allow_can_send_irq_enable = true;
		apple_mbox_can_send_irq_enable(apple_mbox, false);
	}
	spin_unlock_irqrestore(&apple_mbox->lock, flags);

	return IRQ_HANDLED;
}

static irqreturn_t apple_mbox_recv_irq_handler(int irq, void *data)
{
	struct apple_mbox *mbox = data;
	struct apple_mbox_msg msg;
	bool wake = false;
	size_t len;
	unsigned long flags;

	while (apple_mbox_hw_can_recv(mbox)) {
		spin_lock_irqsave(&mbox->lock, flags);
		if (unlikely(kfifo_avail(&mbox->recv_fifo) < 1)) {
			apple_mbox_can_recv_irq_enable(mbox, false);
			mbox->recv_full = true;
			spin_unlock_irqrestore(&mbox->lock, flags);
			return IRQ_WAKE_THREAD;
		}
		spin_unlock_irqrestore(&mbox->lock, flags);

		apple_mbox_hw_recv(mbox, &msg);
		len = kfifo_in(&mbox->recv_fifo, &msg, 1);
		WARN_ON(len != 1);
		wake = true;
	}

	if (wake)
		return IRQ_WAKE_THREAD;
	else
		return IRQ_HANDLED;
}

static irqreturn_t apple_mbox_recv_irq_thread(int irq, void *data)
{
	struct apple_mbox *apple_mbox = data;
	struct mbox_controller *mbox = &apple_mbox->controller;
	struct apple_mbox_msg msg;
	struct apple_chan_priv *chan_priv;
	int i;
	u8 endpoint;
	s8 chan_idx;
	size_t len;
	unsigned long flags;

	while (kfifo_len(&apple_mbox->recv_fifo) >= 1) {
		len = kfifo_out(&apple_mbox->recv_fifo, &msg, 1);
		WARN_ON(len != 1);

		endpoint = msg.info & 0xff;
		chan_idx = apple_mbox->epmap[endpoint];
		if (chan_idx < 0) {
			dev_err(mbox->dev,
				"Received message for unknown endpoint #0x%02x.",
				endpoint);
			continue;
		}

		chan_priv = mbox->chans[i].con_priv;
		if (!chan_priv->enabled) {
			dev_err(mbox->dev,
				"Received message for disabled endpoint #0x%02x.",
				endpoint);
			continue;
		}

		mbox_chan_received_data(&mbox->chans[chan_idx],
					(void *)msg.msg);
	}

	spin_lock_irqsave(&apple_mbox->lock, flags);
	if (apple_mbox->recv_full) {
		apple_mbox->recv_full = false;
		apple_mbox_can_recv_irq_enable(apple_mbox, true);
	}
	spin_unlock_irqrestore(&apple_mbox->lock, flags);

	return IRQ_HANDLED;
}

static struct mbox_chan *apple_mbox_of_xlate(struct mbox_controller *mbox,
					     const struct of_phandle_args *spec)
{
	struct apple_mbox *apple_mbox = dev_get_drvdata(mbox->dev);
	struct mbox_chan *chan;
	struct apple_chan_priv *chan_priv;
	int i, endpoint, free_chan_idx;

	if (spec->args_count != 1)
		return ERR_PTR(-EINVAL);

	endpoint = spec->args[0];
	if (endpoint > APPLE_IOP_MAX_ENDPOINTS || endpoint < 0)
		return ERR_PTR(-EINVAL);

	if (apple_mbox->epmap[endpoint] >= 0)
		return ERR_PTR(-EBUSY);

	free_chan_idx = -1;
	for (i = 0; i < mbox->num_chans; i++) {
		chan_priv = mbox->chans[i].con_priv;

		if (!chan_priv) {
			free_chan_idx = i;
			break;
		}
	}

	if (free_chan_idx < 0) {
		dev_err(mbox->dev, "No free channels left\n");
		return ERR_PTR(-EBUSY);
	}

	chan = &mbox->chans[free_chan_idx];
	chan_priv = devm_kzalloc(mbox->dev, sizeof(*chan_priv), GFP_KERNEL);
	if (!chan_priv)
		return ERR_PTR(-ENOMEM);

	apple_mbox->epmap[endpoint] = free_chan_idx;
	chan_priv->endpoint = endpoint;
	chan_priv->apple_mbox = apple_mbox;
	chan->con_priv = chan_priv;
	return chan;
}

static int apple_mbox_chan_send_data(struct mbox_chan *chan, void *data)
{
	struct apple_chan_priv *chan_priv = chan->con_priv;
	struct apple_mbox *apple_mbox = chan_priv->apple_mbox;
	struct apple_mbox_msg msg;
	unsigned long flags;
	int ret = 0;

	msg.info = chan_priv->endpoint;
	msg.msg = (u64)data;

	spin_lock_irqsave(&apple_mbox->lock, flags);

	if (apple_mbox_hw_can_send(apple_mbox)) {
		apple_mbox_hw_send(apple_mbox, &msg);
	} else {
		ret = -EBUSY;
		if (apple_mbox->allow_can_send_irq_enable) {
			apple_mbox->allow_can_send_irq_enable = false;
			apple_mbox_can_send_irq_enable(apple_mbox, true);
		}
		chan_priv->needs_irq = true;
	}

	spin_unlock_irqrestore(&apple_mbox->lock, flags);
	return ret;
}

static int apple_mbox_chan_startup(struct mbox_chan *chan)
{
	struct apple_chan_priv *chan_priv = chan->con_priv;

	chan_priv->enabled = true;
	return 0;
}

static void apple_mbox_chan_shutdown(struct mbox_chan *chan)
{
	struct apple_chan_priv *chan_priv = chan->con_priv;

	chan_priv->enabled = false;
}

static const struct mbox_chan_ops apple_mbox_ops = {
	.send_data = &apple_mbox_chan_send_data,
	.startup = &apple_mbox_chan_startup,
	.shutdown = &apple_mbox_chan_shutdown,
};

static int apple_mbox_probe(struct platform_device *pdev)
{
	int ret, i;
	struct apple_mbox *mbox;
	struct resource *regs;
	struct device *dev = &pdev->dev;

	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;
	platform_set_drvdata(pdev, mbox);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret)
		return ret;

	mbox->dev = dev;
	spin_lock_init(&mbox->lock);
	INIT_KFIFO(mbox->recv_fifo);

	for (i = 0; i < APPLE_IOP_MAX_ENDPOINTS; ++i)
		mbox->epmap[i] = -1;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs)
		return -EINVAL;

	mbox->regs = devm_ioremap_resource(dev, regs);
	if (IS_ERR(mbox->regs))
		return PTR_ERR(mbox->regs);

	mbox->irq_can_send = platform_get_irq_byname(pdev, "can-send");
	if (mbox->irq_can_send < 0)
		return -ENODEV;
	mbox->irq_can_recv = platform_get_irq_byname(pdev, "can-recv");
	if (mbox->irq_can_recv < 0)
		return -ENODEV;

	ret = devm_clk_bulk_get_all(dev, &mbox->clks);
	if (ret)
		return ret;
	mbox->num_clks = ret;

	ret = clk_bulk_prepare_enable(mbox->num_clks, mbox->clks);
	if (ret)
		return ret;

	mbox->controller.dev = mbox->dev;
	mbox->controller.num_chans = APPLE_IOP_MAX_ACTIVE_CHANS;
	mbox->controller.chans = mbox->chans;
	mbox->controller.ops = &apple_mbox_ops;
	mbox->controller.of_xlate = &apple_mbox_of_xlate;
	mbox->controller.txdone_irq = true;
	mbox->controller.txdone_direct = true;
	mbox->allow_can_send_irq_enable = true;

	ret = request_irq(mbox->irq_can_send, &apple_mbox_can_send_irq_handler,
			  IRQF_NO_AUTOEN, dev_name(dev), mbox);
	if (ret)
		goto err_clk_disable;

	ret = request_threaded_irq(mbox->irq_can_recv,
				   &apple_mbox_recv_irq_handler,
				   &apple_mbox_recv_irq_thread, 0,
				   dev_name(dev), mbox);
	if (ret)
		goto free_can_send_irq;

	ret = devm_mbox_controller_register(dev, &mbox->controller);
	if (ret)
		goto free_can_recv_irq;

	return ret;

free_can_recv_irq:
	free_irq(mbox->irq_can_recv, mbox);
free_can_send_irq:
	free_irq(mbox->irq_can_send, mbox);
err_clk_disable:
	clk_bulk_disable_unprepare(mbox->num_clks, mbox->clks);
	return ret;
}

/*
 * these are probably all the same but use different compatible just in case
 * we discover quirks later on
 */
static const struct of_device_id apple_mbox_of_match[] = {
	{
		.compatible = "apple,t8103-ans-mailbox",
	},
	{
		.compatible = "apple,t8103-smc-mailbox",
	},
	{
		.compatible = "apple,t8103-rtkit-mailbox",
	},
	{
		.compatible = "apple,t8103-sepos-mailbox",
	},
	{},
};
MODULE_DEVICE_TABLE(of, apple_mbox_of_match);

static int apple_mbox_remove(struct platform_device *pdev)
{
	struct apple_mbox *apple_mbox = platform_get_drvdata(pdev);

	free_irq(apple_mbox->irq_can_recv, apple_mbox);
	free_irq(apple_mbox->irq_can_send, apple_mbox);

	return 0;
}

static void apple_mbox_shutdown(struct platform_device *pdev)
{
	apple_mbox_remove(pdev);
}

static struct platform_driver apple_mbox_driver = {
	.driver = {
		.name = "apple-mailbox",
		.of_match_table = apple_mbox_of_match,
	},
	.probe = apple_mbox_probe,
	.remove = apple_mbox_remove,
	.shutdown = apple_mbox_shutdown,
};
module_platform_driver(apple_mbox_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Sven Peter <sven@svenpeter.dev>");
MODULE_DESCRIPTION("Apple mailbox driver");
