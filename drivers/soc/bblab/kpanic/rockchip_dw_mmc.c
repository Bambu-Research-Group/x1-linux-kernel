/*
 * Copyright (c) 2013 Google, Inc
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <linux/types.h>
#include <linux/bitops.h>

#include <asm/io.h>

#include "common.h"
#include "dwmmc.h"
#include "kpanic.h"

#include "cru_rv1126.h"
#include "rv1126-cru.h"


struct rockchip_mmc_plat {
	//struct mmc_config cfg;
	struct mmc mmc;
};

struct rockchip_dwmmc_priv {
    struct rockchip_mmc_plat plat;

	void __iomem *emmc_base;
	void __iomem *cru_base;

	struct dwmci_host host;
	int fifo_depth;
	bool fifo_mode;
	u32 minmax[2];
};


static struct rockchip_dwmmc_priv rk_dwmmc_priv;


#define rk_clrsetreg(addr, clr, set)	\
				writel(((clr) | (set)) << 16 | (set), addr)
#define rk_clrreg(addr, clr)		writel((clr) << 16, addr)
#define rk_setreg(addr, set)		writel((set) << 16 | (set), addr)

#define DIV_TO_RATE(input_rate, div)    ((input_rate) / ((div) + 1))

static ulong clk_get_rate(struct rockchip_dwmmc_priv * priv, ulong clk_id)
{
	ulong gpll_hz = 1188000000;
	ulong cpll_hz = 500000000;
	void * cru_base = priv->cru_base;
	u32 offset = 0;
	u32 div, sel, con;

	switch (clk_id) {
	case HCLK_EMMC:
	case CLK_EMMC:
	case SCLK_EMMC_SAMPLE:
		offset = 0x01e4;
		break;
	default:
		offset = 0xffffffff;
		debug(" clkid:%lu not suppported\n", clk_id);
		break;
	}

	con = readl(cru_base + offset);
	div = (con & EMMC_DIV_MASK) >> EMMC_DIV_SHIFT;
	sel = (con & EMMC_SEL_MASK) >> EMMC_SEL_SHIFT;
	if (sel == EMMC_SEL_GPLL)
		return DIV_TO_RATE(gpll_hz, div) / 2;
	else if (sel == EMMC_SEL_CPLL)
		return DIV_TO_RATE(cpll_hz, div) / 2;
	else if (sel == EMMC_SEL_XIN24M)
		return DIV_TO_RATE(OSC_HZ, div) / 2;

	return 0;
}

static ulong clk_set_rate(struct rockchip_dwmmc_priv * priv, ulong clk_id, ulong rate)
{
	ulong gpll_hz = 1188000000;
	int src_clk_div;
	void * cru_base = priv->cru_base;
	u32 offset = 0;

	switch (clk_id) {
	case HCLK_EMMC:
	case CLK_EMMC:
		offset = 0x01e4;
		break;
	default:
		offset = 0xffffffff;
		debug(" clkid:%lu not suppported\n", clk_id);
		break;
	}

	/* Select clk_sdmmc/emmc source from GPLL by default */
	/* mmc clock defaulg div 2 internal, need provide double in cru */
	src_clk_div = DIV_ROUND_UP(gpll_hz / 2, rate);

	if (src_clk_div > 127) {
		/* use 24MHz source for 400KHz clock */
		src_clk_div = DIV_ROUND_UP(OSC_HZ / 2, rate);
		rk_clrsetreg(cru_base + offset,
			     EMMC_SEL_MASK | EMMC_DIV_MASK,
			     EMMC_SEL_XIN24M << EMMC_SEL_SHIFT |
			     (src_clk_div - 1) << EMMC_DIV_SHIFT);

		debug("gpll_hz =%lu, src_clk_div=%d, reg=%p, rate=%lx\n",
                gpll_hz,
                src_clk_div,
                (void*)cru_base + offset,
                rate);
	} else {
		rk_clrsetreg(cru_base + offset,
			     EMMC_SEL_MASK | EMMC_DIV_MASK,
			     EMMC_SEL_GPLL << EMMC_SEL_SHIFT |
			     (src_clk_div - 1) << EMMC_DIV_SHIFT);
		debug(" gpll_hz =%lu, src_clk_div=%d, reg=%p, rate=%lx\n",
                gpll_hz,
                src_clk_div,
                (void*)cru_base + offset,
                rate);
	}

	return clk_get_rate(priv, clk_id);
}


#define ROCKCHIP_MMC_DELAY_SEL		BIT(10)
#define ROCKCHIP_MMC_DEGREE_MASK	0x3
#define ROCKCHIP_MMC_DELAYNUM_OFFSET	2
#define ROCKCHIP_MMC_DELAYNUM_MASK	(0xff << ROCKCHIP_MMC_DELAYNUM_OFFSET)

#define PSECS_PER_SEC 1000000000000LL
/*
 * Each fine delay is between 44ps-77ps. Assume each fine delay is 60ps to
 * simplify calculations. So 45degs could be anywhere between 33deg and 57.8deg.
 */
#define ROCKCHIP_MMC_DELAY_ELEMENT_PSEC 60

static int clk_get_phase(struct rockchip_dwmmc_priv * priv, ulong clk_id)
{
	u32 raw_value, delay_num;
	u16 degrees = 0;
	ulong rate;

	void * cru_base = priv->cru_base;
	u32 offset = 0;

	switch (clk_id) {
	case SCLK_EMMC_SAMPLE:
		offset = 0x0454;
		break;
	case SCLK_SDMMC_SAMPLE:
		offset = 0x0444;
		break;
	case SCLK_SDIO_SAMPLE:
		offset = 0x044c;
		break;
	default:
		offset = 0xffffffff;
		debug(" clkid:%lu not suppported\n", clk_id);
		break;
	}

	rate = clk_get_rate(priv, clk_id);
	if (rate < 0)
		return rate;

	raw_value = readl(cru_base + offset);

	raw_value >>= 1;
	degrees = (raw_value & ROCKCHIP_MMC_DEGREE_MASK) * 90;

	if (raw_value & ROCKCHIP_MMC_DELAY_SEL) {
		/* degrees/delaynum * 10000 */
		unsigned long factor = (ROCKCHIP_MMC_DELAY_ELEMENT_PSEC / 10) *
					36 * (rate / 1000000);

		delay_num = (raw_value & ROCKCHIP_MMC_DELAYNUM_MASK);
		delay_num >>= ROCKCHIP_MMC_DELAYNUM_OFFSET;
		degrees += DIV_ROUND_CLOSEST(delay_num * factor, 10000);
	}

	return degrees % 360;
}

static int clk_set_phase(struct rockchip_dwmmc_priv * priv, ulong clk_id, u32 degrees)
{
	u8 nineties, remainder, delay_num;
	u32 raw_value, delay;
	ulong rate;

	void * cru_base = priv->cru_base;
	u32 offset = 0;

	switch (clk_id) {
	case SCLK_EMMC_SAMPLE:
		offset = 0x0454;
		break;
	case SCLK_SDMMC_SAMPLE:
		offset = 0x0444;
		break;
	case SCLK_SDIO_SAMPLE:
		offset = 0x044c;
		break;
	default:
		offset = 0xffffffff;
		debug(" clkid:%lu not suppported\n", clk_id);
		break;
	}

	rate = clk_get_rate(priv, clk_id);
	if (rate < 0)
		return rate;

	nineties = degrees / 90;
	remainder = (degrees % 90);

	/*
	 * Convert to delay; do a little extra work to make sure we
	 * don't overflow 32-bit / 64-bit numbers.
	 */
	delay = 10000000; /* PSECS_PER_SEC / 10000 / 10 */
	delay *= remainder;
	delay = DIV_ROUND_CLOSEST(delay, (rate / 1000) * 36 *
				  (ROCKCHIP_MMC_DELAY_ELEMENT_PSEC / 10));

	delay_num = (u8)min_t(u32, delay, 255);

	raw_value = delay_num ? ROCKCHIP_MMC_DELAY_SEL : 0;
	raw_value |= delay_num << ROCKCHIP_MMC_DELAYNUM_OFFSET;
	raw_value |= nineties;

	raw_value <<= 1;
	writel(raw_value | 0xffff0000, cru_base + offset);

	debug("mmc set_phase(%d) delay_nums=%u reg=%#x actual_degrees=%d\n",
	      degrees, delay_num, raw_value, clk_get_phase(priv, clk_id));

	return 0;

}

static uint rockchip_dwmmc_get_mmc_clk(struct dwmci_host *host, uint freq)
{
	struct rockchip_dwmmc_priv *priv = host->priv;
	int ret;

	/*
	 * If DDR52 8bit mode(only emmc work in 8bit mode),
	 * divider must be set 1
	 */
	if (mmc_card_ddr52(host->mmc) && host->mmc->bus_width == 8)
		freq *= 2;

	ret = clk_set_rate(priv, CLK_EMMC, freq);
	if (ret < 0) {
		debug("%s: err=%d\n", __func__, ret);
		return ret;
	}

	return freq;
}

static int rockchip_dwmmc_ofdata_to_platdata(struct rockchip_dwmmc_priv * priv)
{
	struct dwmci_host *host = &priv->host;

	host->name = "Rockchip DWMMC";
	host->ioaddr = priv->emmc_base;
	host->buswidth = 8;
	host->get_mmc_clk = rockchip_dwmmc_get_mmc_clk;
	host->priv = priv;

	/* use non-removeable as sdcard and emmc as judgement */
	host->dev_index = 1;

	priv->fifo_depth = 0x100;

	if (priv->fifo_depth < 0)
		return -EINVAL;
	priv->fifo_mode = false;

	priv->minmax[0] = 400000;  /* 400 kHz */
	priv->minmax[1] = 200000000;

	printk("dev_index=%d, fifo_depth=%d, buswidth=%d, fifo_mode=%d, max freq=%d\n",
			host->dev_index,
			priv->fifo_depth,
			host->buswidth,
			priv->fifo_mode,
			priv->minmax[1]);

	return 0;
}

static int rockchip_dwmmc_execute_tuning(struct dwmci_host *host, u32 opcode)
{
	int i = 0;
	int ret = -1;
	struct mmc *mmc = host->mmc;
	struct rockchip_dwmmc_priv *priv = host->priv;

	if (mmc->default_phase > 0 && mmc->default_phase < 360) {
		ret = clk_set_phase(priv, SCLK_EMMC_SAMPLE, mmc->default_phase);
		if (ret)
			printk("set clk phase fail\n");
		else
			ret = emmc_send_tuning(mmc, opcode);
		mmc->default_phase = 0;
	}
	/*
	 * If use default_phase to tune successfully, return.
	 * Otherwise, use the othe phase to tune.
	 */
	if (!ret)
		return ret;

	for (i = 0; i < 5; i++) {
		/* mmc->init_retry must be 0, 1, 2, 3 */
		if (mmc->init_retry == 4)
			mmc->init_retry = 0;

		ret = clk_set_phase(priv, SCLK_EMMC_SAMPLE, 90 * mmc->init_retry);
		if (ret) {
			printk("set clk phase fail\n");
			break;
		}
		ret = emmc_send_tuning(mmc, opcode);
		debug("Tuning phase is %d, ret is %d\n", mmc->init_retry * 90, ret);
		mmc->init_retry++;
		if (!ret)
			break;
	}

	return ret;
}

int rockchip_dwmmc_probe(struct kpanic_context * ctx)
{
    struct rockchip_dwmmc_priv * priv = &rk_dwmmc_priv;
	struct rockchip_mmc_plat *plat = &priv->plat;
	struct dwmci_host *host = &priv->host;

	priv->emmc_base = ctx->emmc_base;
	priv->cru_base = ctx->cru_base;

	rockchip_dwmmc_ofdata_to_platdata(priv);

	host->execute_tuning = rockchip_dwmmc_execute_tuning;

	host->fifoth_val = MSIZE(DWMCI_MSIZE) |
		RX_WMARK(priv->fifo_depth / 2 - 1) |
		TX_WMARK(priv->fifo_depth / 2);

	host->fifo_mode = priv->fifo_mode;

	host->stride_pio = false;

    //dwmci_setup_cfg(&plat->cfg, host, priv->minmax[1], priv->minmax[0]);

	//plat->cfg.host_caps |= MMC_MODE_HS200;

	plat->mmc.default_phase = 90;

	plat->mmc.init_retry = 0;
	host->mmc = &plat->mmc;
	host->mmc->priv = &priv->host;

    return add_dwmci(host, priv->minmax[1], priv->minmax[0]);
}

struct mmc *init_mmc_device(int dev, bool force_init)
{
	struct mmc *mmc;
	mmc = find_mmc_device(dev);
	if (!mmc) {
		printk("no mmc device at slot %x\n", dev);
		return NULL;
	}

	if (force_init)
		mmc->has_init = 0;
	if (mmc_init(mmc))
		return NULL;
	return mmc;
}
