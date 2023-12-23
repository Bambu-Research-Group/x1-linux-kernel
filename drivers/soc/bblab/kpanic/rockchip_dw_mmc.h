#ifndef RK_DW_MMC__H
#define RK_DW_MMC__H


#include "common.h"

#include "mmc.h"
#include "kpanic.h"


int rockchip_dwmmc_probe(struct kpanic_context * ctx);

struct mmc *init_mmc_device(int dev, bool force_init);

#endif
