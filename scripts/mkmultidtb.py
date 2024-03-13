#!/usr/bin/env python
# SPDX-License-Identifier: (GPL-2.0+ OR MIT)
# Copyright (c) 2018 Fuzhou Rockchip Electronics Co., Ltd
#


"""
Multiple dtb package tool

Usage: scripts/mkmultidtb.py board
The board is what you defined in DTBS dictionary like DTBS['board'],
Such as: PX30-EVB, RK3308-EVB

"""
import os
import sys
import shutil
from collections import OrderedDict

DTBS = {}

DTBS['PX30-EVB'] = OrderedDict([('px30-evb-ddr3-v10', '#_saradc_ch0=1024'),
				('px30-evb-ddr3-lvds-v10', '#_saradc_ch0=512')])

DTBS['RK3308-EVB'] = OrderedDict([('rk3308-evb-dmic-i2s-v10', '#_saradc_ch3=288'),
				  ('rk3308-evb-dmic-pdm-v10', '#_saradc_ch3=1024'),
				  ('rk3308-evb-amic-v10', '#_saradc_ch3=407')])

DTBS['rv1126-bl-p001-v5.dtb'] = OrderedDict([
                                #('rv1126-bl-p001-v2', '#_saradc_ch1=512'), # 0.9v
				#('rv1126-bl-p001-v3', '#_saradc_ch1=543'), # 0.954v
				#('rv1126-bl-p001-v4_a', '#_saradc_ch1=626#_saradc_ch2=333'), # 1.1v
				#('rv1126-bl-p001-v4_b', '#_saradc_ch1=626#_saradc_ch2=0'), # 1.1v
				# ('rv1126-bl-p001-v5_a', '#_saradc_ch1=683#_saradc_ch2=333'), # 1.2v_0.6v
				# ('rv1126-bl-p001-v5_b', '#_saradc_ch1=683#_saradc_ch2=0'), # 1.2v_0.0v
				# ('rv1126-bl-p001-v5_c', '#_saradc_ch1=683#_saradc_ch2=505')]) # 1.2v
				('rv1126-bl-p001-v5', '#_saradc_ch1=683'),])# 1.2v


DTBS['rv1126-n2-v1.dtb'] = OrderedDict([
				('rv1126-n2-v1', '#_saradc_ch1=0'),])# 0v

DTBS['rv1126-c13-v2.dtb'] = OrderedDict([
                                ('rv1126-c13-v1', '#_saradc_ch1=427'),# 0.75v
				('rv1126-c13-v2', '#_saradc_ch1=512')])# 0.9v

dts_path = 'arch/arm/boot/dts/'

def main():
    if (len(sys.argv) < 2) or (sys.argv[1] == '-h'):
        print(__doc__)
        sys.exit(2)

    BOARD = sys.argv[1]
    TARGET_DTBS = DTBS[BOARD]
    target_dtb_list = ''
    default_dtb = True

    for dtb, value in TARGET_DTBS.items():
        if default_dtb:
            ori_file = dts_path + dtb + '.dtb'
            shutil.copyfile(ori_file, "rk-kernel.dtb")
            target_dtb_list += 'rk-kernel.dtb '
            default_dtb = False
        new_file = dtb + value + '.dtb'
        ori_file = dts_path + dtb + '.dtb'
        shutil.copyfile(ori_file, new_file)
        target_dtb_list += ' ' + new_file

    print(target_dtb_list)
    os.system('scripts/resource_tool logo.bmp logo_kernel.bmp ' + target_dtb_list)
    os.system('rm ' + target_dtb_list)

if __name__ == '__main__':
    main()
