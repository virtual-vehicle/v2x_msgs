#!/usr/bin/env python

"""This file replaces duplicates, because some messages are generated twice from two different asn-files.
The paths to the duplicates are static and therefor are deleted by path instandly. """

#Note: Import built-in modules first, followed by third-party modules, followed by any changes to the path and your own modules

__author__ = "Patrizia Neubauer"
__copyright__ = "Copyright 2023, vehicleCAPTAIN toolbox"
__credits__ = ["Patrizia Neubauer", "Christoph Pilz"]
__license__ = "BSD_3-Clause"
__version__ = "1.0"
__maintainer__ = "Patrizia Neubauer"
__email__ = "patrizia.neubauer@v2c2.at"
__status__ = "Production"


import os

# The purpose of this file is to remove all the duplicated files.
# We choose ETSI over ISO and ISO over DSRC!

rel_path_msgs = "build/msg"

# DriverCharacteristics.msg:
filePath = rel_path_msgs + "/is_ts103301/iso-patched/ISO14906_2018_EfcDsrcApplicationv6/DriverCharacteristics.msg"
if os.path.exists(filePath):
    os.remove(filePath)

# TrailerCharacteristics.msg:
filePath = rel_path_msgs + "/is_ts103301/iso-patched/ISO14906_2018_EfcDsrcApplicationv6/TrailerCharacteristics.msg"
if os.path.exists(filePath):
    os.remove(filePath)

# SpeedConfidence.msg:
filePath = rel_path_msgs + "/additional_modules/ISO-TS-19091-addgrp-C-2018-patched/SpeedConfidence.msg"
if os.path.exists(filePath):
    os.remove(filePath)

# Version.msg:
filePath = rel_path_msgs + "/additional_modules/TS17419_2014_CITSapplMgmtIDs/Version.msg"
if os.path.exists(filePath):
    os.remove(filePath)

# StationType.msg:
filePath = rel_path_msgs + "/is_ts103301/iso-patched/ISO14906_2018_EfcDsrcApplicationv6/StationType.msg"
if os.path.exists(filePath):
    os.remove(filePath)

# Temperature.msg:
filePath = rel_path_msgs + "/additional_modules/ISO19321IVIv2/Temperature.msg"
if os.path.exists(filePath):
    os.remove(filePath)

# Heading.msg:
filePath = rel_path_msgs + "/additional_modules/ISO-TS-19091-addgrp-C-2018-patched/Heading.msg"
if os.path.exists(filePath):
    os.remove(filePath)
