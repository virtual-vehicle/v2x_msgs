#!/usr/bin/env python

"""This file replaces the original generated message files with the adapted message files from the "hotfix"-folder.
All file names from all folders in the directory msg, except msg/hotfix itself, are going to be compared to all the file names from the hotfix folder. If the name matches. it will be replaced with the hotfix file.
An exception is the REGEXTIDANDTYPE.msg, which is deleted."""

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
import shutil

hotfixFiles = []
i = 0

for (root, dirs, files) in os.walk('hotfix'):
    hotfixFiles.append(files)

for (root, dirs, files) in os.walk('../'):
    if str(root) != "../generator/hotfix":
        i = 0
        for file in files:
            for hotfix in hotfixFiles[0]:
                targetPath = ""
                hotfixPath = ""
                if str(file) == "REGEXTIDANDTYPE.msg":
                    targetPath = root + "/"
                    targetPath += file
                    os.remove(targetPath)
                    break
                elif str(hotfix) == str(file):
                    hotfixPath = "../generator/hotfix/" + hotfix
                    targetPath = root + "/"
                    targetPath += file
                    shutil.copy(hotfixPath, targetPath)
                i += 1