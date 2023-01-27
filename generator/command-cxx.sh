#!/bin/bash -e
# ----------------------------------------------------
# Project: vehicleCAPTAIN ITS library
#
# Authors:
#   - Christoph Pilz (christoph.pilz@v2c2.at)
#
# Things to Improve:
# - Add: coloring to output
# - add: return value for are_standards_present
# - add: other standards
# - handle: not all standards present -> go to fail
# - refactor: relative path handling load_additional_modules
# - refactor: clean-all for removing also git ETSI ITS ASN1 git repo
#
# ----------------------------------------------------

### ### Defines ### ###
VC_ITS_LIB_NAME="v2x_msgs"
VC_ITS_LIB_VERSION="1.0"

ROOT_DIR=$(pwd)
ASN1C_DIR=/usr/local/share/asn1c
VC_ITS_ASN1_SPECS_DIR=$ROOT_DIR/../"vehicle_captain_its_asn1_specifications"
ETSI_ITS_BUILD_ROOT_DIR=$ROOT_DIR/"build"
CXX_EXCEPTIONS_DIR=$ROOT_DIR/"exceptions"/"CXX"
CXX_PARSER_DIR=$ROOT_DIR/"parser"/"CXX"
VC_ITS_LIB_ROOT_DIR=$ROOT_DIR/$VC_ITS_LIB_NAME
VC_ITS_LIB_INCLUDE_DIR=$VC_ITS_LIB_ROOT_DIR/"include"/$VC_ITS_LIB_NAME
VC_ITS_LIB_SRC_DIR=$VC_ITS_LIB_ROOT_DIR/"src"
HOTFIX_DIR=$ROOT_DIR/"hotfix.py"
DUPLICATES_DIR=$ROOT_DIR/"remove-duplicates.py"

ISO_TS_19091_ADDGRP_C_2018_SPEC=$VC_ITS_ASN1_SPECS_DIR/hotfix/ISO-TS-19091-addgrp-C-2018.asn

CDD_SPEC_v2=etsi/cdd_ts102894_2/ITS-Container.asn
CAM_SPEC=etsi/cam_en302637_2/CAM-PDU-Descriptions.asn
CPM_SPEC=experimental/CPM-PDU-Descriptions.asn
DENM_SPEC=etsi/denm_en302637_3/DENM-PDU-Descriptions.asn
EVCSN_SPEC=etsi/evcsn-ts101556_1/EVCSN-PDU-Descriptions.asn
EVRSR_SPEC=etsi/evrsr_ts101556_3/EV-RSR-PDU-Descriptions.asn
IVIM_SPEC=etsi/is_ts103301/IVIM-PDU-Descriptions.asn
MAPEM_SPEC=etsi/is_ts103301/MAPEM-PDU-Descriptions.asn
RTCMEM_SPEC=etsi/is_ts103301/RTCMEM-PDU-Descriptions.asn
SPATEM_SPEC=etsi/is_ts103301/SPATEM-PDU-Descriptions.asn
SREM_SPEC=etsi/is_ts103301/SREM-PDU-Descriptions.asn
SSEM_SPEC=etsi/is_ts103301/SSEM-PDU-Descriptions.asn
VC_CONTAINER_SPEC=custom/VC-Container-PDU-Descriptions.asn

ADDITIONAL_MODULES_DIR=$ETSI_ITS_BUILD_ROOT_DIR/"asn1"
IS_TS_ISO_DIR=$VC_ITS_ASN1_SPECS_DIR/"etsi/is_ts103301/iso-patched"
MSG_ROOT_DIR=$ROOT_DIR/'msg'
MSG_ADDITIONAL_MODULES_DIR=$MSG_ROOT_DIR/"additional_modules"
MSG_ISO_PATCHED_DIR=$MSG_ROOT_DIR/"is_ts103301/iso-patched"

# Echo Headers
AUTHOR_ECHO="# Authors:
# - Christoph Pilz (christoph.pilz@v2c2.at)
# - Patrizia Neubauer (patrizia.neubauer@v2c2.at)
# "

### ### Functions ### ###
function load_vehicle_captain_its_asn1_specifications() {
  cd "$ROOT_DIR" || return
  echo "Loading vehicleCAPTAIN ITS ASN1 specifications ..."

  if [ ! -d "$VC_ITS_ASN1_SPECS_DIR" ]
    then
      cd .. || return
      echo "Cloning vehicleCAPTAIN ITS ASN1 specifications into $(pwd) ..."
      git clone https://github.com/virtual-vehicle/vehicle_captain_its_asn1_specifications.git --recurse-submodules
    else
      echo "vehicleCAPTAIN ITS ASN1 specifications present"
  fi
}

function are_standards_present() {
  # Basics
  if ! [[ -f "$VC_ITS_ASN1_SPECS_DIR"/"$CDD_SPEC_v2" ]]; then
      echo "WARNING: $VC_ITS_ASN1_SPECS_DIR/$CDD_SPEC_v2 missing."
    fi

    # custom
    if ! [[ -f "$VC_ITS_ASN1_SPECS_DIR"/"$VC_CONTAINER_SPEC" ]]; then
      echo "WARNING: $VC_SPECS_DIR/$VC_CONTAINER_SPEC missing."
    fi

    # experimental
    if ! [[ -f "$VC_ITS_ASN1_SPECS_DIR"/"$CPM_SPEC" ]]; then
      echo "WARNING: $VC_ITS_ASN1_SPECS_DIR/$CPM_SPEC missing."
    fi

    # standardized
    if ! [[ -f "$VC_ITS_ASN1_SPECS_DIR"/"$CAM_SPEC" ]]; then
      echo "WARNING: $VC_ITS_ASN1_SPECS_DIR/$CAM_SPEC missing."
    fi
    if ! [[ -f "$VC_ITS_ASN1_SPECS_DIR"/"$DENM_SPEC" ]]; then
      echo "WARNING: $VC_ITS_ASN1_SPECS_DIR/$DENM_SPEC missing."
    fi
    if ! [[ -f "$VC_ITS_ASN1_SPECS_DIR"/"$EVCSN_SPEC" ]]; then
      echo "WARNING: $VC_ITS_ASN1_SPECS_DIR/$EVCSN_SPEC missing."
    fi
    if ! [[ -f "$VC_ITS_ASN1_SPECS_DIR"/"$EVRSR_SPEC" ]]; then
      echo "WARNING: $VC_ITS_ASN1_SPECS_DIR/$EVRSR_SPEC missing."
    fi
    if ! [[ -f "$VC_ITS_ASN1_SPECS_DIR"/"$IVIM_SPEC" ]]; then
      echo "WARNING: $VC_ITS_ASN1_SPECS_DIR/$IVIM_SPEC missing."
    fi
    if ! [[ -f "$VC_ITS_ASN1_SPECS_DIR"/"$MAPEM_SPEC" ]]; then
      echo "WARNING: $VC_ITS_ASN1_SPECS_DIR/$MAPEM_SPEC missing."
    fi
    if ! [[ -f "$VC_ITS_ASN1_SPECS_DIR"/"$RTCMEM_SPEC" ]]; then
      echo "WARNING: $VC_ITS_ASN1_SPECS_DIR/$RTCMEM_SPEC missing."
    fi
    if ! [[ -f "$VC_ITS_ASN1_SPECS_DIR"/"$SPATEM_SPEC" ]]; then
      echo "WARNING: $VC_ITS_ASN1_SPECS_DIR/$SPATEM_SPEC missing."
    fi
    if ! [[ -f "$VC_ITS_ASN1_SPECS_DIR"/"$SREM_SPEC" ]]; then
      echo "WARNING: $VC_ITS_ASN1_SPECS_DIR/$SREM_SPEC missing."
    fi
    if ! [[ -f "$VC_ITS_ASN1_SPECS_DIR"/"$SSEM_SPEC" ]]; then
      echo "WARNING: $VC_ITS_ASN1_SPECS_DIR/$SSEM_SPEC missing."
    fi
}

function load_additional_modules() {
  # additional modules for IVIM, MAPEM, RTCMEM, SPATEM, SREM, SSEM
  install -d "$ETSI_ITS_BUILD_ROOT_DIR"/asn1

  if [ ! -f "$ETSI_ITS_BUILD_ROOT_DIR"/asn1/ISO-TS-19091-addgrp-C-2018.asn ]; then
        wget -P "$ETSI_ITS_BUILD_ROOT_DIR"/asn1 https://standards.iso.org/iso/ts/19091/ed-2/en/ISO-TS-19091-addgrp-C-2018.asn
        sed -e 's/\bHeadingConfidence\b/HeadingConfidenceDSRC/g' \
          -e 's/\bSpeedConfidence\b/SpeedConfidenceDSRC/g' \
          -e 's/\bHeading\b/HeadingDSRC/g' \
          "$ISO_TS_19091_ADDGRP_C_2018_SPEC" >"$ETSI_ITS_BUILD_ROOT_DIR"/asn1/ISO-TS-19091-addgrp-C-2018-patched.asn
  fi
  if [ ! -f "$ETSI_ITS_BUILD_ROOT_DIR"/asn1/ISO19321IVIv2.asn ]; then
        wget -P "$ETSI_ITS_BUILD_ROOT_DIR"/asn1 'https://standards.iso.org/iso/ts/19321/ed-2/en/ISO19321IVIv2.asn'
  fi
  if [ ! -f "$ETSI_ITS_BUILD_ROOT_DIR"/asn1/ISO24534-3_ElectronicRegistrationIdentificationVehicleDataModule-patched.asn ]; then
        wget -P "$ETSI_ITS_BUILD_ROOT_DIR"/asn1 'https://forge.etsi.org/rep/ITS/asn1/is_ts103301/-/raw/release2/iso-patched/ISO24534-3_ElectronicRegistrationIdentificationVehicleDataModule-patched.asn'
      fi
  if [ ! -f "$ETSI_ITS_BUILD_ROOT_DIR"/asn1/ISO14816_AVIAEINumberingAndDataStructures.asn ]; then
        wget -P "$ETSI_ITS_BUILD_ROOT_DIR"/asn1 https://standards.iso.org/iso/14816/ISO14816%20ASN.1%20repository/ISO14816_AVIAEINumberingAndDataStructures.asn
  fi
  if [ ! -f "$ETSI_ITS_BUILD_ROOT_DIR"/asn1/"ISO14906(2018)EfcDsrcApplicationv6.asn" ]; then
        wget -P "$ETSI_ITS_BUILD_ROOT_DIR"/asn1 'https://standards.iso.org/iso/14906/ed-3/en/ISO14906(2018)EfcDsrcApplicationv6.asn'
  fi
  if [ ! -f "$ETSI_ITS_BUILD_ROOT_DIR"/asn1/"ISO14906(2018)EfcDsrcGenericv7.asn" ]; then
        wget -P "$ETSI_ITS_BUILD_ROOT_DIR"/asn1 'https://standards.iso.org/iso/14906/ed-3/en/ISO14906(2018)EfcDsrcGenericv7.asn'
  fi
  if [ ! -f "$ETSI_ITS_BUILD_ROOT_DIR"/asn1/TS17419_2014_CITSapplMgmtIDs.asn ]; then
        wget -P "$ETSI_ITS_BUILD_ROOT_DIR"/asn1 'https://standards.iso.org/iso/ts/17419/TS%2017419%20ASN.1%20repository/TS17419_2014_CITSapplMgmtIDs.asn'
  fi
  #if [ ! -f "$ETSI_ITS_BUILD_ROOT_DIR"/asn1/"ISO14906(2018)EfcDsrcGenericv7.asn" ]; then
  #      wget -P "$ETSI_ITS_BUILD_ROOT_DIR"/asn1 'https://standards.iso.org/iso/14906/ed-3/en/ISO14906(2018)EfcDsrcGenericv7.asn'
  #fi
  if [ ! -f "$ETSI_ITS_BUILD_ROOT_DIR"/asn1/ISO14823-missing.asn ]; then
        wget -P "$ETSI_ITS_BUILD_ROOT_DIR"/asn1 'https://forge.etsi.org/rep/ITS/asn1/is_ts103301/-/raw/release2/iso-patched/ISO14823-missing.asn'
    fi

}

function compile_additional_modules_ROS() {

  install -d "$MSG_ADDITIONAL_MODULES_DIR"/ISO-TS-19091-addgrp-C-2018-patched/
  asn1c -D "$MSG_ADDITIONAL_MODULES_DIR"/ISO-TS-19091-addgrp-C-2018-patched/ -E -generate-ros-message -R -no-gen-example -fcompound-names -fno-include-deps \
    "$ETSI_ITS_BUILD_ROOT_DIR"/asn1/'ISO-TS-19091-addgrp-C-2018-patched.asn'

  install -d "$MSG_ADDITIONAL_MODULES_DIR"/ISO14816_AVIAEINumberingAndDataStructures/
  asn1c -D "$MSG_ADDITIONAL_MODULES_DIR"/ISO14816_AVIAEINumberingAndDataStructures/ -E -generate-ros-message -R -no-gen-example -fcompound-names -fno-include-deps \
    "$ETSI_ITS_BUILD_ROOT_DIR"/asn1/'ISO14816_AVIAEINumberingAndDataStructures.asn'

  install -d "$MSG_ADDITIONAL_MODULES_DIR"/ISO19321IVIv2/
  asn1c -D "$MSG_ADDITIONAL_MODULES_DIR"/ISO19321IVIv2/ -E -generate-ros-message -R -no-gen-example -fcompound-names -fno-include-deps \
    "$ETSI_ITS_BUILD_ROOT_DIR"/asn1/'ISO19321IVIv2.asn'

  install -d "$MSG_ADDITIONAL_MODULES_DIR"/TS17419_2014_CITSapplMgmtIDs/
  asn1c -D "$MSG_ADDITIONAL_MODULES_DIR"/TS17419_2014_CITSapplMgmtIDs/ -E -generate-ros-message -R -no-gen-example -fcompound-names -fno-include-deps \
    "$ETSI_ITS_BUILD_ROOT_DIR"/asn1/'TS17419_2014_CITSapplMgmtIDs.asn'

  install -d "$MSG_ISO_PATCHED_DIR"/ISO14823-missing/
  asn1c -D "$MSG_ISO_PATCHED_DIR"/ISO14823-missing/ -E -generate-ros-message -R -no-gen-example -fcompound-names -fno-include-deps \
    "$ETSI_ITS_BUILD_ROOT_DIR"/asn1/'ISO14823-missing.asn'

  install -d "$MSG_ISO_PATCHED_DIR"/ISO14906_2018_EfcDsrcApplicationv6/
  asn1c -D "$MSG_ISO_PATCHED_DIR"/ISO14906_2018_EfcDsrcApplicationv6/ -E -generate-ros-message -R -no-gen-example -fcompound-names -fno-include-deps \
    "$ETSI_ITS_BUILD_ROOT_DIR"/asn1/'ISO14906(2018)EfcDsrcApplicationv6.asn'

  install -d "$MSG_ISO_PATCHED_DIR"/ISO14906_2018_EfcDsrcGenericv7/
  asn1c -D "$MSG_ISO_PATCHED_DIR"/ISO14906_2018_EfcDsrcGenericv7/ -E -generate-ros-message -R -no-gen-example -fcompound-names -fno-include-deps \
    "$ETSI_ITS_BUILD_ROOT_DIR"/asn1/'ISO14906(2018)EfcDsrcGenericv7.asn'

  install -d "$MSG_ISO_PATCHED_DIR"/ISO24534-3_ElectronicRegistrationIdentificationVehicleDataModule/
  asn1c -D "$MSG_ISO_PATCHED_DIR"/ISO24534-3_ElectronicRegistrationIdentificationVehicleDataModule/ -E -generate-ros-message -R -no-gen-example -fcompound-names -fno-include-deps \
    "$ETSI_ITS_BUILD_ROOT_DIR"/asn1/'ISO24534-3_ElectronicRegistrationIdentificationVehicleDataModule-patched.asn'

  echo "ROS - Compiled: Additional Modules"
}
function compile_its_asn1_specs_dir_ROS() {
    install -d "$MSG_ROOT_DIR"/cdd/
      asn1c -D "$MSG_ROOT_DIR"/cdd/ -E -generate-ros-message -R -no-gen-example -fcompound-names -fno-include-deps \
      "$VC_ITS_ASN1_SPECS_DIR"/"$CDD_SPEC_v2"

    echo "ROS - Compiled: ITS ASN1 SPECS DIR"
}
function compile_ROS_DENM() {
  install -d "$MSG_ROOT_DIR"/denm/
  asn1c -D "$MSG_ROOT_DIR"/denm/ -E -generate-ros-message -R -no-gen-example -fcompound-names -fno-include-deps \
    "$VC_ITS_ASN1_SPECS_DIR"/"$DENM_SPEC"

  echo "ROS - Compiled PDU: DENM"
}
function compile_ROS_CAM() {
 install -d "$MSG_ROOT_DIR"/cam/
  asn1c -D "$MSG_ROOT_DIR"/cam/ -E -generate-ros-message -R -no-gen-example -fcompound-names -fno-include-deps \
    "$VC_ITS_ASN1_SPECS_DIR"/"$CAM_SPEC"

  echo "ROS - Compiled PDU: CAM"
}
function compile_ROS_CPM() {
  install -d "$MSG_ROOT_DIR"/cpm/
  asn1c -D "$MSG_ROOT_DIR"/cpm/ -E -generate-ros-message -R -no-gen-example -fcompound-names -fno-include-deps \
    "$VC_ITS_ASN1_SPECS_DIR"/"$CPM_SPEC" \

  echo "ROS - Compiled PDU: CPM"
}
function compile_ROS_POI() {
    echo "ROS - POI not yet implemented"
}
function compile_ROS_SPATEM() {
  install -d "$MSG_ROOT_DIR"/spatem/
  asn1c -D "$MSG_ROOT_DIR"/spatem/ -E -generate-ros-message -R -no-gen-example -fcompound-names -fno-include-deps \
    "$VC_ITS_ASN1_SPECS_DIR"/"$SPATEM_SPEC"

  echo "ROS - Compiled PDU: SPATEM"
}
function compile_ROS_MAPEM() {
  install -d "$MSG_ROOT_DIR"/mapem/
  asn1c -D "$MSG_ROOT_DIR"/mapem/ -E -generate-ros-message -R -no-gen-example -fcompound-names -fno-include-deps \
    "$VC_ITS_ASN1_SPECS_DIR"/"$MAPEM_SPEC"

  echo "ROS - Compiled PDU: MAPEM"
}
function compile_ROS_IVIM() {
  install -d "$MSG_ROOT_DIR"/ivim/
  asn1c -D "$MSG_ROOT_DIR"/ivim/ -E -generate-ros-message -R -no-gen-example -fcompound-names -fno-include-deps \
    "$VC_ITS_ASN1_SPECS_DIR"/"$IVIM_SPEC"

  echo "ROS - Compiled PDU: IVIM"
}
function compile_ROS_EVRSR() {
  echo "ROS - EV-RSR not yet implemented"
}
function compile_ROS_TISTPGTRANSACTION() {
  echo "ROS - TISTPGTRANSACTION not yet implemented"
}
function compile_ROS_SREM() {
  install -d "$MSG_ROOT_DIR"/srem/
  asn1c -D "$MSG_ROOT_DIR"/srem/ -E -generate-ros-message -R -no-gen-example -fcompound-names -fno-include-deps \
    "$VC_ITS_ASN1_SPECS_DIR"/"$SREM_SPEC"

  echo "ROS - Compiled PDU: SREM"
}
function compile_ROS_SSEM() {
  install -d "$MSG_ROOT_DIR"/ssem/
  asn1c -D "$MSG_ROOT_DIR"/ssem/ -E -generate-ros-message -R -no-gen-example -fcompound-names -fno-include-deps \
    "$VC_ITS_ASN1_SPECS_DIR"/"$SSEM_SPEC"

  echo "ROS - Compiled PDU: SSEM"
}
function compile_ROS_EVCSN() {
    echo "ROS - EVCSN not yet implemented"
}
function compile_ROS_SAEM() {
    echo "ROS - SAEM not yet implemented"
}
function compile_ROS_RTCMEM() {
  install -d "$MSG_ROOT_DIR"/rtcmem/
  asn1c -D "$MSG_ROOT_DIR"/rtcmem/ -E -generate-ros-message -R -no-gen-example -fcompound-names -fno-include-deps \
    "$VC_ITS_ASN1_SPECS_DIR"/"$RTCMEM_SPEC"

  echo "ROS - Compiled PDU: RTCMEM"
}
function compile_ROS_VCContainer() {
  install -d "$MSG_ROOT_DIR"/vccontainer/
  asn1c -D "$MSG_ROOT_DIR"/vccontainer/ -E -generate-ros-message -R -no-gen-example -fcompound-names -fno-include-deps \
    "$VC_ITS_ASN1_SPECS_DIR"/"$VC_CONTAINER_SPEC"

  echo "ROS - Compiled PDUs: RawRxMessage, RawTxMessage"
}

function pack_cmake_msg_module() {
  # creates cmake file

  echo "... creating  CMakeLists"
  {
    echo "cmake_minimum_required(VERSION 3.5)"
    echo "project($VC_ITS_LIB_NAME)"

    echo ""
    echo "# Default to C99"
    echo "if (NOT CMAKE_C_STANDARD)"
    echo "    set(CMAKE_C_STANDARD 99)"
    echo "endif()"
    echo ""
    echo "# Default to C++14"
    echo "if (NOT CMAKE_CXX_STANDARD)"
    echo "    set(CMAKE_CXX_STANDARD 14)"
    echo "endif ()"
    echo ""
    echo "if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES '"Clang"')"
    echo "    add_compile_options(-Wall -Wextra -Wpedantic)"
    echo "endif ()"
    echo ""
    echo "# find dependencies"
    echo "find_package(ament_cmake REQUIRED)"
    echo "find_package(std_msgs REQUIRED)"
    echo "find_package(rosidl_default_generators REQUIRED)"
    echo ""
    echo ""
    echo -n "rosidl_generate_interfaces("
    echo -n "$"
    echo -n "{"
    echo -n "PROJECT_NAME"
    echo -n "}"
    echo ""

    SUB='hotfix'
    SUB2='v2x_msgs'

for dir in */*/*/*/*.msg; do
      if [[ "$dir" != *"$SUB"*  ]] && [[  "$dir" != *"$SUB2"* ]]; then
       echo -n '"'
       echo -n $dir
       echo '"'
       fi
    done
    for dir in */*/*/*.msg; do
      if [[ "$dir" != *"$SUB"* ]] && [[  "$dir" != *"$SUB2"* ]]; then
       echo -n '"'
       echo -n $dir
       echo '"'
       fi
    done
    for dir in */*/*.msg; do
      if [[ "$dir" != *"$SUB"* ]] && [[  "$dir" != *"$SUB2"* ]]; then
       echo -n '"'
       echo -n $dir
       echo '"'
       fi
     done
     for dir in */*.msg; do
       if [[ "$dir" != *"$SUB"* ]] && [[ "$dir" != *"$SUB2"* ]]; then
            echo -n '"'
            echo -n $dir
            echo '"'
       fi
      done

    echo "DEPENDENCIES std_msgs"
    echo ")"
    echo ""
    echo "ament_package()"
    echo ""
  } >"$ROOT_DIR/.."/CMakeLists.txt
}
function pack_cmake() {
  # create cmake for .msg-files
  pack_cmake_msg_module

  echo "Packed ROS MSG Library"
  echo ""
  echo "$MSG_ROOT_DIR: $MSG_ROOT_DIR"
  echo ""
}


function clean_build() {
  rm -rf "$ETSI_ITS_BUILD_ROOT_DIR"
  rm -rf "$VC_ITS_LIB_ROOT_DIR"
}
function clean_all() {
  clean_build
  rm -rf "$VC_ITS_ASN1_SPECS_DIR"
}

function replace_file_hotfix() {
  python "$HOTFIX_DIR"
  echo "Replaced files from hotfix folder"
}
function remove_duplicates() {
  python "$DUPLICATES_DIR"
  echo "Removed duplicates"
}
function create_v2x_msgs_folder() {
  install -d "$ROOT_DIR"/v2x_msgs/
  cd "$ROOT_DIR"/v2x_msgs/
  cp -r "$MSG_ROOT_DIR" "$ROOT_DIR"/v2x_msgs/
  cp "$ROOT_DIR"/../CMakeLists.txt "$ROOT_DIR"/v2x_msgs/
  cp "$ROOT_DIR"/../package.xml "$ROOT_DIR"/v2x_msgs/
}


### ### Main Logic ### ###
# load vehicle_captain_its_asn1_specifications library
load_vehicle_captain_its_asn1_specifications

# Check presence of standards
are_standards_present

#loading additional modules for IVIM, MAPEM, RTCMEM, SPATEM, SREM, SSEM
load_additional_modules

# select standard to compile
echo "Select one of the following for compilation"
echo "0 ... Exit"
echo "vc... VC-Container - Container Messages with Rx/Tx Info"
echo "all ... generate all ros messages"
echo "ros ... generate all ros messages"
echo "cmake ... create vehicleCAPTAIN ITS library"
echo "clean ... delete build and lib files"
echo "clean-all ... clean + delete standards"
echo "hotfix ... replace files from hotfix folder"
echo "rd ... remove duplicates"
echo -n "Choice: "
read compilation_choice

case $compilation_choice in

"0")
  echo "exit selected ... nothing to do"
  ;;

"all")
  # generate all ros messages
  compile_ROS_DENM
  compile_ROS_CPM
  compile_ROS_CAM
  compile_ROS_POI
  compile_ROS_SPATEM
  compile_ROS_MAPEM
  compile_ROS_IVIM
  compile_ROS_EVRSR
  compile_ROS_TISTPGTRANSACTION
  compile_ROS_SREM
  compile_ROS_SSEM
  compile_ROS_EVCSN
  compile_ROS_SAEM
  compile_ROS_RTCMEM
  compile_ROS_VCContainer
  compile_additional_modules_ROS
  replace_file_hotfix
  remove_duplicates
  compile_its_asn1_specs_dir_ROS
  pack_cmake
  echo ""
  echo "PDUs ready for cmake packing"
  echo ""
  create_v2x_msgs_folder
  ;;

"vc")
  # VC Rx/Tx Container
  compile_ROS_VCContainer
  pack_cmake
  ;;

"ros")
  # generate all ros messages
  compile_ROS_DENM
  compile_ROS_CAM
  compile_ROS_POI
  compile_ROS_SPATEM
  compile_ROS_MAPEM
  compile_ROS_IVIM
  compile_ROS_EVRSR
  compile_ROS_TISTPGTRANSACTION
  compile_ROS_SREM
  compile_ROS_SSEM
  compile_ROS_EVCSN
  compile_ROS_SAEM
  compile_ROS_RTCMEM
  compile_ROS_VCContainer
  compile_additional_modules_ROS
  replace_file_hotfix
  remove_duplicates
  compile_its_asn1_specs_dir_ROS
  pack_cmake
  create_v2x_msgs_folder
  echo ""
  echo "PDUs ready for cmake packing"
  echo ""
  ;;

"cmake")
  # create cmake package
  pack_cmake
  ;;

"clean")
  # remove build and lib folder
  clean_build
  ;;

"hotfix")
  # remove build and lib folder
  replace_file_hotfix
  pack_cmake
  ;;

"rd")
  # remove duplicates
  remove_duplicates
  pack_cmake
  ;;

"clean-all")
  # remove build and lib folder
  clean_all
  ;;

*)
  echo "invalid choice"
  ;;
esac

echo "++++++++++++++++++++++++"
echo "+++ Done"
echo "++++++++++++++++++++++++"

