# Usage Guide

This library is used to generate ROS2 messages in vehicleCAPTAIN toolbox.

Quickstart: simply include into your ROS2 project.

## Quickstart Guide for Docker environment (by Christoph Pilz)
**Release versions** have been tested in use and should be stable. Some scenarios are still in progess.

If you want to make changes in the branch, you can visit https://confluence.v2c2.at/display/VVP/ASN.1ToROS-Generator for more information about the adapted asn1ToROS-code.

This approach requires [Docker](https://www.docker.com/).

**Important:** You have to supply the docker with a keyfile for accessing the vif repo.
Simply put a working id_rsa in the root folder. The Dockerfile does the rest.

```bash
# Generate the docker
./docker-generation-environment-build-and-run.sh

# Connect to the docker
docker exec -it ros2_msg_gen bash

# Execute the script
./command.sh

# exit the docker
exit

# copy the generated library from the docker
docker cp ros2_msg_gen:/tmp/gen_env/build/v2x_msgs ./v2x_msgs

# your generated files are now in the vifits folder
cp -r v2x_msgs <destination>
```


## Generation per hand
If you want to generate .msg-files per hand you have to follow these steps:
```
1. Download branch 'ros2_msg_parser_from_velichkov': https://github.com/virtual-vehicle/vehicle_captain_asn1_parser
2. Use following commands:
    test -f configure || autoreconf -iv
    ./configure
    make
3. Create new directory for your new .msg-files:
    install -d "$MSG_ROOT_DIR"/denm/ 
4. Generate the messages (use "-generate-ros-message"-tag):
    asn1c -D "$dir_for_new_msg" -E -generate-ros-message -R -no-gen-example -fcompound-names -fno-include-deps "$ETSI_SPECS"
5. Check if new messages are in new folder.
6. Done.
```

## "hotfix"-Folder:
(Because some cases are not handled with the parser at the moment.)
```
In the "hotfix"folder are messages for scenarios which are not handled with the parser at the moment.
A python-file replaces the original files with the adapted files from the "hotfix"-folder.
In each message-file you can find a short description what has been changed manually.
```

## Author and License
Main Author: Patrizia Neubauer

License: BSD
