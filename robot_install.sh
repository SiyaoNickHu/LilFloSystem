#!/bin/bash

set -e

## Adding to the bashrc file
grep -qxF 'source ~/catkin_ws/src/LilFloSystem/bash_includes' ~/.bashrc || echo 'source ~/catkin_ws/src/LilFloSystem/bash_includes' >> ~/.bashrc

## Installing AWS CLI
prior=$(pwd)
cd ~/Downloads
curl "https://s3.amazonaws.com/aws-cli/awscli-bundle.zip" -o "awscli-bundle.zip"
unzip awscli-bundle.zip
sudo ./awscli-bundle/install -i /usr/local/aws -b /usr/local/bin/aws
rm aswcli-bundle.zip
rm -rf awscli-bundle
cd $prior

### Installing boto3 for AWS stuff. Not 100% sure this is needed
# It looks like rosdep now handles this
#sudo apt install -y python3-pip
#pip3 install -U boto3

source gen_install.sh