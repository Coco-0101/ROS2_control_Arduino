#!/bin/bash
# Program:
#       This program runs for TEL frisbee competition (SSH).
# History:
# 2023/12/07    Betty Lin       First release
PATH=/bin:/sbin:/usr/bin:/usr/sbin:/usr/local/bin:/usr/local/sbin:~/bin
export PATH
echo -e "Happy Cat ヽ(=^･ω･^=)丿- SSH \a \n"
 
#REMOTE_IP="140.127.205.178"
REMOTE_IP="192.168.0.165"

PORT_NUMBER="22"
USERNAME="ical"
 
# 複製腳本到遠端AGX
scp -P ${PORT_NUMBER} tel_agx.sh ${USERNAME}@${REMOTE_IP}:~/bin/tel_agx.sh

# 透過 SSH 連線到遠端主機並執行腳本
ssh -p ${PORT_NUMBER} ${USERNAME}@${REMOTE_IP} 'bash -s' < ~/bin/tel_agx.sh

  
exit 0

