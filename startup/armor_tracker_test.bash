#!/bin/bash

# HACK: 因为fakeap需要两次signal才能退出(ftxui、iceoryx)，
# 需要将signal重新映射为执行空命令， 以避免bash捕获第二次signal而提前退出脚本
trap ':' SIGINT
process-compose -f configs/startup/armor_tracker_test.yaml -D >/dev/null
build/linux/x86_64/release/fake_ap
process-compose down
