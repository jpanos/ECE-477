#!/bin/bash

wait_file() {
	  local file="$1"; shift
	    local wait_seconds="${1:-10}"; shift # 10 seconds as default timeout

	      until test $((wait_seconds--)) -eq 0 -o -e "$file" ; do sleep 1; done

	        ((++wait_seconds))
	}


rm /tmp/flower.socket
./camera_jpeg_capture --max-disparity=160 --blocksize=11 --algorithm=bm --scale=.50 -i=intrinsics.yml -e=extrinsics.yml --no-display &
wait_file "/tmp/flower.socket" && {
  python3 uart_mod.py "./camera_jpeg_capture --max-disparity=160 --blocksize=11 --algorithm=bm --scale=.50 -i=intrinsics.yml -e=extrinsics.yml --no-display"
}
pkill -SIGINT 
wait
rm /tmp/flower.socket
