1) verify that the jetson sees the camera on the eth
    ip addr
2) now test 
    ping 192.168.144.25
3) Verify the camera’s HTTP API is reachable
    curl http://192.168.144.25:82/cgi-bin/media.cgi/api/v1/getdirectories?media_type=0

    should see 
{
  "success": true,
  "data": {
    "directories": [
      {"name": "101SIYI_IMG", "path": "101SIYI_IMG"}
    ]
  }
}

4) 
mkdir -p ~/siyi_test
cd ~/siyi_test
mkdir -p ~/downloads

5) save the script as siyi-download.py
make it executable 
chmod +x siyi-download.py

6) run it 
    python3 siyi-download.py --ipaddr 192.168.144.25 --dest ~/siyi_test/downloads

7) You should see logs like:

siyi-download.py: downloading image files
siyi-download.py: 1 directories
siyi-download.py: 25 files
siyi-download.py: downloading IMG_0001.JPG from http://192.168.144.25:82/..