#!/usr/bin/env python3

"""
Simple test: Download all images from SD card without camera trigger
This tests the siyi.py download functionality in isolation
"""

import requests
import os
import time
from urllib.parse import urlencode

CAM_IP = "192.168.144.25"
MEDIA_PORT = 82
BASE_URL = f"http://{CAM_IP}:{MEDIA_PORT}/cgi-bin/media.cgi/api/v1"

DEST_DIR = "/home/astra-dev/astra/ros2_ws/video_cam_data/test_download"

def download_all_from_sd():
    """Download all photos and videos from SD card"""
    
    os.makedirs(DEST_DIR, exist_ok=True)
    
    print("=" * 80)
    print(" TESTING SD CARD DOWNLOAD")
    print("=" * 80)
    
    total_downloaded = 0
    
    # Download images (media_type=0)
    for media_type in [0]:
        media_type_str = "images" if media_type == 0 else "videos"
        print(f"📥 Downloading {media_type_str}...")
        
        # Get directories
        dir_url = f"{BASE_URL}/getdirectories?media_type={media_type}"
        response = requests.get(dir_url, timeout=5)
        
        if response.status_code != 200:
            print(f"✗ Failed to get directories: HTTP {response.status_code}")
            continue
        
        dir_data = response.json()
        
        if not dir_data.get('success', False):
            print("✗ API returned success=false")
            continue
        
        directories = dir_data.get('data', {}).get('directories', [])
        print(f"  Found {len(directories)} directories")
        
        # Process each directory
        for directory in directories:
            dir_path = directory.get('path', '')
            if not dir_path:
                continue
            
            print(f"  📁 Directory: {dir_path}")
            
            # Get file list
            file_url = f"{BASE_URL}/getmedialist"
            params = {
                'media_type': str(media_type),
                'path': dir_path,
                'start': 0,
                'count': 9999
            }
            
            file_response = requests.get(file_url, params=params, timeout=5)
            
            if file_response.status_code != 200:
                print(f"    ✗ Failed to get file list: HTTP {file_response.status_code}")
                continue
            
            file_data = file_response.json()
            
            if not file_data.get('success', False):
                print("    ✗ File list API returned success=false")
                continue
            
            file_list = file_data.get('data', {}).get('list', [])
            print(f"    📄 {len(file_list)} files to download")
            
            # Download first 5 files as a test
            for i, fileinfo in enumerate(file_list[:5]):
                filename = fileinfo.get('name', '')
                file_url = fileinfo.get('url', '')
                
                if not filename or not file_url:
                    continue
                
                # Fix IP address
                file_url = file_url.replace("192.168.144.25", CAM_IP)
                
                dest_file = os.path.join(DEST_DIR, filename)
                
                # Skip if exists
                if os.path.exists(dest_file):
                    print(f"      ⏭️  Skip (exists): {filename}")
                    continue
                
                try:
                    print(f"      ⬇️  Downloading {i+1}/5: {filename}")
                    
                    response = requests.get(file_url, timeout=30)
                    if response.status_code == 200:
                        with open(dest_file, 'wb') as f:
                            f.write(response.content)
                        
                        size_kb = len(response.content) / 1024
                        print(f"      ✅ Saved: {filename} ({size_kb:.1f}KB)")
                        total_downloaded += 1
                    else:
                        print(f"      ✗ HTTP {response.status_code}: {filename}")
                except Exception as e:
                    print(f"      ✗ Error: {e}")
    
    print("=" * 80)
    print(f" ✅ TEST COMPLETE: {total_downloaded} files downloaded")
    print(f"    Destination: {DEST_DIR}")
    print("=" * 80)
    
    return total_downloaded

if __name__ == '__main__':
    download_all_from_sd()
