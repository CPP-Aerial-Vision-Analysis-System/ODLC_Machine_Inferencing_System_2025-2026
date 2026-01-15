#!/usr/bin/env python3

"""
Enhanced SIYI Camera SD Card Download Utility
Downloads files from a SIYI camera connected via Ethernet

This version includes:
- Better error handling and reporting
- Progress indicators
- File verification
- Automatic retry on failure
- Detailed logging

AP_FLAKE8_CLEAN
"""

import os
import json
import time
import sys
from argparse import ArgumentParser
from urllib.request import urlopen, urlretrieve
from urllib.parse import urlencode
from urllib.error import URLError, HTTPError
from enum import Enum

# Prefix for output messages
PREFIX = "siyi2.py: "
IP_DEFAULT = "192.168.144.25"


class MediaTypes(Enum):
    """Media types on SD card"""
    IMAGE = 0
    VIDEO = 1


# Media type display strings
MEDIA_TYPE_STR = ["images", "videos"]


class Colors:
    """ANSI color codes for terminal output"""
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BLUE = '\033[94m'
    END = '\033[0m'
    BOLD = '\033[1m'


def print_success(msg):
    """Print success message in green"""
    print(f"{Colors.GREEN}✓ {msg}{Colors.END}")


def print_warning(msg):
    """Print warning message in yellow"""
    print(f"{Colors.YELLOW}⚠ {msg}{Colors.END}")


def print_error(msg):
    """Print error message in red"""
    print(f"{Colors.RED}✗ {msg}{Colors.END}")


def print_info(msg):
    """Print info message in blue"""
    print(f"{Colors.BLUE}ℹ {msg}{Colors.END}")


def print_header(msg):
    """Print header message"""
    print(f"\n{Colors.BOLD}{'=' * 80}{Colors.END}")
    print(f"{Colors.BOLD}{msg}{Colors.END}")
    print(f"{Colors.BOLD}{'=' * 80}{Colors.END}\n")


def get_dirlist_url(ip_address, media_type):
    """Get URL for listing directories on SD card"""
    params = {'media_type': media_type}
    return f"http://{ip_address}:82/cgi-bin/media.cgi/api/v1/getdirectories?" + urlencode(params)


def get_filelist_url(ip_address, media_type, dir_path):
    """Get URL for listing files in a directory"""
    params = {
        'media_type': str(media_type),
        'path': dir_path,
        'start': 0,
        'count': 9999
    }
    return f"http://{ip_address}:82/cgi-bin/media.cgi/api/v1/getmedialist?" + urlencode(params)


def test_connection(ip_address):
    """
    Test connection to camera before attempting download
    Returns True if connection is successful
    """
    print_info(f"Testing connection to camera at {ip_address}...")
    
    try:
        # Try to get directories for images
        url = get_dirlist_url(ip_address, MediaTypes.IMAGE.value)
        with urlopen(url, timeout=5) as response:
            data = json.load(response)
            if data.get('success', False):
                print_success("Connection successful!")
                return True
            else:
                print_error(f"Camera returned error: {data.get('message', 'Unknown')}")
                return False
    except URLError as e:
        print_error(f"Cannot connect to camera: {e}")
        print_info("Troubleshooting steps:")
        print_info("  1. Check Ethernet cable is connected")
        print_info("  2. Verify camera IP with: ping 192.168.144.25")
        print_info("  3. Test API with: curl http://192.168.144.25:82/cgi-bin/media.cgi/api/v1/getdirectories?media_type=0")
        return False
    except Exception as e:
        print_error(f"Connection test failed: {e}")
        return False


def verify_file(filepath, min_size=1024):
    """
    Verify that downloaded file exists and has valid size
    Returns True if file is valid
    """
    try:
        if not os.path.exists(filepath):
            print_error(f"File not found: {filepath}")
            return False
        
        size = os.path.getsize(filepath)
        if size < min_size:
            print_error(f"File too small ({size} bytes): {filepath}")
            return False
        
        return True
    except Exception as e:
        print_error(f"Error verifying file: {e}")
        return False


def download_with_retry(url, dest_path, max_retries=3):
    """
    Download file with automatic retry on failure
    Returns True if successful
    """
    for attempt in range(max_retries):
        try:
            if attempt > 0:
                print_warning(f"Retry attempt {attempt + 1}/{max_retries}")
                time.sleep(1)
            
            # Download file
            urlretrieve(url, dest_path)
            
            # Verify downloaded file
            if verify_file(dest_path):
                size = os.path.getsize(dest_path)
                print_success(f"Downloaded: {os.path.basename(dest_path)} ({size/1024:.1f}KB)")
                return True
            else:
                # Remove invalid file
                try:
                    os.remove(dest_path)
                except:
                    pass
                
        except (URLError, HTTPError) as e:
            print_error(f"Download error: {e}")
        except Exception as e:
            print_error(f"Unexpected error: {e}")
    
    print_error(f"Failed to download after {max_retries} attempts")
    return False


def download_files(ip_address, dest_dir, media_types=None, verbose=False):
    """
    Download files from camera's SD card
    
    Args:
        ip_address: Camera IP address
        dest_dir: Destination directory for downloads
        media_types: List of media types to download (None = all)
        verbose: Print detailed information
    
    Returns:
        Dictionary with statistics
    """
    if media_types is None:
        media_types = [mt.value for mt in MediaTypes]
    
    stats = {
        'total_files': 0,
        'successful': 0,
        'failed': 0,
        'total_size': 0
    }
    
    # Test connection first
    if not test_connection(ip_address):
        print_error("Connection test failed. Aborting download.")
        return stats
    
    # Download each media type
    for media_type in media_types:
        print_header(f"DOWNLOADING {MEDIA_TYPE_STR[media_type].upper()}")
        
        try:
            # Get list of directories
            dir_list_url = get_dirlist_url(ip_address, media_type)
            
            if verbose:
                print_info(f"Fetching directories from: {dir_list_url}")
            
            with urlopen(dir_list_url, timeout=10) as get_dir_url:
                dir_dict = json.load(get_dir_url)
                
                # Check success
                if not dir_dict.get('success', False):
                    print_error(f"Failed to get directories: {dir_dict.get('message', 'Unknown error')}")
                    continue
                
                # Get directories
                if 'data' not in dir_dict or 'directories' not in dir_dict['data']:
                    print_warning("No directories found")
                    continue
                
                directories = dir_dict['data']['directories']
                dir_list = [d['path'] for d in directories if 'path' in d]
                
                print_info(f"Found {len(dir_list)} directories")
                
                if verbose:
                    for d in dir_list:
                        print(f"  - {d}")
                
                # Process each directory
                for dir_index, dir_path in enumerate(dir_list, 1):
                    print(f"\n[{dir_index}/{len(dir_list)}] Processing: {dir_path}")
                    
                    # Get file list
                    filenames_url = get_filelist_url(ip_address, media_type, dir_path)
                    
                    if verbose:
                        print_info(f"Fetching file list from: {filenames_url}")
                    
                    with urlopen(filenames_url, timeout=10) as get_filenames_url:
                        filename_dict = json.load(get_filenames_url)
                        
                        # Check success
                        if not filename_dict.get('success', False):
                            print_error(f"Failed to get file list: {filename_dict.get('message', 'Unknown')}")
                            continue
                        
                        # Get file list
                        if 'data' not in filename_dict or 'list' not in filename_dict['data']:
                            print_warning("No files found in directory")
                            continue
                        
                        file_list = filename_dict['data']['list']
                        print_info(f"Found {len(file_list)} files")
                        
                        # Download each file
                        for file_index, fileinfo in enumerate(file_list, 1):
                            if 'name' not in fileinfo or 'url' not in fileinfo:
                                print_warning("Invalid file info, skipping")
                                continue
                            
                            filename = fileinfo['name']
                            file_url = fileinfo['url']
                            
                            # Fix incorrect IP in URL
                            file_url_fixed = file_url.replace(IP_DEFAULT, ip_address)
                            
                            # Destination path
                            dest_filename = os.path.join(dest_dir, filename)
                            
                            # Skip if already exists
                            if os.path.exists(dest_filename):
                                if verify_file(dest_filename):
                                    print_info(f"[{file_index}/{len(file_list)}] Already exists: {filename}")
                                    stats['total_files'] += 1
                                    stats['successful'] += 1
                                    stats['total_size'] += os.path.getsize(dest_filename)
                                    continue
                            
                            # Download
                            print(f"[{file_index}/{len(file_list)}] Downloading: {filename}")
                            
                            if verbose:
                                print_info(f"URL: {file_url_fixed}")
                            
                            stats['total_files'] += 1
                            
                            if download_with_retry(file_url_fixed, dest_filename):
                                stats['successful'] += 1
                                stats['total_size'] += os.path.getsize(dest_filename)
                            else:
                                stats['failed'] += 1
        
        except URLError as e:
            print_error(f"Network error: {e}")
        except HTTPError as e:
            print_error(f"HTTP error: {e}")
        except Exception as e:
            print_error(f"Unexpected error: {e}")
    
    return stats


def main():
    """Main function"""
    parser = ArgumentParser(description=__doc__)
    parser.add_argument(
        "--ipaddr", 
        default=IP_DEFAULT, 
        help=f"IP address of camera (default: {IP_DEFAULT})")
    parser.add_argument(
        "--dest", 
        default=".", 
        help="Destination directory for downloads (default: current directory)")
    parser.add_argument(
        "--images-only", 
        action="store_true", 
        help="Download only images, not videos")
    parser.add_argument(
        "--videos-only", 
        action="store_true", 
        help="Download only videos, not images")
    parser.add_argument(
        "--verbose", "-v",
        action="store_true",
        help="Print detailed information")
    args = parser.parse_args()
    
    # Print header
    print_header("SIYI CAMERA SD CARD DOWNLOAD UTILITY")
    
    # Check destination directory
    if not os.path.exists(args.dest):
        print_error(f"Destination directory does not exist: {args.dest}")
        print_info("Creating directory...")
        try:
            os.makedirs(args.dest, exist_ok=True)
            print_success(f"Created: {args.dest}")
        except Exception as e:
            print_error(f"Failed to create directory: {e}")
            sys.exit(1)
    
    # Determine media types to download
    if args.images_only:
        media_types = [MediaTypes.IMAGE.value]
    elif args.videos_only:
        media_types = [MediaTypes.VIDEO.value]
    else:
        media_types = [MediaTypes.IMAGE.value, MediaTypes.VIDEO.value]
    
    # Print configuration
    print_info(f"Camera IP: {args.ipaddr}")
    print_info(f"Destination: {os.path.abspath(args.dest)}")
    media_str = " and ".join([MEDIA_TYPE_STR[mt] for mt in media_types])
    print_info(f"Downloading: {media_str}")
    
    # Start download
    start_time = time.time()
    stats = download_files(args.ipaddr, args.dest, media_types, args.verbose)
    elapsed = time.time() - start_time
    
    # Print summary
    print_header("DOWNLOAD SUMMARY")
    print(f"  Total files:      {stats['total_files']}")
    print(f"  {Colors.GREEN}Successful:       {stats['successful']}{Colors.END}")
    if stats['failed'] > 0:
        print(f"  {Colors.RED}Failed:           {stats['failed']}{Colors.END}")
    else:
        print(f"  Failed:           {stats['failed']}")
    print(f"  Total size:       {stats['total_size']/1024/1024:.2f} MB")
    print(f"  Time elapsed:     {elapsed:.1f} seconds")
    
    if stats['successful'] == stats['total_files'] and stats['total_files'] > 0:
        print_success("All files downloaded successfully!")
        sys.exit(0)
    elif stats['failed'] > 0:
        print_error("Some files failed to download")
        sys.exit(1)
    else:
        print_warning("No files found to download")
        sys.exit(0)


if __name__ == "__main__":
    main()
